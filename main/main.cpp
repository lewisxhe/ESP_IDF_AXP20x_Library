/////////////////////////////////////////////////////////////////
/*
MIT License

Copyright (c) 2020 lewis he

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

main.cpp - AXP202X_Library idf Programming example
Created by Lewis he on July 29, 2020.
github:https://github.com/lewisxhe/ESP_IDF_AXP20x_Library
*/
/////////////////////////////////////////////////////////////////

extern "C" {

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
}
#include "axp20x.h"



static const char *TAG = "AXP20x";

#define _I2C_NUMBER(num)            I2C_NUM_##num
#define I2C_NUMBER(num)             _I2C_NUMBER(num)

#define AXP_ISR_GPIO                (gpio_num_t)CONFIG_AXPxxx_INTERRUPT_PIN  /*!< axp power chip interrupt Pin*/
#define GPIO_INPUT_PIN_SEL          (1ULL<<AXP_ISR_GPIO)

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL                   /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA                   /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM)  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY             /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                                       /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                                       /*!< I2C master doesn't need buffer */

#define WRITE_BIT                   I2C_MASTER_WRITE                        /*!< I2C master write */
#define READ_BIT                    I2C_MASTER_READ                         /*!< I2C master read */
#define ACK_CHECK_EN                0x1                                     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS               0x0                                     /*!< I2C master will not check ack from slave */
#define ACK_VAL                     (i2c_ack_type_t)0x0                     /*!< I2C ack value */
#define NACK_VAL                    (i2c_ack_type_t)0x1                     /*!< I2C nack value */

AXP20X_Class axp;

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR axp_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void axp_irq_init()
{

    gpio_config_t io_conf;
    //enable interrupt
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    //change gpio intrrupt type for one pin
    gpio_set_intr_type(AXP_ISR_GPIO, GPIO_INTR_NEGEDGE);
    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(AXP_ISR_GPIO, axp_isr_handler, (void *) AXP_ISR_GPIO);
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief apx library i2c read callblack
 */
uint8_t twi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    if (len == 0) {
        return ESP_OK;
    }
    if (data == NULL) {
        return ESP_FAIL;
    }
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret =  i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | READ_BIT, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, &data[len - 1], NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief apx library i2c write callblack
 */
uint8_t twi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    if (data == NULL) {
        return ESP_FAIL;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, data, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());

    axp_irq_init();

    if (axp.begin(twi_read, twi_write)) {
        ESP_LOGE(TAG, "Error init axp20x !!!");
        while (1);
    }
    ESP_LOGI(TAG, "Success init axp20x !!!");

    axp.setPowerOutPut(AXP202_DCDC3, AXP202_ON);
    axp.setPowerOutPut(AXP202_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP202_LDO2, AXP202_ON);
    axp.setPowerOutPut(AXP202_LDO4, AXP202_ON);
    axp.setPowerOutPut(AXP202_DCDC2, AXP202_ON);
    axp.setLDO4Voltage(AXP202_LDO4_1800MV);
    axp.setLDO3Voltage(3500);
    axp.setPowerOutPut(AXP202_LDO3, AXP202_ON);

    if ( axp.isDCDC2Enable()) {
        ESP_LOGI(TAG, "DC2: %u mV", axp.getDCDC2Voltage());
    } else {
        ESP_LOGI(TAG, "DC2: DISABLE");
    }

    if (axp.isDCDC3Enable()) {
        ESP_LOGI(TAG, "DC3: %u mV", axp.getDCDC3Voltage());
    } else {
        ESP_LOGI(TAG, "DC3: DISABLE");
    }

    if (axp.isLDO2Enable()) {
        ESP_LOGI(TAG, "LDO2: %u mV", axp.getLDO2Voltage());
    } else {
        ESP_LOGI(TAG, "LDO2: DISABLE");
    }

    if (axp.isLDO3Enable()) {
        ESP_LOGI(TAG, "LDO3: %u mV", axp.getLDO3Voltage());
    } else {
        ESP_LOGI(TAG, "LDO3: DISABLE");
    }

    if (axp.isLDO4Enable()) {
        ESP_LOGI(TAG, "LDO4: %u mV", axp.getLDO4Voltage());
    } else {
        ESP_LOGI(TAG, "LDO4: DISABLE");
    }

    if (axp.isExtenEnable()) {
        ESP_LOGI(TAG, "Exten: ENABLE");
    } else {
        ESP_LOGI(TAG, "Exten: DISABLE");
    }

    //When the chip is axp192 / 173, the allowed values are 0 ~ 15,
    //corresponding to the axp1xx_charge_current_t enumeration
    // axp.setChargeControlCur(AXP1XX_CHARGE_CUR_550MA);

    //axp202 allows maximum charging current of 1800mA, minimum 300mA
    axp.setChargeControlCur(300);

    ESP_LOGI(TAG, "setChargeControlCur:%u", axp.getChargeControlCur());

    //! Enable all irq channel
    axp.enableIRQ(AXP202_ALL_IRQ, true);

    axp.clearIRQ();

    uint32_t io_num;
    for (;;) {

        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if (io_num == AXP_ISR_GPIO) {
                axp.readIRQ();
                if (axp.isPEKShortPressIRQ()) {
                    ESP_LOGI(TAG, "AXP202 PEK key Click");
                }
                axp.clearIRQ();
            }
        }
    }
}
