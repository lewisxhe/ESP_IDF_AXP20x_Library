# ESP-IDF AXP20x_Library example

This is an example of using [AXP202X_Library](https://github.com/lewisxhe/AXP202X_Library) to program in esp-idf. For more examples, please refer to [AXP202X_Library](https://github.com/lewisxhe/AXP202X_Library)



-------------------------------
After setting up the correct environment

```
git clone --recursive https://github.com/lewisxhe/ESP_IDF_AXP20x_Library.git
cd ESP_IDF_AXP20x_Library
idf.py menuconfig
idf.py build
idf.py -p [PORT] -b [BAUD] flash
```