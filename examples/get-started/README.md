# BSP: Generic I2C and LCD Example

## Menu config with predefined configuration

### 0.85 128*128 GC9107

https://item.taobao.com/item.htm?id=778379851892

    idf.py -D "SDKCONFIG_DEFAULTS=sdkconfig.npf085b" menuconfig

### 0.96 80*160 ST7735S

https://item.taobao.com/item.htm?id=611336611828

    idf.py -D "SDKCONFIG_DEFAULTS=sdkconfig.npf096h" menuconfig

### 1.77 128*160 ST7735

    idf.py -D "SDKCONFIG_DEFAULTS=sdkconfig.hd17015c12" menuconfig

### 2.4 240*320 ST7789

    idf.py -D "SDKCONFIG_DEFAULTS=sdkconfig.tm024hdh26" menuconfig

## Build with 2.4 240*320 ST7789 on ESP32 chip

    idf.py -D "SDKCONFIG_DEFAULTS=sdkconfig.tm024hdh26;sdkconfig.esp32" build
