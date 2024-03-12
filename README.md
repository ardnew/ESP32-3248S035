# esp32-3248s035

This repository contains a barebones board support package (BSP) for the Sunton ESP32-3248S035(C) with a minimal API that supports all onboard peripherals:

|:Peripheral:|:Interface:|
|LCD (ST7796)|SPI|
|Capacitive touch (GT911)|I²C|
|Audio amplifier|PWM|
|RGB LED|PWM|
|Photoresistor|ADC|

The LCD graphics and touch support are provided directly by [lvgl](https://github.com/lvgl/lvgl) version 9.0. To minimize dependencies and overhead, it drives the SoC peripherals directly instead of using intermediate driver libraries (e.g., TFT_eSPI or LoyvanGFX).

[Example projects](examples) exist for both PlatformIO and Arduino.

## Layout

The following image is sourced from [macsbug](https://macsbug.wordpress.com/2022/10/02/esp32-3248s035/). Note this represents the board variant with resistive touch (XPT2046) instead of capacitive touch (GT911). It is extremely useful, regardless:

![ESP32-3248S035R](docs/pcb-layout.jpg)
