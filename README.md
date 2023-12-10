ESP32-CAM with ILI9341 example
===

# Taken from the ESP examples, and expanded

* This sketch is a extension/expansion/rework of the 'official' ESP32 Camera example sketch from Espressif:
* [https://github.com/espressif/esp32-camera/examples/camera_example](https://github.com/espressif/esp32-camera/examples/camera_example)

* But expanded with:
  * Show image on ili9341 tft lcd screen decoded by tjpgd.
  * Set wifi's ssid and password via cmake paramerater.

* Test on AI-THINKER Board with OV2640 camera and ILI9341 2.8 inch TFT SPI 240x320 v1.2 screen
* ![test_esp32_cam.jpg](test_esp32_cam.jpg)