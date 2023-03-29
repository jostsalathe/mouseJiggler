# mouseJiggler
An ESP32 based bluetooth mouse jiggler to subtly keep the computer awake.
It uses SCROLL_LOCK to enable jigglieness and can also control it with a button.
State is shown with 9 WS2812B LEDs.

WARNING! This project is really messy!

It is directly based on [this example from Espressif](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/ble/ble_hid_device_demo) and [this small WS2812B lib](https://github.com/JSchaenzle/ESP32-NeoPixel-WS2812-RMT).
