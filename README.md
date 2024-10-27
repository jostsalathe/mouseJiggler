# mouseJiggler
An ESP32 based bluetooth mouse jiggler to subtly keep the computer awake.
It uses SCROLL_LOCK to enable jigglieness and can also control it with a button.
State is shown with 9 WS2812B LEDs.

WARNING! This project is really messy!

It is directly based on [this example from Espressif](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/ble/ble_hid_device_demo) and [this small WS2812B lib](https://github.com/JSchaenzle/ESP32-NeoPixel-WS2812-RMT).

## Functional description

The button on GPIO 13 has dual function:
- Short press toggles Scroll Lock through Bluetooth HID
- Long press toggles through wiggle patterns

The number of the LEDs lit shows the selected pattern:
0. jiggleModeStealth - 1 LED: quickly jumping from left to right by one pixel (10 steps/second)
1. jiggleModeCircle - 3 LEDs: quickly draw a circle of 20 pixels in diameter (66,7 steps/second)
2. jiggleModePathSlow - 5 LEDs: slowly draw a dick (376 steps with 1 step/second)
3. jiggleModePathFast - 9 LEDs: quickly draw a dick (376 steps with 66,7 steps/second)

Color of the LEDs:
- Blue: Bluetooth unconnected and advertising connection
- Green: Bluetooth connected but Scroll Lock (aka. jiggling) is **off**
- Red: Bluetooth connected and Scroll Lock (aka. jiggling) is **on**
