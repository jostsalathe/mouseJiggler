/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"

#include "esp_timer.h"
#include <math.h>
#include "ws2812_control.h"

/**
 * Brief:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/**
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor,
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

#define HID_DEMO_TAG "MouseJiggler_App"

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "MouseJiggler"
static uint8_t hidd_service_uuid128[] =
{
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;


typedef enum
{
    jiggleModeStealth = 0,
    jiggleModeCircle,
    jiggleModePathSlow,
    jiggleModePathFast,
    jiggleModeInvalid
} jiggleMode_t;

#define LED_EN_GPIO 16
#define BUT_USR_GPIO 13
#define COLOR_DISCONNECTED 0x000004
#define COLOR_JIGGLE_FORTH 0x000500
#define COLOR_JIGGLE_BACK 0x000400
#define COLOR_JIGGLE_NOT 0x040000

jiggleMode_t jiggleMode = jiggleModeStealth;
bool jiggleEnabled = false;
uint32_t jigglePathPos = 0;
int64_t nextJiggle_us;

void enterJiggleModeStealth()
{
    jiggleMode = jiggleModeStealth;
    jigglePathPos = 0;
    nextJiggle_us = esp_timer_get_time();
    ESP_LOGI(HID_DEMO_TAG, "selected jiggleModeStealth");
}

void doJiggleModeStealth()
{
    if (esp_timer_get_time() >= nextJiggle_us)
    {
        nextJiggle_us += 100 * 1000;
        esp_hidd_send_mouse_value(hid_conn_id, 0,
                                  jigglePathPos ? 1 : -1,
                                  0);
        jigglePathPos = !jigglePathPos;
    }
}

void enterJiggleModeCircle()
{
    jiggleMode = jiggleModeCircle;
    jigglePathPos = 0;
    nextJiggle_us = esp_timer_get_time();
    ESP_LOGI(HID_DEMO_TAG, "selected jiggleModeCircle");
}

void doJiggleModeCircle()
{
    if (esp_timer_get_time() >= nextJiggle_us)
    {
        nextJiggle_us += 15 * 1000;
        esp_hidd_send_mouse_value(hid_conn_id, 0,
                                  sin(M_TWOPI*jigglePathPos/360)*-10,
                                  cos(M_TWOPI*jigglePathPos/360)*10);
        jigglePathPos = (jigglePathPos+6)%360;
    }
}

void enterJiggleModePathSlow()
{
    jiggleMode = jiggleModePathSlow;
    jigglePathPos = 0;
    nextJiggle_us = esp_timer_get_time();
    ESP_LOGI(HID_DEMO_TAG, "selected jiggleModePathSlow");
}

void enterJiggleModePathFast()
{
    jiggleMode = jiggleModePathFast;
    jigglePathPos = 0;
    nextJiggle_us = esp_timer_get_time();
    ESP_LOGI(HID_DEMO_TAG, "selected jiggleModePathFast");
}

// x positive is right, y positive is down
#define N_VECTORS 376
const uint8_t jiggleVectors[N_VECTORS][2] =
{
    {5, 6}, {6, 5}, {6, 4}, {6, 4}, {6, 4}, {7, 4}, {7, 3}, {7, 2},
    {7, 2}, {7, 2}, {8, 1}, {7, 0}, {8, 1}, {7, -1}, {8, 0}, {7, -2},
    {7, -1}, {8, -3}, {7, -2}, {6, -3}, {7, -4}, {6, -4}, {6, -4}, {6, -5},
    {6, -5}, {5, -5}, {5, -6}, {4, -6}, {4, -6}, {4, -7}, {3, -7}, {3, -7},
    {2, -7}, {2, -7}, {2, -7}, {0, -8}, {1, -7}, {0, -8}, {-1, -7}, {-1, -8},
    {-1, -7}, {-2, -7}, {-2, -7}, {-3, -7}, {-3, -7}, {-4, -6}, {-4, -7}, {-5, -6},
    {-5, -5}, {-5, -6}, {-5, -5}, {-6, -4}, {-6, -5}, {-7, -4}, {-6, -3}, {-7, -3},
    {-7, -3}, {-7, -2}, {-8, -1}, {-10, -2}, {-4, -1}, {-4, -3}, {-4, -2}, {-4, -4},
    {-2, -4}, {-3, -4}, {-1, -4}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5},
    {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5},
    {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5},
    {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5},
    {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5},
    {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5},
    {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5},
    {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5},
    {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5}, {0, -5},
    {0, -5}, {0, -5}, {0, -5}, {0, -5}, {7, -5}, {3, -3}, {4, -3}, {3, -4},
    {3, -3}, {3, -7}, {3, -5}, {1, -6}, {1, -3}, {0, -3}, {1, -3}, {0, -4},
    {0, -4}, {0, -4}, {-1, -8}, {0, -5}, {-2, -9}, {-1, -4}, {-2, -10}, {-2, -5},
    {-3, -10}, {-2, -5}, {-4, -9}, {-5, -10}, {-3, -5}, {-3, -4}, {-2, -5}, {-3, -4},
    {-4, -5}, {-3, -4}, {-3, -4}, {-3, -4}, {-4, -3}, {-3, -4}, {-4, -3}, {-4, -3},
    {-4, -3}, {-3, -2}, {-4, -3}, {-4, -2}, {-4, -2}, {-5, -1}, {-4, -1}, {-4, -1},
    {-4, -1}, {-5, -1}, {-4, 0}, {-4, 0}, {-5, 1}, {-4, 1}, {-4, 1}, {-4, 1},
    {-5, 1}, {-4, 2}, {-4, 2}, {-4, 3}, {-3, 2}, {-4, 3}, {-4, 3}, {-4, 3},
    {-3, 4}, {-4, 3}, {-3, 4}, {-3, 4}, {-3, 4}, {-4, 5}, {-3, 4}, {-2, 5},
    {-3, 4}, {-5, 10}, {-3, 5}, {-4, 9}, {-4, 10}, {-3, 10}, {-2, 10}, {-1, 4},
    {-2, 9}, {0, 5}, {-1, 8}, {0, 4}, {0, 4}, {0, 4}, {1, 3}, {0, 3},
    {1, 3}, {1, 3}, {1, 5}, {2, 3}, {2, 5}, {4, 6}, {3, 3}, {4, 3},
    {3, 3}, {7, 6}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5},
    {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5},
    {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5},
    {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5},
    {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5},
    {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5},
    {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5},
    {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5},
    {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5}, {0, 5},
    {0, 5}, {0, 5}, {-1, 6}, {-1, 4}, {-2, 2}, {-2, 3}, {-2, 3}, {-4, 3},
    {-4, 3}, {-4, 1}, {-3, 1}, {-9, 2}, {-8, 1}, {-7, 2}, {-7, 3}, {-7, 3},
    {-6, 3}, {-7, 4}, {-6, 5}, {-6, 4}, {-5, 5}, {-5, 6}, {-5, 5}, {-5, 6},
    {-4, 7}, {-4, 6}, {-3, 7}, {-3, 7}, {-2, 7}, {-2, 7}, {-1, 7}, {-1, 8},
    {-1, 7}, {0, 8}, {1, 7}, {0, 8}, {2, 7}, {2, 7}, {2, 7}, {3, 7},
    {3, 7}, {4, 7}, {4, 6}, {4, 6}, {5, 6}, {5, 5}, {6, 5}, {6, 5},
    {6, 4}, {6, 4}, {7, 4}, {6, 3}, {7, 2}, {8, 3}, {7, 1}, {7, 2},
    {8, 0}, {7, 1}, {8, -1}, {7, 0}, {8, -1}, {7, -2}, {7, -2}, {7, -2},
    {7, -3}, {7, -4}, {6, -4}, {6, -4}, {6, -4}, {6, -5}, {5, -6}, {1, 0}
};

void doJiggleModePath()
{
    if (esp_timer_get_time() >= nextJiggle_us)
    {
        nextJiggle_us += (jiggleMode==jiggleModePathSlow ? 1000 : 15) * 1000;
        esp_hidd_send_mouse_value(hid_conn_id, 0,
                                  jiggleVectors[jigglePathPos][0],
                                  jiggleVectors[jigglePathPos][1]);
        jigglePathPos = (jigglePathPos+1)%N_VECTORS;
    }
}


static esp_ble_adv_data_t hidd_adv_data =
{
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params =
{
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event)
    {
        case ESP_HIDD_EVENT_REG_FINISH:
        {
            if (param->init_finish.state == ESP_HIDD_INIT_OK)
            {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);
            }
            break;
        }
        case ESP_BAT_EVENT_REG:
        {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
        {
            break;
        }
        case ESP_HIDD_EVENT_BLE_CONNECT:
        {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT:
        {
            sec_conn = false;
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT:
        {
            ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->vendor_write.data, param->vendor_write.length);
            break;
        }
        case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT:
        {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->led_write.data, param->led_write.length);
            jiggleEnabled = param->led_write.data[0] & 0x4;
            break;
        }
        default:
        {
            break;
        }
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    {
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    }
    case ESP_GAP_BLE_SEC_REQ_EVT:
    {
        for (int i = 0; i < ESP_BD_ADDR_LEN; i++)
        {
             ESP_LOGD(HID_DEMO_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    }
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
    {
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_DEMO_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_DEMO_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_DEMO_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (!param->ble_security.auth_cmpl.success)
        {
            ESP_LOGE(HID_DEMO_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    }
    default:
    {
        break;
    }
    }
}

void driveLeds()
{
    static int64_t nextLedShow_us = 0;
    if (nextLedShow_us == 0)
    {
        nextLedShow_us = esp_timer_get_time();
    }

    if (esp_timer_get_time() >= nextLedShow_us)
    {
        nextLedShow_us += 30 * 1000;
        led_state_t newState;
        uint32_t color = !sec_conn
                         ? COLOR_DISCONNECTED
                         : !jiggleEnabled
                           ? COLOR_JIGGLE_NOT
                           : jigglePathPos==0
                             ? COLOR_JIGGLE_FORTH
                             : COLOR_JIGGLE_BACK;
        switch (jiggleMode)
        {
        case jiggleModeStealth:
        {
            for (uint8_t iLed=0; iLed<CONFIG_WS2812_NUM_LEDS; ++iLed)
            {
                if (iLed>=4 && iLed<CONFIG_WS2812_NUM_LEDS-4)
                {
                    newState.leds[iLed] = color;
                }
                else
                {
                    newState.leds[iLed] = 0;
                }
            }
            break;
        }
        case jiggleModeCircle:
        {
            for (uint8_t iLed=0; iLed<CONFIG_WS2812_NUM_LEDS; ++iLed)
            {
                if (iLed>=3 && iLed<CONFIG_WS2812_NUM_LEDS-3)
                {
                    newState.leds[iLed] = color;
                }
                else
                {
                    newState.leds[iLed] = 0;
                }
            }
            break;
        }
        case jiggleModePathSlow:
        {
            for (uint8_t iLed=0; iLed<CONFIG_WS2812_NUM_LEDS; ++iLed)
            {
                if (iLed>=2 && iLed<CONFIG_WS2812_NUM_LEDS-2)
                {
                    newState.leds[iLed] = color;
                }
                else
                {
                    newState.leds[iLed] = 0;
                }
            }
            break;
        }
        case jiggleModePathFast:
        {
            for (uint8_t iLed=0; iLed<CONFIG_WS2812_NUM_LEDS; ++iLed)
            {
                newState.leds[iLed] = color;
            }
            break;
        }
        default:
        {
            break;
        }
        }
        ws2812_write_leds(newState);
    }
}

#define PRESS_DEBOUNCE_US 10 * 1000
#define PRESS_LONG_US 500 * 1000
void handleButton()
{
    static int64_t buttonPressedAt_us;
    static bool buttonWasPressed;
    static bool buttonWasPressedLong;

    bool buttonIsPressed = gpio_get_level(BUT_USR_GPIO);
    if (buttonIsPressed)
    {
        int64_t now_us = esp_timer_get_time();
        if (!buttonWasPressed)
        {
            ESP_LOGI(HID_DEMO_TAG, "user button pressed");
            buttonPressedAt_us = now_us;
            buttonWasPressedLong = false;
        }
        else if (!buttonWasPressedLong
                 && now_us >= buttonPressedAt_us + PRESS_LONG_US)
        {
            ESP_LOGI(HID_DEMO_TAG, "user button long press detected");
            buttonWasPressedLong = true;
            switch (jiggleMode)
            {
            case jiggleModeStealth:
            {
                enterJiggleModeCircle();
                break;
            }
            case jiggleModeCircle:
            {
                enterJiggleModePathSlow();
                break;
            }
            case jiggleModePathSlow:
            {
                enterJiggleModePathFast();
                break;
            }
            case jiggleModePathFast:
            default:
            {
                enterJiggleModeStealth();
                break;
            }
            }
        }
    }
    else if (buttonWasPressed)
           /*&& !buttonIsPressed*/
    {
        int64_t now_us = esp_timer_get_time();
        ESP_LOGI(HID_DEMO_TAG, "user button released");
        if (!buttonWasPressedLong
            && now_us >= buttonPressedAt_us + PRESS_DEBOUNCE_US)
        {
            // was short press
            ESP_LOGI(HID_DEMO_TAG, "toggling SCROLL_LOCK");
            uint8_t cmd[1] = {HID_KEY_SCROLL_LOCK};
            esp_hidd_send_keyboard_value(hid_conn_id, 0, cmd, 1);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            esp_hidd_send_keyboard_value(hid_conn_id, 0, 0, 0);
        }
    }
    buttonWasPressed = buttonIsPressed;
}

void hid_demo_task(void *pvParameters)
{
    gpio_set_direction(LED_EN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUT_USR_GPIO, GPIO_MODE_INPUT);
    gpio_set_level(LED_EN_GPIO, 1);
    ws2812_control_init();

    while(1)
    {
        handleButton();
        if (sec_conn && jiggleEnabled)
        {
            // BLE connected
            switch (jiggleMode)
            {
            case jiggleModeStealth:
            {
                doJiggleModeStealth();
                break;
            }
            case jiggleModeCircle:
            {
                doJiggleModeCircle();
                break;
            }
            case jiggleModePathSlow:
            case jiggleModePathFast:
            {
                doJiggleModePath();
                break;
            }
            default:
            {
                break;
            }
            }
        }
        driveLeds();
    }
}


void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES
        || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    if ((ret = esp_hidd_profile_init()) != ESP_OK)
    {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);
}
