#include "Arduino.h"
#include "TFT_eSPI.h" /* Please use the TFT library provided in the library. */
#include <esp_adc_cal.h>
#include "pin_config.h"
// include logging library
#include "esp32-hal-log.h"
#include "stdlib.h"
#include <numeric>
#include <algorithm>
#include <map>

#include "DeviceIds.h"
#include "Protocol.hpp"
#include "EspUart.hpp"
#include "MessageId.h"
#include "Master.hpp"
#include "Leaf.hpp"
#include "MemoryMap.h"

#if CORE_DEBUG_LEVEL > 1
constexpr uint32_t _XERXES_TIMEOUT_US = (1e9 / __XERXES_BAUD_RATE) + 2e4;
#else
constexpr uint32_t _XERXES_TIMEOUT_US = (1e9 / __XERXES_BAUD_RATE);
#endif

#define _FONT_SIZE 4

constexpr uint8_t _pin_tx = 17;
constexpr uint8_t _pin_rx = 18;
auto xn = Xerxes::EspUart(_pin_tx, _pin_rx);
auto xp = Xerxes::Protocol(&xn);
auto master = Xerxes::Master(&xp, 0xFE, _XERXES_TIMEOUT_US);

/* The product now has two screens, and the initialization code needs a small change in the new version. The LCD_MODULE_CMD_1 is used to define the
 * switch macro. */
#define LCD_MODULE_CMD_1

#ifndef __MONITOR_SPEED
#error "Please define the monitor speed."
#endif

TFT_eSPI tft = TFT_eSPI();
unsigned long targetTime = 0;

#if defined(LCD_MODULE_CMD_1)
typedef struct
{
    uint8_t cmd;
    uint8_t data[14];
    uint8_t len;
} lcd_cmd_t;

lcd_cmd_t lcd_st7789v[] = {
    {0x11, {0}, 0 | 0x80},
    {0x3A, {0X05}, 1},
    {0xB2, {0X0B, 0X0B, 0X00, 0X33, 0X33}, 5},
    {0xB7, {0X75}, 1},
    {0xBB, {0X28}, 1},
    {0xC0, {0X2C}, 1},
    {0xC2, {0X01}, 1},
    {0xC3, {0X1F}, 1},
    {0xC6, {0X13}, 1},
    {0xD0, {0XA7}, 1},
    {0xD0, {0XA4, 0XA1}, 2},
    {0xD6, {0XA1}, 1},
    {0xE0, {0XF0, 0X05, 0X0A, 0X06, 0X06, 0X03, 0X2B, 0X32, 0X43, 0X36, 0X11, 0X10, 0X2B, 0X32}, 14},
    {0xE1, {0XF0, 0X08, 0X0C, 0X0B, 0X09, 0X24, 0X2B, 0X22, 0X43, 0X38, 0X15, 0X16, 0X2F, 0X37}, 14},
};
#endif

auto devices = std::vector<uint8_t>();

int discoverXerxesDevices(std::vector<uint8_t> &devices, uint8_t range_min = 0, uint8_t range_max = 0x1F)
{
    int ret = 0; // amount of devices found
    ping_reply_t ping_reply;

    // create vector to store all adresses
    std::vector<uint8_t> adresses(range_max - range_min + 1);

    // fill vector with all adresses to be pinged
    std::iota(adresses.begin(), adresses.end(), range_min);
    tft.print("Discovering devices...\n");

    // try to ping all devices
    for (auto &j : adresses)
    {
        // skip if already found device
        if (std::find(devices.begin(), devices.end(), j) != devices.end())
        {
            ESP_LOGD("main", "Skipping device with address: %d, already found", j);
            continue;
        }

        try
        {
            xn.flush();
            tft.printf("%d, ", j);
            ping_reply = master.ping(j);
            devices.push_back(j);
            ESP_LOGI("main", "Found device with id: %d, latency: %.1fms, address: %d", ping_reply.device_id, ping_reply.latency_ms, j);
            ret++;
        }
        catch (const Xerxes::TimeoutError &e)
        {
            // this is expected if device is not responding
            ESP_LOGD("main", "Timeout while pinging device with address: %d", j);
        }
        catch (const std::exception &e)
        {
            // this is unexpected
            ESP_LOGE("main", "Error while pinging device with address: %d, %s", j, e.what());
        }
    }
    return ret;
}

/**
 * @brief Read data from leaf and return it as float
 *
 * @param _dst destination variable
 * @param addr leaf's address
 * @param offset offset of the value to read
 * @return true if read was successful
 * @return false if read failed
 */
bool readSafe(float &_dst, const uint8_t addr, const uint16_t offset)
{
    try
    {
        _dst = master.readValue<float>(addr, offset);
        ESP_LOGI("main", "Address[offset] - %d[%d]: %.2f ", addr, offset, _dst);
        return true;
    }
    catch (const std::exception &e)
    {
        ESP_LOGW("main", "Error while reading value from device with address: %d, %s", addr, e.what());
        return false;
    }
}

void setup()
{

    // This IO15 must be set to HIGH, otherwise nothing will be displayed when USB is not connected.
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);

    Serial.begin(__MONITOR_SPEED);
    delay(10);
    ESP_LOGI("setup", "Start");

    tft.begin();
    tft.setRotation(3);
    tft.setTextSize(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    ESP_LOGI("setup", "LCD init");

#if defined(LCD_MODULE_CMD_1)
    for (uint8_t i = 0; i < (sizeof(lcd_st7789v) / sizeof(lcd_cmd_t)); i++)
    {
        tft.writecommand(lcd_st7789v[i].cmd);
        for (int j = 0; j < lcd_st7789v[i].len & 0x7f; j++)
        {
            tft.writedata(lcd_st7789v[i].data[j]);
        }

        if (lcd_st7789v[i].len & 0x80)
        {
            delay(120);
        }
    }
#endif

    // Turn on backlight
    ledcSetup(0, 2000, 8);
    ledcAttachPin(PIN_LCD_BL, 0);
    ledcWrite(0, 255);
    ESP_LOGI("setup", "Backlight on");

    // print all definitions:
    tft.print("Xerxes display v1.0.0\n");
    tft.printf("Monitor speed: %d\n", __MONITOR_SPEED);
    tft.printf("Xerxes baud: %d\n", __XERXES_BAUD_RATE);
    tft.printf("Xerxes timeout: %dus\n", _XERXES_TIMEOUT_US);
    tft.printf("CPU freq: %dMHz\n", ESP.getCpuFreqMHz());

    /*
    // wait for button press
    tft.print("Press button to start");
    while (digitalRead(PIN_BUTTON_1) == HIGH)
    {
        delay(10);
    }
    */

    discoverXerxesDevices(devices, 0x00, 0x1F);
}

void loop()
{
    if (devices.size() == 0)
    {
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(0, 0);
        tft.drawString("No devices found", 0, 0, 4);
        delay(1000);
        discoverXerxesDevices(devices, 0x00, 0x1F);
    }

    int y_offset = 0;
    for (auto &device : devices)
    {
        bool status = false;
        float pv0 = 0;
        float pv1 = 0;
        float pv2 = 0;
        float pv3 = 0;
        status |= readSafe(pv0, device, MEAN_PV0_OFFSET);
        status |= readSafe(pv1, device, MEAN_PV1_OFFSET);
        status |= readSafe(pv2, device, MEAN_PV2_OFFSET);
        status |= readSafe(pv3, device, MEAN_PV3_OFFSET);

        if (status)
        {
            tft.fillScreen(TFT_BLACK);
            tft.setCursor(0, 0);
            tft.drawString("[" + String(device) + "]", 0, y_offset, _FONT_SIZE);
            tft.drawString(String(pv0, 1), 80, y_offset, _FONT_SIZE);
            tft.drawString(String(pv1, 1), 160, y_offset, _FONT_SIZE);
            // tft.drawString(String(pv2, 1), 180, y_offset, _FONT_SIZE);
            tft.drawString(String(pv3, 1), 240, y_offset, _FONT_SIZE);
        }

        y_offset += 20;
    }
    delay(200);

    // go to sleep after 5 minutes
    if (millis() > 300000)
    {
        ESP_LOGI("loop", "Going to sleep");
        esp_deep_sleep_start();
    }
}

// TFT Pin check
#if PIN_LCD_WR != TFT_WR ||     \
    PIN_LCD_RD != TFT_RD ||     \
    PIN_LCD_CS != TFT_CS ||     \
    PIN_LCD_DC != TFT_DC ||     \
    PIN_LCD_RES != TFT_RST ||   \
    PIN_LCD_D0 != TFT_D0 ||     \
    PIN_LCD_D1 != TFT_D1 ||     \
    PIN_LCD_D2 != TFT_D2 ||     \
    PIN_LCD_D3 != TFT_D3 ||     \
    PIN_LCD_D4 != TFT_D4 ||     \
    PIN_LCD_D5 != TFT_D5 ||     \
    PIN_LCD_D6 != TFT_D6 ||     \
    PIN_LCD_D7 != TFT_D7 ||     \
    PIN_LCD_BL != TFT_BL ||     \
    TFT_BACKLIGHT_ON != HIGH || \
    170 != TFT_WIDTH ||         \
    320 != TFT_HEIGHT
#error "Error! Please make sure <User_Setups/Setup206_LilyGo_T_Display_S3.h> is selected in <TFT_eSPI/User_Setup_Select.h>"
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#error "The current version is not supported for the time being, please use a version below Arduino ESP32 3.0"
#endif