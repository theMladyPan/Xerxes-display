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
#define _XERXES_TIMEOUT_US 1e9 / __XERXES_BAUD_RATE + 1e4
#else
#define _XERXES_TIMEOUT_US 1e9 / __XERXES_BAUD_RATE
#endif

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
    int ret = 0;
    ping_reply_t ping_reply;

    // create vector to store all adresses
    std::vector<uint8_t> adresses(range_max - range_min + 1);

    // fill vector with all adresses to be pinged
    std::iota(adresses.begin(), adresses.end(), range_min);

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
            ping_reply = master.ping(j);
            devices.push_back(j);
            ESP_LOGI("main", "Found device with id: %d, latency: %.1fms, address: %d", ping_reply.device_id, ping_reply.latency_ms, j);
            tft.print("Found device with id: ");
            tft.print(ping_reply.device_id);
            tft.print(", latency: ");
            tft.print(ping_reply.latency_ms);

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
    tft.print("Xerxes display v1.0.0");
    tft.print("Monitor speed: ");
    tft.print(__MONITOR_SPEED);
    tft.print("\n");
    tft.print("Xerxes baud: ");
    tft.print(__XERXES_BAUD_RATE);
    tft.print("\n");
    tft.print("Xerxes timeout: ");
    tft.print(_XERXES_TIMEOUT_US);
    tft.print("us\n");
    tft.print("CPU freq: ");
    tft.print(ESP.getCpuFreqMHz());
    tft.print("MHz\n");

    // wait for button press
    tft.print("Press button to start");
    while (digitalRead(PIN_BUTTON_1) == HIGH)
    {
        delay(10);
    }

    discoverXerxesDevices(devices, 0x00, 0x1F);
}

void loop()
{
    if (millis() > targetTime)
    {
        esp_adc_cal_characteristics_t adc_chars;

        // Get the internal calibration value of the chip
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
        uint32_t raw = analogRead(PIN_BAT_VOLT);
        uint32_t v1 = esp_adc_cal_raw_to_voltage(raw, &adc_chars) * 2; // The partial pressure is one-half

        tft.fillScreen(TFT_BLACK);
        tft.setCursor(0, 0);

        // If the battery is not connected, the ADC detects the charging voltage of TP4056, which is inaccurate.
        // Can judge whether it is greater than 4300mV. If it is less than this value, it means the battery exists.
        if (v1 > 4300)
        {
            tft.print("No battery connect!");
            tft.drawCentreString("Font size 4", 80, 30, 4); // Draw text centre at position 80, 24 using font 4
            ESP_LOGI("loop", "No battery connect!");
        }
        else
        {
            tft.print(v1);
            tft.print("mV");
            ESP_LOGI("loop", "Battery voltage: %dmV", v1);
        }
        targetTime = millis() + 1000;
    }
    delay(20);
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
#error "Error! Please make sure <User_Setups/Setup206_LilyGo_T_Display_S3.h> is selected in <TFT_eSPI/User_Setup_Select.h>"
#error "Error! Please make sure <User_Setups/Setup206_LilyGo_T_Display_S3.h> is selected in <TFT_eSPI/User_Setup_Select.h>"
#error "Error! Please make sure <User_Setups/Setup206_LilyGo_T_Display_S3.h> is selected in <TFT_eSPI/User_Setup_Select.h>"
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#error "The current version is not supported for the time being, please use a version below Arduino ESP32 3.0"
#endif