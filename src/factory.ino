#include "SD_MMC.h"
#include "Wire.h"
#include "ui/ui.h"
#include "img.h"
#include "lvgl.h"
#include <Arduino.h>
#include "OneButton.h"
#include "pin_config.h"
#include "lcd_touch_display.h"
#include "wifi_lib.hpp"
#include "time_provider.hpp"
#include <ArduinoJson.h>
// #include "WiFi.h"
// #include "HTTPClient.h"


// void deep_sleep(void);
// void SD_init(void);
void tft_init(void);
void lcd_cmd(const uint8_t cmd);
void lcd_data(const uint8_t *data, int len);
void update_weather(void* string_response);
void update_time(void* tm_Obj);

OneButton button(0, true);
// TaskHandle_t pvCreatedTask;
wifi_handler wifi;
wifi_credentials wifi_handler::credentials;
GET_params wifi_handler::get_params;
TaskHandle_t wifi_handler::wifi_task;
TaskHandle_t wifiTask;
HTTPClient wifi_handler::http_client;
WiFiClass wifi_handler::wifi_connection;

// time_handler time_provider;
WiFiClass time_handler::wifi_connection;
// tm time_handler::time_container;
TaskHandle_t time_handler::time_task;
time_handler_params time_handler::time_params;
DynamicJsonDocument currentData(1024);

// HTTPClient http_client;

bool click = false;
const int backlightPin = EXAMPLE_PIN_NUM_BK_LIGHT;
void print_chip_info(void)
{
    Serial.print("Chip: ");
    Serial.println(ESP.getChipModel());
    Serial.print("ChipRevision: ");
    Serial.println(ESP.getChipRevision());
    Serial.print("Psram size: ");
    Serial.print(ESP.getPsramSize() / 1024);
    Serial.println("KB");
    Serial.print("Flash size: ");
    Serial.print(ESP.getFlashChipSize() / 1024);
    Serial.println("KB");
    Serial.print("CPU frequency: ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println("MHz");
}

void setup()
{
    Serial.begin(115200);

    esp_lcd_panel_handle_t panel_handle = lcd_touch_setup();
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions
    static lv_indev_drv_t indev_drv;
    lv_init();
    // alloc draw buffers used by LVGL from PSRAM
    lv_color_t *buf1 = (lv_color_t *)ps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t));
    assert(buf1);
    lv_color_t *buf2 = (lv_color_t *)ps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t));
    assert(buf2);
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);

    Serial.println("Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lv_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    // // Test screen color
    // LV_IMG_DECLARE(photo2);
    // LV_IMG_DECLARE(photo4);
    // LV_IMG_DECLARE(photo5);

    // lv_obj_t *img = lv_img_create(lv_scr_act());
    // lv_img_set_src(img, &photo2);
    // lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);

    // const lv_img_dsc_t *photo[] = {&photo2, &photo4, &photo5};

    // button.attachClick([]()
    //                    { click = true; });

    // // Wait for the GT911 interrupt signal to be ready
    waitInterruptReady();

    lv_task_handler();

    pinMode(backlightPin, OUTPUT);
    // LilyGo T-RGB control backlight chip has 16 levels of adjustment range
    for (int i = 0; i < 16; ++i)
    {
        setBrightness(i);
        delay(30);
    }

    ui_init();

    wifi.connect(WIFI_SSID, WIFI_PASSWORD);
    
    // int i = 1;
    // while (i <= 3)
    // {
    //     if (click || !digitalRead(TP_INT_PIN))
    //     {
    //         click = false;
    //         lv_img_set_src(img, photo[i]);
    //         i++;
    //         waitInterruptReady();
    //     }

    //     button.tick();
    //     lv_task_handler();
    //     delay(5);
    // }

    // lv_obj_del(img);
    print_chip_info();
    time_handler time_provider;
    _ui_screen_change( &ui_Screen1, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0, &ui_Screen1_screen_init);
    // ui_begin();
    wifi.GET_req("http://192.168.1.178:5000/air_quality", update_weather, true, 60000);
    time_provider.get_time(update_time, true, 120000);
    // delay(1000);
    // SecondHandRotate_Animation(ui_comp_get_child(ui_ClockEdge01, UI_COMP_CLOCKEDGE_CLOCKEDGECLOCKGROUP_SECONDHAND), 0);
    // MinuteHandRotate_Animation(ui_comp_get_child(ui_ClockEdge01, UI_COMP_CLOCKEDGE_CLOCKEDGECLOCKGROUP_MINUTEHAND), 0);
    // HourHandRotate_Animation(ui_comp_get_child(ui_ClockEdge01, UI_COMP_CLOCKEDGE_CLOCKEDGECLOCKGROUP_HOURHAND), 0);
    // delay(500);
    // startClockAnimations();
    // String weather_response = wifi.GET_req("http://192.168.1.178:5000/air_quality");
    // xTaskCreatePinnedToCore(update_weather, "weather_update_task", 1024 * 6, &weather_response, 1, &wifiTask, CONFIG_ARDUINO_RUNNING_CORE);
    
    // xTaskCreate(update_time, "time_task", 1024 * 6, NULL, 1, &time_update_task);
}

bool lastStatus = false;

void loop()
{
    // put your main code here, to run repeatedly:
    // wifi_handler wifi;
    
    lv_timer_handler();
    // static uint32_t Millis;
    // if (millis() - Millis > 5000) {
    //     // float v = (analogRead(BAT_VOLT_PIN) * 2 * 3.3) / 4096;
    //     // lv_msg_send(MSG_BAT_VOLT_UPDATE, &v);
    //     // Serial.printf("Wifi Stat: %d\n", wifi_handler::wifi_connection.status());
    //     // wifi.GET_req("http://192.168.1.178:5000/air_quality", update_wifi, true);

    //     Millis = millis();
    // }

#ifndef USING_2_1_INC_CST820
    bool touched = digitalRead(TP_INT_PIN) == LOW;
    if (touched) {
        lastStatus = touched;
        lv_msg_send(MSG_TOUCH_INT_UPDATE, &touched);
    } else if (!touched && lastStatus) {
        lastStatus = false;
        lv_msg_send(MSG_TOUCH_INT_UPDATE, &touched);
    }
#endif
}

void update_weather(void* string_response){
    deserializeJson(currentData, *((String*)string_response));
    float temp = currentData["temp"];
    int16_t itemp = round(temp*10);
    String tempLabel = String(temp, 1);
    tempLabel += " °C";
    lv_msg_send(TEMP_LABEL_UPDATE, &tempLabel);
    lv_msg_send(TEMP_NUMERIC_UPDATE, &itemp);

    float press = currentData["pres"];
    int16_t ipress = round(press*10);
    String pressLabel = String(press, 2);
    pressLabel += " kPa";
    lv_msg_send(PRESS_LABEL_UPDATE, &pressLabel);
    lv_msg_send(PRESS_NUMERIC_UPDATE, &ipress);

    float humid = currentData["humid"];
    int16_t ihumid = round(humid);
    String humidLabel = String(humid, 1);
    humidLabel += " %%rh";
    lv_msg_send(HUMID_LABEL_UPDATE, &humidLabel);
    lv_msg_send(HUMID_NUMERIC_UPDATE, &ihumid);

    float pm25 = currentData["pm25"];
    int16_t ipm25 = round(pm25);
    String pm25Label = String(pm25, 0);
    pm25Label += " PM2.5";
    lv_msg_send(PM25_LABEL_UPDATE, &pm25Label);
    lv_msg_send(PM25_NUMERIC_UPDATE, &ipm25);

};

void update_time(void* tm_Obj){
    tm time_info = *((tm*)tm_Obj);
    int16_t time[3] = {time_info.tm_hour, time_info.tm_min, time_info.tm_sec};
    Serial.printf("Time update %D:%D:%D\n", time_info.tm_hour, time_info.tm_min, time_info.tm_sec);
    lv_msg_send(TIME_UPDATE, &time);
};