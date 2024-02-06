#pragma once
#include "WiFi.h"
// #include <esp32-hal-timer.h>

struct time_handler_params {
    void (*timeCallback)(void* time_obj);
    bool repeat;
    int delayMs;
    tm* time_container;
};

class time_handler{
    public:
        static WiFiClass wifi_connection;
        static TaskHandle_t time_task;
        static time_handler_params time_params;
        tm time_container;

        time_handler();
        time_handler(String ntpServerURL, long GMTZone, int daylightOffsetSec);
        void update_params(String ntpServerURL, long GMTZone, int daylightOffsetSec);
        tm get_time();
        void get_time(void (*responseCallback)(void* string_resp), bool repeat, int delayMs);
};