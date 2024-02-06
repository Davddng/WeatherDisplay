#include "time_provider.hpp"

// ntpServerURL = URL of NTP Time server
// GMTZone GMT time zone. Greenwich Mean Time = 0, Pacific time = -8, etc

time_handler::time_handler(){
    tm time_container;
    update_params("pool.ntp.org", -7, 3600);
}

time_handler::time_handler(String ntpServerURL, long GMTZone, int daylightOffsetSec){
    tm time_container;
    update_params(ntpServerURL, GMTZone, daylightOffsetSec);
}

void time_handler::update_params(String ntpServerURL, long GMTZone, int daylightOffsetSec){
    GMTZone = 3600 * GMTZone;
    // configTime(GMTZone, daylightOffsetSec, ntpServerURL.c_str());
    configTime(GMTZone, daylightOffsetSec, "pool.ntp.org");
}

void time_handler::get_time(void (*responseCallback)(void* tm_container), bool repeat, int delayMs){
    // time_handler_params params;
    time_params.repeat = repeat;
    time_params.timeCallback = responseCallback;
    time_params.delayMs = delayMs;
    time_params.time_container = &time_container;

    if(wifi_connection.status() != WL_CONNECTED){
        Serial.println("Wifi not connected");
    }else{
        Serial.println("Wifi connected");
    }
    xTaskCreatePinnedToCore([](void *param){
        time_handler_params time_params = *((time_handler_params*)param);
        tm time_cont;
        delay(1000);
        do{
            if(!getLocalTime(&time_cont)){
                Serial.printf("Failed to obtain time, trying again in %d ms\n", time_params.delayMs);
            }else{
                Serial.println("Time update");
                time_params.timeCallback(&time_cont);
            }
            delay(time_params.delayMs);
        }while(time_params.repeat);
        vTaskDelete(NULL);

    }, "time_update_task_01", 1024 * 12, &time_params, 1, &time_task, CONFIG_ARDUINO_RUNNING_CORE);
}

tm time_handler::get_time(){
    if(wifi_connection.status() != WL_CONNECTED){
        Serial.println("Wifi not connected");
        // return;
    }
    if(!getLocalTime(&time_container)){
        Serial.println("Failed to obtain time");
        // return;
    }
    return time_container;
}