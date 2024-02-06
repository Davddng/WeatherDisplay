#pragma once

#include "WiFi.h"
#include "pin_config.h"
#include "HTTPClient.h"

struct wifi_credentials {
    String SSID;
    String PWD;
};

struct GET_params {
    String url;
    void (*callbackFn)(void* string_resp);
    // HTTPClient* http_client;
    bool repeat;
    int delayMs;
};

class wifi_handler{
    // private:
    public:
        // static HTTPClient* http_client;
        static wifi_credentials credentials;
        static GET_params get_params;
        static TaskHandle_t wifi_task;
        static HTTPClient http_client;
        static WiFiClass wifi_connection;
        
        void connect(String SSID, String pwd);
        void GET_req(String url, void (*responseCallback)(void* string_resp), bool repeat, int delayMs);
        String GET_req(String url);
        void disconnect();
};