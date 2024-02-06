#include "wifi_lib.hpp"

void wifi_handler::connect(String SSID, String pwd){
    Serial.println("Connect");
    wifi_connection.mode(WIFI_STA);
    credentials.SSID = SSID;
    credentials.PWD = pwd;

    // xTaskCreatePinnedToCore([](void *param){
        // wifi_credentials credentials = *((wifi_credentials*)param);
        uint32_t last_m = millis();
        wifi_connection.begin(credentials.SSID, credentials.PWD);
        uint32_t timestamp = millis() + 30000;
        while (wifi_connection.status() != WL_CONNECTED) {
            if (timestamp < millis()) {
                Serial.println("WiFi Connect failed!");
                // lv_msg_send(MSG_WIFI_UPDATE, "WiFi Connect failed!");
                wifi_task = NULL;
                vTaskDelete(NULL);
                while (1) {
                    delay(30000);
                }
            }
            Serial.print(".");
            vTaskDelay(500);
        }
        Serial.printf("\r\n-- wifi connect success! --\r\n");
        Serial.println(wifi_connection.macAddress());
        Serial.printf("It takes %d milliseconds\r\n", millis() - last_m);
        // vTaskDelete(NULL);
    // }, "wifi_task", 1024 * 6, &credentials, 1, &wifiTask, CONFIG_ARDUINO_RUNNING_CORE);
}

void wifi_handler::GET_req(String url, void (*responseCallback)(void* string_resp), bool repeat, int delayMs){
    get_params.url = url;
    get_params.callbackFn = responseCallback;
    // get_params.http_client = &http_client;
    get_params.repeat = repeat;
    get_params.delayMs = delayMs;
    xTaskCreatePinnedToCore([](void *param){
        GET_params get_params = *((GET_params*)param);
        String rsp;
        do {
            http_client.begin(get_params.url);
            int http_code = http_client.GET();
            Serial.println(http_code);
            if (http_code == HTTP_CODE_OK) {
                rsp = http_client.getString();
                get_params.callbackFn(&rsp);
            } else {
                Serial.printf("[HTTP] GET... failed, error: %s\n", http_client.errorToString(http_code).c_str());
                Serial.printf("Wifi Status: %d\n", wifi_connection.status());
                if(wifi_connection.status() != WL_CONNECTED) {
                    wifi_connection.begin(credentials.SSID, credentials.PWD);
                }

                // rsp = "";
                // get_params.callbackFn(&rsp);
            }
            http_client.end();
            delay(get_params.delayMs);
        } while (get_params.repeat);
        vTaskDelete(NULL);
    }, "wifi_task", 1024 * 6, &get_params, 1, &wifi_task, CONFIG_ARDUINO_RUNNING_CORE);
}

String wifi_handler::GET_req(String url){
    bool is_get_http = false;
    http_client.begin(url);
    int http_code = http_client.GET();
    if (http_code == HTTP_CODE_OK) {
        return http_client.getString();;
    }else{
        Serial.printf("[HTTP] GET... failed, error: %s\n", http_client.errorToString(http_code).c_str());
        Serial.printf("Wifi Status: %d\n", wifi_connection.status());
        return "";
    }

}   

void wifi_handler::disconnect(){
    wifi_connection.disconnect();
}