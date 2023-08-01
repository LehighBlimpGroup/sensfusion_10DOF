/*
 * @Author       : Hanqing Qi
 * @Date         : 2023-08-01 15:54:19
 * @LastEditors  : Hanqing Qi
 * @LastEditTime : 2023-08-01 16:18:31
 * @FilePath     : /sensfusion_10DOF/ESPNOW.cpp
 * @Description  : This is the file for the ESPNOW class
 */

#include <ESPNOW.h>

volatile bool esp_ready;
volatile unsigned long esp_time_now;
Control_Input ESPNOW_Input;

ESPNOW::ESPNOW()
{
    esp_ready = false;
}

// Callback function that will be executed when data is received
void ESPNOW_DataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    esp_time_now = millis();
    esp_ready = false;
    memcpy(&ESPNOW_Input, incomingData, sizeof(ESPNOW_Input));
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("Data: ");
    Serial.print(ESPNOW_Input.p1);
    Serial.print("|");
    Serial.print(ESPNOW_Input.p2);
    Serial.print("|");
    Serial.print(ESPNOW_Input.p3);
    Serial.print("|");
    Serial.print(ESPNOW_Input.p4);
    Serial.print("|");
    Serial.print(ESPNOW_Input.p5);
    Serial.print("|");
    Serial.print(ESPNOW_Input.p6);
    Serial.print("|");
    Serial.print(ESPNOW_Input.p7);
    Serial.print("|");
    Serial.print(ESPNOW_Input.p8);
    Serial.print("|");
    Serial.print(ESPNOW_Input.p9);
    Serial.print("|");
    Serial.print(ESPNOW_Input.p10);
    Serial.print("|");
    Serial.print(ESPNOW_Input.p11);
    Serial.print("|");
    Serial.print(ESPNOW_Input.p12);
    Serial.print("|");
    Serial.print(ESPNOW_Input.p13);
    Serial.println();
    esp_ready = true;
}

void ESPNOW::init(int port)
{
    active = true;
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    // Once ESPNow is successfully Init, we will register for recv CB to
    // Get recv packer info
    esp_now_register_recv_cb(ESPNOW_DataRecv);
    esp_time_now = millis();
    // Print your local IP address
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

/**
 * @description: This function gets the controller inputs from the ESPNOW
 * @param       {controller_t} *controls:
 * @return      {*}
 */
void ESPNOW::getControllerInputs(controller_t *controls)
{

    if (esp_ready && millis() - esp_time_now < delayMS)
    {
        controls->flag = (int)ESPNOW_Input.p1;
        controls->fx = ESPNOW_Input.p2;
        controls->fy = ESPNOW_Input.p3;
        controls->fz = ESPNOW_Input.p4;
        controls->tx = ESPNOW_Input.p5;
        controls->ty = ESPNOW_Input.p6;
        controls->tz = ESPNOW_Input.p7;
        controls->absz = ESPNOW_Input.p8;
        controls->ready = ESPNOW_Input.p9 != 0;
        controls->snapshot = (int)ESPNOW_Input.p10;
    }
    else
    {
        controls->ready = false;
    }
    return;
}