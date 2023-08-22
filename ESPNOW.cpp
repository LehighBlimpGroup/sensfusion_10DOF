/*
 * @Author       : Hanqing Qi
 * @Date         : 2023-08-01 15:54:19
 * @LastEditors  : Hanqing Qi
 * @LastEditTime : 2023-08-01 17:35:04
 * @FilePath     : /undefined/Users/hanqingqi/Desktop/sensfusion_10DOF/ESPNOW.cpp
 * @Description  : This is the file for the ESPNOW class
 */

#include <ESPNOW.h>

volatile bool esp_ready;
volatile unsigned long esp_time_now;
Control_Input ESPNOW_Input;

ESPNOW::ESPNOW()
{
    ESPNOW_Input.p1 = 0;
    ESPNOW_Input.p2 = 0;
    ESPNOW_Input.p3 = 0;
    ESPNOW_Input.p4 = 0;
    ESPNOW_Input.p5 = 0;
    ESPNOW_Input.p6 = 0;
    ESPNOW_Input.p7 = 0;
    ESPNOW_Input.p8 = 0;
    ESPNOW_Input.p9 = 0;
    ESPNOW_Input.p10 = 0;
    ESPNOW_Input.p11 = 0;
    ESPNOW_Input.p12 = 0;
    ESPNOW_Input.p13 = 0;
    esp_ready = false;
}

// Callback function that will be executed when data is received
void ESPNOW_DataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    esp_time_now = millis();
    esp_ready = false;
    memcpy(&ESPNOW_Input, incomingData, sizeof(ESPNOW_Input));
    // Serial.print("Bytes received: ");
    // Serial.println(len);
    // Serial.print("Data: ");
    // Serial.print(ESPNOW_Input.p1);
    // Serial.print("|");
    // Serial.print(ESPNOW_Input.p2);
    // Serial.print("|");
    // Serial.print(ESPNOW_Input.p3);
    // Serial.print("|");
    // Serial.print(ESPNOW_Input.p4);
    // Serial.print("|");
    // Serial.print(ESPNOW_Input.p5);
    // Serial.print("|");
    // Serial.print(ESPNOW_Input.p6);
    // Serial.print("|");
    // Serial.print(ESPNOW_Input.p7);
    // Serial.print("|");
    // Serial.print(ESPNOW_Input.p8);
    // Serial.print("|");
    // Serial.print(ESPNOW_Input.p9);
    // Serial.print("|");
    // Serial.print(ESPNOW_Input.p10);
    // Serial.print("|");
    // Serial.print(ESPNOW_Input.p11);
    // Serial.print("|");
    // Serial.print(ESPNOW_Input.p12);
    // Serial.print("|");
    // Serial.print(ESPNOW_Input.p13);
    // Serial.println();
    esp_ready = true;
}

void ESPNOW::init()
{
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
    // Print your local MAC address
    Serial.println();
    Serial.print("ESP Board MAC Address:  ");
    Serial.println(WiFi.macAddress());
}

/**
 * @description: This function gets the controller inputs from the ESPNOW
 * @param       {controller_t} *controls:
 * @return      {*}
 */
void ESPNOW::getControllerInputs(controller_t *controls)
{

    // Serial.print(esp_ready);
    // Serial.print(',');
    // Serial.println(millis() - esp_time_now);
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
        controls->ready = (bool)((int)(ESPNOW_Input.p9) != 0);
        controls->snapshot = (int)ESPNOW_Input.p10;
    }
    else
    {
        controls->ready = false;
    }
    return;
}

/**
 * @description: This function gets the controller inputs from the ESPNOW
 * @param       {raw_t} *controls:
 * @return      {*}
 */
void ESPNOW::getControllerRaws(raw_t *raws)
{

    // Serial.print(esp_ready);
    // Serial.print(',');
    // Serial.println(millis() - esp_time_now);
    if (esp_ready && millis() - esp_time_now < delayMS)
    {
        raws->flag = (int)ESPNOW_Input.p1;
        raws->ready = (int)ESPNOW_Input.p2 == 1;
        raws->data[0] = ESPNOW_Input.p3;
        raws->data[1] = ESPNOW_Input.p4;
        raws->data[2] = ESPNOW_Input.p5;
        raws->data[3] = ESPNOW_Input.p6;
        raws->data[4] = ESPNOW_Input.p7;
        raws->data[5] = ESPNOW_Input.p8;
        raws->data[6] = ESPNOW_Input.p9;
        raws->data[7] = ESPNOW_Input.p10;
        raws->data[8] = ESPNOW_Input.p11;
        raws->data[9] = ESPNOW_Input.p12;
        raws->data[10] = ESPNOW_Input.p13;
        
    }
    else
    {
        raws->ready = false;
    }
    return;
}