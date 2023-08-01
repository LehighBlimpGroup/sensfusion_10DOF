/*
 * @Author       : Hanqing Qi
 * @Date         : 2023-07-18 16:29:47
 * @LastEditors  : Hanqing Qi
 * @LastEditTime : 2023-07-28 14:36:58
 * @FilePath     : /ESP_NOW/Sender/Sender.ino
 * @Description  : This is the sender of ESP-NOW.
 */

#include <esp_now.h>
#include <WiFi.h>

// MAC addresses of the receivers
uint8_t broadcastAddress1[] = {0x34, 0x85, 0x18, 0x91, 0x49, 0xC0}; // Receiver 1

// Add MAC addresses for additional receivers here
// uint8_t broadcastAddress2[] = {0xFF, , , , , }; // Receiver 2
// uint8_t broadcastAddress3[] = {0xFF, , , , , }; // Receiver 3

// Structure definition for control parameters
// This must match the receiver structure
typedef struct Control_Input
{
  float p1;
  float p2;
  float p3;
  float p4;
  float p5;
  float p6;
  float p7;
  float p8;
  float p9;
  float p10;
  float p11;
  float p12;
  float p13;
} Control_Input;

esp_now_peer_info_t peerInfo; // Struct for peer info

// This function gets called once the data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Convert MAC address from bytes to a human-readable format
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  Serial.print("Packet to: ");
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Setting up ESP-NOW and registering callback function for when data is sent
void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // Add all the peers
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  // Repeat this block for additional peers
}

// Global variable for storing received parameters
Control_Input params;

// Flag for tracking if the data stream has started
bool isDataStarted = false;

// Flag indicates if we have received any data
bool HaveData = false;

// Buffer for storing incoming serial data
String data;

// Main function which is called repeatedly
void loop()
{
  HaveData = false;
  delay(20);
  while (Serial.available())
  {
    char c = Serial.read();

    // When '<' is detected, start recording data
    if (c == '<')
    {
      HaveData = true;
      isDataStarted = true;
      data = "";
    }
    // When '>' is detected, stop recording data and process it
    else if (c == '>')
    {
      isDataStarted = false;
      process_data(data);
    }
    // Record data if it's started
    else if (isDataStarted)
    {
      data += c;
    }
  }
  if (HaveData) {
    esp_err_t result = esp_now_send(0, (uint8_t *) &params, sizeof(Control_Input));

    if (result == ESP_OK) {
      // Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }
}

/**
 * @description: Process the received data
 * @param       {String} &data:
 * @return      {void} -
 */
void process_data(String &data)
{
  int count = 0;
  while (data.indexOf('|') != -1 && count < 13)
  {
    int split_position = data.indexOf('|');
    String param = data.substring(0, split_position);
    save_to_params(count, param.toFloat());
    count++;
    data = data.substring(split_position + 1);
  }

  if (data.length() > 0 && count == 12 && data.indexOf('|') == -1)
  {
    save_to_params(count, data.toFloat());
    count++;
  }

  // Checking if the received data is complete
  if (count != 13)
  {
    Serial.println("Error: data is broken");
  }
  else
  {
    Serial.print("Success: ");
    print_params();
  }
}

/**
 * @description: Save the received data to the params struct
 * @param       {int} count: The index of the parameter
 * @param       {float} value: The value of the parameter
 * @return      {void} -
 */
void save_to_params(int count, float value)
{
  // A switch-case structure is used to assign the correct parameter
  switch (count)
  {
  case 0:
    params.p1 = value;
    break;
  case 1:
    params.p2 = value;
    break;
  case 2:
    params.p3 = value;
    break;
  case 3:
    params.p4 = value;
    break;
  case 4:
    params.p5 = value;
    break;
  case 5:
    params.p6 = value;
    break;
  case 6:
    params.p7 = value;
    break;
  case 7:
    params.p8 = value;
    break;
  case 8:
    params.p9 = value;
    break;
  case 9:
    params.p10 = value;
    break;
  case 10:
    params.p11 = value;
    break;
  case 11:
    params.p12 = value;
    break;
  case 12:
    params.p13 = value;
    break;
  }
}

/**
 * @description: Print the parameters
 * @return      {void} -
 */
void print_params()
{
  Serial.print(params.p1);
  Serial.print(", ");
  Serial.print(params.p2);
  Serial.print(", ");
  Serial.print(params.p3);
  Serial.print(", ");
  Serial.print(params.p4);
  Serial.print(", ");
  Serial.print(params.p5);
  Serial.print(", ");
  Serial.print(params.p6);
  Serial.print(", ");
  Serial.print(params.p7);
  Serial.print(", ");
  Serial.print(params.p8);
  Serial.print(", ");
  Serial.print(params.p9);
  Serial.print(", ");
  Serial.print(params.p10);
  Serial.print(", ");
  Serial.print(params.p11);
  Serial.print(", ");
  Serial.print(params.p12);
  Serial.print(", ");
  Serial.print(params.p13);
  Serial.print("----");
}

