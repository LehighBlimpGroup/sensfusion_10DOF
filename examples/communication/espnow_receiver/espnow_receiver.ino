/*
 * @Author       : Hanqing Qi
 * @Date         : 2023-07-18 16:35:57
 * @LastEditors  : Hanqing Qi
 * @LastEditTime : 2023-10-03 15:08:52
 * @FilePath     : /DTR_Projects/ESP_NOW/Multi_Mac_Address_V1/Receiver/Receiver.ino
 * @Description  : This is the receiver of ESP-NOW
 */

#include <esp_now.h>
#include <WiFi.h>

const int NUM_CONTROL_PARAMS = 13; // Number of parameters used for control
const int CHANNEL = 0;

typedef struct ControlInput
{
  float params[NUM_CONTROL_PARAMS];
  int channel; // The channel to broadcast on
} ControlInput;

// Callback when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  
  ControlInput *incomingData = (ControlInput *)data; // Cast data to our structure type
  
  if (incomingData->channel == -1) // Check if the data is P2P
  {
    Serial.print("Packet from: ");
    Serial.println(macStr);
    Serial.print("Control params: ");
    for (int i = 0; i < NUM_CONTROL_PARAMS; i++)
    {
      Serial.print(incomingData->params[i]);
      if (i < NUM_CONTROL_PARAMS - 1)
      {
        Serial.print(", ");
      }
    }
    Serial.println("\tListening from P2P");
  }else if (incomingData->channel == CHANNEL){ // Check if the data is broadcast
    Serial.print("Packet from: ");
    Serial.println(macStr);
    Serial.print("Control params: ");
    for (int i = 0; i < NUM_CONTROL_PARAMS; i++)
    {
      Serial.print(incomingData->params[i]);
      if (i < NUM_CONTROL_PARAMS - 1)
      {
        Serial.print(", ");
      }
    }
    Serial.print("\tListening on channel: ");
    Serial.println(incomingData->channel);
  }
  else
  {
    Serial.println("Data received on an unexpected channel. Ignoring.");
  }
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }else{
    Serial.println("ESP-NOW initialized");
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  // Nothing to do here since we just wait for the callback to be triggered
}