/*
 * This example receives the information from two distance sensors, 
 * approximates a line, and identifies the angle, and the distance,
 * to a wall in front of the sensors.
 */


#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress1[] = {0x48, 0x27, 0xE2, 0xE6, 0xE4, 0x0C};
const int NUM_PARAMS = 15;         // Number of parameters contained in the serial messages
const int NUM_CONTROL_PARAMS = 13; // Number of parameters used for control



// Distance sensors
float dist_1, dist_2;

// Constants to project the distance to the bodoy frame
float cos_mbeta, sin_mbeta, cos_pbeta, sin_pbeta;
// Position in the body from
double px1, px2, py1, py2;



/// Control message for ESP-NOW ///
typedef struct ControlInput
{
  float params[NUM_CONTROL_PARAMS];
  int channel; // The channel to broadcast on
} ControlInput;

ControlInput controlParams;
esp_now_peer_info_t peerInfo; // The peer info structure



void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  Serial.print("Packet to: ");
  Serial.print(macStr);
  Serial.print(" Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}



void setup() {
  // put your setup code here, to run once:
  // Angle between the y-axis and the ray of the distance sensor.
  float beta = 20; 
  beta = beta * PI / 180;  // Convert to radians


  // Projections (constants)
  cos_mbeta = cos(PI / 2 - beta); 
  sin_mbeta = sin(PI / 2 - beta);
  cos_pbeta = cos(PI / 2 + beta);
  sin_pbeta = sin(PI / 2 + beta);


  Serial.begin(115200);

  ////////// Communication ////////////////
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
   
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

}


void loop() {
  // Get from distance sensors
  

  dist_1 = 0.7;
  dist_2 = 1;



  px1 = dist_1 * cos_mbeta;
  py1 = dist_1 * sin_mbeta;
  px2 = dist_2 * cos_pbeta;
  py2 = dist_2 * sin_pbeta;

  // Line y = ax + b
  float a = (py1 - py2) / (px1 - px2);
  float b = py2 - a * px2;   // Distance to the front of the robot (y-axis)
                
  // relative angle
  float ang = atan2(py1 - py2, px1 - px2);

  Serial.print(b);
  Serial.print(" , ");
  Serial.println(ang);


  controlParams.params[0] = b;
  controlParams.params[1] = ang;

  esp_now_send(broadcastAddress1, (uint8_t *)&controlParams, sizeof(ControlInput));


}
