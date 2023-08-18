/*
 * @Author       : Hanqing Qi
 * @Date         : 2023-07-18 16:32:31
 * @LastEditors  : Hanqing Qi
 * @LastEditTime : 2023-07-21 14:04:57
 * @FilePath     : /ESP_NOW/Get_MAC_Address/Get_MAC_Address.ino
 * @Description  : Get the MAC address of the ESP board
 */

// Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/

#include <WiFi.h>

void setup(){
  Serial.begin(9600);
  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
}
 
void loop(){

}