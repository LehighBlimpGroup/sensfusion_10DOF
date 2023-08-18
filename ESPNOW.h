/*
 * @Author       : Hanqing Qi
 * @Date         : 2023-08-01 15:54:11
 * @LastEditors  : Hanqing Qi
 * @LastEditTime : 2023-08-01 16:47:12
 * @FilePath     : /sensfusion_10DOF-main/ESPNOW.h
 * @Description  : Header file for ESPNOW.cpp
 */

#include "WiFi.h"
#include <esp_now.h> // This is the arduino library for ESP-NOW
#include <data_types.h>
void ESPNOW_DataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

// Receiver side data structures
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

class ESPNOW
{
private:
  int delayMS = 1000;

public:
  ESPNOW();
  void init();
  void getControllerInputs(controller_t *controls);
};