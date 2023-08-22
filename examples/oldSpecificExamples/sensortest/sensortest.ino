#include <crazyflieComplementary.h>

SensFusion sensorSuite;

void setup() {
  Serial.begin(115200);
  delay(500);
  while(!Serial);
  sensorSuite.initSensors();
  
  sensorSuite.enterTransform();
  sensorSuite.updateKp(20,-1,5);//20,-1,0
  //sensorSuite.recordData();
}


void loop() {
  //gyro, acc, mag, euler, z
  sensorSuite.sensfusionLoop(true, 3);


}