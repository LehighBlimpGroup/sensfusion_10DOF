#include <crazyflieComplementary.h>

SensFusion sensorSuite;

void setup() {
  Serial.begin(9600);
  delay(500);
  while(!Serial);
  sensorSuite.initSensors();
  float transformationMatrix[3][3] = {
    {1.0f, 9.693f, 0.6187f},
    {9.6624f, -0.6822f, 0.3864f},
    {-0.4155f, 0.6628f, -10.7386f}
  };
  float offsets[3] = {11.98f, 7.01f, 21.77f};
  sensorSuite.enterTransform(offsets, transformationMatrix);
  sensorSuite.updateKp(5,-1,0.3);//20,-1,0
  //sensorSuite.recordData();
}


void loop() {
  //gyro, acc, mag, euler, z
  sensorSuite.sensfusionLoop(true, 5);


}