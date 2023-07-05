#include <crazyflieComplementary.h>

SensFusion sensorSuite;

void setup() {
  Serial.begin(9600);
  delay(500);
  while(!Serial);
  sensorSuite.initSensors();
  float transformationMatrix[3][3] = {
   {      1.0000f,   46.5266f,   -3.0944f},
   { 44.4423f,    2.3206f,    0.3207f},
    { 0.2423f,   -2.4094f,  -42.0017f}
  };
  float offsets[3] = {48.82f,     -6.04f,    -69.82f};
  sensorSuite.enterTransform(offsets, transformationMatrix);
  sensorSuite.updateKp(20,-1,2);//20,-1,0
  //sensorSuite.recordData();
}


void loop() {
  //gyro, acc, mag, euler, z
  sensorSuite.sensfusionLoop(true, 4);


}