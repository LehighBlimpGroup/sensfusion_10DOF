#include "BNO55.h"

BNO55::BNO55(){
    
  //pass
    
    
}

void BNO55::init(){
    
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    
  } else{
    Serial.println("BNO started!");
    bnoOn = true;
  }
    
    
}

void BNO55::updateSensors(sensors_t *sensors){
    // Serial.println("BNO Update!");

  if (bnoOn){
    sensors_event_t orientationData, angVelocityData;//, linearAccelData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    sensors->yaw = orientationData.orientation.x* 3.1416f/180.0f;
    if (sensors->yaw > 3.1416f){
      sensors->yaw -= 3.1416f*2;
    }
    sensors->roll = orientationData.orientation.y* 3.1416f/180.0f;
    sensors->pitch = orientationData.orientation.z* 3.1416f/180.0f;
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    sensors->yawrate = sensors->yawrate *.7 + angVelocityData.gyro.z* 3.1416f/180.0f * .3;
    sensors->rollrate = sensors->rollrate *.7+ angVelocityData.gyro.y* 3.1416f/180.0f* .3;
    sensors->pitchrate = sensors->pitchrate *.7+  angVelocityData.gyro.x* 3.1416f/180.0f* .3;
    //bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  } else {
    sensors->yaw = 0;
    sensors->roll = 0;
    sensors->pitch = 0;
    
    sensors->yawrate = 0;
    sensors->rollrate = 0;
    sensors->pitchrate = 0;
  }
}
