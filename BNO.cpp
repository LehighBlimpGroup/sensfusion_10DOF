

// #include <Adafruit_BNO055.h>
// #include <Adafruit_BMP280.h>

// typedef struct sensors_s {
//     float roll, pitch, yaw;
//     float rollrate, pitchrate, yawrate;
//     float estimatedZ, velocityZ, groundZ;
// } sensors_t;

// sensors_t sensors;
// // Check I2C device address and correct line below (by default address is 0x29 or 0x28)
// //                                   id, address
// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
// Adafruit_BMP280 bme; // I2C

// float oldZ = 0;
// float rawZ = 0;
// float refZ = 0;
// float barorate = 50;
// bool baroOn = false;
// time_t barotime;

// void setup(){
//       /* Initialise the sensor */
//   if (!bno.begin())
//   {
//     /* There was a problem detecting the BNO055 ... check your connections */
//     Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//     while (1);
//   }
//   /* Initialise the sensor */
//   int countTries = 0;
//   baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
//   while (!baroOn) {
//       delay(100);
//       if (countTries > 10) {
//         Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
//                           "try a different address!"));
//         Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
//         Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
//         Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
//         Serial.print("        ID of 0x60 represents a BME 280.\n");
//         Serial.print("        ID of 0x61 represents a BME 680.\n");
//         break;
//       }
//       countTries += 1;
//       baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
//   }
  
//   barotime = micros();
//   if (baroOn){
//     sensors.groundZ = bme.readAltitude();
//     sensors.estimatedZ = sensors.groundZ;
//     refZ = sensors.groundZ;
//     rawZ = sensors.groundZ;
//   } else {
//     sensors.groundZ = 0;
//     sensors.estimatedZ = 0;
//     refZ = sensors.groundZ;
//     rawZ = sensors.groundZ;
//   }
// }

// void getLatestSensorData(sensors_t *sensors) {
//   sensors_event_t orientationData, angVelocityData, linearAccelData;
  
  
//   bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
//   sensors->yaw = orientationData.orientation.x* 3.1416f/180.0f;
//   if (sensors->yaw > 3.1416f){
//     sensors->yaw -= 3.1416f*2;
//   }
//   sensors->roll = orientationData.orientation.y* 3.1416f/180.0f;
//   sensors->pitch = orientationData.orientation.z* 3.1416f/180.0f;
//   bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
//   sensors->yawrate = sensors->yawrate *.7 + angVelocityData.gyro.z* 3.1416f/180.0f * .3;
//   sensors->rollrate = sensors->rollrate *.7+ angVelocityData.gyro.y* 3.1416f/180.0f* .3;
//   sensors->pitchrate = sensors->pitchrate *.7+  angVelocityData.gyro.x* 3.1416f/180.0f* .3;
//   bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

//   oldZ = rawZ;
//   time_t newtime = micros();
//   int barotimer = newtime - barotime; 
//   if (barotimer > 1/barorate * 1000000) {
//     if (baroOn) {
//       float newHeight = bme.readAltitude();
//       if (newHeight > 400 or newHeight < 100){
//         baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
//       } else {
//         rawZ = newHeight;
//         // Serial.print(sensors->velocityZ);
//         // Serial.print(",");
//         // Serial.println(sensors->estimatedZ);
//         refZ += (rawZ - refZ) * 0.02;     
//         sensors->estimatedZ = sensors->estimatedZ * .8 +  refZ* .2;
//         sensors->velocityZ = sensors->velocityZ *.8 + (sensors->estimatedZ - oldZ)*.2;
//       }
//     } else {
//       baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);

//     }
//     barotime = newtime;
//     //baroHeightave = baroHeightave*.95 + baroHeight*.05;

//   }
//   //sensors->groundZ = 0;
// }
