#include "modBlimp.h"
#include <Adafruit_BNO055.h>


ModBlimp blimp;

String blimpcommand = "Custom";//"Automatic"; //"Custom"


IBusBM IBus; 
//HardwareSerial MySerial0(0);
/*
flags to be used in the init 
-bool verbose: allows some debug print statments
-bool sensors: enables or disables the sensorsuite package: if false all values will be 0, and sensorReady =false in the sensor 
-bool UDP: starts up the UDP connection such that other UDP functions will be enabled
-int motor_type: determines if you are using brushless or brushed motors: 0 = brushless, 1 = brushed;
-int mode: sets which controller to listen to: 0 = UDP, 1 = IBUS,2 = espnow, -1 = None;
-int control: sets which type of controller to use: 0 = bicopter, 1 = spinning(TODO), -1 = None;
*/
init_flags_t init_flags = {
  .verbose = false,
  .sensors = false,
  .escarm = true,
  .UDP = false,
  .Ibus = true,
  .ESPNOW = true,
  .PORT = 1345,
  .motor_type = 0,
  .mode = 2,
  .control = 0,
};


/*
sensor values that control the sensors - if you want to turn off sensors use init_flags (sensors = false)
- float Kacc: kp value in implementation for accelerometer
- float Kgyro: kp value in implementation for gyroscope
- float Kmag: kp value in implementation for magnetometer
- bool baro: enables or disables barometer
- float eulerGamma: is the weight of the weighted average on euler angles (0 means that it always uses the newest value)
- float rateGamma: is the weight of the weighted average on gyroscope rates (0 means that it always uses the newest value)
- float zGamma: is the weight of the weighted average on estimatedZ (0 means that it always uses the newest value)
*/
init_sensors_t init_sensors = {
  .Kacc = 5,
  .Kgyro = -1,
  .Kmag = 0,
  .baro = true,
  .eulerGamma = 0,
  .rateGamma = 0.9f,
  .zGamma = 0.9f,
};


/*
PD terms for use in the feedback controller 
- bool roll, pitch, yaw, x, y, z, rotation: 
          enables that type of feedback (true means feedback is on for that state variable)
- float Croll, Cpitch, Cyaw, Cx, Cy, Cz, Cabsz: 
          KP term applied to the controller input
- float kproll, kdroll, kppitch, kdpitch, kpyaw, kdyaw: 
- float kpx, kdx, kpy, kdy, kpz, kdz;
          Kp and kd terms applied to each feedback mechanism using the sensors 
          (some do not have sensor availibility like x and y)
- float lx;
          a control variable used as the arm between the center of mass and the propellers
*/
feedback_t feedbackPD = {
  .roll = false,
  .pitch = false, 
  .yaw = true,
  .x = false,
  .y = false,
  .z = true,
  .rotation = false,

  .Croll = 1,
  .Cpitch = 0, 
  .Cyaw = 1,
  .Cx = 1,
  .Cy = 0,
  .Cz = 1,
  .Cabsz = 1,

  .kproll = 0,
  .kdroll = 0.0f,
  .kppitch = 0,
  .kdpitch = 0,
  .kpyaw = 3.0f,
  .kdyaw = -150.0f,//-70//5f,

  .kpx = 0,
  .kdx = 0,
  .kpy = 0,
  .kdy = 0,
  .kpz = 0.1f,//.4f
  .kdz = -0.5f,

  .lx = .15,
};
feedback_t * PDterms = &feedbackPD;
//storage variables
sensors_t sensors;
controller_t controls;
//rawInput_t rawInputs;TODO
actuation_t outputs;



// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP280 bme; // I2C

float oldZ = 0;
float rawZ = 0;
float refZ = 0;
float barorate = 50;
bool baroOn = false;
time_t barotime;

bool bnoOn = false;

void setup() {
  
    
  //initializes systems based on flags and saves flags into the system
  blimp.init(&init_flags, &init_sensors, &feedbackPD);
    

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    
  } else{
    bnoOn = true;
  }
  /* Initialise the sensor */
  int countTries = 0;
  baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  while (!baroOn) {
      delay(100);
      if (countTries > 10) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                          "try a different address!"));
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        break;
      }
      countTries += 1;
      baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  }
  
  barotime = micros();
  if (baroOn){
    sensors.groundZ = bme.readAltitude();
    sensors.estimatedZ = sensors.groundZ;
    refZ = sensors.groundZ;
    rawZ = sensors.groundZ;
  } else {
    sensors.groundZ = 0;
    sensors.estimatedZ = 0;
    refZ = sensors.groundZ;
    rawZ = sensors.groundZ;
  }
  controls.snapshot = 0;

  delay(1000);


}
float absoluteyawave = 0;
bool snapon = 0;
// float resyncPitch = 0.09;
// float resyncPitchTemp = 0;
// float resyncTimer = 0;
void loop() {
  /*  
  //    attempts to get the lastest information about the SENSORS and places them into the 
  //    sensor_t data structure
  //    contains: roll, pitch, yaw, rollrate, pitchrate, yawrate, estimatedZ, velocityZ, groundZ
  //    will return 0 for all sensors if sensors == false
  */
  getLatestSensorData(&sensors);

  sensors.pitch =  sensors.pitch -3.1416 - 0.14;//hack to invert pitch due to orientation of the sensor
  while (sensors.pitch > 3.1416) {
    sensors.pitch -= 3.1416*2;
  }
  while (sensors.pitch < -3.1416) {
    sensors.pitch += 3.1416*2;
  }
  


  /*
  //    attempts to get the lastest information about the CONTROLLER and places them into the 
  //    controller_t data structure
  //    contrains: fx, fy, fz, absz, tx, ty, tz, ready
  */
  blimp.getControllerData(&controls);

  controlAlgorithm(&controls, &sensors);
  

  /* TODO- NOT IMPLEMENTED
  //    optionally you can get the lastest information about the controller as raw values labeled as I1, I2, I3...
  */
  //rawInputs = blimp.getRawInputs();


  /*
  //    adds feedback directly into the controller terms using sensor feedback
  //    replace this with your own custom function for more customization
  //        example is placed below
  */
  //blimp.addFeedback(&controls, &sensors);
  addFeedback(&controls, &sensors); //this function is implemented here for you to customize


  // Serial.print(controls.fz);
  // Serial.print(", ");
  // Serial.print(controls.tz);
  // Serial.print(", ");
  // Serial.println(sensors.yawrate*100);

  // Serial.print(sensors.yawrate*100);
  // Serial.print(", ");
  // Serial.print(sensors.roll);
  // Serial.print(", ");
  // Serial.println(sensors.pitch);

  /*
  //    uses the mode to determine the control scheme for the motor/servo outputs
  //    currently only implementation is for the bicopter blimp
  //    replace this with your own custom function for more customization
  //    actuation_t data type contains: m1, m2, s1, s2 for each motor and servo
  //        example is placed below
  */
  blimp.getOutputs(&controls, &sensors, &outputs);
  //getOutputs(&controls, &sensors, &outputs); //this function is implemented here for you to customize
  
  /*
  //    uses the mode to determine the ouput scheme for the motor/servo outputs
  //    currently only implementation is for the bicopter blimp
  //    outputs should be floats between 0 and 1
  */
  blimp.executeOutputs(&outputs);
  delay(4);
  


}

time_t snaptime;

float aveyaw = 0;
float tempyaw = 0;
int oldsnap = 0;
float ballz = 0;

void controlAlgorithm(controller_t *controls, sensors_t *sensors) {
  blimp.IBus.loop();
  motionControl(controls, sensors);

//   if (false){
//     randomWalk(controls, sensors);
//   } else {
//     followBall(controls, sensors);
//   }
}

time_t randtime = 0;
float randYaw = 0;
void randomWalk(controller_t *controls, sensors_t *sensors){
  // int cx = blimp.IBus.readChannel(0);
  // int cy = blimp.IBus.readChannel(1);
  // int sig = blimp.IBus.readChannel(2);
  // int tof = blimp.IBus.readChannel(3);
  // time_t currTimme = millis();
  // if (currtime-randtime > 10 * 1000){

  // }
  // if (tof < 1000) {
  //   controls->fx = 0;
  //   tempYaw = sensors->yaw - 3.1416f
  // }

}

void motionControl(controller_t *controls, sensors_t *sensors){
  int yaw = blimp.IBus.readChannel(0);  // rad * 1000
  int z = blimp.IBus.readChannel(1);   // in cm (max 65536)
  int fx = blimp.IBus.readChannel(2);
//  int tof = blimp.IBus.readChannel(3);

  if (controls->snapshot != 0 ) {
      controls->tz = (float) yaw / 1000.0;
      controls->fz = (float) z / 100.0;
      controls->fx = (float) fx / 100.0;
  }
}


void followBall(controller_t *controls, sensors_t *sensors){
  int cx = blimp.IBus.readChannel(0);
  int cy = blimp.IBus.readChannel(1);
  int sig = blimp.IBus.readChannel(2);
  int tof = blimp.IBus.readChannel(3);
  if (controls->snapshot != 0 ) {
    controls->snapshot = sig;
    controls->tz  = (float)((cx-120)/3)*3.14159f/180.0f;
    controls->fz = ballz;
    if (controls->snapshot != oldsnap) {
      ballz += (float)((cy-120)/120*2);
      if (abs(ballz) > 15){
        ballz = 15 * ballz/abs(ballz);
      }
      snaptime = millis();
    //  `tempyaw` = sensors->yaw + controls->tz;//controls->tz;//
      oldsnap = controls->snapshot;
      while (tempyaw > 3.1416f){
        tempyaw -= 6.283f;
      }
      while (tempyaw < -3.1416f){
        tempyaw += 6.283f;
      }
    } else if (millis() - snaptime > 5000){// if time since last snap > 5 seconds do patterned walk on yaw
      tempyaw = sensors->yaw + 3.1415/4.0f;
      snaptime = millis();

    }
    controls->tz = 0;
    float tempdiff = -1*(tempyaw - sensors->yaw);
    while (tempdiff > 3.1416f){
      tempdiff -= 6.283f;
    }
    while (tempdiff < -3.1416f){
      tempdiff += 6.283f;
    }
    // float dyaw = tempdiff - oldyaw;
    // oldyaw = tempdiff;
    // float kdabsyaw = 1;
    float actyaw = clamp(tempdiff,-1,1);//- dyaw*kdabsyaw;
    
    // actyaw = actyaw;
    aveyaw = aveyaw * 0 + (actyaw) * 1;
  } else {
    tempyaw = sensors->yaw;
    aveyaw = 0;
    oldsnap = 0;
  }

  Serial.print(controls->snapshot);
  Serial.print(", ");
  Serial.print(ballz);
  Serial.print(", ");
  Serial.print(cy);
  Serial.print(", ");
  Serial.print(cx);
  Serial.print(", ");
  Serial.println(tof);
}

/*
  -----------------------------------------------------------------------------------------------------
  EXAMPLE FUNCTIONS for full customization on outputs
  If you want to add a new sensor, you can try to go into firmware (crazyflieComplementary.cpp)
      or just implement it in this program
  -----------------------------------------------------------------------------------------------------
*/

void getLatestSensorData(sensors_t *sensors) {
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
    sensors->yawrate = sensors->yawrate *.9 + angVelocityData.gyro.z* 3.1416f/180.0f * .1;
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
  oldZ = rawZ;
  time_t newtime = micros();
  int barotimer = newtime - barotime; 
  if (barotimer > 1/barorate * 1000000) {
    if (baroOn) {
      float newHeight = bme.readAltitude();
      if (newHeight > 400 or newHeight < 100){
        baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
      } else {
        rawZ = newHeight;
        // Serial.print(sensors->velocityZ);
        // Serial.print(",");
        // Serial.println(sensors->estimatedZ);
        refZ += (rawZ - refZ) * 0.02;     
        sensors->estimatedZ = sensors->estimatedZ * .8 +  refZ* .2;
        sensors->velocityZ = sensors->velocityZ *.8 + (sensors->estimatedZ - oldZ)*.2;
      }
    } else {
      baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);

    }
    barotime = newtime;
    //baroHeightave = baroHeightave*.95 + baroHeight*.05;

  }
  //sensors->groundZ = 0;
}

float fzave = 0;
float tzave = 0;
// float tempyaw = 0;
// float oldyaw = 0;
// float aveyaw = 0;
// float oldsnap = 0;

//adds sensor feedback into the control values
//this set is specifically made for bicopter
void addFeedback(controller_t *controls, sensors_t *sensors) {
    //controller weights
    controls->fx *= PDterms->Cx;
    controls->fy *= PDterms->Cy;
    controls->fz *= PDterms->Cz;
    controls->tx *= PDterms->Croll;
    controls->ty *= PDterms->Cpitch;
    controls->tz *= PDterms->Cyaw;
    controls->absz *= PDterms->Cabsz;

    //z feedback 
    if (PDterms->z) { 
    controls->fz = (controls->fz + controls->absz - (sensors->estimatedZ-sensors->groundZ))*PDterms->kpz 
                    - (sensors->velocityZ)*PDterms->kdz;


    // fzave = fzave * .9 + controls->fz * .1;
    // controls->fz = fzave;
    }
    
    //yaw feedback
    if (PDterms->yaw) {
      //controls->tz = controls->tz + aveyaw * PDterms->kpyaw - sensors->yawrate * PDterms->kdyaw;
      float e_yaw = -(controls->tz - sensors->yaw);
      while (e_yaw > 3.1416f){
      e_yaw -= 6.283f;
      }
      while (e_yaw < -3.1416f){
        e_yaw += 6.283f;
      }
     //e_yaw = clamp(e_yaw,-1,1);//- dyaw*kdabsyaw;




      controls->tz = PDterms->kpyaw * e_yaw - PDterms->kdyaw * sensors->yawrate;
    }
    
    //roll feedback
    if (PDterms->roll) { 
      controls->tx = controls->tx - sensors->roll* PDterms->kproll - sensors->rollrate * PDterms->kdroll;
    }

    //roll and pitch rotation state feedback
    if (PDterms->rotation) { 
      float cosp = (float) cos(sensors->pitch);
      float sinp = (float) sin(sensors->pitch);
      float cosr = (float) cos(sensors->roll);
      float ifx = controls->fx;
      controls->fx = ifx*cosp + controls->fz*sinp;
      controls->fz = (-1*ifx*sinp + controls->fz* cosp)/cosr;
    }
}


//creates the output values used for actuation from the control values
void getOutputs(controller_t *controls, sensors_t *sensors, actuation_t *out ){
    
    //set up output
    

    //set output to default if controls not ready
    if (controls->ready == false){
      out->s1 = .5f;
      out->s2 = .5f;
      out->m1 = 0;
      out->m2 = 0;
      out->ready = false;
      return;
    }

    out->ready = true;
    //inputs to the A-Matrix
    float l = PDterms->lx; //.3
    
    float fx = clamp(controls->fx, -1 , 1);//setpoint->bicopter.fx;
    float fz = clamp(controls->fz, 0.1 , 2);//setpoint->bicopter.fz;
    float maxRadsYaw = .25;//.175;
    float magxz = sqrt(fz*fz + fx*fx)* tan(maxRadsYaw); //limits the yaw based on the magnitude of the force
    float taux = clamp(controls->tx, -l + (float)0.01 , l - (float) 0.01);
    float tauz = clamp(controls->tz, -1 , 1)*magxz;// limit should be .25 setpoint->bicopter.tauz; //- stateAttitudeRateYaw


    //inverse A-Matrix calculations
    float term1 = l*l*fx*fx + l*l*fz*fz + taux*taux + tauz*tauz;
    float term2 = 2*fz*l*taux - 2*fx*l*tauz;
    float term3 = sqrt(term1+term2);
    float term4 = sqrt(term1-term2);
    float f1 = term3/(2*l); // in unknown units
    float f2 = term4/(2*l);
    float t1 = atan2((fz*l - taux)/term3, (fx*l + tauz)/term3 ) - sensors->pitch;// in radians
    float t2 = atan2((fz*l + taux)/term4, (fx*l - tauz)/term4 ) - sensors->pitch;

    //checking for full rotations
    while (t1 < -PI/2) {
      t1 = t1 + 2 * PI;
    }
    while (t1 > 3*PI/2) {
      t1 = t1 - 2 * PI;
    }
    while (t2 < -PI/2) {
      t2 = t2 + 2 * PI;
    }
    while (t2 > 3*PI/2) {
      t2 = t2 - 2 * PI;
    }

    //converting values to a more stable form
    out->s1 = clamp(t1, 0, PI)/(PI);// cant handle values between PI and 2PI
    out->s2 = clamp(t2, 0, PI)/(PI);
    out->m1 = clamp(f1, 0, 1);
    out->m2 = clamp(f2, 0, 1);
    if (out->m1 < 0.02f ){
      out->s1 = 0.5f; 
    }
    if (out->m2 < 0.02f ){
      out->s2 = 0.5f; 
    }
    return;
}
float clamp(float in, float min, float max){
  if (in< min){
    return min;
  } else if (in > max){
    return max;
  } else {
    return in;
  }
}




