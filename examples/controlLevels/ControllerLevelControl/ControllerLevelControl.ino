#include "modBlimp.h"
#include "BNO55.h"
#include "baro280.h"


ModBlimp blimp;
BNO55 bno;
baro280 baro;


IBusBM IBus; 


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
raw_t raws;
actuation_t outputs;


void setup() {
  
    
  //initializes systems based on flags and saves flags into the system
  blimp.init(&init_flags, &init_sensors, &feedbackPD);
    
  delay(100);
  baro.init();
  bno.init();

  getLatestSensorData(&sensors);
  sensors.groundZ = baro.getEstimatedZ();


}
float absoluteyawave = 0;
bool snapon = 0;
// float resyncPitch = 0.09;
// float resyncPitchTemp = 0;
// float resyncTimer = 0;
void loop() {

  

  /*
  //    attempts to get the lastest information about the CONTROLLER and places them into the 
  //    raw_t data structure
  //    contains: flag, ready, data[11]
  */
  blimp.getControllerRaws(&raws);


  /*
  if flag = 0: normal control logic
  if flag = 1 or 2: use old magnetometer calibration
  if flag = 10, 11 or 12: do flag changes // flag changes should turn off or on sensor feedback as well as PID controls
  if flag = 20: do low level control
  if flag = 21: do high level control (same as 0)
  if flag = 22: do nicla low level control
  if flag = 23: do nicla high level control
  */
  int flag = raws.flag;
  
  getLatestSensorData(&sensors);
  sensors.pitch =  sensors.pitch -3.1416 - 0.14;//hack to invert pitch due to orientation of the sensor
  while (sensors.pitch > 3.1416) {
    sensors.pitch -= 3.1416*2;
  }
  while (sensors.pitch < -3.1416) {
    sensors.pitch += 3.1416*2;
  }

  if (flag == 0 or flag == 1 or flag == 2){// uses control of what used to be the correct way
    oldLogic(); //changes outputs using the old format
  } else if (flag == 10 or flag == 11 or flag == 12){
    //set FLAGS for other stuff
  } else if (flag == 20 or flag == 22){ // low level control
    if (flag == 20){
      outputs.ready = raws.ready;
      outputs.m1 = raws.data[0];
      outputs.m2 = raws.data[1];
      outputs.s1 = raws.data[2];
      outputs.s2 = raws.data[3];
    } else { //nicla control
      IBus.loop();
      outputs.ready = raws.ready;
      outputs.m1 = (float)IBus.readChannel(0)/1000.0f;
      outputs.m2 = (float)IBus.readChannel(1)/1000.0f;
      outputs.s1 = (float)IBus.readChannel(2)/1000.0f;
      outputs.s2 = (float)IBus.readChannel(3)/1000.0f;

    }

  }
  else if (flag == 21 or flag == 23){ // high level control
    if (flag == 21){
      controls.ready = raws.ready;
      controls.fx = raws.data[0];
      controls.fy = raws.data[1];
      controls.fz = raws.data[2];
      controls.tx = raws.data[3];
      controls.ty = raws.data[4];
      controls.tz = raws.data[5];
    } else { //nicla control
      IBus.loop();
      controls.ready = raws.ready;
      controls.fx = (float)IBus.readChannel(0)/1000.0f;
      controls.fy = (float)IBus.readChannel(1)/1000.0f;
      controls.fz = (float)IBus.readChannel(2)/1000.0f;
      controls.tx = (float)IBus.readChannel(3)/1000.0f;
      controls.ty = (float)IBus.readChannel(4)/1000.0f;
      controls.tz = (float)IBus.readChannel(5)/1000.0f;
    }
    
    addFeedback(&controls, &sensors); //this function is implemented here for you to customize
    blimp.getOutputs(&controls, &sensors, &outputs);

  }


  blimp.executeOutputs(&outputs);
  delay(4);
  


}

void oldLogic() {

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

  // Serial.print(sensors.yaw);
  // Serial.print(", ");
  // Serial.print(sensors.roll);
  // Serial.print(", ");
  // Serial.println(sensors.estimatedZ);

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
  // blimp.executeOutputs(&outputs);
  // delay(4);
}

time_t snaptime;

float aveyaw = 0;
float tempyaw = 0;
int oldsnap = 0;
float ballz = 0;

void controlAlgorithm(controller_t *controls, sensors_t *sensors) {
  blimp.IBus.loop();
  
  //randomWalk(controls, sensors);
  
  followBall(controls, sensors);
    
}

time_t randtime = 0;
float randYaw = 0;

void randomWalk(controller_t *controls, sensors_t *sensors){
  // int cx = blimp.IBus.readChannel(0);
  // int cy = blimp.IBus.readChannel(1);
  // int sig = blimp.IBus.readChannel(2);
  int tof = blimp.IBus.readChannel(3);
  // time_t currTimme = millis();
  // if (currtime-randtime > 10 * 1000){

  // }
  
  if (controls->snapshot != 0 ) {
    if (tof < 1300) {
      randYaw = sensors->yaw + 2.0f* 3.1416f/3.0f ;
    }
    float tempdiff = -1*(randYaw - sensors->yaw);
    while (tempdiff > 3.1416f){
      tempdiff -= 6.283f;
    }
    while (tempdiff < -3.1416f){
      tempdiff += 6.283f;
    }
    // float dyaw = tempdiff - oldyaw;
    // oldyaw = tempdiff;
    // float kdabsyaw = 1;
    float actyaw = clamp(tempdiff,-1.5,1.5);//- dyaw*kdabsyaw;
    if (abs(actyaw )< .15){
      controls->fx = .23;
    } else 
    
    // actyaw = actyaw;
    aveyaw = aveyaw * 0 + (actyaw) * 1;
  }
  Serial.print(aveyaw);
  Serial.print(",");
  Serial.println(tof);
}

time_t  tofTime = 0;

void followBall(controller_t *controls, sensors_t *sensors){
  int cx = blimp.IBus.readChannel(0);
  int cy = blimp.IBus.readChannel(1);
  int sig = blimp.IBus.readChannel(2);
  int tof = blimp.IBus.readChannel(3);
  if (controls->snapshot != 0 ) {
    controls->snapshot = sig;
    controls->tz  = (float)((cx-120)/3)*3.14159f/180.0f;
    controls->fz = ballz - sensors->estimatedZ;
    if (controls->snapshot != oldsnap) {
      ballz = sensors->estimatedZ - (float)(((float)cy-170.0f)/120.0f) * 1.5;
      snaptime = millis();
      tempyaw = sensors->yaw + controls->tz;//controls->tz;//
      oldsnap = controls->snapshot;
      while (tempyaw > 3.1416f){
        tempyaw -= 6.283f;
      }
      while (tempyaw < -3.1416f){
        tempyaw += 6.283f;
      }
    } else if (millis() - snaptime > 8000){// if time since last snap > 5 seconds do patterned walk on yaw
      // tempyaw = sensors->yaw + 3.1415/4.0f;
      // snaptime = millis();
      controls->fz = (sensors->groundZ+3) - sensors->estimatedZ;

      if (tof < 1500) {
        tofTime = millis();
        tempyaw = sensors->yaw + 2.0f* 3.1416f/3.0f ;
        while (tempyaw > 3.1416f){
          tempyaw -= 6.283f;
        }
        while (tempyaw < -3.1416f){
          tempyaw += 6.283f;
        }
      }
      if (millis() - tofTime <3000){
        controls->fx  = -0.3f;
      }

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
    if (abs(actyaw )< .15){
      controls->fx = .23;
    }
    
    // actyaw = actyaw;
    aveyaw = aveyaw * 0 + (actyaw) * 1;
  } else {
    tempyaw = sensors->yaw;
    aveyaw = 0;
    oldsnap = 0;
  }

  // Serial.print(controls->snapshot);
  // Serial.print(", ");
  // Serial.print(ballz);
  // Serial.print(", ");
  // Serial.print(aveyaw);
  // Serial.print(", ");
  // Serial.print(millis() - snaptime);
  // Serial.print(", ");
  // Serial.println(tof);
}

/*
  -----------------------------------------------------------------------------------------------------
  EXAMPLE FUNCTIONS for full customization on outputs
  If you want to add a new sensor, you can try to go into firmware (crazyflieComplementary.cpp)
      or just implement it in this program
  -----------------------------------------------------------------------------------------------------
*/

void getLatestSensorData(sensors_t *sensors) {
  bno.updateSensors(sensors);
  sensors->estimatedZ = sensors->estimatedZ * .8  + baro.getEstimatedZ()*.2;
  sensors->velocityZ = sensors->velocityZ * .8 + baro.getVelocityZ()*.2;
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
      
      controls->tz = controls->tz + aveyaw * PDterms->kpyaw - sensors->yawrate*PDterms->kdyaw;
      
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




