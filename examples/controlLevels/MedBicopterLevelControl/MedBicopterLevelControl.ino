#include "modBlimp.h"
#include "BNO55.h"
#include "baro390.h"


ModBlimp blimp;
BNO55 bno;
baro390 baro;


IBusBM IBus; 


/*
flags to be used in the init 
-bool verbose: allows some debug print statments
-bool sensors: enables or disables the sensorsuite package: if false all values will be 0, and sensorReady =false in the sensor 
-bool UDP: starts up the UDP connection such that other UDP functions will be enabled
-int motor_type: determines if you are using brushless or brushed motors: 0 = brushless, 1 = brushed;
-int mode: sets which controller to listen to: 0 = UDP, 1 = IBUS,2 = espnow, -1 = None;
-int control: sets which type of controller to use: 0 = bicopter, 1 = spinning(TODO),2 = s-blimp, -1 = None;
*/
init_flags_t init_flags = {
  .verbose = false,
  .sensors = false,
  .escarm = true,
  .UDP = false,
  .Ibus = false,
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
sensor values that control the sensors - if you want to turn off sensors use init_flags (sensors = false)
- float Kacc: kp value in implementation for accelerometer
- float Kgyro: kp value in implementation for gyroscope
- float Kmag: kp value in implementation for magnetometer
- bool baro: enables or disables barometer
- float eulerGamma: is the weight of the weighted average on euler angles (0 means that it always uses the newest value)
- float rateGamma: is the weight of the weighted average on gyroscope rates (0 means that it always uses the newest value)
- float zGamma: is the weight of the weighted average on estimatedZ (0 means that it always uses the newest value)
*/
sensor_weights_t weights = {
  .eulerGamma = 0.0f,
  .rollRateGamma = 0.7f,
  .yawRateGamma = 0.7f,
  .pitchRateGamma = 0.7f,
  .zGamma = 0.9f,
  .vzGamma = 0.9f,
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
  .yaw = false,
  .x = false,
  .y = false,
  .z = false,
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
// EXTRA TERMS
float pitchSign = 1;
float pitchOffset = 0;
float kiz = 0;
float integral_dt = .01;
float z_int_low = 0;
float z_int_high = 50;

//active terms
float z_integral = 0;

//passive term
float servo1offset = 0;
float servo2offset = 0;


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
  // baro.init();
  // bno.init();

  getLatestSensorData(&sensors);
  sensors.groundZ = baro.getEstimatedZ();


}
float absoluteyawave = 0;
bool snapon = 0;
// float resyncPitch = 0.09;
// float resyncPitchTemp = 0;
// float resyncTimer = 0;
unsigned long timed = millis();
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
  // sensors.pitch =  sensors.pitch -3.1416 - 0.14;//hack to invert pitch due to orientation of the sensor
  // while (sensors.pitch > 3.1416) {
  //   sensors.pitch -= 3.1416*2;
  // }
  // while (sensors.pitch < -3.1416) {
  //   sensors.pitch += 3.1416*2;
  // }

  if ((int)(flag/10) == 0){// flag == 0, 1, 2uses control of what used to be the correct way
    return; //changes outputs using the old format
  } else if ((int)(flag/10) == 1){ //flag == 10, 11, 12
    //set FLAGS for other stuff
    setPDflags(PDterms,&weights, &raws);
    outputs.m1 = 0;
    outputs.m2 = 0;
    outputs.s1 = 0;
    outputs.s2 = 0;
    outputs.ready = false;
    z_integral = 0;
    
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
      controls.absz = raws.data[6];
    } else { //nicla control
      IBus.loop();
      controls.ready = raws.ready;
      controls.fx = (float)IBus.readChannel(0)/1000.0f;
      controls.fy = (float)IBus.readChannel(1)/1000.0f;
      controls.fz = (float)IBus.readChannel(2)/1000.0f;
      controls.tx = (float)IBus.readChannel(3)/1000.0f;
      controls.ty = (float)IBus.readChannel(4)/1000.0f;
      controls.tz = (float)IBus.readChannel(5)/1000.0f;
      controls.absz = (float)IBus.readChannel(6)/1000.0f;
    }
    
    addFeedback(&controls, &sensors); //this function is implemented here for you to customize
    getOutputs(&controls, &sensors, &outputs);

  }

  blimp.executeOutputs(&outputs);
  delay(4);
  


}

void setPDflags(feedback_t *PDterms, sensor_weights_t *weights, raw_t *raws){
  Serial.print("Set flags: ");
  Serial.println(raws->flag);
  if (raws->flag == 10){// enables or disables feedback in these terms
    PDterms->roll = raws->data[0] == 1.0f;
    PDterms->pitch = raws->data[1] == 1.0f;
    PDterms->yaw = raws->data[2] == 1.0f;
    PDterms->x = raws->data[3] == 1.0f;
    PDterms->y = raws->data[4] == 1.0f;
    PDterms->z = raws->data[5] == 1.0f;
    PDterms->rotation = raws->data[6] == 1.0f;
  }
  else if (raws->flag == 11){
    PDterms->Croll = raws->data[0];
    PDterms->Cpitch = raws->data[1];
    PDterms->Cyaw = raws->data[2];
    PDterms->Cx = raws->data[3];
    PDterms->Cy = raws->data[4];
    PDterms->Cz = raws->data[5];
    PDterms->Cabsz = raws->data[6];
  }
  else if (raws->flag == 12){
    PDterms->kproll = raws->data[0];
    PDterms->kdroll = raws->data[1];
    PDterms->kppitch = raws->data[2];
    PDterms->kdpitch = raws->data[3];
    PDterms->kpyaw = raws->data[4];
    PDterms->kdyaw = raws->data[5];
  }
  else if (raws->flag == 13){
    PDterms->kpx = raws->data[0];
    PDterms->kdx = raws->data[1];
    PDterms->kpy = raws->data[2];
    PDterms->kdy = raws->data[3];
    PDterms->kpz = raws->data[4];
    PDterms->kdz = raws->data[5];
    PDterms->lx = raws->data[6];
    pitchSign = raws->data[7];
    pitchOffset = raws->data[8];

  }
  else if (raws->flag == 14){
    weights->eulerGamma = raws->data[0];
    weights->rollRateGamma = raws->data[1];
    weights->pitchRateGamma = raws->data[2];
    weights->yawRateGamma = raws->data[3];
    weights->zGamma = raws->data[4];
    weights->vzGamma = raws->data[5];
  }
  else if (raws->flag == 15){
    kiz = raws->data[0];
    integral_dt = raws->data[1];
    z_int_low = raws->data[2];
    z_int_high = raws->data[3];
    servo1offset = raws->data[4];
    servo2offset = raws->data[5];
    
  }


}


void testMotors() {
  
  timed = millis();
  while (millis() - timed < 1000) {
    outputs.ready = true;
    outputs.m1 = 0.5;
    outputs.m2 = 0;
    outputs.s1 = 0;
    outputs.s2 = 0;
    blimp.executeOutputs(&outputs);
    
    delay (5);
  }
  timed = millis();
  while (millis() - timed < 1000) {
    outputs.m1 = 0;
    outputs.m2 = 0.5;
    outputs.s1 = 0;
    outputs.s2 = 0;
    blimp.executeOutputs(&outputs);
    
    delay (5);
  }
  timed = millis();
  while (millis() - timed < 1000) {
    outputs.m1 = 0;
    outputs.m2 = 0;
    outputs.s1 = 0.5;
    outputs.s2 = 0;
    blimp.executeOutputs(&outputs);
    
    delay (5);
  }
  timed = millis();
  while (millis() - timed < 1000) {
    outputs.m1 = 0;
    outputs.m2 = 0;
    outputs.s1 = 0;
    outputs.s2 = 0.5;
    blimp.executeOutputs(&outputs);
    
    delay (5);
  }
  timed = millis();
  while (millis() - timed < 1000) {
    outputs.ready = false;
    blimp.executeOutputs(&outputs);
    
    delay (5);
  }
  return;
}



/*
  -----------------------------------------------------------------------------------------------------
  EXAMPLE FUNCTIONS for full customization on outputs
  If you want to add a new sensor, you can try to go into firmware (crazyflieComplementary.cpp)
      or just implement it in this program
  -----------------------------------------------------------------------------------------------------
*/

void getLatestSensorData(sensors_t *sensors) {
  bno.updateSensors(sensors, &weights);
  sensors->estimatedZ = sensors->estimatedZ * weights.zGamma  + baro.getEstimatedZ()* (1 - weights.zGamma);
  sensors->velocityZ = sensors->velocityZ * weights.vzGamma + baro.getVelocityZ()*(1 - weights.zGamma);
}

float fzave = 0;
float tzave = 0;
// float tempyaw = 0;
// float oldyaw = 0;
float aveyaw = 0;
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
      if (controls->ready){
        z_integral += (controls->fz + controls->absz - (sensors->estimatedZ-sensors->groundZ)) * integral_dt;
        z_integral = clamp(z_integral, z_int_low,z_int_high);
        //Serial.println(z_integral);
      } 
      controls->fz = (controls->fz + controls->absz - (sensors->estimatedZ-sensors->groundZ))*PDterms->kpz 
                      - (sensors->velocityZ)*PDterms->kdz + (z_integral) * kiz;
      
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
void getOutputs(controller_t *controls, sensors_t *sensors, actuation_t *out)
{

  // set up output

  // set output to default if controls not ready
  if (controls->ready == false)
  {
    out->s1 = .5f;
    out->s2 = .5f;
    out->m1 = 0;
    out->m2 = 0;
    out->ready = false;
    return;
  }

  out->ready = true;
  // inputs to the A-Matrix
  float l = PDterms->lx; //.3

  float fx = clamp(controls->fx, -1, 1);                  // setpoint->bicopter.fx;
  float fz = clamp(controls->fz, 0.1, 2);                 // setpoint->bicopter.fz;
  //float maxRadsYaw = .07; //.1f                                 //.175;
  //float magxz = max(fz * tan(maxRadsYaw), fx * l * 0.17f); // limits the yaw based on the magnitude of the force
  float taux = clamp(controls->tx, -l + (float)0.01, l - (float)0.01);
  float tauz = clamp(controls->tz, -1, 1) ;//* magxz; // limit should be .25 setpoint->bicopter.tauz; //- stateAttitudeRateYaw

  // inverse A-Matrix calculations
  float term1 = l * l * fx * fx + l * l * fz * fz + taux * taux + tauz * tauz;
  float term2 = 2 * fz * l * taux - 2 * fx * l * tauz;
  float term3 = sqrt(term1 + term2);
  float term4 = sqrt(term1 - term2);
  float f1 = term3 / (2 * l); // in unknown units
  float f2 = term4 / (2 * l);
  float t1 = atan2((fz * l - taux) / term3, (fx * l + tauz) / term3) - sensors->pitch; // in radians
  float t2 = atan2((fz * l + taux) / term4, (fx * l - tauz) / term4) - sensors->pitch;

  // checking for full rotations
  while (t1 < -PI / 2)
  {
    t1 = t1 + 2 * PI;
  }
  while (t1 > 3 * PI / 2)
  {
    t1 = t1 - 2 * PI;
  }
  while (t2 < -PI / 2)
  {
    t2 = t2 + 2 * PI;
  }
  while (t2 > 3 * PI / 2)
  {
    t2 = t2 - 2 * PI;
  }

  // converting values to a more stable form
  
  out->s1 = clamp(t1 + servo1offset, 0, PI) / (PI); // cant handle values between PI and 2PI
  out->s2 = clamp(t2 + servo2offset, 0, PI) / (PI);
  out->m1 = clamp(f1, 0, 1);
  out->m2 = clamp(f2, 0, 1);
  if (out->m1 < 0.02f)
  {
    out->s1 = 0.5f;
  }
  if (out->m2 < 0.02f)
  {
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




