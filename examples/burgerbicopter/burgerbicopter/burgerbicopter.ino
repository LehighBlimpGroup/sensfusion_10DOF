#include "modBlimp.h"

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
  .sensors = true,
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
  .kpyaw = 2.0f,
  .kdyaw = 5.0f,//1.5f,

  .kpx = 0,
  .kdx = 0,
  .kpy = 0,
  .kdy = 0,
  .kpz = 0.4f,
  .kdz = 70,

  .lx = .15,
};
feedback_t * PDterms = &feedbackPD;
//storage variables
sensors_t sensors;
controller_t controls;
//rawInput_t rawInputs;TODO
actuation_t outputs;




void setup() {
  
    
  //initializes systems based on flags and saves flags into the system
  blimp.init(&init_flags, &init_sensors, &feedbackPD);
    



}
float yawabs = 0;
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
  blimp.getLatestSensorData(&sensors);

  sensors.pitch =  sensors.pitch - .23;//hack to invert pitch due to orientation of the sensor
  
  
  // Serial.print(sensors.roll);
  // // Serial.print(", ");
  // Serial.println(sensors.yaw);

  /*
  //    attempts to get the lastest information about the CONTROLLER and places them into the 
  //    controller_t data structure
  //    contrains: fx, fy, fz, absz, tx, ty, tz, ready
  */
  blimp.getControllerData(&controls);
  blimp.IBus.loop();
  int cx = blimp.IBus.readChannel(0);
  int cy = blimp.IBus.readChannel(1);
  // if (controls.snapshot != 0){ // code is in abs yaw following mode
  //   if (cx !=0 || cy != 0){
  //     //balloon is detected
  //     yawabs = yawabs * .9f + ((((float)cx) /120.0f-.5)*20.0f)*.1f


  //   } else {
  //     //no balloon detected

  //   }
  // }
  // Serial.print(cx);
  // Serial.print(", ");
  // Serial.print(cy);
  // Serial.print(", ");
  // Serial.println(1);


  // if (resyncTimer > 1000){
  //   resyncTimer = 0;
  //   resyncPitch = resyncPitchTemp;
  //   Serial.println(resyncPitch); 

  // } else if (controls.tz == 0 && controls.fx ==0 && controls.ready){
  //   resyncPitchTemp = resyncPitchTemp*.995 + sensors.pitch * .005;
  //   resyncTimer += 1;

  // } else {
  //   resyncTimer = 0;
  //   resyncPitchTemp = sensors.pitch;
  // }

  // controls.ready = true;
  // controls.fz = .3;
  // controls.fx = .5;

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
  delay(5);
  


}

/*
  -----------------------------------------------------------------------------------------------------
  EXAMPLE FUNCTIONS for full customization on outputs
  If you want to add a new sensor, you can try to go into firmware (crazyflieComplementary.cpp)
      or just implement it in this program
  -----------------------------------------------------------------------------------------------------
*/
float fzave = 0;
float tzave = 0;
float tempyaw = 0;
float oldyaw = 0;
float aveyaw = 0;
float oldsnap = 0;

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
    controls->fz = (controls->fz - (sensors->estimatedZ-sensors->groundZ))*PDterms->kpz 
                    - (sensors->velocityZ)*PDterms->kdz + controls->absz;
    fzave = fzave * .975 + controls->fz * .025;
    controls->fz = fzave;
    }
    
    //yaw feedback
    if (PDterms->yaw) { 
      if (controls->snapshot != 0 ) {
        if (controls->snapshot != oldsnap){
          tempyaw = sensors->yaw + controls->tz;
          oldsnap = controls->snapshot;
          if (tempyaw > 3.1416f){
            tempyaw -= 6.283f;
          }
          else if (tempyaw < -3.1416f){
            tempyaw += 6.283f;
          }
        } 
        controls->tz = 0;
        float tempdiff = (tempyaw - sensors->yaw);
        if (tempdiff > 3.1416f){
          tempdiff -= 6.283f;
        }
        else if (tempdiff < -3.1416f){
          tempdiff += 6.283f;
        }
        // float dyaw = tempdiff - oldyaw;
        // oldyaw = tempdiff;
        // float kdabsyaw = 1;
        float actyaw = clamp(tempdiff,-1,1);//- dyaw*kdabsyaw;
        
        // actyaw = actyaw;
        aveyaw = aveyaw * .99 + actyaw * .01;
      } else {
        tempyaw = sensors->yaw;
        aveyaw = 0;
        oldsnap = 0;
      }

      // Serial.print(controls->snapshot);
      // Serial.print(", ");
      // Serial.print(aveyaw);
      // Serial.print(", ");
      // Serial.print(sensors->yaw);
      // Serial.print(", ");
      // Serial.println(tempyaw);
      controls->tz = controls->tz  - sensors->yawrate*PDterms->kdyaw;
      tzave = tzave * .9 + controls->tz * .1;
      controls->tz = tzave+ aveyaw * PDterms->kpyaw;
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




