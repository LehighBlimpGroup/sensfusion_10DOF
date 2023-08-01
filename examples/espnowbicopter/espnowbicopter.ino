/*
 * @Author       : Hanqing Qi
 * @Date         : 2023-08-01 16:19:19
 * @LastEditors  : Hanqing Qi
 * @LastEditTime : 2023-08-01 16:23:56
 * @FilePath     : /sensfusion_10DOF/examples/espnowbicopter/espnowbicopter.ino
 * @Description  : 
 */

#include "modBlimp.h"

ModBlimp blimp;


//This code is a basic framework with IBUS in mind

/*
flags to be used in the init 
-bool verbose: allows some debug print statments
-bool sensors: enables or disables the sensorsuite package: if false all values will be 0, and sensorReady =false in the sensor 
-bool UDP: starts up the UDP connection such that other UDP functions will be enabled
-int motor_type: determines if you are using brushless or brushed motors: 0 = brushed, 1 = brushless;
-int mode: sets which controller to listen to: 0 = UDP, 1 = IBUS, -1 = None;
-int control: sets which type of controller to use: 0 = bicopter, 1 = spinning(TODO), -1 = None;
*/
init_flags_t init_flags = {
  .verbose = false,
  .sensors = false,
  .escarm = false,
  .UDP = false,
  .Ibus = false,
  .ESPNOW = true,
  .PORT = 1333,
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
  .rateGamma = .8f,
  .zGamma = 0,
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
  .rotation = true,

  .Croll = 0,
  .Cpitch = 0, 
  .Cyaw = -.4,
  .Cx = 1.2,
  .Cy = 0,
  .Cz = 6,
  .Cabsz = 0,

  .kproll = 0,
  .kdroll = 0,
  .kppitch = 0,
  .kdpitch = 0,
  .kpyaw = 0,
  .kdyaw = 0.07f,

  .kpx = 0,
  .kdx = 0,
  .kpy = 0,
  .kdy = 0,
  .kpz = 0.4f,
  .kdz = 0,

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


void loop() {
  /*  
  //    attempts to get the lastest information about the SENSORS and places them into the 
  //    sensor_t data structure
  //    contains: roll, pitch, yaw, rollrate, pitchrate, yawrate, estimatedZ, velocityZ, groundZ
  //    will return 0 for all sensors if sensors == false
  */
  blimp.getLatestSensorData(&sensors);

  //sensors.pitch = -1* sensors.pitch;//hack to invert pitch due to orientation of the sensor
  
  


  /*
  //    attempts to get the lastest information about the CONTROLLER and places them into the 
  //    controller_t data structure
  //    contrains: fx, fy, fz, absz, tx, ty, tz, ready
  */
  blimp.getControllerData(&controls);


  /* TODO- NOT IMPLEMENTED
  //    optionally you can get the lastest information about the controller as raw values labeled as I1, I2, I3...
  */
  //rawInputs = blimp.getRawInputs();


  /*
  //    adds feedback directly into the controller terms using sensor feedback
  //    replace this with your own custom function for more customization
  //        example is placed below
  */
  blimp.addFeedback(&controls, &sensors);
  //addFeedback(&controls, &sensors); //this function is implemented here for you to customize
  

  /*
  //    uses the mode to determine the control scheme for the motor/servo outputs
  //    currently only implementation is for the bicopter blimp
  //    replace this with your own custom function for more customization
  //    actuation_t data type contains: m1, m2, s1, s2 for each motor and servo
  //        example is placed below
  */
  blimp.getOutputs(&controls, &sensors, &outputs);
  //getOutputs(&controls, &outputs); //this function is implemented here for you to customize
  


  /*
  //    uses the mode to determine the ouput scheme for the motor/servo outputs
  //    currently only implementation is for the bicopter blimp
  //    outputs should be floats between 0 and 1
  */
  blimp.executeOutputs(&outputs);
  delay(5);
    

  
}



