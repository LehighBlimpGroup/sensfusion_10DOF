#include "modBlimp.h"

ModBlimp blimp;

String blimpcommand = "Automatic"; //"Custom"


/*
flags to be used in the init 
-bool verbose: allows some debug print statments
-bool sensors: enables or disables the sensorsuite package: if false all values will be 0, and sensorReady =false in the sensor 
-bool UDP: starts up the UDP connection such that other UDP functions will be enabled
-int mode: sets which controller to listen to: 0 = UDP, 1 = IBUS, -1 = None;
-int control: sets which type of controller to use: 0 = bicopter, 1 = spinning(TODO), -1 = None;
*/
init_flags_t init_flags = {
  .verbose = false,
  .sensors = true,
  .escarm = true,
  .UDP = true,
  .Ibus = true,
  .mode = 0,
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
  .rateGamma = .95f,
  .zGamma = 0,
};

//storage variables
sensors_t sensors;
controller_t controls;
//rawInput_t rawInputs;TODO
actuation_t outputs;


//Constants for the custom functions
float kpz = 0.4f;
float kdz = 0;
float kptz = .7;
float kdtz = 0.1;
float lx = .15;
// float kptx = .01;
// float kdtx = .01;


void setup() {
  if (blimpcommand == "Automatic"){
    //initializes all values to default
    blimp.initDefault(); //contains an example for the initialization
  } else if (blimpcommand == "Custom"){
    
    //initializes systems based on flags and saves flags into the system
    blimp.init(init_flags, init_sensors);

    //initializes magnetometer with some calibration values
    // these values have an automatic init inside, but it is better to make your own
    float transformationMatrix[3][3] = {
      {     1.0000f,  -32.2488f,   -0.4705f},
    {-30.6786f,   -0.2169f,   -5.6020f},
      {-1.1802f,    0.0597f,   35.5136f}
    };
    float offsets[3] = {20.45f, 64.11f, -67.0f};
    blimp.magnetometerCalibration(offsets, transformationMatrix);
  }

}



void loop() {
  /*
  // this code is for using the internal automatic structure based on your parameters
  // meant for almost no customization for things that have already been completed
  //        Ask Eddie to add any new modules to the internal structure, or follow the notes in modblimp .h and .cpp and README files
  */
  if (blimpcommand == "Automatic"){
    blimp.defaultControl(); //contains an example for the entire stack of control
  }
  //this code is for getting your own ability to customize the inputs and outputs of your program
  else if (blimpcommand == "Custom"){

    /*  
    //    attempts to get the lastest information about the SENSORS and places them into the 
    //    modSensor_t data structure
    //    will return 0 for all sensors if sensors == false
    */
    *sensors = blimp.getLatestSensorData();
    

    /*
    //    attempts to get the lastest information about the CONTROLLER and places them into the 
    //    controller_t data structure
    */
    *controls = blimp.getControllerData();


    /* TODO- NOT IMPLEMENTED
    //    optionally you can get the lastest information about the controller as raw values labeled as I1, I2, I3...
    */
    //rawInputs = blimp.getRawInputs();


    /*
    //    adds feedback into the controller terms from the sensors
    //    replace this with your own custom function for more customization
    */
    blimp.addFeedback(&controls, &sensors);
    //addFeedback(&controls, &sensors); //this function is implemented here for you to customize
    

    /*
    //    uses the mode to determine the control scheme for the motor/servo outputs
    //    currently only implementation is for the bicopter blimp
    //    replace this with your own custom function for more customization
    */
    *outputs = blimp.getOutputs(&controls);
    //*outputs = getOutputs(&controls); //this function is implemented here for you to customize


    /*
    //    uses the mode to determine the ouput scheme for the motor/servo outputs
    //    currently only implementation is for the bicopter blimp
    //    outputs should be floats between 0 and 1
    */
    blimp.exectuteOutputs(&outputs);

  }
}

/*
  -----------------------------------------------------------------------------------------------------
  EXAMPLE FUNCTIONS for full customization on outputs
  If you want to add a new sensor, you can try to go into firmware (crazyflieComplementary.cpp)
      or just implement it in this program
  -----------------------------------------------------------------------------------------------------
*/

/*
//adds sensor feedback into the control values
void addFeedback(controller_t *controls, sensor_t sensors) {
    //z feedback 
    controls->fz = (controls->fz  - (sensors->estimatedZ-sensors->groundZ))*kpz 
                    - (sensors->velocityZ)*kdz + controls->abz;
    
    //yaw feedback
    controls->tz = controls->tz * kptz - sensors->yawrateave*kdtz;
    
    //roll feedback
    //*tx = *tx - roll* kptx - rollrate *kdtx;

    //roll and pitch state feedback
    float cosp = (float) cos(sensors->pitch);
    float sinp = (float) sin(sensors->pitch);
    float cosr = (float) cos(sensors->roll);
    float ifx = controls->fx;
    controls->fx = ifx*cosp + controls->fz*sinp;
    controls->fz = (ifx*sinp + controls->fz* cosp)/cosr;
}


//creates the output values used for actuation from the control values
actuation_t getOutputs(controller_t *controls){
    //set up output
    actuation_t out;

    //set output to default if controls not ready
    if (controls->ready == false){
      out.s1 = .5f;
      out.s2 = .5f;
      out.m1 = 0f;
      out.m2 = 0f;
      return out;
    }

    //inputs to the A-Matrix
    float l = lx; //.3
    float fx = clamp(controls->fx, -1 , 1);//setpoint->bicopter.fx;
    float fz = clamp(controls->fz, 0 , 2);//setpoint->bicopter.fz;
    float taux = clamp(controls->tx, -l + (float)0.01 , l - (float) 0.01);
    float tauz = clamp(controls->tz, -.3 , .3);// limit should be .25 setpoint->bicopter.tauz; //- stateAttitudeRateYaw


    //inverse A-Matrix calculations
    float term1 = l*l*fx*fx + l*l*fz*fz + taux*taux + tauz*tauz;
    float term2 = 2*fz*l*taux - 2*fx*l*tauz;
    float term3 = sqrt(term1+term2);
    float term4 = sqrt(term1-term2);
    float f1 = term3/(2*l); // in unknown units
    float f2 = term4/(2*l);
    float t1 = atan2((fz*l - taux)/term3, (fx*l + tauz)/term3 );// in radians
    float t2 = atan2((fz*l + taux)/term4, (fx*l - tauz)/term4 );

    //checking for full rotations
    while (t1 < 0) {
      t1 = t1 + 2 * PI;
    }
    while (t1 > 2*PI) {
      t1 = t1 - 2 * PI;
    }
    while (t2 < 0) {
      t2 = t2 + 2 * PI;
    }
    while (t2 > 2*PI) {
      t2 = t2 - 2 * PI;
    }

    //converting values to a more stable form
    out.s1 = clamp(t1, 0, PI)/(PI);// cant handle values between PI and 2PI
    out.s2 = clamp(t2, 0, PI)/(PI);
    out.m1 = clamp(f1, 0, 1);
    out.m2 = clamp(f2, 0, 1);
    if (out.m1 < 0.02f ){
      out.s1 = 0.5f; 
    }
    if (out.m2 < 0.02f ){
      out.s2 = 0.5f; 
    }
    return out;
}

//helper function to keep values within a certain range
float clamp(float in, float min, float max){
  if (in< min){
    return min;
  } else if (in > max){
    return max;
  } else {
    return in;
  }
}

*/
