#include "modBlimp.h"


ModBlimp::ModBlimp(){ //constructor


}

//initialization functions
void ModBlimp::initDefault() { //contains an example of how to initialize the system
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

    init(&init_flags, &init_sensors);

    //initializes magnetometer with some calibration values
    // these values have an automatic init inside, but it is better to make your own
    float transformationMatrix[3][3] = {
      {     1.0000f,  -32.2488f,   -0.4705f},
    {-30.6786f,   -0.2169f,   -5.6020f},
      {-1.1802f,    0.0597f,   35.5136f}
    };
    float offsets[3] = {20.45f, 64.11f, -67.0f};
    magnetometerCalibration(offsets, transformationMatrix);

}
void ModBlimp::init(init_flags_t *init_flags, init_sensors_t  *sensors){//sets up the control flags in the system
    //save flags
    //initialize serial
    //initialize motor + servos
    //initialize sensors
    //initialize UDP
    //initialize IBUS

}
void ModBlimp::magnetometerCalibration(float (&offset)[3], float (&matrix)[3][3]){

}

//loop functions
void ModBlimp::defaultControl(){ //contains an example of the entire control stack

}
sensor_t ModBlimp::getLatestSensorData(){

}
controller_t ModBlimp::getControllerData(){

}
//TODO rawInput_t getRawInputs();
void ModBlimp::addFeedback(controller_t *controls, sensor_t *sensors){

}
void ModBlimp::getOutputs(controller_t *controls){

}
void ModBlimp::executeOutputs(actuation_t *outputs){

}