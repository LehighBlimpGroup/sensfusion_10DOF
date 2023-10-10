#include "modBlimp.h"

ModBlimp::ModBlimp()
{ // constructor
}

// initialization functions
void ModBlimp::initDefault()
{ // contains an example of how to initialize the system
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
      .sensors = true,
      .escarm = true,
      .UDP = true,
      .Ibus = true,
      .motor_type = 0,
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
      .Kmag = 5,
      .baro = true,
      .eulerGamma = 0,
      .rateGamma = .95f,
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
      .Cyaw = 0,
      .Cx = 0,
      .Cy = 0,
      .Cz = 0,
      .Cabsz = 0,

      .kproll = 0,
      .kdroll = 0,
      .kppitch = 0,
      .kdpitch = 0,
      .kpyaw = 0.7f,
      .kdyaw = 0.1f,

      .kpx = 0,
      .kdx = 0,
      .kpy = 0,
      .kdy = 0,
      .kpz = 0.4f,
      .kdz = 0,

      .lx = .15,
  };

  init(&init_flags, &init_sensors, &feedbackPD);

  // initializes magnetometer with some calibration values
  //  these values have an automatic init inside, but it is better to make your own
  //  float transformationMatrix[3][3] = {
  //    {     1.0000f,  -32.2488f,   -0.4705f},
  //  {-30.6786f,   -0.2169f,   -5.6020f},
  //    {-1.1802f,    0.0597f,   35.5136f}
  //  };
  //  float offsets[3] = {20.45f, 64.11f, -67.0f};
  //  magnetometerCalibration(offsets, transformationMatrix);
}

void ModBlimp::changeServoPins(){
  
}

HardwareSerial MySerial0(0);

void ModBlimp::setFlags(init_flags_t *init_flagsIn, init_sensors_t *init_sensorsIn, feedback_t *feedbackPDIn){
  init_sensors = init_sensorsIn;
  init_flags = init_flagsIn;
  PDterms = feedbackPDIn;
}

void ModBlimp::init(init_flags_t *init_flagsIn, init_sensors_t *init_sensorsIn, feedback_t *feedbackPDIn)
{ // sets up the control flags in the system
  // save flags
  setFlags(init_flagsIn, init_sensorsIn, feedbackPDIn);
  time_end = millis();

  // initialize serial
  Serial.begin(115200);
  delay(1000);

  // initialize IBUS
  if (init_flags->Ibus)
  {
    Serial.println("Starting IBUS Init");
    MySerial0.begin(115200, SERIAL_8N1, -1, -1);
    IBus.begin(MySerial0, IBUSBM_NOTIMER);
    IBus.loop();
  }

  // initialize motor + servos
  Serial.println("Starting Motor Servo Init");
  pinMode(SERVO1, OUTPUT);
  pinMode(SERVO2, OUTPUT);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  if (init_flags->control == 0) {
  servo1.setPeriodHertz(50); // Standard 50hz servo
  servo2.setPeriodHertz(50); // Standard 50hz servo
  servo1.attach(SERVO1, 450, 2550);
  servo2.attach(SERVO2, 450, 2550);
  pinMode(THRUST1, OUTPUT);
  pinMode(THRUST2, OUTPUT);
  if (init_flags->motor_type == 0)
  {
    thrust1.attach(THRUST1, 1000, 2000);
    thrust2.attach(THRUST2, 1000, 2000);
    thrust1.setPeriodHertz(55);
    thrust2.setPeriodHertz(55);

    if (init_flags->escarm)
    {
      escarm(thrust1, thrust2);
    }
  }
  } else if (init_flags->control == 2) {
    servo1.attach(SERVO1, 1000, 2000);
    servo2.attach(SERVO2, 1000, 2000);
    servo1.setPeriodHertz(50); // Standard 50hz servo
    servo2.setPeriodHertz(50); // Standard 50hz servo
    thrust1.attach(THRUST1, 1000, 2000);
    thrust2.attach(THRUST2, 1000, 2000);
    thrust1.setPeriodHertz(55);
    thrust2.setPeriodHertz(55);

  }

  // initialize sensors
  if (init_flags->sensors)
  {
    Serial.println("Starting Sensor Init");
    sensorSuite.initSensors();
    sensorSuite.updateKp(init_sensors->Kacc, init_sensors->Kgyro, init_sensors->Kmag); // 5,-1,0.3
    groundZ = sensorSuite.returnZ();
    sensorSuite.enterTransform();
  }

  // initialize UDP
  if (init_flags->UDP)
  {
    Serial.println("Starting UDP Init");
    udpSuite.init(init_flags->PORT);
  }

  // initialize ESP-NOW
  if (init_flags->ESPNOW)
  {
    Serial.println("Starting ESP-NOW Init");
    espNow.init();
  }

  Serial.println("Finished Init");
}
void ModBlimp::magnetometerCalibration(float (&offset)[3], float (&matrix)[3][3])
{
  sensorSuite.saveTransform(offset, matrix);
  sensorSuite.enterTransform();
}

// loop functions
void ModBlimp::defaultControl()
{ // contains an example of the entire control stack

  /*
  //    attempts to get the lastest information about the SENSORS and places them into the
  //    sensor_t data structure
  //    contains: roll, pitch, yaw, rollrate, pitchrate, yawrate, estimatedZ, velocityZ, groundZ
  //    will return 0 for all sensors if sensors == false
  */
  getLatestSensorData(&sensorsEx);

  sensorsEx.pitch = -1 * sensorsEx.pitch; // hack to invert pitch due to orientation of the sensor

  /*
  //    attempts to get the lastest information about the CONTROLLER and places them into the
  //    controller_t data structure
  //    contrains: fx, fy, fz, absz, tx, ty, tz, ready
  */
  getControllerData(&controlsEx);

  /* TODO- NOT IMPLEMENTED
  //    optionally you can get the lastest information about the controller as raw values labeled as I1, I2, I3...
  */
  // rawInputs = blimp.getRawInputs();

  /*
  //    adds feedback directly into the controller terms using sensor feedback
  //    replace this with your own custom function for more customization
  //        example is placed below
  */
  addFeedback(&controlsEx, &sensorsEx);

  /*
  //    uses the mode to determine the control scheme for the motor/servo outputs
  //    currently only implementation is for the bicopter blimp
  //    replace this with your own custom function for more customization
  //    actuation_t data type contains: m1, m2, s1, s2 for each motor and servo
  //        example is placed below
  */
  getOutputs(&controlsEx, &outputsEx);

  /*
  //    uses the mode to determine the ouput scheme for the motor/servo outputs
  //    currently only implementation is for the bicopter blimp
  //    outputs should be floats between 0 and 1
  */
  executeOutputs(&outputsEx);
}
void ModBlimp::getLatestSensorData(sensors_t *sensors)
{
  // interface with Sensorsuite
  if (init_flags->sensors)
  {
    sensorSuite.sensfusionLoop(false, 4);

    sensors->roll = sensors->roll * init_sensors->eulerGamma + sensorSuite.getRoll() * (1 - init_sensors->eulerGamma);
    sensors->pitch = sensors->pitch * init_sensors->eulerGamma + sensorSuite.getPitch() * (1 - init_sensors->eulerGamma);
    sensors->yaw = sensors->yaw * init_sensors->eulerGamma + sensorSuite.getYaw() * (1 - init_sensors->eulerGamma);

    sensors->rollrate = sensors->rollrate * init_sensors->rateGamma + sensorSuite.getRollRate() * (1 - init_sensors->rateGamma);
    sensors->pitchrate = sensors->pitchrate * init_sensors->rateGamma + sensorSuite.getPitchRate() * (1 - init_sensors->rateGamma);
    sensors->yawrate = sensors->yawrate * init_sensors->rateGamma + sensorSuite.getYawRate() * (1 - init_sensors->rateGamma);

    if (init_sensors->baro)
    {

      sensors->estimatedZ = sensors->estimatedZ * init_sensors->zGamma + sensorSuite.returnZ() * (1 - init_sensors->zGamma);
      sensors->velocityZ = sensors->velocityZ * init_sensors->zGamma + sensorSuite.returnVZ() * (1 - init_sensors->zGamma);
    }
    else
    {
      sensors->estimatedZ = 0;
      sensors->velocityZ = 0;
    }
    sensors->groundZ = groundZ;
  }
  // recieve data and place it into the sensor_t data_type
  return;
}


void ModBlimp::getControllerRaws(raw_t *raws)
{
  // check for which flag for controller is being used
  int mode = init_flags->mode;
  // access IBUS or UDP to get data
  if (mode == 0 && init_flags->UDP)
  { // UDP
    udpSuite.getControllerRaws(raws);
  }
  else if (mode == 1 && init_flags->Ibus)
  { // TODO: IBUS
    IBus.loop();// ibus integers only two bytes (inclusive) [0, 65535] 2**16
    raws->flag = IBus.readChannel(0);
    raws->ready = IBus.readChannel(1);
    for (int x = 0; x < 11; x ++ ){
      raws->data[x] = IBus.readChannel(x + 2);
    }
  }
  else if (mode == 2 && init_flags->ESPNOW)
  {
    espNow.getControllerRaws(raws);
  }
  else
  { // control will be empty if no control input is given
    raws->flag = 0;
    raws->ready = false;
    for (int x = 0; x < 11; x ++ ){
      raws->data[x] = 0; 
    }
  }
}

void ModBlimp::getControllerData(controller_t *controls)
{
  // check for which flag for controller is being used
  int mode = init_flags->mode;
  // access IBUS or UDP to get data
  if (mode == 0 && init_flags->UDP)
  { // UDP
    udpSuite.getControllerInputs(controls);
    calibrationMode(controls->flag); // runs calibration mode if controls->flag != 0;
  }
  else if (mode == 1 && init_flags->Ibus)
  { // TODO: IBUS
    IBus.loop();
    controls->fx = ((float)IBus.readChannel(1) - (float)1500) / (float)500 +
                   ((float)IBus.readChannel(5) - (float)1500) / (float)500;
    controls->fy = 0;
    controls->fz = ((float)IBus.readChannel(2) - (float)1000) / (float)500;
    controls->absz = 0;
    controls->tx = 0; //((float)IBus.readChannel(3)-(float)1000)/(float)1000;
    controls->ty = 0;
    controls->tz = ((float)IBus.readChannel(0) - (float)1500) / (float)500;
    controls->ready = (bool)(IBus.readChannel(4) > 1500);
    controls->flag = 0;
    controls->snapshot = 0;
  }
  else if (mode == 2 && init_flags->ESPNOW)
  {
    espNow.getControllerInputs(controls);
  }
  else
  { // control will be empty if no control input is given
    controls->fx = 0;
    controls->fy = 0;
    controls->fz = 0;
    controls->absz = 0;
    controls->tx = 0;
    controls->ty = 0;
    controls->tz = 0;
    controls->ready = 0;
    controls->flag = 0;
    controls->snapshot = 0;
  }
}

void ModBlimp::calibrationMode(int flag)
{
  if (flag != 0)
  {
    Serial.println("I got to Calibrate!");
    float input_data[13] = {(float)flag, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float calibration_data[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool uncalibrated = true;
    while (flag != 0)
    {

      if (flag == 1)
      { // start sending data
        sensorSuite.prepCalibrationData(calibration_data);
        udpSuite.send_mag_acc(calibration_data);
      }
      else if (flag == 2)
      { // recieve calibration command
        if (uncalibrated == true)
        {
          sensorSuite.saveCalibration(input_data);
          uncalibrated = false;
        }
        udpSuite.sendAck();
      }
      delay(125);
      udpSuite.getCalibrationInputs(input_data);
      flag = input_data[0];
    }
    // restart system
    Serial.println("No longer Calibrating!");
  }
}

// TODO rawInput_t getRawInputs(){}

void ModBlimp::addFeedback(controller_t *controls, sensors_t *sensors)
{
  // controller weights
  controls->fx *= PDterms->Cx;
  controls->fy *= PDterms->Cy;
  controls->fz *= PDterms->Cz;
  controls->tx *= PDterms->Croll;
  controls->ty *= PDterms->Cpitch;
  controls->tz *= PDterms->Cyaw;
  controls->absz *= PDterms->Cabsz;

  // z feedback
  if (PDterms->z)
  {
    controls->fz = (controls->fz - (sensors->estimatedZ - sensors->groundZ)) * PDterms->kpz - (sensors->velocityZ) * PDterms->kdz + controls->absz;
  }

  // yaw feedback
  if (PDterms->yaw)
  {
    controls->tz = controls->tz - sensors->yawrate * PDterms->kdyaw;
  }

  // roll feedback
  if (PDterms->roll)
  {
    controls->tx = controls->tx - sensors->roll * PDterms->kproll - sensors->rollrate * PDterms->kdroll;
  }

  // roll and pitch rotation state feedback
  if (PDterms->rotation)
  {
    float cosp = (float)cos(sensors->pitch);
    float sinp = (float)sin(sensors->pitch);
    float cosr = (float)cos(sensors->roll);
    float ifx = controls->fx;
    controls->fx = ifx * cosp + controls->fz * sinp;
    controls->fz = (-1 * ifx * sinp + controls->fz * cosp) / cosr;
  }
}

void ModBlimp::getOutputs(controller_t *controls, actuation_t *out)
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
  avex = avex * 0.9 + controls->fx * .1;
  avez = avez * 0.9 + controls->fz * .1;
  float fx = clamp(avex, -1, 1);                          // setpoint->bicopter.fx;
  float fz = clamp(avez, 0.1, 2);                         // setpoint->bicopter.fz;
  float maxRadsYaw = .175;                                //.175;
  float magxz = max(fz * tan(maxRadsYaw), fx * l * 0.7f); // limits the yaw based on the magnitude of the force
  float taux = clamp(controls->tx, -l + (float)0.01, l - (float)0.01);
  float tauz = clamp(controls->tz, -1, 1) * magxz; // limit should be .25 setpoint->bicopter.tauz; //- stateAttitudeRateYaw

  // inverse A-Matrix calculations
  float term1 = l * l * fx * fx + l * l * fz * fz + taux * taux + tauz * tauz;
  float term2 = 2 * fz * l * taux - 2 * fx * l * tauz;
  float term3 = sqrt(term1 + term2);
  float term4 = sqrt(term1 - term2);
  float f1 = term3 / (2 * l); // in unknown units
  float f2 = term4 / (2 * l);
  float t1 = atan2((fz * l - taux) / term3, (fx * l + tauz) / term3); // in radians
  float t2 = atan2((fz * l + taux) / term4, (fx * l - tauz) / term4);

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
  out->s1 = clamp(t1, 0, PI) / (PI); // cant handle values between PI and 2PI
  out->s2 = clamp(t2, 0, PI) / (PI);
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
void ModBlimp::getOutputs(controller_t *controls, sensors_t *sensors, actuation_t *out)
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
  float maxRadsYaw = .07; //.1f                                 //.175;
  float magxz = max(fz * tan(maxRadsYaw), fx * l * 0.17f); // limits the yaw based on the magnitude of the force
  float taux = clamp(controls->tx, -l + (float)0.01, l - (float)0.01);
  float tauz = clamp(controls->tz, -1, 1) * magxz; // limit should be .25 setpoint->bicopter.tauz; //- stateAttitudeRateYaw

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
  out->s1 = clamp(t1, 0, PI) / (PI); // cant handle values between PI and 2PI
  out->s2 = clamp(t2, 0, PI) / (PI);
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

void ModBlimp::executeOutputs(actuation_t *outputs)
{
  if (init_flags->control == 0){
    servo1.write((int)(outputs->s1 * 180));
    servo2.write((int)((1 - outputs->s2) * 180));
    if (init_flags->motor_type == 0)
    {
      if (outputs->ready)
      {
        thrust1.writeMicroseconds((int)((outputs->m1) * 900 + 1100));
        thrust2.writeMicroseconds((int)((outputs->m2) * 900 + 1100));
      }
      else
      {

        thrust1.writeMicroseconds((int)0);
        thrust2.writeMicroseconds((int)0);
      }
    }
    else if (init_flags->motor_type == 1)
    {
      if (outputs->ready)
      {
        analogWrite(THRUST1, (int)(0 + (outputs->m1) * 255));
        analogWrite(THRUST2, (int)(0 + (outputs->m2) * 255));
      }
      else
      {
        analogWrite(THRUST1, (int)0);
        analogWrite(THRUST2, (int)0);
      }
    }
    time_end = millis();
  }
  else if (init_flags->control == 2){
      if (outputs->ready)
      {
        analogWrite(THRUST1, (int)(0 + (outputs->m1) * 255));
        analogWrite(THRUST2, (int)(0 + (outputs->m2) * 255));
        analogWrite(SERVO1, (int)(0 + (outputs->s1) * 255));
        analogWrite(SERVO2, (int)(0 + (outputs->s2) * 255));
      }
      else
      {
        analogWrite(THRUST1, (int)0);
        analogWrite(THRUST2, (int)0);
        analogWrite(SERVO1, (int)0);
        analogWrite(SERVO2, (int)0);
      }
  }

}
void ModBlimp::send_udp_feedback(String dat1, String dat2, String dat3, String dat4)
{
  udpSuite.send_udp_feedback(dat1, dat2, dat3, dat4);
}
float ModBlimp::clamp(float in, float min, float max)
{
  if (in < min)
  {
    return min;
  }
  else if (in > max)
  {
    return max;
  }
  else
  {
    return in;
  }
}

// Enter arming sequence for ESC
void ModBlimp::escarm(Servo &thrust1, Servo &thrust2)
{
  // ESC arming sequence for BLHeli S
  thrust1.writeMicroseconds(1000);
  delay(10);
  thrust2.writeMicroseconds(1000);
  delay(10);

  // Sweep up
  for (int i = 1100; i < 1500; i++)
  {
    thrust1.writeMicroseconds(i);
    delay(5);
    thrust2.writeMicroseconds(i);
    delay(5);
  }
  // Sweep down
  for (int i = 1500; i > 1100; i--)
  {
    thrust1.writeMicroseconds(i);
    delay(5);
    thrust2.writeMicroseconds(i);
    delay(5);
  }
  // Back to minimum value
  thrust1.writeMicroseconds(1000);
  delay(10);
  thrust2.writeMicroseconds(1000);
  delay(10);
}