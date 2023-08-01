#include <crazyflieComplementary.h>

SensFusion::SensFusion(){
  M_PI_F = 3.14159f;
  G = 9.81f;
  //float LSB = 0.48828125f;
  sensorflag = 2;

  //Parameters

  // float TWO_KP_DEF = (2.0f * 0.4f); // 2 * proportional gain
  //float twoKp = 2.0f;//TWO_KP_DEF;    // 2 * proportional gain (Kp)
  Kpacc = 20;//4;//strength of the accelerometer on Q
  Kpgyro = -1;//-1;//strength of the gyroscope on Q
  Kpmag = 1;//1//strength of the magnetometer on Q
  TWO_KI_DEF = (2.0f * 0.001f); // 2 * integral gain
  twoKi = TWO_KI_DEF;    // 2 * integral gain (Ki)

  baroOn = false;

  vAccDeadband = 0.02f;
  velZAlpha = 0.95f;//0.995f;

  estAlphaAsl = 4.0f; //strength of the barometer added to the estimatedZ
  velocityFactor = 0.0f;//1.0f; //strength of the accelerometer speed added to the estimatedZ
  aslGamma = 0.9f; //averaging factor to smooth after z calculations
  rate = 200;
  barorate = 50;

  //helper variables
  qw = 1;
  qx = 0;
  qy = 0;
  qz = 0;
  integralFBx = 0.0f;
  integralFBy = 0.0f;
  integralFBz = 0.0f;  // integral error terms scaled by Ki
  

  //SimpleFusion fuser;               // Initialize the SimpleFusion object (currently only using for the ability to modulate rate)
  //Adafruit_BMP085 bmp;

  
  

  isCalibrated = false;
  
  gammaMag = 0;//.95;

  // Calibration variables //-1.53,14.27,-14.04
  bx = 1;//cos(magneticInclincation* M_PI_f /180.0f);
  bz = 0;//sin(magneticInclincation* M_PI_f /180.0f);
  magInc = -11.0f;//-70.0f; //magnetic inclination of bethlehem


  //outputs
  roll = 0;
  pitch = 0;
  yaw = 0;
  velocityZ = 0;
  velocityZbaro = 0;
  velocityZbaroAve = 0;
  estimatedZold = 0;
  accZ;
  estimatedZ = 0.0f;
  estimatedZave = 0.0f;


  min = 0;
  max = 0;
  // float transformationMatrixBackup[3][3] = {
  //   {1.0f, 9.693f, 0.6187f},
  //   {9.6624f, -0.6822f, 0.3864f},
  //   {-0.4155f, 0.6628f, -10.7386f}
  // };
  // float offsetsBackup[3] = {11.98f, 7.01f, 21.77f};
  // saveTransform(offsetsBackup, transformationMatrixBackup);
  enterTransform();
}



void SensFusion::updateSensors(){

    if (mySensor.accelUpdate() == 0) {
      accx = mySensor.accelX();
      accy = mySensor.accelY();
      accz = mySensor.accelZ();
      
    }else{
    mySensor.beginAccel();
    }

    if (mySensor.gyroUpdate() == 0) {
      gyrox = mySensor.gyroX();
      gyroy = mySensor.gyroY();
      gyroz = mySensor.gyroZ();
    }else{
    mySensor.beginGyro();
    }

    if (mySensor.magUpdate() == 0) {
      magx = mySensor.magX();
      magy = mySensor.magY();
      magz = mySensor.magZ();
      transform(&magx, &magy, &magz);
      
    } else{
    mySensor.beginMag();
    }
    

    time_t newtime = micros();
    int barotimer = newtime - barotime; 
    if (barotimer > 1/barorate * 1000000) {
      if (baroOn) {
        float newHeight = bme.readAltitude();
        if (newHeight > 1000 or newHeight < 1){
          baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
        } else {
          baroHeight = newHeight;
        }
      } else {
        baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);

      }
      barotime = newtime;
      //baroHeightave = baroHeightave*.95 + baroHeight*.05;

    }
    

  
}

void SensFusion::transform(float *x, float *y, float *z) {
  // Transformation matrix
  // float transformationMatrix[3][3] = {
  //   {1.0f, 0.0464f, -0.0211f},
  //   {-.0337f, -0.9517f, 0.0072f},
  //   {0.0129f, -0.0436f, -1.0101f}
  // };
  //float offsets[3] = {-1.33f, 15.40f, -2.11f};
  // float transformationMatrix[3][3] = {
  //   {1.0f, -20.5657f, 0.4194f},
  //   {-21.2752f, -0.5661f, -1.259f},
  //   {-0.6610f, 0.8231f, 22.5804f}
  // };
  // float transformationMatrix[3][3] = {
  //   {1.0f, -0.0021f, 0.0155f},
  //   {-0.0021f, 0.9808f, 0.0208f},
  //   {0.0155f, 0.0208f, 1.0625f}
  // };
  //float offsets[3] = {20.37f, 17.72f, -80.59f};


  // Input array
  float inputArray[3] = {*x , *y, *z};

  for (int i = 0; i < 3; i++) {
    inputArray[i] += -1.0f* offsets[i];
  }

  // Output array
  float outputArray[3] = {0,0,0};

  // Apply transformation
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      outputArray[i] += transformationMatrix[i][j] * inputArray[j];
    }
  }
  *x = outputArray[0];
  *y = outputArray[1];
  *z = outputArray[2];

}

void SensFusion::sensfusionAll(float dt){
  sensfusion6UpdateQ(gyrox, gyroy, gyroz,
                      accx, accy, accz,
                      magx, magy, magz,
                      dt);
  // Save attitude, adjusted for the legacy CF2 body coordinate system
  sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
  positionZupdate(dt);
  estimatedZave = estimatedZave * aslGamma + (1- aslGamma) * estimatedZ;
}
void SensFusion::printSerial(int flag){

  if (flag == 1){
    Serial.print(-180);
    Serial.print(",");
    Serial.print(180);
    Serial.print(",");
    Serial.print(gyrox);
    Serial.print(",");
    Serial.print(gyroy);
    Serial.print(",");
    Serial.println(gyroz);
  } else if (flag == 2) {
    minmax(accx);
    minmax(accy);
    minmax(accz);
    Serial.print(min);
    Serial.print(",");
    Serial.print(max);
    Serial.print(",");
    Serial.print(accx);
    Serial.print(",");
    Serial.print(accy);
    Serial.print(",");
    Serial.println(accz);
  } else if (flag == 3) {
    minmax(magx);
    minmax(magy);
    minmax(magz);
    Serial.print(min);
    Serial.print(",");
    Serial.print(max);
    Serial.print(",");
    Serial.print(magx);
    Serial.print(",");
    Serial.print(magy);
    Serial.print(",");
    Serial.println(magz);
  } else if (flag == 4) {
    Serial.print(-180);
    Serial.print(",");
    Serial.print(180);
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    // Serial.print(magxave);
    // Serial.print(",");
    // Serial.print(magyave);
    // Serial.print(",");
    // Serial.print(magzave);
    // Serial.print(",");
    Serial.println(yaw);
  } else if (flag == 5) {
    Serial.print(-2);
    Serial.print(",");
    Serial.print(2);
    Serial.print(",");
    Serial.print(baroHeight - baroGround);
    Serial.print(",");
    Serial.print(estimatedZ - baroGround);
    Serial.print(",");
    Serial.print(velocityZ);
    Serial.print(",");
    Serial.println(accZ);
  } else if (flag == 6) {
    Serial.print(-2);
    Serial.print(",");
    Serial.print(2);
    Serial.print(",");
    Serial.print(qw);
    Serial.print(",");
    Serial.print(qx);
    Serial.print(",");
    Serial.print(qy);
    Serial.print(",");
    Serial.println(qz);
  }
}

void SensFusion::minmax(float val) {
  if (val > max){
    max = val;
  } else if (val < min) {
    min = val;
  }
}

float SensFusion::degreesRotate(float d) {
  while (d<= -180){
    d += 360;
  } 
  while (d > 180) {
    d += -360;
  }
  return d;
}

float SensFusion::invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

// Madgwick's implementation of Mahony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
void SensFusion::sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  
  //converts degrees to radians
  gx = Kpgyro * gx * M_PI_F / 180;
  gy = Kpgyro * gy * M_PI_F / 180;
  gz = Kpgyro * gz * M_PI_F / 180;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = qx * qz - qw * qy;
    halfvy = qw * qx + qy * qz;
    halfvz = qw * qw - 0.5f + qz * qz;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += Kpacc * halfex;
    gy += Kpacc * halfey;
    gz += Kpacc * halfez;
    // Serial.print(gx);
    // Serial.print(",");
    // Serial.print(gy);
    // Serial.print(",");
    // Serial.println(gz);
  }
  

  // Compute feedback only if magnetometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)))
  {
    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Estimated direction of north and vector parallel to magnetic flux
    halfvx = 2*bx*(-qy * qy + 0.5f - qz * qz) + 2*bz*(qx * qz - qw * qy) ;
    halfvy = 2*bx*(qx * qy - qw * qz) + 2*bz*(qw * qx + qy * qz);
    halfvz = 2*bx*(qx * qz + qw * qy) + 2*bz*(0.5f- qx * qx - qy * qy);

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (my * halfvz - mz * halfvy);//* 180/M_PI_F;
    halfey = (mz * halfvx - mx * halfvz);//* 180/M_PI_F;
    halfez = (mx * halfvy - my * halfvx);//* 180/M_PI_F;
    // Serial.print(-180);
    // Serial.print(",");
    // Serial.print(180);
    // Serial.print(",");
    // Serial.print(halfex );
    // Serial.print(",");
    // Serial.print(halfey);
    // Serial.print(",");
    // Serial.println(halfez);
    gx += Kpmag * halfex;
    gy += Kpmag * halfey;
    gz += Kpmag * halfez;

  }
  

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);   // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = qw;
  qb = qx;
  qc = qy;
  qw += (-qb * gx - qc * gy - qz * gz);
  qx += (qa * gx + qc * gz - qz * gy);
  qy += (qa * gy - qb * gz + qz * gx);
  qz += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  qw *= recipNorm;
  qx *= recipNorm;
  qy *= recipNorm;
  qz *= recipNorm;


}



void SensFusion::sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  float gx = gravX;
  float gy = gravY;
  float gz = gravZ;

  if (gx>1) gx=1;
  if (gx<-1) gx=-1;

  *yaw = atan2f(2*(qw*qz + qx*qy), qw*qw + qx*qx - qy*qy - qz*qz) * 180 / M_PI_F;
  *pitch = asinf(gx) * 180 / M_PI_F; //Pitch seems to be inverted
  *roll = atan2f(gy, gz) * 180 / M_PI_F;
}

float SensFusion::sensfusion6GetAccZ(const float ax, const float ay, const float az)
{
  // return vertical acceleration
  // (A dot G) / |G|,  (|G| = 1) -> (A dot G)
  return (ax * gravX + ay * gravY + az * gravZ);
}

float SensFusion::sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az)
{
  return sensfusion6GetAccZ(ax, ay, az) - abs(baseZacc);
}

void SensFusion::estimatedGravityDirection()
{
  gravX = 2 * (qx * qz - qw * qy);
  gravY = 2 * (qw * qx + qy * qz);
  gravZ = qw * qw - qx * qx - qy * qy + qz * qz;
}

void SensFusion::sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,  float dt)
{
  sensfusion6UpdateQImpl(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
  estimatedGravityDirection();

  if (!isCalibrated) {
    baseZacc = sensfusion6GetAccZ(ax, ay, az);
    isCalibrated = true;
  }
}


void SensFusion::positionUpdateVelocityInternal(float accWZ, float dt) {
  velocityZ += deadband(accWZ, vAccDeadband) * dt * G;
  velocityZ *= velZAlpha;
}

//taken from num.c
float SensFusion::deadband(float value, const float threshold)
{
  if (fabsf(value) < threshold)
  {
    value = 0;
  }
  else if (value > 0)
  {
    value -= threshold;
  }
  else if (value < 0)
  {
    value += threshold;
  }
  return value;
}

void SensFusion::positionZupdate(float dt) {
  accZ = sensfusion6GetAccZWithoutGravity(accx, accy, accz);
  positionUpdateVelocityInternal(accZ, dt);
  if (estimatedZ == 0.0f) {
    
      estimatedZ = baroHeight;
  } 
    estimatedZ += (velocityFactor * velocityZ + estAlphaAsl * (baroHeight-estimatedZ)) * dt;
    
    velocityZbaro = estimatedZ - estimatedZold;
    
    estimatedZold = estimatedZ;
}

void SensFusion::updateKp(float kpa, float kpg, float kpm){
  Kpacc = kpa;
  Kpgyro = kpg;
  Kpmag = kpm;
}

void SensFusion::initSensors(){
  oldtime  = micros();
  barotime = micros();
  //magneticInclincation = -11.0f;
  magInc = -20;
  bx = cos(magInc * M_PI_F /180.0f);
  bz = sin(magInc * M_PI_F /180.0f);
  // #ifdef _ESP32_HAL_I2C_H_ // For ESP32
  // Wire.begin(4, 5);//da, cl
  // mySensor.setWire(&Wire);
  // #else
  //   Wire.begin();
  //   mySensor.setWire(&Wire);
  // #endif
  sensorflag = 2;
  // if (!bme.begin()){

  //   // if (!IMU.begin() || !bmp.begin()) {
  //   //   Serial.println("Failed to initialize IMU!");
  //   // Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  //   sensorflag = 0;

  //   // } else {
  //   //   sensorflag = 1;
  //   // }
  // } else {
  //   sensorflag = 2;
  // }

  if (sensorflag == 1) {
    sensorflag = 1;
    // baroGround = bmp.readAltitude();
    // estimatedZ = baroGround;
    // rate = IMU.accelerationSampleRate();
  } else if (sensorflag == 2) {
    //unsigned status = ;
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
    
    
    if (baroOn){
      baroGround = bme.readAltitude();
      estimatedZ = baroGround;
    } else {
      baroGround = 0;
      estimatedZ = 0;
    }
    rate = 144;//144
    mySensor.beginAccel();
    mySensor.beginGyro();
    mySensor.beginMag();
  } else {
      Serial.println("Failed to initialize any IMU!");
      while(1);
  }

}
void SensFusion::sensfusionLoop(bool verbose, int flag) {
  time_t newtime = micros();
  int delta = newtime - oldtime;
  float dt = float(delta)/1000000.0f;
  updateSensors();// updates gyro, accelerometer, magnometer, and barometer
  
  if ( delta > 1/rate * 1000000) {
    
    oldtime = newtime;

    sensfusionAll(dt);
    if (verbose) {
      printSerial(flag);
    }
  } 
}
void SensFusion::enterTransform(){
  char* name = new char[3];
  char str[256];
  preferences.begin("calibration", true); 
  name[2] = (char)'\0';
  
  name[0] = (char)'m';
  itoa((int)(0),str,16);
  name[1] = str[0];
  if ((float)preferences.getFloat(name) == NULL){
    preferences.end();
    float transformationMatrixBackup[3][3] = {
      {1.0f, 0.0f, 0.0f},
      {0.0f, 1.0f, 0.0f},
      {0.0f, 0.0f, 1.0f}
    };
    float offsetsBackup[3] = {0.0f, 0.0f, 0.0f};
    saveTransform(offsetsBackup, transformationMatrixBackup);

    preferences.begin("calibration", true); 

  }
  for (int i = 0; i < 3; i ++) {
    for (int j = 0; j < 3; j ++) {
      itoa((int)(i*3 + j),str,16);
      name[1] = str[0];
      transformationMatrix[i][j] = (float)preferences.getFloat(name);// place UDP recieve here to get the values?
      Serial.print(name);
      Serial.print(": ");
      Serial.println((float)preferences.getFloat(name));
    }
  }
  name[0] = (char)'o';
  for (int i = 0; i < 3; i ++) {
    itoa(i,str,16);
    name[1] = str[0];
    offsets[i] = (float)preferences.getFloat(name);// place UDP recieve here to get the values?
    Serial.print(name);
    Serial.print(": ");
    Serial.println((float)preferences.getFloat(name));
  }
  preferences.end();
}

void SensFusion::saveTransform(float (&offset)[3], float (&matrix)[3][3]){
  char* name = new char[3];
  char str[256];
  preferences.begin("calibration", false); 
  preferences.clear();
  name[2] = (char)'\0';
  
  name[0] = (char)'m';
  for (int i = 0; i < 3; i ++) {
    for (int j = 0; j < 3; j ++) {
      itoa((int)(i*3 + j),str,16);
      name[1] = str[0];
      preferences.putFloat(name, (float_t)(matrix[i][j]));// place UDP recieve here to get the values?
      Serial.print(name);
      Serial.print(": ");
      Serial.println((float)matrix[i][j]);
    }
  }
  name[0] = (char)'o';
  for (int i = 0; i < 3; i ++) {
    itoa(i,str,16);
    name[1] = str[0];
    preferences.putFloat(name, (float_t)(offset[i]));// place UDP recieve here to get the values?
  }
  preferences.end();
}
//allows you to record data with putty 
void SensFusion::recordData() {
  rate = 8;

  int counter = 10000;
  while (counter > 0){
    time_t newtime = micros();
    int delta = newtime - oldtime;
    

    
    

    if ( delta > 1/rate * 1000000 && mySensor.magUpdate() == 0 && mySensor.accelUpdate() == 0){
      
      auto result1 = mySensor.magUpdate();
      if (result1 != 0) {
        mySensor.beginMag();
        result1 = mySensor.magUpdate();
      }
      magx = mySensor.magX();
      magy = mySensor.magY();
      magz = mySensor.magZ();

      auto result2 = mySensor.accelUpdate();
      if (result2 != 0) {
        mySensor.beginAccel();
        result2 = mySensor.accelUpdate();
      }
      accx = mySensor.accelX();
      accy = mySensor.accelY();
      accz = mySensor.accelZ();
      counter += -1;
      float dt = float(delta)/1000000.0f;
      oldtime = newtime;

      //     // calibration data
      float recipNorm = invSqrt(accx * accx + accy * accy + accz * accz);
      accx *= recipNorm;
      accy *= recipNorm;
      accz *= recipNorm;
      //Serial.print("[");
      Serial.print(magx);
      Serial.print(",");
      Serial.print(magy);
      Serial.print(",");
      Serial.print(magz);
      Serial.print(",");
      Serial.print(accx);
      Serial.print(",");
      Serial.print(accy);
      Serial.print(",");
      Serial.println(accz);
      //Serial.println("],");
    }
    
    
  }
  
  Serial.println("Data written to the file successfully.");
  while(1);

}

//allows you to record data with putty 
void SensFusion::prepCalibrationData(float sensor_data[6]) {

  auto result1 = mySensor.magUpdate();
  while (result1 != 0) {
    mySensor.beginMag();
    result1 = mySensor.magUpdate();
    delay(5);
  }
  auto result2 = mySensor.accelUpdate();
  while (result2 != 0) {
    mySensor.beginAccel();
    result2 = mySensor.accelUpdate();
    delay(5);
  }
  
  magx = mySensor.magX();
  magy = mySensor.magY();
  magz = mySensor.magZ();
  accx = mySensor.accelX();
  accy = mySensor.accelY();
  accz = mySensor.accelZ();
  

  //     // calibration data
  float recipNorm = invSqrt(accx * accx + accy * accy + accz * accz);
  accx *= recipNorm;
  accy *= recipNorm;
  accz *= recipNorm;
  
  sensor_data[0] = magx;
  sensor_data[1] = magy;
  sensor_data[2] = magz;
  sensor_data[3] = accx;
  sensor_data[4] = accy;
  sensor_data[5] = accz;

  
}

void SensFusion::saveCalibration(float input_data[13]) {

  float transformationMatrixBackup[3][3] = {
    {input_data[1],input_data[2],input_data[3]},
    {input_data[4],input_data[5],input_data[6]},
    {input_data[7],input_data[8],input_data[9]}
  };
  float offsetsBackup[3] = {input_data[10],input_data[11],input_data[12]};
  saveTransform(offsetsBackup, transformationMatrixBackup);
  enterTransform();
}

float SensFusion::getRoll(){
  return roll * M_PI_F/180.0f;
}  
float SensFusion::getPitch(){
  return pitch * M_PI_F/180.0f;
}
float SensFusion::getYaw(){
  return yaw * M_PI_F/180.0f;
}
float SensFusion::getRollRate(){
  return gyrox * M_PI_F/180.0f;
}  
float SensFusion::getPitchRate(){
  return gyroy * M_PI_F/180.0f;
}
float SensFusion::getYawRate(){
  return gyroz * M_PI_F/180.0f;
}

float SensFusion::getMagx(){
  return magx;
}  
float SensFusion::getMagy(){
  return magy;
}
float SensFusion::getMagz(){
  return magz;
}
float SensFusion::returnZ(){
  return estimatedZ;
}
float SensFusion::returnVZ(){
  return velocityZbaro;
}

