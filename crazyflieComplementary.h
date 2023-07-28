
#define CRAZYFLIE_H
#include <Arduino.h>
//imports

//#include <Adafruit_BMP085.h>
#include <Adafruit_BMP280.h>

//#include <Arduino_LSM6DS3.h> //nano 33 IOT
//#include <Arduino_LSM9DS1.h> //nano 33 ble

#include <MPU9250_asukiaaa.h>

#include <TimeLib.h>
#include <math.h>
#include <Preferences.h>

// Class declaration and definition

class SensFusion;
class SensFusion {
    private:
        float M_PI_F;
        float G;
        //float LSB = 0.48828125f;

        //Parameters

        // float TWO_KP_DEF = (2.0f * 0.4f); // 2 * proportional gain
        //float twoKp = 2.0f;//TWO_KP_DEF;    // 2 * proportional gain (Kp)
        float Kpacc;//4;//strength of the accelerometer on Q
        float Kpgyro;//-3;//strength of the gyro on Q
        float Kpmag;//strength of the gyro on Q
        float TWO_KI_DEF; // 2 * integral gain
        float twoKi;    // 2 * integral gain (Ki)

        float vAccDeadband;
        float velZAlpha;//0.995f;

        float estAlphaAsl; //strength of the barometer added to the estimatedZ
        float velocityFactor ;//1.0f; //strength of the accelerometer speed added to the estimatedZ
        float aslGamma; //averaging factor to smooth after z calculations

        float rate ;
        float barorate;

        //helper variables
        float qw;
        float qx;
        float qy;
        float qz;
        float integralFBx;
        float integralFBy;
        float integralFBz;  // integral error terms scaled by Ki

        float gravX, gravY, gravZ; // Unit vector in the estimated gravity direction

        //SimpleFusion fuser;               // Initialize the SimpleFusion object (currently only using for the ability to modulate rate)
        //Adafruit_BMP085 bmp;

        Adafruit_BMP280 bme; // I2C
        MPU9250_asukiaaa mySensor;

        float gyrox, gyroy, gyroz, accx, accy, accz, magx, magy, magz;
        float baroHeight;
        float baroHeightave;
        float baroGround;
        float baseZacc;
        bool isCalibrated;
        float filteredZ;
        time_t oldtime;
        time_t barotime;
        float magxave, magyave, magzave;
        float gammaMag;//.95;

        // Calibration variables //-1.53,14.27,-14.04
        float bx;//cos(magneticInclincation* M_PI_f /180.0f);
        float bz;//sin(magneticInclincation* M_PI_f /180.0f);
        float magInc; //magnetic inclination of bethlehem

        Preferences preferences;
        float transformationMatrix[3][3];
        float offsets[3];


        bool baroOn;


        //outputs
        float roll;
        float pitch;
        float yaw;
        float velocityZ ;
        float velocityZbaro;
        float velocityZbaroAve;
        float estimatedZold;
        float accZ;
        float estimatedZ ;
        float estimatedZave;


        int sensorflag;
        float min;
        float max;
        void updateSensors();
        void transform(float* x, float* y, float* z);
        void sensfusionAll(float dt);
        void printSerial(int flag);
        void minmax(float val);
        float degreesRotate(float d);
        float invSqrt(float x);
        void sensfusion6UpdateQImpl(float gx, float gy, float gz, 
                                    float ax, float ay, float az, 
                                    float mx, float my, float mz, 
                                    float dt);
        void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw);
        float sensfusion6GetAccZ(const float ax, const float ay, const float az);
        float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az);
        void estimatedGravityDirection();
        void sensfusion6UpdateQ(float gx, float gy, float gz, 
                                float ax, float ay, float az, 
                                float mx, float my, float mz,  
                                float dt);
        void positionUpdateVelocityInternal(float accWZ, float dt);
        float deadband(float value, const float threshold);
        void positionZupdate(float dt);
    
    public:
        SensFusion();
        void initSensors();
        void updateKp(float kpa, float kpg, float kpm);
        void sensfusionLoop(bool verbose, int flag);
        void recordData();
        void enterTransform();
        void saveTransform(float (&offset)[3], float (&matrix)[3][3]);
        float getRoll();
        float getPitch();
        float getYaw();
        float getMagx();
        float getMagy();
        float getMagz();
        float getRollRate();
        float getPitchRate();
        float getYawRate();
        float returnZ();
        float returnVZ();
        void prepCalibrationData(float sensor_data[6]);
        void saveCalibration(float input_data[13]);
};

