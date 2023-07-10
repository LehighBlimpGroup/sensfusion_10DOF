#ifndef CRAZYFLIE_H
#define CRAZYFLIE_H
#include <crazyflieComplementary.h>
#include <data_types.h>
#include <udpCom.h>
#include <math.h>
#include <IBusBM.h>
#include <ESP32Servo.h>


#define SERVO1 D2
#define SERVO2 D3
#define THRUST1 D0
#define THRUST2 D1


class ModBlimp;
class ModBlimp {
    private:

        IBusBM IBus; 
        HardwareSerial MySerial0(0);
        const char * ssid = "AIRLab-BigLab";
        const char * password = "Airlabrocks2022";
        float M_PI_F;
        
        //flag holders
        init_sensors_t  *sensors;
        init_flags_t *init_flags;
        feedback_t *feedbackPD;
        
        SensFusion sensorSuite;

        void initMotors(); //runs the escarm and initializes motors and servos
        void initSensors(init_sensors_t  *sensors); //attempts to connect to the sensors using sensorswuite.

    public:
        ModBlimp(); //constructor

        //initialization functions
        void initDefault(); //contains an example of how to initialize the system
        void init(init_flags_t *init_flags, init_sensors_t  *sensors, feedback_t *PDterms);//sets up the control flags in the system
        void magnetometerCalibration(float (&offset)[3], float (&matrix)[3][3]);

        //loop functions
        void defaultControl(); //contains an example of the entire control stack
        sensor_t getLatestSensorData(); //gets the latest sensor data and returns it in the sensor_t datatype
        controller_t getControllerData(); //gets the latest controller data and returns it in the controller_t datatype
        //TODO rawInput_t getRawInputs();
        void addFeedback(controller_t *controls, sensor_t *sensors); //uses the sensor data to add feedback directly into controller_t
        actuation_t getOutputs(controller_t *controls); //converts control
        void executeOutputs(actuation_t *outputs);
        float clamp(float in, float min, float max);


};

#endif