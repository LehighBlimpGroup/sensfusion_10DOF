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


        float M_PI_F;
        Servo servo1;
        Servo servo2; 
        Servo thrust1;
        Servo thrust2;

        float avex = 0;
        float avez = 0; 
        
        //flag holders
        init_sensors_t  *init_sensors;
        init_flags_t *init_flags;
        feedback_t *PDterms;


        sensors_t sensorsEx;
        controller_t controlsEx;
        //rawInput_t rawInputs;TODO
        actuation_t outputsEx;

        float groundZ;
        
        UDPCom udpSuite;
        IBusBM IBus; 

        volatile unsigned long time_end;
        void escarm(Servo& thrust1, Servo& thrust2);
        void initMotors(); //runs the escarm and initializes motors and servos
        void initSensors(init_sensors_t  *sensors); //attempts to connect to the sensors using sensorswuite.

    public:
        SensFusion sensorSuite;
        ModBlimp(); //constructor

        //initialization functions
        void initDefault(); //contains an example of how to initialize the system
        void init(init_flags_t *init_flags, init_sensors_t  *sensors, feedback_t *PDterms);//sets up the control flags in the system
        void magnetometerCalibration(float (&offset)[3], float (&matrix)[3][3]);

        //loop functions
        void defaultControl(); //contains an example of the entire control stack
        void getLatestSensorData(sensors_t* sensors); //gets the latest sensor data and returns it in the sensor_t datatype
        void getControllerData(controller_t* controls); //gets the latest controller data and returns it in the controller_t datatype
        //TODO rawInput_t getRawInputs();
        void addFeedback(controller_t *controls, sensors_t *sensors); //uses the sensor data to add feedback directly into controller_t
        void getOutputs(controller_t *controls, actuation_t* outputs); //converts control
        void getOutputs(controller_t *controls, sensors_t *sensors, actuation_t* outputs); //converts control
        void executeOutputs(actuation_t *out);
        float clamp(float in, float min, float max);
        void send_udp_feedback(String dat1, String dat2, String dat3, String dat4);
        void calibrationMode(int flag);
};

#endif