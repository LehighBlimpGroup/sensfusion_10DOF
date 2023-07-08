#ifndef CRAZYFLIE_H
#define CRAZYFLIE_H
#include <crazyflieComplementary.h>
#include <data_types.h>
#include <math.h>
#include "WiFi.h"
#include "AsyncUDP.h"
#include <IBusBM.h>
#include <ESP32Servo.h>



class ModBlimp;
class ModBlimp {
    private:
        float M_PI_F;
        float Kacc, Kgyro, Kmag;
        bool baro;
        float eulerGamma, rateGamma, zGamma;

        bool verbose, sensors, UDP, Ibus;
        int mode, control;
        feedback_t *PDterms;
        init_sensors_t  *sensors;
        init_flags_t *init_flags;

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
        sensor_t getLatestSensorData();
        controller_t getControllerData();
        //TODO rawInput_t getRawInputs();
        void addFeedback(controller_t *controls, sensor_t *sensors);
        actuation_t getOutputs(controller_t *controls);
        void executeOutputs(actuation_t *outputs);
        float clamp(float in, float min, float max);


};

#endif