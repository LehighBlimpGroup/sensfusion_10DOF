#ifndef CRAZYFLIE_H
#define CRAZYFLIE_H
#include <crazyflieComplementary.h>
#include <data_types.h>


typedef struct init_sensors_s {
    float Kacc, Kgyro, Kmag;
    bool baro;
    float eulerGamma, rateGamma, zGamma;
} init_sensors_t;

typedef struct init_flags_s {
    
    bool verbose, sensors, escarm, UDP, Ibus;
    int mode, control;
} init_flags_t;

class ModBlimp;
class ModBlimp {
    private:
        float Kacc, Kgyro, Kmag;
        bool baro;
        float eulerGamma, rateGamma, zGamma;

        bool verbose, sensors, UDP, Ibus;
        int mode, control;

        void initMotors(); //runs the escarm and initializes motors and servos
        void initSensors(init_sensors_t  *sensors); //attempts to connect to the sensors using sensorswuite.

    public:
        ModBlimp(); //constructor

        //initialization functions
        void initDefault(); //contains an example of how to initialize the system
        void init(init_flags_t *init_flags, init_sensors_t  *sensors);//sets up the control flags in the system
        void magnetometerCalibration(float (&offset)[3], float (&matrix)[3][3]);

        //loop functions
        void defaultControl(); //contains an example of the entire control stack
        sensor_t getLatestSensorData();
        controller_t getControllerData();
        //TODO rawInput_t getRawInputs();
        void addFeedback(controller_t *controls, sensor_t *sensors);
        void getOutputs(controller_t *controls);
        void executeOutputs(actuation_t *outputs);


};

#endif