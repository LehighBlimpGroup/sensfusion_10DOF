#ifndef CRAZYFLIE_H
#define CRAZYFLIE_H
#include <crazyflieComplementary.h>
#include <data_types.h>


typedef struct init_sensors_s {
    float acc, gyro, mag;
    bool baro;
    float eulerGamma, rateGamma, zGamma;
} init_sensors_t;

typedef struct init_flags_s {
    
    bool verbose, sensors, UDP;
    int mode, control;
} init_flags_t;

class ModBlimp;
class ModBlimp {
    private:
        float M_PI_F;
    public:
        ModBlimp();
        void initMotors();
        void initSensors(init_sensors_t init_sensors);
};

#endif