#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <data_types.h>

class BNO55
{
private:
    bool bnoOn = false;
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
    
public:
    BNO55();
    void init();
    void updateSensors(sensors_t *sensors, sensor_weights_t *weights);
};