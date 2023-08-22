#include <Adafruit_BMP280.h>
class baro280
{
private:
    unsigned long tStart = micros();
    bool baroOn = false;
    Adafruit_BMP280 bme;
    uint16_t BMP280_SAMPLERATE_DELAY_MS = 63; //how often to read data from the board
    float estimatedZ = 0;
    float oldZ = 0;
public:
    baro280();
    void init();
    float getEstimatedZ();
    float getVelocityZ();
};