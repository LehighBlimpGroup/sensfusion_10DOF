#include <Adafruit_BMP3XX.h>
class baro390
{
private:
    unsigned long tStart = micros();
    bool baroOn = false;
    Adafruit_BMP3XX bme;
    uint16_t BMP390_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
    float estimatedZ = 0;
    float oldZ = 0;
public:
    baro390();
    void init();
    float getEstimatedZ();
    float getVelocityZ();
};