#include "baro390.h"

baro390::baro390(){
    
    //pass
    
}
void baro390::init(){
    /* Initialise the sensor */
    int countTries = 0;
    baroOn = bme.begin_I2C();
    while (!baroOn) {
        delay(100);
        if (countTries > 10) {
            Serial.println(F("Could not find a valid BMP390 sensor, check wiring or "
                            "try a different address!"));
                            
            break;
        }
        countTries += 1;
        baroOn = bme.begin_I2C();
    }

    bme.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bme.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bme.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bme.setOutputDataRate(BMP3_ODR_100_HZ);
    delay(500);
    if (baroOn){
        estimatedZ = bme.readAltitude(1013.25);
        oldZ = estimatedZ;
    } else {
        estimatedZ = 0;
        oldZ = 0;
    }
    
    
}

float baro390::getEstimatedZ(){
    if (baroOn){
        if ((micros() - tStart) > (BMP390_SAMPLERATE_DELAY_MS * 1000)) {
            oldZ = estimatedZ;
            estimatedZ = bme.readAltitude(1013.25);
            tStart = micros();
        }
    }
    return estimatedZ;


}
float baro390::getVelocityZ(){
    return estimatedZ - oldZ;
}