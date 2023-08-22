

#include "WiFi.h"
#include "AsyncUDP.h"
#include <data_types.h>
void unpack_joystick(float *dat, const unsigned char *buffer);
    


class UDPCom {
    private:
        const char * ssid = "AIRLab-BigLab";
        const char * password = "Airlabrocks2022";
        int UDPport = 1600; //TODO check if sending feedback uses a different UDP port
        int delayMS = 1000;
        AsyncUDP udp;
        bool active = false;
    public:
        UDPCom();
        void init(int port);
        void send_udp_feedback(String dat1, String dat2, String dat3, String dat4);
        void getControllerInputs(controller_t *controls);
        void getControllerRaws(raw_t *raws);
        void getCalibrationInputs(float input_data[13]);
        void send_mag_acc(float calibration_data[6]); //const unsigned char *buffer
        void sendAck(); //const unsigned char *buffer
};