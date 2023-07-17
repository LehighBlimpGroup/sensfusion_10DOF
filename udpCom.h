

#include "WiFi.h"
#include "AsyncUDP.h"
#include <data_types.h>
void unpack_joystick(float *dat, const unsigned char *buffer);
    


class UDPCom {
    private:
        const char * ssid = "AIRLab-BigLab";
        const char * password = "Airlabrocks2022";
        int UDPport = 1333; //TODO check if sending feedback uses a different UDP port
        int delayMS = 1000;
        AsyncUDP udp;
    public:
        UDPCom();
        void init();
        void send_udp_feedback(String dat1, String dat2, String dat3, String dat4);
        void getControllerInputs(controller_t *controls);
        void send_udp_feedback(String dat1, String dat2, String dat3, String dat4);
        void send_mag_acc(String dat1, String dat2, String dat3, String dat4, String dat5, String dat6){ //const unsigned char *buffer

};