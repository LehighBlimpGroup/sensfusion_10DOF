

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
        void getControllerInputs(controller_t *controls);
        void send_udp_feedback(float dat1, float dat2, float dat3, float dat4);
};