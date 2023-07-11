
#include <udpCom.h>


volatile bool joy_ready;
volatile unsigned long time_now;
float joy_data[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

UDPCom::UDPCom(){
  joy_ready = false;

}


void UDPCom::init(){
  //following code block connects you to the internet
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("WiFi Failed");
        //pinMode(LED_BUILTIN, OUTPUT);
        while(1) {
          delay(100);
          //digitalWrite(LED_BUILTIN, HIGH);
          delay(100);
          //digitalWrite(LED_BUILTIN, LOW);
        }
    }

    //following code block determines what happens when a packet
    //    is recieved through the wifi
    time_now = millis();
    if(udp.listen(UDPport)) {
        Serial.print("UDP Listening on IP: ");
        Serial.println(WiFi.localIP());

        // setup callback functions of the udp
        udp.onPacket([](AsyncUDPPacket packet) {
            joy_ready = false;
            time_now = millis();
            unsigned char *buffer = packet.data();
            unpack_joystick(joy_data, buffer);//converts data into joystick 
            joy_ready = true;
            //reply to the client
            //packet.printf("Got %u bytes of data", packet.length());
        });
    }
}


// send udp feedback on roll, pitch, and yaw
// TODO finish this off
void UDPCom::send_udp_feedback(String dat1, String dat2, String dat3, String dat4){ //const unsigned char *buffer

  String blimp_feedback = String("");
  blimp_feedback = dat1 + String(", ") + dat2 + String(", ") + dat3 + String(", ") + dat4;
  
  udp.broadcastTo(blimp_feedback.c_str(), UDPport);
}


//unpacks the data into joystick data list
void unpack_joystick(float *dat, const unsigned char *buffer) {
  int num_floats = 8;
  int num_bytes = 4;
  int i, j;

  for(i = 0; i < num_floats; i++) {
    char temp[4] = {0, 0, 0, 0};
    for(j = 0; j < num_bytes; j++) {
      temp[j] = buffer[4*i + j];
    }
    dat[i] = *((float*) temp);
  }
}


//takes saved joystick data and puts it into the interfaceable packet.
//    also makes sure that the data has been recieved within the last second 
//    to prevent drone from flying away if losing controller connection
void UDPCom::getControllerInputs(controller_t *controls){
  
  if (joy_ready && millis() - time_now < delayMS){
    controls->fx = joy_data[0];
    controls->fy = joy_data[1];
    controls->fz = joy_data[2];
    controls->tx = joy_data[3];
    controls->ty = joy_data[4];
    controls->tz = joy_data[5];
    controls->absz = joy_data[6];
    controls->ready = joy_data[7];
  } else {
    controls->ready = false;
  }
  
  return;
}