
#include <udpCom.h>


volatile bool joy_ready;
volatile unsigned long time_now;
float joy_data[13] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

UDPCom::UDPCom(){
  joy_ready = false;

}


void UDPCom::init(int port){
  //following code block connects you to the internet
    active = true;
    UDPport = port;
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
void UDPCom::send_udp_feedback(String dat1, String dat2, String dat3, String dat4){ //const unsigned char *buffer
  if (active){
    String blimp_feedback = String("0,");
    blimp_feedback += dat1 + String(",") 
                    + dat2 + String(",") 
                    + dat3 + String(",") + dat4;
  
    udp.broadcastTo(blimp_feedback.c_str(), UDPport);
  }
}


// send udp feedback on roll, pitch, and yaw
void UDPCom::send_mag_acc(float calibration_data[6]){ //const unsigned char *buffer

  if (active) {
    String blimp_feedback = String("");
    blimp_feedback = String("1,") + String(calibration_data[0]) + 
                    String(",") + String(calibration_data[1]) + 
                    String(",") + String(calibration_data[2]) + 
                    String(",") + String(calibration_data[3])+ 
                    String(",") + String(calibration_data[4]) + 
                    String(",") + String(calibration_data[5]);
    // Serial.println(blimp_feedback);
    udp.broadcastTo(blimp_feedback.c_str(), UDPport);
  }
}

void UDPCom::sendAck() {
  if (active) {
    String blimp_feedback = String("2,0");
    udp.broadcastTo(blimp_feedback.c_str(), UDPport);
  }
}



//unpacks the data into joystick data list
void unpack_joystick(float *dat, const unsigned char *buffer) {
  int num_floats = 13;
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
    controls->flag = joy_data[0];
    controls->fx = joy_data[1];
    controls->fy = joy_data[2];
    controls->fz = joy_data[3];
    controls->tx = joy_data[4];
    controls->ty = joy_data[5];
    controls->tz = joy_data[6];
    controls->absz = joy_data[7];
    controls->ready = joy_data[8] != 0;
    controls->snapshot = joy_data[9];
  } else {
    controls->ready = false;
  }
  
  return;
}


void UDPCom::getCalibrationInputs(float input_data[13]){
  
  if (joy_ready && millis() - time_now < delayMS){
    for (int i; i < 13; i++) {
      input_data[i] = joy_data[i];
    }
  }
  return;
}


//takes saved joystick data and puts it into the interfaceable packet.
//    also makes sure that the data has been recieved within the last second 
//    to prevent drone from flying away if losing controller connection
void UDPCom::getControllerRaws(raw_t *raws){
  
  if (joy_ready && millis() - time_now < delayMS){
    raws->flag = joy_data[0];
    raws->ready = joy_data[1] != 0;
    raws->data[0] = joy_data[2];
    raws->data[1] = joy_data[3];
    raws->data[2] = joy_data[4];
    raws->data[3] = joy_data[5];
    raws->data[4] = joy_data[6];
    raws->data[5] = joy_data[7];
    raws->data[6] = joy_data[8];
    raws->data[7] = joy_data[9];
    raws->data[8] = joy_data[10];
    raws->data[9] = joy_data[11];
    raws->data[10] = joy_data[12];
  } else {
    raws->ready = false;
  }
  
  return;
}