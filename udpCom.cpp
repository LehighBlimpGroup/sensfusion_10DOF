// send udp feedback on roll, pitch, and yaw
void send_udp_feedback(){ //const unsigned char *buffer
  int num_floats = 4;
  int num_bytes = 4;
  float dat[4] = {roll, pitch, yaw, yawrate};
  int i, j;
  /*
  for (i = 0; i < num_floats; i++){
    char temp[4] = {0, 0, 0, 0};
    for (j = 0; j < num_bytes; j++){
      temp[j] = buffer[4*i + j];
    }
    dat[i] = *((float*) temp);
  }
  */
  String blimp_feedback = String("");
  blimp_feedback = String((float)roll) + String(", ") + String((float)pitch) + String(", ") + String((float)yaw + String(", ") + String((float)yawrate));
  
  udp.broadcastTo(blimp_feedback.c_str(), 1444);
}

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
    // if(i == 1 || i == 3){
    //   dat[i] = -*((float*) temp);
    // } else {
    //   dat[i] = *((float*) temp);
    // }
  }
}


   WiFi.mode(WIFI_STA);
   WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while(1) {
      delay(3000);
    servo1.write((int) 180);
    servo2.write((int) 0);
      delay(3000);
    servo1.write((int) 0);
    servo2.write((int) 180);
    }
  }
   if(udp.listen(1444)) {
     Serial.print("UDP Listening on IP: ");
     Serial.println(WiFi.localIP());

     // setup callback functions of the udp
     udp.onPacket([](AsyncUDPPacket packet) {
       joy_ready = false;
       time_now = millis();
       unsigned char *buffer = packet.data();
       //buff = *buffer;
       unpack_joystick(joy_data, buffer);
       joy_ready = true;
       //reply to the client
       //packet.printf("Got %u bytes of data", packet.length());
     });
   }


void getControllerInputs(float *fx, float *fy, float *fz, float *tx, float *ty, float *tz, float *abz){
    
  *fx = joy_data[0];
  *fy = joy_data[1];
  *fz = joy_data[2];
  *tx = joy_data[3];
  *ty = joy_data[4];
  *tz = joy_data[5];
  *abz = joy_data[6];
  
}