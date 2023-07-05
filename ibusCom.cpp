
  MySerial0.begin(115200, SERIAL_8N1, -1, -1);
  IBus.begin(MySerial0, IBUSBM_NOTIMER);
  ready = (bool)(IBus.readChannel(4)>1500);
void getControllerInputs(float *fx, float *fy, float *fz, float *tx, float *ty, float *tz, float *abz){
  
  
  *fx = 1.2*((float)IBus.readChannel(1)-(float)1500)/(float)500 + ((float)IBus.readChannel(5)-(float)1500)/(float)500;
  *fy = 0;
  *fz = 6 * ((float)IBus.readChannel(2)-(float)1000)/(float)500;
  *tx = 0;//((float)IBus.readChannel(3)-(float)1000)/(float)1000;
  *ty = 0;
  *tz = -.6*((float)IBus.readChannel(0)-(float)1500)/(float)500;
  *abz = 0;
  
}