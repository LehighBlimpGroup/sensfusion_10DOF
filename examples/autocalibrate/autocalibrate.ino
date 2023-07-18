
#include <Preferences.h>
Preferences preferences;
char* name = new char[3];
char str[256];
void setup() {
  Serial.begin(115200);
  // preferences.begin("calibration", false); 
  // preferences.clear();
  // name[2] = (char)'\0';
  
  // name[0] = (char)'m';
  // for (int i = 0; i < 9; i ++) {
  //   itoa(i,str,16);
  //   name[1] = str[0];
  //   Serial.println(name);
  //   preferences.putFloat(name, (float_t)(2.1f + (float)(i)));// place UDP recieve here to get the values?
  // }
  // name[0] = (char)'o';
  // for (int i = 0; i < 3; i ++) {
  //   itoa(i,str,16);
  //   name[1] = str[0];
  //   Serial.println(name);
  //   preferences.putFloat(name, (float_t)(3.2f + (float)(i)));// place UDP recieve here to get the values?
  // }
  // preferences.end();
  preferences.begin("calibration", true); 
}

  
void loop() {
  

  name[2] = (char)'\0';
  name[0] = (char)'m';
  for (int i = 0; i < 9; i ++) {
    itoa(i,str,16);
    name[1] = str[0];
    Serial.print(preferences.getFloat(name,(float_t)0));// place UDP recieve here to get the values?
    Serial.print(", ");
  }
  name[0] = (char)'o';
  for (int i = 0; i < 3; i ++) {
    itoa(i,str,16);
    name[1] = str[0];
    Serial.print(preferences.getFloat(name,(float_t)0));// place UDP recieve here to get the values?
    Serial.print(", ");
  }
  Serial.println();
  delay(1000);
  

}
