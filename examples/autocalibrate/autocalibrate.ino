
#include <Preferences.h>
Preferences preferences;
void setup() {
  Serial.begin(115200);
  preferences.begin("calibration", false); 
  preferences.clear();
  char* name = new char[3];
  char* intptr = new char[5];
  name[0] = (char)'m';
  for (int i = 0; i < 9; i ++) {
    
    intptr[0] = &((char) i);
    intptr[4] = (char*) '\0';
    name[1] = (char)(i);
    name[2] = '\0';
    Serial.print(intptr);
    //preferences.putFloat(name, (float_t)(2.1f + (float)(i)));// place UDP recieve here to get the values?
  }
  for (int i = 0; i < 3; i ++) {
    char* name = new char[3];
    name[0] = (char)'o';
    char* intptr = new char[5];
    intptr[0] = &((char) i);
    intptr[4] = (char) '\0';
    name[1] = (char)(i);
    name[2] = '\0';
    Serial.print(intptr);
    //preferences.putFloat(name, (float_t)(3.2f + (float)(i)));// place UDP recieve here to get the values?
  }
  preferences.end();
  preferences.begin("calibration", true); 
}

void loop() {
  // put your main code here, to run repeatedly:

  // for (int i = 0; i < 9; i ++) {
  //   char* name = new char[2];
  //   name[0] = (char)'m';
  //   name[1] = (char)i;
  //   Serial.print(preferences.getFloat(name,(float_t)0));// place UDP recieve here to get the values?
  //   Serial.print(", ");
  // }
  // for (int i = 0; i < 3; i ++) {
  //   char* name = new char[2];
  //   name[0] = (char)'o';
  //   name[1] = (char)(i);
  //   Serial.print(preferences.getFloat(name,(float_t)0));// place UDP recieve here to get the values?
  //   Serial.print(", ");
  // }
  // Serial.println();
  delay(1000);

}
