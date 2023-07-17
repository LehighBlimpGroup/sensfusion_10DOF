
#include <Preferences.h>
Preferences preferences;
void setup() {
  preferences.begin("calibration", false); 
  preferences.clear();
  for (int i = 0; i < 9; i ++) {
    putFloat("m" + String(i), 2.0f);// place UDP recieve here to get the values?
  }
  for (int i = 9; i < 12; i ++) {
    putFloat("o" + String(i-9), 2.0f);// place UDP recieve here to get the values?
  }
  preferences.end();
}

void loop() {
  // put your main code here, to run repeatedly:

}
