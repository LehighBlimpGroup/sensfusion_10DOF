/*
 * This example receives the information from two distance sensors, 
 * approximates a line, and identifies the angle, and the distance,
 * to a wall in front of the sensors.
 */




// Distance sensors
float dist_1, dist_2;

float cos_mbeta, sin_mbeta, cos_pbeta, sin_pbeta;
double px1, px2, py1, py2;



void setup() {
  // put your setup code here, to run once:
  // Angle between the y-axis and the ray of the distance sensor.
  float beta = 20; 
  beta = beta * PI / 180;  // Convert to radians


  // Projections (constants)
  cos_mbeta = cos(PI / 2 - beta); 
  sin_mbeta = sin(PI / 2 - beta);
  cos_pbeta = cos(PI / 2 + beta);
  sin_pbeta = sin(PI / 2 + beta);


  Serial.begin(115200);
}


void loop() {
  // Get from distance sensors
  

  dist_1 = 0.7;
  dist_2 = 1;



  px1 = dist_1 * cos_mbeta;
  py1 = dist_1 * sin_mbeta;
  px2 = dist_2 * cos_pbeta;
  py2 = dist_2 * sin_pbeta;

  // Line y = ax + b
  float a = (py1 - py2) / (px1 - px2);
  float b = py2 - a * px2;   // Distance to the front of the robot (y-axis)
                
  // relative angle
  float ang = atan2(py1 - py2, px1 - px2);

  Serial.print(ang);
  Serial.print(" , ");
  Serial.println(b);

}
