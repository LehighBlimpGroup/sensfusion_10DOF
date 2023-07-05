float clamp(float in, float min, float max){
  if (in< min){
    return min;
  } else if (in > max){
    return max;
  } else {
    return in;
  }
}
void controlOutputs(float ifx, float ify, float ifz, float itx, float ity, float itz) {
    //float desiredPitch = wty - self->pitch*(float)g_self.kR_xy - self->pitchrate *(float)g_self.kw_xy;

    float l = lx; //.3
    float fx = clamp(ifx, -1 , 1);//setpoint->bicopter.fx;
    float fz = clamp(ifz, 0 , 2);//setpoint->bicopter.fz;
    float taux = clamp(itx, -l + (float)0.01 , l - (float) 0.01);
    float tauz = clamp(itz, -.3 , .3);// limit should be .25 setpoint->bicopter.tauz; //- stateAttitudeRateYaw

    float term1 = l*l*fx*fx + l*l*fz*fz + taux*taux + tauz*tauz;
    float term2 = 2*fz*l*taux - 2*fx*l*tauz;
    float term3 = sqrt(term1+term2);
    float term4 = sqrt(term1-term2);

    float f1 = term3/(2*l); // in unknown units
    float f2 = term4/(2*l);


    float t1 = atan2((fz*l - taux)/term3, (fx*l + tauz)/term3 );// in radians
    float t2 = atan2((fz*l + taux)/term4, (fx*l - tauz)/term4 );

  
    while (t1 < 0) {
      t1 = t1 + 2 * PI;
    }
    while (t1 > 2*PI) {
      t1 = t1 - 2 * PI;
    }
    while (t2 < 0) {
      t2 = t2 + 2 * PI;
    }
    while (t2 > 2*PI) {
      t2 = t2 - 2 * PI;
    }
    s1 = clamp(t1, 0, PI)/(PI);// cant handle values between PI and 2PI
    s2 = clamp(t2, 0, PI)/(PI);
    m1 = clamp(f1, 0, 1);
    m2 = clamp(f2, 0, 1);
    if (m1 < 0.02f ){
      s1 = 0.5f; 
    }
    if (m2 < 0.02f ){
      s2 = 0.5f; 
    }
}


//without signal
    servo1.write((int) (90));
    servo2.write((int) (90));
    thrust1.writeMicroseconds(1100);
    //delay(5);
    thrust2.writeMicroseconds(1100);


//with signal
servo1.write((int) (s1*180));
    servo2.write((int) ((1-s2)*180));
    am1 = am1 * .98f + m1 * .02f;
    am2 = am2 * .98f + m2 * .02f;
    thrust1.writeMicroseconds((int)(1100 + 400*m1));
    //delay(5);
    thrust2.writeMicroseconds((int)(1100 + 400*m2));