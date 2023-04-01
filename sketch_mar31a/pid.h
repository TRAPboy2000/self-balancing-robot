#ifndef PID_H
#define PID_H

#include<stdlib.h>
#include<Arduino.h>

class PID
{
  private:
    float _KP, _KI, _KD, _TIME_SAMPLING, _MAX_OUT, _MIN_OUT, _SETPOINT;

  public:
    PID(float KP, float KI, float KD, float TIME_SAMPLING, float MAX_OUT, float MIN_OUT, float SETPOINT);
    void setTunning(float KP,float KI, float KD, float TIME_SAMPLING);
    void clamp(float MAX_OUT, float MIN_OUT);
    void setPoint(float setPoint);
    float compute(float mea);
    float getKP();
  
};








#endif
