#include"pid.h"




PID::PID(float KP, float KI, float KD, float TIME_SAMPLING, float MAX_OUT, float MIN_OUT, float SETPOINT)
{
  _KP = KP;
  _KI = KI*TIME_SAMPLING;
  _KD = KD/TIME_SAMPLING;
  _TIME_SAMPLING = TIME_SAMPLING;
  _MAX_OUT = MAX_OUT;
  _MIN_OUT = MIN_OUT;
  _SETPOINT = SETPOINT;
  
}

void PID::setTunning(float KP,float KI, float KD, float TIME_SAMPLING)
{
  _KP = KP;
  _KI = KI*TIME_SAMPLING;
  _KD = KD/TIME_SAMPLING;
  _TIME_SAMPLING = TIME_SAMPLING;  
}

void PID::clamp(float MAX_OUT, float MIN_OUT)
{
  _MAX_OUT = MAX_OUT;
  _MIN_OUT = MIN_OUT;  
}


void PID::setPoint(float setPoint)
{
  _SETPOINT = setPoint;  
}

float PID::compute(float mea)
{
  static float last_error;
  static float I_term;
  float error = _SETPOINT - mea;
  float P_term = _KP*error;
  I_term += _KI*error;
  float D_term = _KD*(error - last_error);
  float out = P_term + I_term + D_term;
  if(out > _MAX_OUT)
  {
    I_term = I_term - _MAX_OUT;
    out = _MAX_OUT; 
  }
  else if(out < _MIN_OUT)
  {
    out = _MIN_OUT;
    I_term = _MIN_OUT - I_term;
  }
  last_error = error;
  return out;
}



float PID::getKP()
{
  return _KP;
}
