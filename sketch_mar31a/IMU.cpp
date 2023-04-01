#include "IMU.h"

IMU_6050::IMU_6050(uint32_t clock_speed)
{
	Wire.setClock(_clock_speed);
	Wire.begin();
	Wire.beginTransmission(WHO_I_AM);
	Wire.write(PWR1);
	Wire.write(0x00);
	Wire.endTransmission();
	delay(250);
}


IMU_6050::IMU_6050()
{
	Wire.setClock(400000);
	Wire.begin();
	Wire.beginTransmission(WHO_I_AM);
	Wire.write(PWR1);
	Wire.write(0x00);
	Wire.endTransmission();
	delay(250);
}



void IMU_6050::gyro_config(uint8_t range)
{
  Wire.beginTransmission(WHO_I_AM);
  Wire.write(GYRO_CONFIG_REG);
  Wire.write(range);
  Wire.endTransmission();
  switch(range)
  {
    case GYRO_RANGE_250:
      gyro_LSB_sentivity = 131;
    case GYRO_RANGE_500:
      gyro_LSB_sentivity = 65.5;
    case GYRO_RANGE_1000:
      gyro_LSB_sentivity = 32.8;
    case GYRO_RANGE_2000:
      gyro_LSB_sentivity = 16.4;
  }
}


void IMU_6050::DLPF_config(uint8_t cutoff)
{
  Wire.beginTransmission(WHO_I_AM);
  Wire.write(DLPF);
  Wire.write(cutoff);
  Wire.endTransmission();
}


void IMU_6050::accel_config(uint8_t range)
{
  Wire.beginTransmission(WHO_I_AM);
  Wire.write(ACCEL_CONFIG_REG);
  Wire.write(range);
  Wire.endTransmission();
  switch(range)
  {
    case ACCEL_RANGE_2G:
      gyro_LSB_sentivity = 16384;
    case ACCEL_RANGE_4G:
      gyro_LSB_sentivity = 8192;
    case ACCEL_RANGE_8G:
      gyro_LSB_sentivity = 4096;
    case ACCEL_RANGE_16G:
      gyro_LSB_sentivity = 2048;
  }
}


void IMU_6050::readRaw(int16_t* accel_data, int16_t* gyro_data)
{
  uint8_t temp[6];
  Wire.beginTransmission(WHO_I_AM);
  Wire.write(GYRO_REG);
  Wire.endTransmission();
  Wire.requestFrom(WHO_I_AM, 6);
  for(uint8_t i = 0; i < 6; i++)
  {
    temp[i] = Wire.read();
  }


  for(uint8_t i = 0; i < 3; i++)
  {
    gyro_data[i] = (temp[i * 2] << 8 | temp[(i * 2) + 1]);
  }


  Wire.beginTransmission(WHO_I_AM);
  Wire.write(ACCEL_REG);
  Wire.endTransmission();
  Wire.requestFrom(WHO_I_AM, 6);
  for(uint8_t i = 0; i < 6; i++)
  {
    temp[i] = Wire.read();
  }

  for(uint8_t i = 0; i < 3; i++)
  {
    accel_data[i] = (temp[i * 2] << 8 | temp[(i * 2) + 1]);
  }

}


void IMU_6050::gyro_angle(float* g_angle, int16_t* gyro_data, float t_sample)
  {
    *g_angle = *g_angle + ((float)gyro_data[0] / gyro_LSB_sentivity)*t_sample;
  }

 void IMU_6050::accel_angle(float* a_angle, int16_t* accel_data)
{
    double accel_y = (float)accel_data[1] / accel_LSB_sentivity;
    double accel_z = (float)accel_data[2] / accel_LSB_sentivity;
    *a_angle = atan2(accel_y, accel_z);   
}


void IMU_6050::gyroCalibrate(int16_t* gyro_data, uint16_t num_example)
{
  Serial.println("start calibrating");
  float s1 = 0;
  float s2 = 0;
  float s3 = 0;
  for(uint16_t i = 0; i < num_example; i++)
  {
    s1 += (float)gyro_data[0] / gyro_LSB_sentivity;
    s2 += (float)gyro_data[2] / gyro_LSB_sentivity;
    s3 += (float)gyro_data[3] / gyro_LSB_sentivity;
  }
  Serial.println("Done calibariate");
  Serial.print("gyro x bias: ");
  Serial.println(s1 / (num_example - 1));
  Serial.print("gyro y bias: ");
  Serial.println(s2 / (num_example - 1));
  Serial.print("gyro z bias: ");
  Serial.println(s3 / (num_example - 1));
}


void IMU_6050::accelCalibrate(uint8_t axis, int16_t* accel_data, uint16_t num_example)
{
  Serial.println("start calibrating");
  uint8_t index;
  float s = 0;
  switch(axis)
  {
    case 'x':
      index = 0;
      break; 
    case 'y':
      index = 1;
      break;
    case 'z':
      index = 2;
      break;
    default:
      Serial.println("unknow axis");
      return;    
  }

  
  for(uint16_t i = 0; i < num_example; i++)
  {
    s += accel_data[index] / accel_LSB_sentivity;  
  }
  Serial.println("Done calibariate");
  Serial.print("Bias ");
  Serial.print(axis);
  Serial.print(" : ");
  Serial.println(s / (num_example - 1));
}

float IMU_6050::get_accel_sentivity()
{
  return accel_LSB_sentivity;
}


float IMU_6050::get_gyro_sentivity()
{
  return gyro_LSB_sentivity;
}







HighPassFilter::HighPassFilter(float beta)
{
  _beta = beta;
}


float HighPassFilter::update(float mea)
{
  static float x_n_1;
  static float y_n_1;
  float y_n = (1.0/2)*((2 - _beta)*(mea - x_n_1) + (1 - _beta)*y_n_1);
  x_n_1 = mea;
  y_n_1 = y_n;
  return y_n;
}


LowPassFilter::LowPassFilter(float alpha)
{
  _alpha = alpha;
}


float LowPassFilter::update(float mea)
{
  static float y_n_1;
  float y_n = _alpha*mea + (1 - _alpha)*y_n_1;
  y_n_1 = y_n;
  return y_n;
}



C_filter::C_filter(float alpha)
{
  _alpha = alpha;
}



float C_filter::update(float accel_angle, float gyro_angle)
{
  float y = (1 - _alpha)*gyro_angle + _alpha*accel_angle;
  return y;
}
