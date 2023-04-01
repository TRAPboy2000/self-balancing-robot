#include "IMU.h"
#include "pid.h"
const uint8_t sample_interval = 4;
uint32_t time_elap = 0;
float accel_angle, gyro_angle;
IMU_6050 MPU_6050 = IMU_6050();
HighPassFilter gyro_filter = HighPassFilter(0.6);
LowPassFilter accel_filter = LowPassFilter(0.7);
C_filter fusion = C_filter(0.05);

int16_t gyro_data[3];
int16_t accel_data[3];
float gyro_LSB = MPU_6050.get_accel_sentivity();
float accel_LSB = MPU_6050.get_gyro_sentivity();

void setup() 
{
  MPU_6050.gyro_config(GYRO_RANGE_500);
  MPU_6050.accel_config(ACCEL_RANGE_4G);
  MPU_6050.DLPF_config(BANDWIDTH_10_10);
  Serial.begin(57600);
}

void loop() 
{
  if(millis() - time_elap >= sample_interval)
  {
    MPU_6050.readRaw(accel_data, gyro_data);
    MPU_6050.accel_angle(&accel_angle, accel_data);
    MPU_6050.gyro_angle(&gyro_angle, gyro_data, (float)sample_interval/1000);
    Serial.println(accel_angle);
    //Serial.println(gyro_angle);   
  }

}
