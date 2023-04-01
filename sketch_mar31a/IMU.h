#ifndef IMU_H
#define IMU_H
#include <Arduino.h>
#include <Wire.h>
#include<math.h>
#define WHO_I_AM 0x68
#define GYRO_REG 0x43
#define PWR1 0x6B
#define ACCEL_REG 0x3B
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define GYRO_RANGE_250 0b00000000
#define GYRO_RANGE_500 0b00001000
#define GYRO_RANGE_1000 0b00010000
#define GYRO_RANGE_2000 0b00011000
#define ACCEL_RANGE_2G 0b00000000
#define ACCEL_RANGE_4G 0b00001000
#define ACCEL_RANGE_8G 0b00010000
#define ACCEL_RANGE_16G 0b00011000
#define DLPF 0x1A
#define BANDWIDTH_260_256 0b00000000
#define BANDWIDTH_184_188 0b00000001
#define BANDWIDTH_94_98 0b00000010
#define BANDWIDTH_44_42 0b00000011
#define BANDWIDTH_21_20 0b00000100
#define BANDWIDTH_10_10 0b00000101
#define BANDWIDTH_5_5 0b00000110






class IMU_6050
{
	private:
		float accel_LSB_sentivity;
		float gyro_LSB_sentivity;
    uint32_t _clock_speed;

	public:
		IMU_6050(uint32_t clock_speed);
		IMU_6050();
		void gyro_config(uint8_t range);
		void accel_config(uint8_t range);
    void DLPF_config(uint8_t cutoff); 
		void readRaw(int16_t* accel_data, int16_t* gyro_data);
		void gyro_angle(float* g_angle, int16_t* gyro_data, float t_sample);
		void accel_angle(float* a_angle, int16_t* accel_data);
    void gyroCalibrate(int16_t* gyro_data, uint16_t num_example);
    void accelCalibrate(uint8_t axis, int16_t* accel_data, uint16_t num_example);
    float get_accel_sentivity();
    float get_gyro_sentivity();
};



class HighPassFilter
{
	private:
		float _beta;
	public:
		HighPassFilter(float beta);
		float update(float mea);
};



class LowPassFilter
{
	private:
		float _alpha;
	public:
		LowPassFilter(float alpha);
		float update(float mea);

};



class C_filter
{
	private:
		float _alpha;
	public:
		C_filter(float alpha);
		float update(float accel_angle, float gyro_angle);
};

#endif
