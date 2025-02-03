#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "variables.h"

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState + (t*KalmanInput);
  KalmanUncertainty=KalmanUncertainty + (t*t*4*4); //here 4 is the vairnece of IMU i.e 4 deg/s
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3); //std deviation of error is 3 deg
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}


void gyro_signals(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29; //*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29;
}

void mpu_signals() {
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(); 
    Wire.requestFrom(0x68,6);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();
    Wire.beginTransmission(0x68);
    Wire.write(0x1B); 
    Wire.write(0x8);
    Wire.endTransmission();                                                   
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68,6);
    int16_t GyroX=Wire.read()<<8 | Wire.read();
    int16_t GyroY=Wire.read()<<8 | Wire.read();
    int16_t GyroZ=Wire.read()<<8 | Wire.read();
    RateRoll=(float)GyroX/65.5;
    RatePitch=(float)GyroY/65.5;
    RateYaw=(float)GyroZ/65.5;
    AccX=(float)AccXLSB/4096;
    AccY=(float)AccYLSB/4096;
    AccZ=(float)AccZLSB/4096;

    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    AccX -= AccXCalibration ;
    AccY -= AccYCalibration ;
    AccZ -= AccZCalibration;

    AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29;
    AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29;

    complementaryAngleRoll=0.991*(complementaryAngleRoll+RateRoll*t) + 0.009*AngleRoll;
    complementaryAnglePitch=0.991*(complementaryAnglePitch+RatePitch*t) + 0.009*AnglePitch;
    // Clamping complementary filter roll angle to ±20 degrees
    complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
    complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);

}

void pin_mode() {
    int led_time=100;
    pinMode(15, OUTPUT);
    digitalWrite(15, LOW);
    delay(led_time);
    digitalWrite(15, HIGH);
    delay(led_time);
    digitalWrite(15, LOW);
    delay(led_time);
    digitalWrite(15, HIGH);
    delay(led_time);
    digitalWrite(15, LOW);
    delay(led_time);
    digitalWrite(15, HIGH);
    delay(led_time);
    digitalWrite(15, LOW);
    delay(led_time);
    digitalWrite(15, HIGH);
    delay(led_time);
    digitalWrite(15, LOW);
    delay(led_time);

    pinMode(channel_1_pin, INPUT_PULLUP);
    pinMode(channel_2_pin, INPUT_PULLUP);
    pinMode(channel_3_pin, INPUT_PULLUP);
    pinMode(channel_4_pin, INPUT_PULLUP);
    pinMode(channel_5_pin, INPUT_PULLUP);
    pinMode(channel_6_pin, INPUT_PULLUP);
}