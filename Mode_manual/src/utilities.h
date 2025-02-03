#ifndef UTILITIES_H
#define UTILITIES_H

#include <ESP32Servo.h>

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
void channelInterruptHandler();
void gyro_signals(void);
void mpu_signals();
void pin_mode();

#endif