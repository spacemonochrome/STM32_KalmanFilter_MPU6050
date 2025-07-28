/*
 * mpu6050.h
 *
 *  Created on: Apr 27, 2025
 *      Author: Hasan Fırat Keskin
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f1xx_hal.h"

typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    int16_t Accel_X_RAW_K;
    int16_t Accel_Y_RAW_K;
    int16_t Accel_Z_RAW_K;
    int16_t Gyro_X_RAW_K;
    int16_t Gyro_Y_RAW_K;
    int16_t Gyro_Z_RAW_K;

    int16_t Temp_RAW;
    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
    double KalmanAngleZ;
} MPU6050_t;

// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

extern Kalman_t KalmanX;

extern Kalman_t KalmanY;

extern uint8_t Rec_Data[14];

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_WithKalman(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct, uint32_t zaman);

void MPU6050_Calc_Kalman(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct, uint32_t zaman);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

//uint32_t Timer_GetElapsed(TIM_HandleTypeDef *htim, uint32_t timer_start);

void MPU6050_Calibration_offset(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All_DMA(I2C_HandleTypeDef *I2Cx);

void MPU6050_Calc_Kalman_DMA(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct, uint32_t zaman);

void DWT_Init(void);

uint32_t DWT_GetMicros(void);

uint32_t DWT_GetDeltaMicros(uint32_t start_time);

#endif /* INC_MPU6050_H_ */
