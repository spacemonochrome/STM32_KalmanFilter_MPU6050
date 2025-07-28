/*
 * mpu6050.c
 *
 *  Created on: Apr 27, 2025
 *      Author: Hasan Fırat Keskin
 */

#include <math.h>
#include "mpu6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

//Setup MPU6050
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define I2C_TIMEOUT 50
#define MPU6050_ADDR 0xD0

Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 2.0f,
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 2.0f,
};

uint8_t Rec_Data[14];

volatile double dt_g = 0.0f;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, I2C_TIMEOUT);

    if (status != HAL_OK)
    {
    	// hata dondur
	}

    if (check == 104 && status == HAL_OK)  // 0x68 will be returned by the sensor if everything goes well 104
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, I2C_TIMEOUT);

        // DLP settings
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x1A, 1, &Data, 1, I2C_TIMEOUT);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, I2C_TIMEOUT);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, I2C_TIMEOUT);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> 250 degree/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, I2C_TIMEOUT);
        return 0;
    }
    return check;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}


void MPU6050_Read_WithKalman(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct, uint32_t zaman)
{

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, (uint8_t *)Rec_Data, 14, I2C_TIMEOUT);

    if (status != HAL_OK)
    {
    	// hata dondur
	}

    DataStruct->Accel_X_RAW = ((int16_t)(Rec_Data[0] << 8 | Rec_Data[1])) - DataStruct->Accel_X_RAW_K;
    DataStruct->Accel_Y_RAW = ((int16_t)(Rec_Data[2] << 8 | Rec_Data[3])) - DataStruct->Accel_Y_RAW_K;
    DataStruct->Accel_Z_RAW = ((int16_t)(Rec_Data[4] << 8 | Rec_Data[5])) - DataStruct->Accel_Z_RAW_K;
    DataStruct->Temp_RAW = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = ((int16_t)(Rec_Data[8] << 8 | Rec_Data[9])) - DataStruct->Gyro_X_RAW_K;
    DataStruct->Gyro_Y_RAW = ((int16_t)(Rec_Data[10] << 8 | Rec_Data[11])) - DataStruct->Gyro_Y_RAW_K;
    DataStruct->Gyro_Z_RAW = ((int16_t)(Rec_Data[12] << 8 | Rec_Data[13])) - DataStruct->Gyro_Z_RAW_K;

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;
    DataStruct->Temperature = (float)(DataStruct->Temp_RAW  / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;


    double dt = DWT_GetDeltaMicros(zaman) * 1e-6;

    double roll = atan2(DataStruct->Ay, sqrt(DataStruct->Ax * DataStruct->Ax + DataStruct->Az * DataStruct->Az)) * RAD_TO_DEG;
    double pitch = atan2(-DataStruct->Ax, DataStruct->Az) * RAD_TO_DEG;

    // Pitch için ±90° kontrolü (Gimbal lock önleme)
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    } else {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }

    // Roll için ±90° kontrolü (Gimbal lock önleme)
    if ((roll < -90 && DataStruct->KalmanAngleX > 90) || (roll > 90 && DataStruct->KalmanAngleX < -90)) {
        KalmanX.angle = roll;
        DataStruct->KalmanAngleX = roll;
    } else {
        DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
    }

    // Jiroskop yön düzeltmesi (pitch ±90° durumunda)
    if (fabs(DataStruct->KalmanAngleY) > 90) {
        DataStruct->Gx = -DataStruct->Gx;
    }
    // Yaw için jiroskop entegrasyonu
    DataStruct->KalmanAngleZ += DataStruct->Gz * dt;
}


void MPU6050_Calc_Kalman(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct, uint32_t zaman)
{

    double dt = DWT_GetDeltaMicros(zaman) * 1e-6;

    double roll = atan2(DataStruct->Ay, sqrt(DataStruct->Ax * DataStruct->Ax + DataStruct->Az * DataStruct->Az)) * RAD_TO_DEG;
    double pitch = atan2(-DataStruct->Ax, DataStruct->Az) * RAD_TO_DEG;

    // Pitch için ±90° kontrolü (Gimbal lock önleme)
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    } else {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }

    // Roll için ±90° kontrolü (Gimbal lock önleme)
    if ((roll < -90 && DataStruct->KalmanAngleX > 90) || (roll > 90 && DataStruct->KalmanAngleX < -90)) {
        KalmanX.angle = roll;
        DataStruct->KalmanAngleX = roll;
    } else {
        DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
    }

    // Jiroskop yön düzeltmesi (pitch ±90° durumunda)
    if (fabs(DataStruct->KalmanAngleY) > 90) {
        DataStruct->Gx = -DataStruct->Gx;
    }
    // Yaw için jiroskop entegrasyonu
    DataStruct->KalmanAngleZ += DataStruct->Gz * dt;
}


void MPU6050_Calibration_offset(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    int32_t kalibreAx = 0, kalibreAy = 0, kalibreAz = 0;
    int32_t kalibreGx = 0, kalibreGy = 0, kalibreGz = 0;

    for (uint8_t sayac = 0; sayac < 250; sayac++)
    {
        MPU6050_Read_All(I2Cx, DataStruct);
        kalibreAx += DataStruct->Accel_X_RAW;
        kalibreAy += DataStruct->Accel_Y_RAW;
        kalibreAz += DataStruct->Accel_Z_RAW;

        kalibreGx += DataStruct->Gyro_X_RAW;
        kalibreGy += DataStruct->Gyro_Y_RAW;
        kalibreGz += DataStruct->Gyro_Z_RAW;
        HAL_Delay(1); // 1 + 1 = 2 millisecond wait
    }

    DataStruct->Accel_X_RAW_K = (int16_t)(kalibreAx / 250);
    DataStruct->Accel_Y_RAW_K = (int16_t)(kalibreAy / 250);
    DataStruct->Accel_Z_RAW_K = (int16_t)((kalibreAz / 250) - 16384);

    DataStruct->Gyro_X_RAW_K = (int16_t)(kalibreGx / 250);
    DataStruct->Gyro_Y_RAW_K = (int16_t)(kalibreGy / 250);
    DataStruct->Gyro_Z_RAW_K = (int16_t)(kalibreGz / 250);
}


void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, (uint8_t *)Rec_Data, 14, I2C_TIMEOUT);

    if (status != HAL_OK)
    {
    	// hata dondur
	}
    DataStruct->Accel_X_RAW = ((int16_t)(Rec_Data[0] << 8 | Rec_Data[1])) - DataStruct->Accel_X_RAW_K;
    DataStruct->Accel_Y_RAW = ((int16_t)(Rec_Data[2] << 8 | Rec_Data[3])) - DataStruct->Accel_Y_RAW_K;
    DataStruct->Accel_Z_RAW = ((int16_t)(Rec_Data[4] << 8 | Rec_Data[5])) - DataStruct->Accel_Z_RAW_K;
    DataStruct->Temp_RAW = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = ((int16_t)(Rec_Data[8] << 8 | Rec_Data[9])) - DataStruct->Gyro_X_RAW_K;
    DataStruct->Gyro_Y_RAW = ((int16_t)(Rec_Data[10] << 8 | Rec_Data[11])) - DataStruct->Gyro_Y_RAW_K;
    DataStruct->Gyro_Z_RAW = ((int16_t)(Rec_Data[12] << 8 | Rec_Data[13])) - DataStruct->Gyro_Z_RAW_K;

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;
    DataStruct->Temperature = (float)(DataStruct->Temp_RAW / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}


void MPU6050_Read_All_DMA(I2C_HandleTypeDef *I2Cx)
{
	HAL_I2C_Mem_Read_DMA(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, (uint8_t *)Rec_Data, 14);
}

void MPU6050_Calc_Kalman_DMA(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct, uint32_t zaman) {

    DataStruct->Accel_X_RAW = ((int16_t)(Rec_Data[0] << 8 | Rec_Data[1])) - DataStruct->Accel_X_RAW_K;
    DataStruct->Accel_Y_RAW = ((int16_t)(Rec_Data[2] << 8 | Rec_Data[3])) - DataStruct->Accel_Y_RAW_K;
    DataStruct->Accel_Z_RAW = ((int16_t)(Rec_Data[4] << 8 | Rec_Data[5])) - DataStruct->Accel_Z_RAW_K;
    DataStruct->Temp_RAW = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = ((int16_t)(Rec_Data[8] << 8 | Rec_Data[9])) - DataStruct->Gyro_X_RAW_K;
    DataStruct->Gyro_Y_RAW = ((int16_t)(Rec_Data[10] << 8 | Rec_Data[11])) - DataStruct->Gyro_Y_RAW_K;
    DataStruct->Gyro_Z_RAW = ((int16_t)(Rec_Data[12] << 8 | Rec_Data[13])) - DataStruct->Gyro_Z_RAW_K;

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;
    DataStruct->Temperature = (float)(DataStruct->Temp_RAW / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    double roll = atan2(DataStruct->Ay, sqrt(DataStruct->Ax * DataStruct->Ax + DataStruct->Az * DataStruct->Az)) * RAD_TO_DEG;
    double pitch = atan2(-DataStruct->Ax, DataStruct->Az) * RAD_TO_DEG;
    //dt_g = DWT_GetDeltaMicros(zaman) * 1e-6;

    // Pitch için ±90° kontrolü (Gimbal lock önleme)
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    } else {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt_g);
    }

    // Roll için ±90° kontrolü (Gimbal lock önleme)
    if ((roll < -90 && DataStruct->KalmanAngleX > 90) || (roll > 90 && DataStruct->KalmanAngleX < -90)) {
        KalmanX.angle = roll;
        DataStruct->KalmanAngleX = roll;
    } else {
        DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt_g);
    }

    // Jiroskop yön düzeltmesi (pitch ±90° durumunda)
    if (fabs(DataStruct->KalmanAngleY) > 90) {
        DataStruct->Gx = -DataStruct->Gx;
    }
    // Yaw için jiroskop entegrasyonu
    DataStruct->KalmanAngleZ += DataStruct->Gz * dt_g;
    dt_g = DWT_GetDeltaMicros(zaman) * 1e-6;
}

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_GetMicros(void)
{
    return (DWT->CYCCNT / (SystemCoreClock / 1000000));
}

uint32_t DWT_GetDeltaMicros(uint32_t start_time)
{
    uint32_t current_time = DWT_GetMicros();
    if (current_time >= start_time)
    {        return (current_time - start_time);    }
    else
    {        return (0xFFFFFFFF - start_time + current_time + 1);    }
}

