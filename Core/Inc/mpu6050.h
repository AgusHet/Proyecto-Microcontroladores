#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "stm32f4xx_hal.h"

// Dirección del MPU6050 (con bit R/W en 0)
#define MPU6050_ADDR        (0x68 << 1)  // Dirección de 7 bits desplazada a la izquierda

// Registros importantes
#define WHO_AM_I_REG        0x75
#define REG_PWR_MGMT        0x6B
#define REG_CONFIG_GYRO     0x1B
#define REG_CONFIG_ACC      0x1C
#define REG_CONFIG          0x1A
#define REG_DATA_START      0x3B  // ACC_XOUT_H

// Valores de configuración
#define FS_GYRO_250         0x00
#define FS_ACC_2G           0x00
#define LPF_44              0x03
#define RESET_SENSOR        0x80

//// Valores de configuracion del giroscopio - RANGOS
//#define FS_GYRO_250 	0
//#define FS_GYRO_500 	8
//#define FS_GYRO_1000 	9
//#define FS_GYRO_2000	10

//// Valores de configuracion del acelerometro - RANGOS
//#define FS_ACC_2G 	0
//#define FS_ACC_4G 	8
//#define FS_ACC_8G 	9
//#define FS_ACC_16G 	10

// Timeout
#define MPU6050_I2C_TIMEOUT 100

// Códigos de retorno
#define MPU6050_OK                 0x00
#define MPU6050_ERR_NO_I2C         0x01
#define MPU6050_ERR_READ_REG       0x02
#define MPU6050_ERR_WHO_AM_I       0x03
#define MPU6050_ERR_PWR_MGMT_WRITE 0x04
#define MPU6050_ERR_GYRO_CONFIG    0x05
#define MPU6050_ERR_ACC_CONFIG     0x06
#define MPU6050_ERR_FILTER_CONFIG  0x07
#define MPU6050_ERR_READ_DATA      0x08
#define MPU6050_ERR_RESET          0x09

// Prototipos de funciones
uint8_t mpu6050_ready(I2C_HandleTypeDef *hi2c);
uint8_t mpu6050_init(I2C_HandleTypeDef *hi2c);
uint8_t mpu6050_config(I2C_HandleTypeDef *hi2c);
uint8_t mpu6050_read(I2C_HandleTypeDef *hi2c,
                     int16_t* x_acc, int16_t* y_acc, int16_t* z_acc,
                     int16_t* x_gyro, int16_t* y_gyro, int16_t* z_gyro);
uint8_t mpu6050_reset(I2C_HandleTypeDef *hi2c);
float mpu6050_gyro_raw_to_dps(int16_t raw_value);

#endif /* INC_MPU6050_H_ */

