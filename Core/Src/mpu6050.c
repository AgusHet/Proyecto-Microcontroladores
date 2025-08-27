#include "mpu6050.h"

uint8_t mpu6050_ready(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef ret;
    // Verificar disponibilidad
    ret = HAL_I2C_IsDeviceReady(hi2c, MPU6050_ADDR, 3, 50);
    if (ret != HAL_OK) {
    	return MPU6050_ERR_NO_I2C;
    }
    return MPU6050_OK;
}

// Inicialización básica del sensor
uint8_t mpu6050_init(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef ret;
    uint8_t check = 0;
    uint8_t temp_data = 0;

    // Verificar identidad
    ret = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, MPU6050_I2C_TIMEOUT);
    if (ret != HAL_OK) return MPU6050_ERR_READ_REG;
    if (check != 0x68) return MPU6050_ERR_WHO_AM_I;

    // Salir de modo sleep
    ret = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_PWR_MGMT, 1, &temp_data, 1, MPU6050_I2C_TIMEOUT);
    if (ret != HAL_OK) return MPU6050_ERR_PWR_MGMT_WRITE;

    return MPU6050_OK;
}

// Configuración de acelerómetro, giroscopio y filtro
uint8_t mpu6050_config(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef ret;
    uint8_t temp_data;

    // Configurar giroscopio
    temp_data = FS_GYRO_250;
    ret = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_CONFIG_GYRO, 1, &temp_data, 1, MPU6050_I2C_TIMEOUT);
    if (ret != HAL_OK) return MPU6050_ERR_GYRO_CONFIG;

    // Configurar acelerómetro
    temp_data = FS_ACC_2G;
    ret = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_CONFIG_ACC, 1, &temp_data, 1, MPU6050_I2C_TIMEOUT);
    if (ret != HAL_OK) return MPU6050_ERR_ACC_CONFIG;

    // Configurar filtro digital pasa-bajos
    temp_data = LPF_44;
    ret = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_CONFIG, 1, &temp_data, 1, MPU6050_I2C_TIMEOUT);
    if (ret != HAL_OK) return MPU6050_ERR_FILTER_CONFIG;

    return MPU6050_OK;
}

// Lectura de datos crudos del sensor
uint8_t mpu6050_read(I2C_HandleTypeDef *hi2c,
                     int16_t* x_acc, int16_t* y_acc, int16_t* z_acc,
                     int16_t* x_gyro, int16_t* y_gyro, int16_t* z_gyro) {
    HAL_StatusTypeDef ret;
    uint8_t data[14];

    // Leer 14 bytes desde el registro 0x3B
    ret = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, REG_DATA_START, 1, data, 14, MPU6050_I2C_TIMEOUT);
    if (ret != HAL_OK) return MPU6050_ERR_READ_DATA;

    // Acelerómetro
    *x_acc = ((int16_t)data[0] << 8) | data[1];
    *y_acc = ((int16_t)data[2] << 8) | data[3];
    *z_acc = ((int16_t)data[4] << 8) | data[5];

    // Giroscopio
    *x_gyro = ((int16_t)data[8] << 8) | data[9];
    *y_gyro = ((int16_t)data[10] << 8) | data[11];
    *z_gyro = ((int16_t)data[12] << 8) | data[13];

    return MPU6050_OK;
}

// Reset total del sensor
uint8_t mpu6050_reset(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef ret;
    uint8_t temp_data = RESET_SENSOR;

    ret = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_PWR_MGMT, 1, &temp_data, 1, MPU6050_I2C_TIMEOUT);
    return (ret == HAL_OK) ? MPU6050_OK : MPU6050_ERR_RESET;
}

// Conversión giroscopio raw a °/s
float mpu6050_gyro_raw_to_dps(int16_t raw_value) {
    return ((float)raw_value) / 131.0f;  // Para ±250°/s
}
