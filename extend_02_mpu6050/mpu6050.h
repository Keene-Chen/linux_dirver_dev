#ifndef __MPU6050_H__
#define __MPU6050_H__

/* MPU6050 register map */
#if 0
#define AUX_VDDIO        0x01 // 1
#define SMPLRT_DIV       0x19 // 25
#define CONFIG           0x1A // 26
#define GYRO_CONFIG      0x1B // 27
#define ACCEL_CONFIG     0x1C // 28
#define FF_THR           0x1D // 29
#define FF_DUR           0x1E // 30
#define MOT_THR          0x1F // 31
#define MOT_DUR          0x20 // 32
#define ZRMOT_THR        0x21 // 33
#define ZRMOT_DUR        0x22 // 34
#define FIFO_EN          0x23 // 35
#define I2C_MST_CTRL     0x24 // 36
#define I2C_SLV0_ADDR    0x25 // 37
#define I2C_SLV0_REG     0x26 // 38
#define I2C_SLV0_CTRL    0x27 // 39
#define I2C_SLV1_ADDR    0x28 // 40
#define I2C_SLV1_REG     0x29 // 41
#define I2C_SLV1_CTRL    0x2A // 42
#define I2C_SLV2_ADDR    0x2B // 43
#define I2C_SLV2_REG     0x2C // 44
#define I2C_SLV2_CTRL    0x2D // 45
#define I2C_SLV3_ADDR    0x2E // 46
#define I2C_SLV3_REG     0x2F // 47
#define I2C_SLV3_CTRL    0x30 // 48
#define I2C_SLV4_ADDR    0x31 // 49
#define I2C_SLV4_REG     0x32 // 50
#define I2C_SLV4_DO      0x33 // 51
#define I2C_SLV4_CTRL    0x34 // 52
#define I2C_SLV4_DI      0x35 // 53
#define I2C_MST_STATUS   0x36 // 54
#define INT_PIN_CFG      0x37 // 55
#define INT_ENABLE       0x38 // 56
#define INT_STATUS       0x3A // 58
#define ACCEL_XOUT_H     0x3B // 59
#define ACCEL_XOUT_L     0x3C // 60
#define ACCEL_YOUT_H     0x3D // 61
#define ACCEL_YOUT_L     0x3E // 62
#define ACCEL_ZOUT_H     0x3F // 63
#define ACCEL_ZOUT_L     0x40 // 64
#define TEMP_OUT_H       0x41 // 65
#define TEMP_OUT_L       0x42 // 66
#define GYRO_XOUT_H      0x43 // 67
#define GYRO_XOUT_L      0x44 // 68
#define GYRO_YOUT_H      0x45 // 69
#define GYRO_YOUT_L      0x46 // 70
#define GYRO_ZOUT_H      0x47 // 71
#define GYRO_ZOUT_L      0x48 // 72
#define EXT_SENS_DATA_00 0x49 // 73
#define EXT_SENS_DATA_01 0x4A // 74
#define EXT_SENS_DATA_02 0x4B // 75
#define EXT_SENS_DATA_03 0x4C // 76
#define EXT_SENS_DATA_04 0x4D // 77
#define EXT_SENS_DATA_05 0x4E // 78
#define EXT_SENS_DATA_06 0x4F // 79
#define EXT_SENS_DATA_07 0x50 // 80
#define EXT_SENS_DATA_08 0x51 // 81
#define EXT_SENS_DATA_09 0x52 // 82
#define EXT_SENS_DATA_10 0x53 // 83
#define EXT_SENS_DATA_11 0x54 // 84
#define EXT_SENS_DATA_12 0x55 // 85
#define EXT_SENS_DATA_13 0x56 // 86
#define EXT_SENS_DATA_14 0x57 // 87
#define EXT_SENS_DATA_15 0x58 // 88
#define EXT_SENS_DATA_16 0x59 // 89
#define EXT_SENS_DATA_17 0x5A // 90
#define EXT_SENS_DATA_18 0x5B // 91
#define EXT_SENS_DATA_19 0x5C // 92
#define EXT_SENS_DATA_20 0x5D // 93
#define EXT_SENS_DATA_21 0x5E // 94
#define EXT_SENS_DATA_22 0x5F // 95
#define EXT_SENS_DATA_23 0x60 // 96
#define MOT_DETECT_STAT  0x61 // 97
#define I2C_SLV0_DO      0x63 // 99
#define I2C_SLV1_DO      0x64 // 100
#define I2C_SLV2_DO      0x65 // 101
#define I2C_SLV3_DO      0x66 // 102
#define I2C_MST_DELAY_CT 0x67 // 103
#define SIGNAL_PATH_RES  0x68 // 104
#define MOT_DETECT_CTRL  0x69 // 105
#define USER_CTRL        0x6A // 106
#define PWR_MGMT_1       0x6B // 107
#define PWR_MGMT_2       0x6C // 108
#define FIFO_COUNTH      0x72 // 114
#define FIFO_COUNTL      0x73 // 115
#define FIFO_R_W         0x74 // 116
#define WHO_AM_I         0x75 // 117
#endif

/* register define */
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_TEMP_OUT_L   0x42
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48
#define MPU6050_PWR_MGMT_1   0x6B /* power managemnt register */
#define MPU6050_PWR_MGMT_2   0X6C
#define MPU6050_WHO_AM_I     0x75 /* default value :  IIC addr 0x68 */
#define MPU_FIFO_EN_REG      0X23

/* interrupt status register */
#define MPU6050_INT_STATUS  0x3A
#define MPU6050_INT_ENABLE  0x38
#define MPU6050_INT_PIN_CFG 0x37

/* 加速度 */
struct acc {
    short x;
    short y;
    short z;
};

/* 角速度 */
struct anv {
    short x;
    short y;
    short z;
};

struct mpu6050_data {
    struct acc acc;
    struct anv anv;
    short temp;
    unsigned char name;
};

#endif // __MPU6050_H__