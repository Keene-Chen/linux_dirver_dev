/**
 * @file    : icm20608_app.c
 * @author  : KeeneChen
 * @date    : 2023.04.20-17:08:41
 * @details : icm20608_app
 */

#include "icm20608_reg.h"
#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

int main(int argc, char const* argv[])
{
    int fd;
    char* filename;
    signed int databuf[7];
    // unsigned char data[14];
    struct icm20608_data data;

    float gyro_x_act, gyro_y_act, gyro_z_act;
    float accel_x_act, accel_y_act, accel_z_act;
    float temp_act;

    int ret = 0;

    if (argc != 2) {
        printf("Error Usage!\r\n");
        return -1;
    }

    filename = argv[1];
    fd       = open(filename, O_RDWR);
    if (fd < 0) {
        printf("can't open file %s\r\n", filename);
        return -1;
    }

    while (1) {
        ret = read(fd, databuf, sizeof(databuf));
        if (ret == 0) { /* 数据读取成功 */
            data.gyro.x = databuf[0];
            data.gyro.y = databuf[1];
            data.gyro.z = databuf[2];
            data.acc.x  = databuf[3];
            data.acc.y  = databuf[4];
            data.acc.z  = databuf[5];
            data.temp   = databuf[6];

            /* 计算实际值 */
            data.gyro.x = (float)(data.gyro.x) / 16.4;
            data.gyro.y = (float)(data.gyro.y) / 16.4;
            data.gyro.z = (float)(data.gyro.z) / 16.4;
            data.acc.x  = (float)(data.acc.x) / 2048;
            data.acc.y  = (float)(data.acc.y) / 2048;
            data.acc.z  = (float)(data.acc.z) / 2048;
            data.temp   = ((float)(data.temp) - 25) / 326.8 + 25;

            printf("\r\n原始值:\r\n");
            printf("gx = %d, gy = %d, gz = %d\r\n", data.gyro.x, data.gyro.y, data.gyro.z);
            printf("ax = %d, ay = %d, az = %d\r\n", data.acc.x, data.acc.y, data.acc.z);
            printf("temp = %d\r\n", data.temp);
            printf("实际值:");
            printf("act gx = %.2f°/S, act gy = %.2f°/S, act gz = %.2f°/S\r\n", data.gyro.x,
                   data.gyro.y, data.gyro.z);
            printf("act ax = %.2fg, act ay = %.2fg, act az = %.2fg\r\n", data.acc.x, data.acc.y,
                   data.acc.z);
            printf("act temp = %.2f°C\r\n", data.temp);
        }
        usleep(100000); /* 100ms */
    }

    ret = close(fd);
    if (ret < 0) {
        printf("file %s close failed!\r\n", argv[1]);
        return -1;
    }

    return 0;
}