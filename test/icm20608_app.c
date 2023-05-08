
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

int main(int argc, char* argv[])
{
    int ret = 0;
    int fd;
    char* filename;
    signed int databuf[7];
    unsigned char data[14];
    signed int gyro_x_adc, gyro_y_adc, gyro_z_adc;
    signed int accel_x_adc, accel_y_adc, accel_z_adc;
    signed int temp_adc;

    float gyro_x_act, gyro_y_act, gyro_z_act;
    float accel_x_act, accel_y_act, accel_z_act;
    float temp_act;
    if (argc != 3) {
        printf("Error uargc\r\n");
        return -1;
    }

    filename = argv[1];
    fd       = open(filename, O_RDWR);
    if (fd < 0) {
        printf(" open file %s failed\r\n", filename);
        return -1;
    }

    while (1) {
        ret = read(fd, databuf, sizeof(databuf));
        if (ret == 0) /* 数据读取成功 */
        {
            gyro_x_adc  = databuf[0];
            gyro_y_adc  = databuf[1];
            gyro_z_adc  = databuf[2];
            accel_x_adc = databuf[3];
            accel_y_adc = databuf[4];
            accel_z_adc = databuf[5];
            temp_adc    = databuf[6];
            /* 计算实际值 */
            gyro_x_act  = (float)(gyro_x_adc) / 16.4;
            gyro_y_act  = (float)(gyro_y_adc) / 16.4;
            gyro_z_act  = (float)(gyro_z_adc) / 16.4;
            accel_x_act = (float)(accel_x_adc) / 2048;
            accel_y_act = (float)(accel_y_adc) / 2048;
            accel_z_act = (float)(accel_z_adc) / 2048;
            temp_act    = ((float)(temp_adc)-25) / 326.8 + 25;

            printf("\r\n 原始值:\r\n");
            printf("gx = %d, gy = %d, gz = %d\r\n", gyro_x_adc, gyro_y_adc, gyro_z_adc);
            printf("ax = %d, ay = %d, az = %d\r\n", accel_x_adc, accel_y_adc, accel_z_adc);
            printf("temp = %d\r\n", temp_adc);

            printf("实际值:");
            printf("act gx = %.2f°/S, act gy = %.2f°/S,act gz = %.2f°/S\r\n", gyro_x_act,
                   gyro_y_act, gyro_z_act);
            printf("act ax = %.2fg, act ay = %.2fg,act az = %.2fg\r\n", accel_x_act, accel_y_act,
                   accel_z_act);
            printf("act temp = %.2f°C\r\n", temp_act);
        }
        usleep(100000); /*100ms */
    }

    ret = close(fd);
    if (ret < 0) {
        printf(" close file %s failed\r\n", filename);
        return -1;
    }

    return 0;
}