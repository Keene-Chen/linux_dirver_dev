#include "mpu6050.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#define m 0.0002663
#define n 0.0001211

/**
 * @description        : main主程序
 * @param - argc     : argv数组元素个数
 * @param - argv     : 具体参数
 * @return             : 0 成功;其他 失败
 */
int main(int argc, char* argv[])
{
    int fd;
    char* filename;
    short databuf[7];
    // short a[3], g[3], tem;
    // unsigned char name, bu[1];
    struct mpu6050_data data;
    float temperature;
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
        if (ret == 0) {
            data.acc.x  = databuf[0];
            data.acc.y  = databuf[1];
            data.acc.z  = databuf[2];
            data.temp   = databuf[3];
            temperature = 36.53 + ((int)data.temp) / 340;
            data.anv.x  = databuf[4];
            data.anv.y  = databuf[5];
            data.anv.z  = databuf[6];
            printf("ax=%.4fg ay=%.4fg az=%.4fg\n", (float)(data.acc.x * n), (float)(data.acc.y * n),
                   (float)(data.acc.z * n));
            printf("gx=%.4fr/s gy=%.4fr/s gz=%.4fr/s\n", (float)(data.anv.x * m),
                   (float)(data.anv.y * m), (float)(data.anv.z * m));
            printf("temp=%.4f\n\n", temperature);
        }
        usleep(1000000); /*100ms */
    }

    close(fd); /* 关闭文件 */
    return 0;
}
