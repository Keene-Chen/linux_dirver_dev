#include "fcntl.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "sys/ioctl.h"
#include "sys/stat.h"
#include "sys/types.h"
#include "unistd.h"
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/time.h>

/* 字符串转数字，将浮点小数字符串转换为浮点数数值 */
#define SENSOR_FLOAT_DATA_GET(ret, index, str, member)   \
    ret         = file_data_read(file_path[index], str); \
    dev->member = atof(str);

/* 字符串转数字，将整数字符串转换为整数数值 */
#define SENSOR_INT_DATA_GET(ret, index, str, member)     \
    ret         = file_data_read(file_path[index], str); \
    dev->member = atoi(str);

/* adc iio框架对应的文件路径 */
static char* file_path[] = {
    "/sys/bus/iio/devices/iio:device0/in_voltage_scale",
    "/sys/bus/iio/devices/iio:device0/in_voltage1_raw",
};

/* 文件路径索引，要和file_path里面的文件顺序对应 */
enum path_index {
    IN_VOLTAGE_SCALE = 0,
    IN_VOLTAGE_RAW,
};

/*
 * ADC数据设备结构体
 */
struct adc_dev {
    int raw;
    float scale;
    float act;
};

struct adc_dev imx6ulladc;

/*
 * @description			: 读取指定文件内容
 * @param - filename 	: 要读取的文件路径
 * @param - str 		: 读取到的文件字符串
 * @return 				: 0 成功;其他 失败
 */
static int file_data_read(char* filename, char* str)
{
    int ret = 0;
    FILE* data_stream;

    data_stream = fopen(filename, "r"); /* 只读打开 */
    if (data_stream == NULL) {
        printf("can't open file %s\r\n", filename);
        return -1;
    }

    ret = fscanf(data_stream, "%s", str);
    if (!ret) {
        printf("file read error!\r\n");
    }
    else if (ret == EOF) {
        /* 读到文件末尾的话将文件指针重新调整到文件头 */
        fseek(data_stream, 0, SEEK_SET);
    }
    fclose(data_stream); /* 关闭文件 */
    return 0;
}

/*
 * @description	: 获取ADC数据
 * @param - dev : 设备结构体
 * @return 		: 0 成功;其他 失败
 */
static int adc_read(struct adc_dev* dev)
{
    int ret = 0;
    char str[50];

    SENSOR_FLOAT_DATA_GET(ret, IN_VOLTAGE_SCALE, str, scale);
    SENSOR_INT_DATA_GET(ret, IN_VOLTAGE_RAW, str, raw);

    /* 转换得到实际电压值mV */
    dev->act = (dev->scale * dev->raw) / 1000.f;
    return ret;
}

int main(int argc, char* argv[])
{
    int ret   = 0;
    int count = 0;
    int avg   = 0;

    if (argc != 1) {
        printf("Error Usage!\r\n");
        return -1;
    }

    /**
     * MATLIB 曲线拟合生成公式
     * 常规模型 Exp2:
     * f(x) = a*exp(b*x) + c*exp(d*x)
     * 系数(置信边界为 95%):
     * a =   3.981e+04  (3.572e+04, 4.389e+04)
     * b =    -0.01534  (-0.01674, -0.01395)
     * c =        3467  (2584, 4349)
     * d =   -0.002379  (-0.002887, -0.00187)
     */
    float a = 39810;
    float b = -0.01534;
    float c = 3467;
    float d = -0.002379;

    while (1) {
        for (int i = 0; i < 20; i++) {
            ret = adc_read(&imx6ulladc);
            count += imx6ulladc.raw;
        }
        avg = count / 20;
        if (ret == 0) { /* 数据读取成功 */
            float lx = (a * exp(b * avg) + c * exp(d * avg));
            printf("ADC1_channels1:\r\n");
            printf("raw   : %d\r\n", avg);
            printf("vol   : %.3fV\r\n", imx6ulladc.act);
            printf("light : %.3fLx\r\n\n", lx);
        }
        count = 0;
        usleep(100000); /*100ms */
    }
    return 0;
}
