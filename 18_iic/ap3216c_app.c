/**
 * @file    : ap3216c_app.c
 * @author  : KeeneChen
 * @date    : 2023.04.17-19:00:29
 * @details : ap3216c_app
 */

#include <ctype.h>
#include <fcntl.h>
#include <linux/input.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/* 定义一个 input_event 变量，存放输入事件信息 */
static struct input_event input_event;

int main(int argc, char const* argv[])
{
    int ret = 0;
    unsigned short databuf[3];
    unsigned short ir, als, ps;

    if (argc != 2) {
        printf("Error Usage!\r\n");
        return -1;
    }

    int fd = open(argv[1], O_RDWR);
    if (fd < 0) {
        printf("file %s open failed!\r\n", argv[1]);
        return -1;
    }

    while (1) {
        ret = read(fd, databuf, sizeof(databuf));
        if (ret == 0) {       /* 数据读取成功 */
            ir  = databuf[0]; /* ir传感器数据 */
            als = databuf[1]; /* als传感器数据 */
            ps  = databuf[2]; /* ps传感器数据 */
            printf("ir = %d, als = %d, ps = %d\r\n", ir, als, ps);
        }
        usleep(200000); /*100ms */
    }

    ret = close(fd);
    if (ret < 0) {
        printf("file %s close failed!\r\n", argv[1]);
        return -1;
    }

    return 0;
}