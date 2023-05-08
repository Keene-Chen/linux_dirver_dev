/**
 * @file    : led_app.c
 * @author  : KeeneChen
 * @date    : 2023.04.03-19:16:26
 * @details : led_app
 */

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
    if (argc != 2) {
        printf("Error Usage!\r\n");
        return -1;
    }

    int fd = open(argv[1], O_RDWR);
    if (fd < 0) {
        printf("file %s open failed!\r\n", argv[1]);
        return -1;
    }

    unsigned char data[5] = { 0, 0, 0, 0, 0 };
    while (1) {
        read(fd, &data, sizeof(data));
        printf("humi:%d,%d\n", data[0], data[1]);
        printf("temp:%d,%d\n\n", data[2], data[3]);
        usleep(200000);
    }

    int ret = close(fd);
    if (ret < 0) {
        printf("file %s close failed!\r\n", argv[1]);
        return -1;
    }

    return 0;
}