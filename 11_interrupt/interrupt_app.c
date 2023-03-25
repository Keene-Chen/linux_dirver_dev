/**
 * @file    : interrupt_app.c
 * @author  : KeeneChen
 * @date    : 2023.03.19-21:28:01
 * @details : interrupt_app
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

    int ret = 0;
    unsigned char data;
    while (1) {
        ret = read(fd, &data, sizeof(data));
        if (ret < 0) { // 数据读取错误或者无效
        }
        else {        // 数据读取正确
            if (data) // 读取到数据
                printf("key value = %#X\r\n", data);
        }
    }

    ret = close(fd);
    if (ret < 0) {
        printf("file %s close failed!\r\n", argv[1]);
        return -1;
    }

    return 0;
}