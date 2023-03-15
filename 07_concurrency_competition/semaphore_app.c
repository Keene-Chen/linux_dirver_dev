/**
 * @file    : spinlock_app.c
 * @author  : KeeneChen
 * @date    : 2023.03.14-15:48:25
 * @details : spinlock_app
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
    if (argc != 3) {
        printf("Error Usage!\r\n");
        return -1;
    }

    int fd = open(argv[1], O_RDWR);
    if (fd < 0) {
        printf("file %s open failed!\r\n", argv[1]);
        return -1;
    }

    unsigned char data_buf[1];
    data_buf[0] = atoi(argv[2]);

    int ret = write(fd, data_buf, sizeof(data_buf));
    if (ret < 0) {
        printf("LED Control Failed!\r\n");
        close(fd);
        return -1;
    }

    /* 模拟应用占用驱动10s */
    int cnt = 0;
    while (1) {
        sleep(5);
        cnt++;
        printf("App runing times:%d\r\n", cnt);
        if (cnt >= 2)
            break;
    }
    printf("App runing finished\r\n");

    ret = close(fd);
    if (ret < 0) {
        printf("file %s close failed!\r\n", argv[1]);
        return -1;
    }

    return 0;
}