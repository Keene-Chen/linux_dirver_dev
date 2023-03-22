/**
 * @file    : timer_app.c
 * @author  : KeeneChen
 * @date    : 2023.03.16-18:02:25
 * @details : timer_app
 */

#include <ctype.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define CMD_OPEN      _IO(0xEF, 1)       // 打开命令
#define CMD_CLOSE     _IO(0xEF, 2)       // 关闭命令
#define CMD_SETPERIOD _IOW(0xEF, 3, int) // 设置周期命令

int main(int argc, char const* argv[])
{
    int ret       = 0;
    uint32_t cmd  = 0;
    uint32_t arg  = 0;
    char str[100] = { "0" };

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
        printf("input cmd: ");
        ret = scanf("%d", &cmd);
        printf("\r\n");
        if (ret != 1) {
            // gets(str); /* 防止卡死 */
        }

        switch (cmd) {
        case 1:
            ioctl(fd, CMD_OPEN, &arg);
            break;
        case 2:
            ioctl(fd, CMD_CLOSE, &arg);
            break;
        case 3:
            printf("input timer period: ");
            ret = scanf("%d", &arg);
            printf("\r\n");
            if (ret != 1) {
                // gets(str); /* 防止卡死 */
            }
            ret = ioctl(fd, CMD_SETPERIOD, &arg);
            printf("input end\r\n");
            if (ret < 0) {
                close(fd);
                printf("input err\r\n");
                return -1;
            }
            break;
        default:
            break;
        }
        if (cmd == 0)
            break;
    }

    if (close(fd) < 0) {
        printf("file %s close failed!\r\n", argv[1]);
        return -1;
    }

    return 0;
}