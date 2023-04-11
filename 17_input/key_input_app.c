/**
 * @file    : key_input_app.c
 * @author  : KeeneChen
 * @date    : 2023.04.10-14:55:13
 * @details : key_input_app
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
        ret = read(fd, &input_event, sizeof(input_event));
        if (ret > 0) {
            switch (input_event.type) {
            case EV_KEY: {
                if (input_event.code < BTN_MISC) { // 键盘键值
                    printf("key %#x %s\r\n", input_event.code,
                           input_event.value ? "press" : "release");
                }
                else {
                    printf("button %#x %s\r\n", input_event.code,
                           input_event.value ? "press" : "release");
                }
                break;
            }
            case EV_SYN:
                printf("key sync\r\n");
                break;
            case EV_REL:
                printf("key rel\r\n");
                break;
            case EV_ABS:
                break;
            case EV_MSC:
                break;
            case EV_SW:
                break;
            case EV_LED:
                break;
            case EV_SND:
                break;
            case EV_REP:
                break;
            case EV_FF:
                break;
            case EV_PWR:
                break;
            case EV_FF_STATUS:
                break;
            default:
                break;
            }
        }
        else {
            printf("按键读取失败\n");
        }
    }

    ret = close(fd);
    if (ret < 0) {
        printf("file %s close failed!\r\n", argv[1]);
        return -1;
    }

    return 0;
}