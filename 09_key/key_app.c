/**
 * @file    : key_app.c
 * @author  : KeeneChen
 * @date    : 2023.03.15-13:52:21
 * @details : key_app
 */

#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define KEY_VALUE   0xF0 // 按键有效值
#define KEY_INVALID 0x00 // 按键无效值

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

    /**
     * 循环读取按键值
     * ?warn 此处采用阻塞IO,实际开发并不适用
     */
    int key_val;
    while (1) {
        read(fd, &key_val, sizeof(key_val));
        if (key_val == KEY_VALUE)
            printf("KEY0 Press, value = %#X\r\n", key_val);
    }

    int ret = close(fd);
    if (ret < 0) {
        printf("file %s close failed!\r\n", argv[1]);
        return -1;
    }

    return 0;
}