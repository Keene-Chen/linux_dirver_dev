/**
 * @file    : chrdevbase_app.c
 * @author  : KeeneChen
 * @date    : 2022.12.20-19:31:58
 * @details : chrdevbase_app
 * @note    : ./chrdevbase /dev/chrdevbase <1>|<2> 
 *            argv[2] 1:读文件 argv[2] 2:写文件
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define MAX_SIZE 100

static char user_data[] = { "user data" };

int main(int argc, char const* argv[])
{
    const char* filename     = argv[1];
    int ret                  = 0;
    char read_buf[MAX_SIZE]  = { 0 };
    char write_buf[MAX_SIZE] = { 0 };

    if (argc != 3) {
        printf("Error Usage!\r\n");
        exit(EXIT_FAILURE);
    }

    int fd = open(filename, O_RDWR);
    if (fd < 0) {
        perror("open failed\r\n");
        exit(EXIT_FAILURE);
    }

    /* 从驱动文件读取数据 */
    if (atoi(argv[2]) == 1) {
        ret = read(fd, read_buf, 50);
        if (ret < 0) {
            printf("read file %s failed!\r\n", filename);
        }
        else {
            printf("read data:%s\r\n", read_buf);
        }
    }

    /* 向设备驱动写数据 */
    if (atoi(argv[2]) == 2) {
        memcpy(write_buf, user_data, sizeof(user_data));
        ret = write(fd, write_buf, 50);
        if (ret < 0) {
            printf("write file %s failed!\r\n", filename);
        }
    }

    close(fd);

    return 0;
}
