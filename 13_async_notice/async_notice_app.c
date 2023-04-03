/**
 * @file    : async_notice_app.c
 * @author  : KeeneChen
 * @date    : 2023.04.03-10:16:55
 * @details : async_notice_app
 */

#include <ctype.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

static int fd = 0;

static void sigio_signal_func(int signum)
{
    int err               = 0;
    unsigned int keyvalue = 0;

    err = read(fd, &keyvalue, sizeof(keyvalue));
    if (err < 0) {
        // 读取错误
    }
    else {
        printf("sigio signal! key value=%d\r\n", keyvalue);
    }
}

int main(int argc, char const* argv[])
{
    int flags = 0;

    if (argc != 2) {
        printf("Error Usage!\r\n");
        return -1;
    }

    fd = open(argv[1], O_RDWR);
    if (fd < 0) {
        printf("file %s open failed!\r\n", argv[1]);
        return -1;
    }

    /* 设置信号SIGIO的处理函数 */
    signal(SIGIO, sigio_signal_func);

    fcntl(fd, F_SETOWN, getpid());      // 设置当前进程接收SIGIO信号
    flags = fcntl(fd, F_GETFL);         // 获取当前的进程状态
    fcntl(fd, F_SETFL, flags | FASYNC); // 设置进程启用异步通知功能

    while (1) {
        sleep(2);
    }

    int ret = close(fd);
    if (ret < 0) {
        printf("file %s close failed!\r\n", argv[1]);
        return -1;
    }

    return 0;
}