/**
 * @file    : nonblock_io_app.c
 * @author  : KeeneChen
 * @date    : 2023.04.02-20:05:57
 * @details : nonblock_io_app
 */

#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/poll.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
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

#if defined(SELECT)
    fd_set readfds;
    struct timeval timeout;

    while (1) {
        FD_ZERO(&readfds);    // 清除readfds
        FD_SET(fd, &readfds); // 将fd添加到readfds集合中
        timeout.tv_sec  = 0;
        timeout.tv_usec = 5000000; // 500ms

        ret = select(fd + 1, &readfds, NULL, NULL, &timeout);
        switch (ret) {
        case 0: // 超时
            printf("timeout!\r\n");
            break;
        case -1: // 错误
            printf("error!\r\n");
            break;
        default:                                     // 可以读取数据
            if (FD_ISSET(fd, &readfds)) {            // 判断是否为 fd 文件描述符
                ret = read(fd, &data, sizeof(data)); // 使用 read 函数读取数据
                if (ret < 0) {                       // 数据读取错误或者无效
                }
                else {        // 数据读取正确
                    if (data) // 读取到数据
                        printf("key value = %#x\r\n", data);
                }
            }
            break;
        }
    }
#elif defined(POLL)
    struct pollfd fds = { .fd = fd, .events = POLLIN };

    while (1) {
        ret = poll(&fds, 1, 500);
        if (ret == 0) { // 超时
            printf("timeout\n");
        }
        else if (ret < 0) { // 错误
            printf("poll error\n");
        }
        else {
            if (fds.events | POLLIN) { // 判断poll是否可读取
                read(fd, &data, sizeof(data));
                printf("key value = %#x\r\n", data);
            }
        }
    }
#else
    int nfds, epollfd;
    struct epoll_event ev, events[10];

    // 创建 epoll 实例
    epollfd = epoll_create1(0);
    if (epollfd == -1) {
        perror("epoll_create1");
        exit(EXIT_FAILURE);
    }

    // 配置要监听的事件类型
    ev.events  = EPOLLIN;
    ev.data.fd = fd;

    // 注册要监听的文件描述符和事件
    ret = epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &ev);
    if (ret == -1) {
        perror("epoll_ctl");
        exit(EXIT_FAILURE);
    }
    // 进入事件循环
    while (1) {
        nfds = epoll_wait(epollfd, events, 10, -1);
        if (nfds == -1) {
            perror("epoll_wait");
            exit(EXIT_FAILURE);
        }

        // 处理所有就绪事件
        for (int n = 0; n < nfds; ++n) {
            if (events[n].data.fd == fd) {
                // 从文件描述符中读取数据
                read(fd, &data, sizeof(data));
                printf("key value = %#x\r\n", data);
            }
        }
    }

    // 关闭 epoll 实例和文件描述符
    close(epollfd);

#endif

    ret = close(fd);
    if (ret < 0) {
        printf("file %s close failed!\r\n", argv[1]);
        return -1;
    }

    return 0;
}