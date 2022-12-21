/**
 * @file    : chrdevbase.c
 * @author  : KeeneChen
 * @date    : 2022.12.20-13:24:48
 * @details : chrdevbase
 */

#include <linux/ide.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>

#define CHRDEVBASE_MAJOR 200          // 驱动设备号
#define CHRDEVBASE_NAME  "chrdevbase" // 驱动设备名
#define MAX_SIZE         100

static char read_buf[MAX_SIZE]  = { 0 }; // 驱动读缓冲区
static char write_buf[MAX_SIZE] = { 0 }; // 驱动写缓存区
static char kernel_data[]       = { "kernel data" };

/**
 * @brief 打开设备
 * @param inode 传递给驱动的inode
 * @param filp  设备文件，file结构体有个叫做private_data的成员变量
 *              一般在open的时候将private_data指向设备结构体。
 * @return 函数调用状态
 * @retval 0     成功
 * @retval other 失败
 */
static int chrdevbase_open(struct inode* inode, struct file* file)
{
    printk("chrdevbase_open\r\n");
    return 0;
}

/**
 * @brief 关闭/释放设备
 * @param filp 要关闭的设备文件(文件描述符)
 * @return 函数调用状态
 * @retval 0     成功
 * @retval other 失败
 */
static int chrdevbase_release(struct inode* inode, struct file* file)
{
    printk("chrdevbase_release\r\n");
    return 0;
}

/**
 * @brief 从设备读取数据
 * @param filp 要打开的设备文件(文件描述符)
 * @param buf  返回给用户空间的数据缓冲区
 * @param cnt  要读取的数据长度
 * @param offt 相对于文件首地址的偏移
 * @return 读取的字节数
 * @retval 负值表示读取失败
 */
static ssize_t chrdevbase_read(struct file* filp, char __user* buf, size_t count, loff_t* ppos)
{
    // printk("chrdevbase_read\r\n");
    int ret = 0;

    memcpy(read_buf, kernel_data, sizeof(kernel_data));
    ret = copy_to_user(buf, read_buf, count);
    if (ret == 0) {
        printk("kernel senddata success\r\n");
    }
    else {
        printk("kernel senddata failed\r\n");
    }

    return ret;
}

/**
 * @brief 向设备写数据
 * @param filp 设备文件，表示打开的文件描述符
 * @param buf  要写给设备写入的数据
 * @param cnt  要写入的数据长度
 * @param offt 相对于文件首地址的偏移
 * @return 写入的字节数
 * @retval 负值表示写入失败
 */
static ssize_t chrdevbase_write(struct file* filp, const char __user* buf, size_t count,
                                loff_t* ppos)
{
    // printk("chrdevbase_write\r\n");

    int ret = 0;

    ret = copy_from_user(write_buf, buf, count);
    if (ret == 0) {
        printk("kernel recedata:%s\r\n", write_buf);
    }
    else {
        printk("kernel recedata failed\r\n");
    }

    return ret;
}

/* 字符设备操作集合 */
static struct file_operations chrdevbase_fops = {
    .owner   = THIS_MODULE,
    .open    = chrdevbase_open,
    .release = chrdevbase_release,
    .read    = chrdevbase_read,
    .write   = chrdevbase_write,
};

/**
 * @brief  驱动入口函数
 * @return 函数调用状态
 * @retval 0     成功
 * @retval other 失败
 */
static int __init chrdevbase_init(void)
{
    int ret = 0;
    printk("chrdevbase_init\r\n");

    ret = register_chrdev(CHRDEVBASE_MAJOR, CHRDEVBASE_NAME, &chrdevbase_fops);
    if (ret < 0) {
        printk("chrdevbase failed\r\n");
    }

    return ret;
}

/**
 * @brief  驱动出口函数
 * @return void
 */
static void __exit chrdevbase_exit(void)
{
    printk("chrdevbase_exit\r\n");
    unregister_chrdev(CHRDEVBASE_MAJOR, CHRDEVBASE_NAME);
}

/* 内核驱动模块加载与卸载 */
module_init(chrdevbase_init);
module_exit(chrdevbase_exit);

/* 内核驱动模块信息 */
MODULE_DESCRIPTION("字符设备驱动");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("KeeneChen");