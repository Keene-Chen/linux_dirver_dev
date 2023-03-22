/**
 * @file    : timer.c
 * @author  : KeeneChen
 * @date    : 2023.03.16-18:02:32
 * @details : timer
 */

/**
 *
 * 处理32位绕回的API函数
 * unkown 通常为 jiffies, known 通常是需要对比的值
 ** time_after(unkown, known)
 ** time_before(unkown, known)
 ** time_after_eq(unkown, known)
 ** time_before_eq(unkown, known)
 *
 * 将 jiffies 类型的参数 j 分别转换为对应的毫秒、微秒、纳秒
 ** int jiffies_to_msecs(const unsigned long j)
 ** int jiffies_to_usecs(const unsigned long j)
 ** u64 jiffies_to_nsecs(const unsigned long j)
 * 将毫秒、微秒、纳秒转换为 jiffies 类型
 ** long msecs_to_jiffies(const unsigned int m)
 ** long usecs_to_jiffies(const unsigned int u)
 ** unsigned long nsecs_to_jiffies(u64 n)
 *
 * 定时器 timer_list 结构体
 * struct timer_list {
 *      struct list_head entry;
 *      unsigned long expires;      // 定时器超时时间,单位是节拍数
 *      struct tvec_base *base;     // 定时处理函数
 *      void (*function)(unsigned long);
 *      unsigned long data;         // 要传递给 function 函数的参数
 *      int slack;
 *   };
 *
 * 操作定时器的API函数
 ** void init_timer(struct timer_list *timer)
 * @brief 负责初始化 timer_list 类型变量
 * @param timer 要初始化定时器
 *
 ** void add_timer(struct timer_list *timer)
 * @brief 用于向 Linux 内核注册定时器
 * @param timer 要注册的定时器
 *
 ** int del_timer(struct timer_list * timer)
 * @brief 用于删除一个定时器
 * @param timer 要删除的定时器
 * @return 0,定时器还没被激活; 1,定时器已经激活
 *
 ** int del_timer_sync(struct timer_list * timer)
 * @brief del_timer的同步版本,会等待其他处理器使用完定时器再删除,del_timer_sync
 *不能使用在中断上下文中
 * @param timer 要删除的定时器
 * @return 0,定时器还没被激活; 1,定时器已经激活
 *
 ** int mod_timer(struct timer_list *timer, unsigned long expires)
 * @brief 用于修改定时值,如果定时器还没有激活的话, mod_timer 函数会激活定时器
 * @param timer 要修改超时时间(定时值)的定时器
 * @param expires 修改后的超时时间
 * @return 0,调用 mod_timer 函数前定时器未被激活; 1,调用 mod_timer 函数前定时器已被激活
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define TIMER_NAME    "timer"            // 设备名
#define TIMER_COUNT   1                  // 设备个数
#define CMD_OPEN      _IO(0xEF, 1)       // 打开命令
#define CMD_CLOSE     _IO(0xEF, 2)       // 关闭命令
#define CMD_SETPERIOD _IOW(0xEF, 3, int) // 设置周期命令

/* 驱动设备结构体 */
struct timer_dev {
    dev_t devid;              // 设备号
    int major;                // 主设备号
    int minor;                // 次设备号
    struct cdev cdev;         // 字符设备
    struct class* class;      // 逻辑类
    struct device* device;    // 设备
    struct device_node* node; // 设备节点
    int led_gpio;             // led所使用的GPIO编号
    struct timer_list timer;  // 定时器
    int period;               // 定时器周期ms
    spinlock_t lock;          // 自旋锁
} timer;

static int timer_open(struct inode* inode, struct file* filp)
{
    // 设置私有数据
    filp->private_data = &timer;
    return 0;
}

static int timer_release(struct inode* inode, struct file* filp)
{
    return 0;
}

static long timer_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    int rc    = 0;
    int value = 0;
    unsigned long flags;                                           // 中断状态标识
    struct timer_dev* dev = (struct timer_dev*)filp->private_data; // 获取私有数据

    switch (cmd) {
    case CMD_OPEN:
        printk("open timer\r\n");
        mod_timer(&dev->timer, jiffies + msecs_to_jiffies(dev->period));
        break;
    case CMD_CLOSE:
        printk("close timer\r\n");
        del_timer_sync(&dev->timer);
        break;
    case CMD_SETPERIOD:
        rc = copy_from_user(&value, (int*)arg, sizeof(int));
        if (rc < 0)
            return rc;

        spin_lock_irqsave(&dev->lock, flags);      /* 加锁 */
        dev->period = value;                       /* 设置周期 */
        spin_unlock_irqrestore(&dev->lock, flags); /* 释放锁 */

        printk("timeperiod = %d\r\n", dev->period);
        mod_timer(&dev->timer, jiffies + msecs_to_jiffies(dev->period)); /* 打开定时器 */

    default:
        break;
    }

    return rc;
}

static const struct file_operations timer_fops = {
    .owner          = THIS_MODULE,
    .open           = timer_open,
    .release        = timer_release,
    .unlocked_ioctl = timer_ioctl,
};

/* 定时器回调函数 */
static void timer_cb(unsigned long arg)
{
    struct timer_dev* dev = (struct timer_dev*)arg; // 定时器初始化时传入的参数
    unsigned long expires = jiffies + msecs_to_jiffies(dev->period); // 定时器周期超时时间
    static int status     = 1;

    status = !status;
    gpio_set_value(dev->led_gpio, status);

    mod_timer(&dev->timer, expires); // 重设定时器初值
}

/* 初始化LED灯 */
int led_init(struct timer_dev* dev)
{
    int rc = 0;

    // 1.根据设备树路径获取设备节点
    dev->node = of_find_node_by_path("/gpioled");
    if (dev->node == NULL) {
        rc = -EINVAL;
        printk("node found\r\n");
        goto fail_node;
    }

    // 2.获取led所对应的GPIO
    dev->led_gpio = of_get_named_gpio(dev->node, "led-gpio", 0);
    if (dev->led_gpio < 0) {
        rc = -EINVAL;
        printk("led gpio not found\r\n");
        goto fail_led_gpio;
    }
    printk("led gpio= %d\r\n", dev->led_gpio);

    // 3.申请IO
    rc = gpio_request(dev->led_gpio, "led-gpio");
    if (rc) {
        rc = -EINVAL;
        pr_err("failed to request the led gpio\r\n");
        goto fail_gpio_req;
    }

    // 4.使用IO,设置为输出关灯
    rc = gpio_direction_output(dev->led_gpio, 1);
    if (rc) {
        goto fail_gpio_out;
    }

fail_gpio_out:
    gpio_free(dev->led_gpio);
fail_gpio_req:
fail_led_gpio:
fail_node:
    return rc;
}

/* 驱动入口函数 */
static int __init timer_init(void)
{
    int rc = 0;

    /* 注册字符设备 */
    // 1.分配设备号
    timer.major = 0; // 内核指定分配设备号
    if (timer.major) {
        timer.devid = MKDEV(timer.major, 0);
        rc          = register_chrdev_region(timer.devid, TIMER_COUNT, TIMER_NAME);
    }
    else {
        rc          = alloc_chrdev_region(&timer.devid, 0, TIMER_COUNT, TIMER_NAME);
        timer.major = MAJOR(timer.devid);
        timer.minor = MINOR(timer.devid);
    }
    if (rc < 0) {
        printk("timer chrdev region error!\r\n");
        goto fail_devid;
    }
    printk("major=%d,minor=%d\r\n", timer.major, timer.minor);

    // 2.注册字符设备
    timer.cdev.owner = THIS_MODULE;
    cdev_init(&timer.cdev, &timer_fops);
    rc = cdev_add(&timer.cdev, timer.devid, TIMER_COUNT);
    if (rc < 0) {
        rc = -EINVAL;
        goto fail_cdev;
    }

    // 3.动态创建设备的逻辑类
    timer.class = class_create(THIS_MODULE, TIMER_NAME);
    if (IS_ERR(timer.class)) {
        rc = PTR_ERR(timer.class);
        goto fail_class;
    }

    // 4.自动创建设备节点
    timer.device = device_create(timer.class, NULL, timer.devid, NULL, TIMER_NAME);
    if (IS_ERR(timer.device)) {
        rc = PTR_ERR(timer.device);
        goto fail_device;
    }
    printk("timer dirver init\r\n");

    /* 初始化LED */
    rc = led_init(&timer);
    if (rc < 0) {
        printk("LED init failed");
        goto fail_led;
    }

    /* 初始化定时器 */
    spin_lock_init(&timer.lock); // 初始化自旋锁
    init_timer(&timer.timer);
    timer.timer.function = timer_cb;                         // 定时器回调函数
    timer.timer.expires  = jiffies + msecs_to_jiffies(2000); // 毫秒转节拍数
    timer.period         = 1000;
    timer.timer.data     = (unsigned long)&timer; // 给回调函数传参
    // add_timer(&timer.timer);                      // 添加定时器到内核

    return rc;

fail_led:
    device_destroy(timer.class, timer.devid);
fail_device:
    class_destroy(timer.class);
fail_class:
    cdev_del(&timer.cdev);
fail_cdev:
    unregister_chrdev_region(timer.devid, TIMER_COUNT);
fail_devid:
    return rc;
}

/* 驱动出口函数 */
static void __exit timer_exit(void)
{
    // 1.删除定时器
    del_timer(&timer.timer);
    // 2.关灯
    gpio_set_value(timer.led_gpio, 1);
    // 3.释放IO
    gpio_free(timer.led_gpio);
    // 4.删除字符设备
    cdev_del(&timer.cdev);
    // 5.注销设备号
    unregister_chrdev_region(timer.devid, TIMER_COUNT);
    // 6.销毁设备节点
    device_destroy(timer.class, timer.devid);
    // 7.销毁设备的逻辑类
    class_destroy(timer.class);

    printk("timer dirver exit\r\n");
}

/* 驱动注册与卸载 */
module_init(timer_init);
module_exit(timer_exit);
MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("timer");