/**
 * @file    : spinlock.c
 * @author  : KeeneChen
 * @date    : 2023.03.14-15:46:53
 * @details : spinlock
 */

/**
 * 自旋锁 API 函数
 ** DEFINE_SPINLOCK(spinlock_t lock)
 * @brief 定义并初始化一个自选变量
 ** int spin_lock_init(spinlock_t *lock)
 * @brief 初始化自旋锁
 ** void spin_lock(spinlock_t *lock)
 * @brief 获取指定的自旋锁,也叫做加锁
 ** void spin_unlock(spinlock_t *lock)
 * @brief 释放指定的自旋锁
 ** int spin_trylock(spinlock_t *lock)
 * @brief 尝试获取指定的自旋锁,如果没有获取到就返回 0
 ** int spin_is_locked(spinlock_t *lock)
 * @brief 检查指定的自旋锁是否被获取,如果没有被获取就返回非 0,否则返回 0
 ** void spin_lock_irq(spinlock_t *lock)
 * @brief 禁止本地中断,并获取自旋锁
 ** void spin_unlock_irq(spinlock_t *lock)
 * @brief 激活本地中断,并释放自旋锁
 ** void spin_lock_irqsave(spinlock_t *lock,unsigned long flags)
 * @brief 保存中断状态,禁止本地中断,并获取自旋锁
 ** void spin_unlock_irqrestore(spinlock_t*lock, unsigned long flags)
 * @brief 将中断状态恢复到以前的状态,并且激活本地中断,释放自旋锁
 ** void spin_lock_bh(spinlock_t *lock)
 * @brief 关闭下半部,并获取自旋锁
 ** void spin_unlock_bh(spinlock_t *lock)
 * @brief 打开下半部,并释放自旋锁
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define GPIOLED_NAME  "spinlock" // 设备名
#define GPIOLED_COUNT 1          // 设备个数
#define LED_ON        1          // 开灯
#define LED_OFF       0          // 关灯

/* 驱动设备结构体 */
struct gpioled_dev {
    dev_t devid;              // 设备号
    int major;                // 主设备号
    int minor;                // 次设备号
    struct cdev cdev;         // 字符设备
    struct class* class;      // 逻辑类
    struct device* device;    // 设备
    struct device_node* node; // 设备节点
    int led_gpio;             // led所使用的GPIO编号
    int dev_status;           // 设备状态,0,设备未使用,>0,设备已经被使用
    spinlock_t lock;          // 自旋锁
} gpioled;

static int gpioled_open(struct inode* inode, struct file* filp)
{
    unsigned long irq_flag;

    // 设置私有数据
    filp->private_data = &gpioled;

#if 0
    spin_lock(&gpioled.lock); // 加锁
    if (gpioled.dev_status) { // 驱动不能使用
        spin_unlock(&gpioled.lock);
        return -EBUSY;
    }
    gpioled.dev_status++;       // 标记驱动被使用
    spin_unlock(&gpioled.lock); // 解锁
#endif

    spin_lock_irqsave(&gpioled.lock, irq_flag); // 加锁,并保存中断状态
    if (gpioled.dev_status) {                   // 驱动不能使用
        spin_unlock(&gpioled.lock);
        return -EBUSY;
    }
    gpioled.dev_status++;                            // 标记驱动被使用
    spin_unlock_irqrestore(&gpioled.lock, irq_flag); // 解锁,并恢复中断状态

    return 0;
}

static int gpioled_release(struct inode* inode, struct file* filp)
{
    unsigned long irq_flag;

    // 取出私有数据
    struct gpioled_dev* dev = (struct gpioled_dev*)filp->private_data;
#if 0
    spin_lock(&dev->lock);
    if (dev->dev_status) {
        dev->dev_status--; // 标记驱动可以使用
    }
    spin_unlock(&dev->lock);
#endif

    spin_lock_irqsave(&dev->lock, irq_flag);
    if (dev->dev_status) {
        dev->dev_status--;
    }
    spin_unlock_irqrestore(&dev->lock, irq_flag);

    return 0;
}

static ssize_t gpioled_write(struct file* filp, const char __user* buf, size_t count, loff_t* offt)
{
    int ret        = 0;
    u8 data_buf[1] = { 0 };

    // 获取私有数据
    struct gpioled_dev* dev = (struct gpioled_dev*)filp->private_data;
    printk("major=%d\r\n", dev->major);
    printk("minor=%d\r\n", dev->minor);
    printk("devid=%d\r\n", dev->devid);

    ret = copy_from_user(data_buf, buf, count);
    if (ret < 0) {
        printk("kernel write failed\r\n");
        return -EFAULT;
    }

    // 判断灯的状态
    if (data_buf[0] == LED_ON) {
        gpio_set_value(dev->led_gpio, 0); // 打开LED灯
    }
    else if (data_buf[0] == LED_OFF) {
        gpio_set_value(dev->led_gpio, 1); // 关闭LED灯
    }

    return 0;
}

static const struct file_operations gpioled_fops = {
    .owner   = THIS_MODULE,
    .open    = gpioled_open,
    .release = gpioled_release,
    .write   = gpioled_write,
};

/* 驱动入口函数 */
static int __init gpioled_init(void)
{
    int rc = 0;

    /* 初始化自旋锁 */
    spin_lock_init(&gpioled.lock);

    /* 注册字符设备 */
    // 1.分配设备号
    gpioled.major = 0; // 内核指定分配设备号
    if (gpioled.major) {
        gpioled.devid = MKDEV(gpioled.major, 0);
        rc            = register_chrdev_region(gpioled.devid, GPIOLED_COUNT, GPIOLED_NAME);
    }
    else {
        rc            = alloc_chrdev_region(&gpioled.devid, 0, GPIOLED_COUNT, GPIOLED_NAME);
        gpioled.major = MAJOR(gpioled.devid);
        gpioled.minor = MINOR(gpioled.devid);
    }
    if (rc < 0) {
        printk("gpioled chrdev region error!\r\n");
        goto fail_devid;
    }
    printk("major=%d,minor=%d\r\n", gpioled.major, gpioled.minor);

    // 2.注册字符设备
    gpioled.cdev.owner = THIS_MODULE;
    cdev_init(&gpioled.cdev, &gpioled_fops);
    rc = cdev_add(&gpioled.cdev, gpioled.devid, GPIOLED_COUNT);
    if (rc < 0) {
        rc = -EINVAL;
        goto fail_cdev;
    }

    // 3.动态创建设备的逻辑类
    gpioled.class = class_create(THIS_MODULE, GPIOLED_NAME);
    if (IS_ERR(gpioled.class)) {
        rc = PTR_ERR(gpioled.class);
        goto fail_class;
    }

    // 4.自动创建设备节点
    gpioled.device = device_create(gpioled.class, NULL, gpioled.devid, NULL, GPIOLED_NAME);
    if (IS_ERR(gpioled.device)) {
        rc = PTR_ERR(gpioled.device);
        goto fail_device;
    }
    printk("gpioled dirver init\r\n");

    /* 获取设备节点 */
    // 1.根据设备树路径获取设备节点
    gpioled.node = of_find_node_by_path("/gpioled");
    if (gpioled.node == NULL) {
        rc = -EINVAL;
        goto fail_node;
    }

    // 2.获取led所对应的GPIO
    gpioled.led_gpio = of_get_named_gpio(gpioled.node, "led-gpio", 0);
    if (gpioled.led_gpio < 0) {
        printk("led gpio not found\r\n");
        rc = -EINVAL;
        goto fail_led_gpio;
    }
    printk("led gpio= %d\r\n", gpioled.led_gpio);

    // 3.申请IO
    rc = gpio_request(gpioled.led_gpio, "led-gpio");
    if (rc) {
        pr_err("failed to request the led gpio\r\n");
        rc = -EINVAL;
        goto fail_gpio_req;
    }

    // 4.使用IO,设置为输出
    rc = gpio_direction_output(gpioled.led_gpio, 1);
    if (rc) {
        goto fail_gpio_out;
    }

    // 5.输出低电平,点亮LED
    gpio_set_value(gpioled.led_gpio, 0);

    return rc;
fail_gpio_out:
    gpio_free(gpioled.led_gpio);
fail_gpio_req:
fail_led_gpio:
fail_node:
    device_destroy(gpioled.class, gpioled.devid);
fail_device:
    class_destroy(gpioled.class);
fail_class:
    cdev_del(&gpioled.cdev);
fail_cdev:
    unregister_chrdev_region(gpioled.devid, GPIOLED_COUNT);
fail_devid:
    return rc;
}

/* 驱动出口函数 */
static void __exit gpioled_exit(void)
{
    // 1.关灯
    gpio_set_value(gpioled.led_gpio, 1);

    // 6.释放IO
    gpio_free(gpioled.led_gpio);

    // 2.删除字符设备
    cdev_del(&gpioled.cdev);

    // 3.注销设备号
    unregister_chrdev_region(gpioled.devid, GPIOLED_COUNT);

    // 4.销毁设备节点
    device_destroy(gpioled.class, gpioled.devid);

    // 5.销毁设备的逻辑类
    class_destroy(gpioled.class);

    printk("gpioled dirver exit\r\n");
}

/* 驱动注册与卸载 */
module_init(gpioled_init);
module_exit(gpioled_exit);
MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("spinlock");