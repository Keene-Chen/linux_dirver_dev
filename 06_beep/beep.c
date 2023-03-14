/**
 * @file    : beep.c
 * @author  : KeeneChen
 * @date    : 2023.03.14-09:50:20
 * @details : beep driver
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
#include <linux/types.h>
#include <linux/uaccess.h>

#define BEEP_NAME  "beep" // 设备名
#define BEEP_COUNT 1         // 设备个数
#define BEEP_ON    0         // 开蜂鸣器
#define BEEP_OFF   1         // 关蜂鸣器

/* beep设备结构体 */
struct beep_dev {
    dev_t devid;              // 设备号
    int major;                // 主设备号
    int minor;                // 次设备号
    struct cdev cdev;         // 字符设备
    struct class* class;      // 设备逻辑类
    struct device* device;    // 设备
    struct device_node* node; // 设备节点
    int beep_gpio;            // beep gpio
} beep;

/* 设备操作集合 */
static int beep_open(struct inode* inode, struct file* filp)
{
    filp->private_data = &beep;
    return 0;
}
static int beep_release(struct inode* inode, struct file* filp)
{
    struct beep_dev* dev = (struct beep_dev*)filp->private_data;
    printk("major=%d\r\n", dev->major);
    printk("minor=%d\r\n", dev->minor);

    return 0;
}
static ssize_t beep_read(struct file* filp, char __user* buf, size_t count, loff_t* offt)
{
    return 0;
}
static ssize_t beep_write(struct file* filp, const char __user* buf, size_t count, loff_t* offt)
{
    int ret;
    unsigned char databuf[1];

    ret = copy_from_user(databuf, buf, count);
    if (ret < 0) {
        printk("kernel write failed!\r\n");
        return -EFAULT;
    }

    if (databuf[0] == BEEP_ON) {
        printk("beep open\r\n");
        gpio_set_value(beep.beep_gpio, 0); /* 打开蜂鸣器 */
    }
    else if (databuf[0] == BEEP_OFF) {
        printk("beep close\r\n");
        gpio_set_value(beep.beep_gpio, 1); /* 关闭蜂鸣器 */
    }

    return 0;
}

static struct file_operations beep_fops = { .owner   = THIS_MODULE,
                                            .open    = beep_open,
                                            .release = beep_release,
                                            .read    = beep_read,
                                            .write   = beep_write };

/* 驱动入口函数与出口函数 */
static int __init beep_init(void)
{
    int rc = 0;

    /* 注册字符设备 */
    beep.major = 0;
    if (beep.major) {
        beep.devid = MKDEV(beep.major, 0);
        rc         = register_chrdev_region(beep.devid, BEEP_COUNT, BEEP_NAME);
    }
    else {
        rc         = alloc_chrdev_region(&beep.devid, 0, BEEP_COUNT, BEEP_NAME);
        beep.major = MAJOR(beep.devid);
        beep.minor = MINOR(beep.devid);
    }
    if (rc < 0) {
        printk("register chrdev region failed\r\n");
        goto fail_devid;
    }
    printk("major=%d,minor=%d\r\n", beep.major, beep.minor);

    beep.cdev.owner = THIS_MODULE;
    cdev_init(&beep.cdev, &beep_fops);
    rc = cdev_add(&beep.cdev, beep.devid, BEEP_COUNT);
    if (rc < 0) {
        rc = -EINVAL;
        printk("cdev add failed\r\n");
        goto fail_cdev;
    }

    beep.class = class_create(THIS_MODULE, BEEP_NAME);
    if (IS_ERR(beep.class)) {
        rc = PTR_ERR(beep.class);
        printk("kernel create class failed\r\n");
        goto fail_class;
    }

    beep.device = device_create(beep.class, NULL, beep.devid, NULL, BEEP_NAME);
    if (IS_ERR(beep.device)) {
        rc = PTR_ERR(beep.device);
        printk("kernel create device failed\r\n");
        goto fail_device;
    }

    /* 获取设备节点 */
    beep.node = of_find_node_by_path("/beep");
    if (beep.node == NULL) {
        rc = -EINVAL;
        printk("kernel get path node failed\r\n");
        goto fail_node;
    }

    beep.beep_gpio = of_get_named_gpio(beep.node, "beep-gpio", 0);
    if (beep.beep_gpio < 0) {
        rc = -EINVAL;
        printk("kernel get names gpio failed\r\n");
        goto fail_beep_gpio;
    }
    printk("beep gpio= %d\r\n", beep.beep_gpio);

    rc = gpio_request(beep.beep_gpio, BEEP_NAME);
    if (rc < 0) {
        pr_err("failed to request the beep gpio\r\n");
        rc = -EINVAL;
        printk("kernel gpio register failed\r\n");
        goto fail_gpio_req;
    }

    // 4.使用IO,设置为输出
    rc = gpio_direction_output(beep.beep_gpio, BEEP_OFF);
    if (rc < 0) {
        rc = -EINVAL;
        printk("kernel gpio setting output failed\r\n");
        goto fail_gpio_out;
    }

    // 5.输出低电平,打开蜂鸣器
    gpio_set_value(beep.beep_gpio, BEEP_OFF);

    return rc;
fail_gpio_out:
    gpio_free(beep.beep_gpio);
fail_gpio_req:
fail_beep_gpio:
fail_node:
    device_destroy(beep.class, beep.devid);
fail_device:
    class_destroy(beep.class);
fail_class:
    cdev_del(&beep.cdev);
fail_cdev:
    unregister_chrdev_region(beep.devid, BEEP_COUNT);
fail_devid:
    return rc;
}

static void __exit beep_exit(void)
{
    gpio_set_value(beep.beep_gpio, BEEP_OFF);
    gpio_free(beep.beep_gpio);
    cdev_del(&beep.cdev);
    unregister_chrdev_region(beep.devid, BEEP_COUNT);
    device_destroy(beep.class, beep.devid);
    class_destroy(beep.class);

    printk("beep dirver exit\r\n");
}

/* 驱动加载与卸载 */
module_init(beep_init);
module_exit(beep_exit);

/* 模块信息 */
MODULE_AUTHOR("KeeneChen");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("beep driver");