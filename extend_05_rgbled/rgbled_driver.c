/**
 * @file    : rgbled_driver.c
 * @author  : KeeneChen
 * @date    : 2023.05.17-13:05:33
 * @details : rgbled_driver
 */

#include "asm/gpio.h"
#include "linux/printk.h"
#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#define RGBLED_NAME  "rgbled" // 设备名
#define RGBLED_COUNT 3        // 设备个数
#define LED_ON       1        // 开灯
#define LED_OFF      0        // 关灯

struct rgbled_data {
    int red_gpio;
    int green_gpio;
    int blue_gpio; // led所使用的GPIO编号
    u8 value[3];   // 状态值
};

/* 驱动设备结构体 */
struct rgbled_dev {
    dev_t devid;              // 设备号
    int major;                // 主设备号
    int minor;                // 次设备号
    struct cdev cdev;         // 字符设备
    struct class* class;      // 逻辑类
    struct device* device;    // 设备
    struct device_node* node; // 设备节点
    struct rgbled_data rgbled_data;
} rgbled;

static int rgbled_open(struct inode* inode, struct file* filp)
{
    // 设置私有数据
    filp->private_data = &rgbled;
    return 0;
}

ssize_t rgbled_read(struct file* filp, char __user* buf, size_t count, loff_t* offt)
{
    int rc;
    unsigned char data[3];
    data[0] = rgbled.rgbled_data.value[0];
    data[1] = rgbled.rgbled_data.value[1];
    data[2] = rgbled.rgbled_data.value[2];

    rc = copy_to_user(buf, data, sizeof(data));

    return 0;
}

static ssize_t rgbled_write(struct file* filp, const char __user* buf, size_t count, loff_t* offt)
{
    int ret        = 0;
    u8 data_buf[3] = { 0 };

    // 获取私有数据
    struct rgbled_dev* dev = (struct rgbled_dev*)filp->private_data;

    ret = copy_from_user(rgbled.rgbled_data.value, buf, count);
    if (ret < 0) {
        printk("kernel write failed\r\n");
        return -EFAULT;
    }

    // 判断灯的状态
    data_buf[0] = rgbled.rgbled_data.value[0];
    if (data_buf[0] == LED_ON)
        gpio_set_value(dev->rgbled_data.red_gpio, 0); // 打开REDLED灯
    else if (data_buf[0] == LED_OFF)
        gpio_set_value(dev->rgbled_data.red_gpio, 1); // 关闭REDLED灯

    // 判断灯的状态
    data_buf[1] = rgbled.rgbled_data.value[1];
    if (data_buf[1] == LED_ON)
        gpio_set_value(dev->rgbled_data.green_gpio, 0); // 打开REDLED灯
    else if (data_buf[1] == LED_OFF)
        gpio_set_value(dev->rgbled_data.green_gpio, 1); // 关闭REDLED灯

    // 判断灯的状态
    data_buf[2] = rgbled.rgbled_data.value[2];
    if (data_buf[2] == LED_ON)
        gpio_set_value(dev->rgbled_data.blue_gpio, 0); // 打开REDLED灯
    else if (data_buf[2] == LED_OFF)
        gpio_set_value(dev->rgbled_data.blue_gpio, 1); // 关闭REDLED灯

    return 0;
}

/* 设备文件操作结构体 */
static const struct file_operations rgbled_fops = {
    .owner = THIS_MODULE,
    .open  = rgbled_open,
    .read  = rgbled_read,
    .write = rgbled_write,
};

/* 驱动加载函数 */
static int led_probe(struct platform_device* dev)
{
    int rc = 0;

    /* 注册字符设备 */
    // 1.分配设备号
    rgbled.major = 0; // 内核指定分配设备号
    if (rgbled.major) {
        rgbled.devid = MKDEV(rgbled.major, 0);
        rc           = register_chrdev_region(rgbled.devid, RGBLED_COUNT, RGBLED_NAME);
    }
    else {
        rc           = alloc_chrdev_region(&rgbled.devid, 0, RGBLED_COUNT, RGBLED_NAME);
        rgbled.major = MAJOR(rgbled.devid);
        rgbled.minor = MINOR(rgbled.devid);
    }
    if (rc < 0) {
        printk("rgbled chrdev region error!\r\n");
        goto fail_devid;
    }
    printk("major=%d,minor=%d\r\n", rgbled.major, rgbled.minor);

    // 2.注册字符设备
    rgbled.cdev.owner = THIS_MODULE;
    cdev_init(&rgbled.cdev, &rgbled_fops);
    rc = cdev_add(&rgbled.cdev, rgbled.devid, RGBLED_COUNT);
    if (rc < 0) {
        rc = -EINVAL;
        goto fail_cdev;
    }

    // 3.动态创建设备的逻辑类
    rgbled.class = class_create(THIS_MODULE, RGBLED_NAME);
    if (IS_ERR(rgbled.class)) {
        rc = PTR_ERR(rgbled.class);
        goto fail_class;
    }

    // 4.自动创建设备节点
    rgbled.device = device_create(rgbled.class, NULL, rgbled.devid, NULL, RGBLED_NAME);
    if (IS_ERR(rgbled.device)) {
        rc = PTR_ERR(rgbled.device);
        goto fail_device;
    }
    printk("rgbled dirver init\r\n");

    /* 获取 red led 设备节点 */
    // 1.根据设备树路径获取设备节点
    rgbled.node = of_find_node_by_path("/rgbled/red-led");
    if (rgbled.node == NULL) {
        rc = -EINVAL;
        goto fail_node;
    }

    // 2.获取led所对应的GPIO
    rgbled.rgbled_data.red_gpio = of_get_named_gpio(rgbled.node, "led-gpio", 0);
    if (rgbled.rgbled_data.red_gpio < 0) {
        printk("led gpio not found\r\n");
        rc = -EINVAL;
        goto fail_led_gpio;
    }
    printk("red led gpio= %d\r\n", rgbled.rgbled_data.red_gpio);

    // 3.申请IO
    rc = gpio_request(rgbled.rgbled_data.red_gpio, "red-led");
    if (rc) {
        pr_err("failed to request the led gpio\r\n");
        rc = -EINVAL;
        goto fail_gpio_req;
    }

    // 4.使用IO,设置为输出
    rc = gpio_direction_output(rgbled.rgbled_data.red_gpio, 1);
    if (rc) {
        goto fail_gpio_out;
    }

    /* 获取 green led 设备节点 */
    rgbled.node                   = of_find_node_by_path("/rgbled/green-led");
    rgbled.rgbled_data.green_gpio = of_get_named_gpio(rgbled.node, "led-gpio", 0);
    printk("green led gpio= %d\r\n", rgbled.rgbled_data.green_gpio);
    rc = gpio_request(rgbled.rgbled_data.green_gpio, "green-led");
    rc = gpio_direction_output(rgbled.rgbled_data.green_gpio, 1);

    /* 获取 blue led 设备节点 */
    rgbled.node                  = of_find_node_by_path("/rgbled/blue-led");
    rgbled.rgbled_data.blue_gpio = of_get_named_gpio(rgbled.node, "led-gpio", 0);
    printk("blue led gpio= %d\r\n", rgbled.rgbled_data.blue_gpio);
    rc = gpio_request(rgbled.rgbled_data.blue_gpio, "blue-led");
    rc = gpio_direction_output(rgbled.rgbled_data.blue_gpio, 1);

    printk("led probe\r\n");
    return rc;

fail_gpio_out:
    gpio_free(rgbled.rgbled_data.red_gpio);
fail_gpio_req:
fail_led_gpio:
fail_node:
    device_destroy(rgbled.class, rgbled.devid);
fail_device:
    class_destroy(rgbled.class);
fail_class:
    cdev_del(&rgbled.cdev);
fail_cdev:
    unregister_chrdev_region(rgbled.devid, RGBLED_COUNT);
fail_devid:
    return rc;
}

/* 驱动卸载函数 */
static int led_remove(struct platform_device* dev)
{
    // 2.释放IO
    gpio_free(rgbled.rgbled_data.red_gpio);
    gpio_free(rgbled.rgbled_data.green_gpio);
    gpio_free(rgbled.rgbled_data.blue_gpio);

    // 3.删除字符设备
    cdev_del(&rgbled.cdev);

    // 4.注销设备号
    unregister_chrdev_region(rgbled.devid, RGBLED_COUNT);

    // 5.销毁设备节点
    device_destroy(rgbled.class, rgbled.devid);

    // 6.销毁设备的逻辑类
    class_destroy(rgbled.class);

    printk("led remove\r\n");
    return 0;
}

static struct of_device_id led_of_match[] = {
    { .compatible = "atkmini,rgbled" },
    { /* Sentinel */ },
};

MODULE_DEVICE_TABLE(of, led_of_match);

/* platform driver 结构体 */
static struct platform_driver led_driver = {
    .probe  = led_probe,
    .remove = led_remove,
    .driver = {
        .name = "imx6ull-led", // 无设备树与设备资源进行匹配,驱动名称
        .of_match_table=led_of_match, // 设备树匹配表
    },
};

/* 驱动注册与卸载宏 */
module_platform_driver(led_driver);

MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("dts platform driver");