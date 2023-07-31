/**
 * @file    : led_driver.c
 * @author  : KeeneChen
 * @date    : 2023.04.05-13:38:57
 * @details : led_driver
 */

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

#define GPIOLED_NAME  "blueled" // 设备名
#define GPIOLED_COUNT 1        // 设备个数
#define LED_ON        1        // 开灯
#define LED_OFF       0        // 关灯

/* 驱动设备结构体 */
struct blueled_dev {
    dev_t devid;              // 设备号
    int major;                // 主设备号
    int minor;                // 次设备号
    struct cdev cdev;         // 字符设备
    struct class* class;      // 逻辑类
    struct device* device;    // 设备
    struct device_node* node; // 设备节点
    int led_gpio;             // led所使用的GPIO编号
    atomic_t value;           // 状态值
} blueled;

static int blueled_open(struct inode* inode, struct file* filp)
{
    // 设置私有数据
    filp->private_data = &blueled;
    return 0;
}

ssize_t blueled_read(struct file* filp, char __user* buf, size_t count, loff_t* offt)
{
    int rc;
    unsigned char data[1];
    data[0] = atomic_read(&blueled.value);

    rc = copy_to_user(buf, data, sizeof(data));

    return 0;
}

static ssize_t blueled_write(struct file* filp, const char __user* buf, size_t count, loff_t* offt)
{
    int ret        = 0;
    u8 data_buf[1] = { 0 };

    // 获取私有数据
    struct blueled_dev* dev = (struct blueled_dev*)filp->private_data;

    ret = copy_from_user(&blueled.value, buf, count);
    if (ret < 0) {
        printk("kernel write failed\r\n");
        return -EFAULT;
    }

    // 判断灯的状态
    data_buf[0] = atomic_read(&blueled.value);
    if (data_buf[0] == LED_ON) {
        gpio_set_value(dev->led_gpio, 0); // 打开LED灯
    }
    else if (data_buf[0] == LED_OFF) {
        gpio_set_value(dev->led_gpio, 1); // 关闭LED灯
    }

    return 0;
}

/* 设备文件操作结构体 */
static const struct file_operations blueled_fops = {
    .owner = THIS_MODULE,
    .open  = blueled_open,
    .read  = blueled_read,
    .write = blueled_write,
};

/* 驱动加载函数 */
static int led_probe(struct platform_device* dev)
{
    int rc = 0;

    /* 注册字符设备 */
    // 1.分配设备号
    blueled.major = 0; // 内核指定分配设备号
    if (blueled.major) {
        blueled.devid = MKDEV(blueled.major, 0);
        rc           = register_chrdev_region(blueled.devid, GPIOLED_COUNT, GPIOLED_NAME);
    }
    else {
        rc           = alloc_chrdev_region(&blueled.devid, 0, GPIOLED_COUNT, GPIOLED_NAME);
        blueled.major = MAJOR(blueled.devid);
        blueled.minor = MINOR(blueled.devid);
    }
    if (rc < 0) {
        printk("blueled chrdev region error!\r\n");
        goto fail_devid;
    }
    printk("major=%d,minor=%d\r\n", blueled.major, blueled.minor);

    // 2.注册字符设备
    blueled.cdev.owner = THIS_MODULE;
    cdev_init(&blueled.cdev, &blueled_fops);
    rc = cdev_add(&blueled.cdev, blueled.devid, GPIOLED_COUNT);
    if (rc < 0) {
        rc = -EINVAL;
        goto fail_cdev;
    }

    // 3.动态创建设备的逻辑类
    blueled.class = class_create(THIS_MODULE, GPIOLED_NAME);
    if (IS_ERR(blueled.class)) {
        rc = PTR_ERR(blueled.class);
        goto fail_class;
    }

    // 4.自动创建设备节点
    blueled.device = device_create(blueled.class, NULL, blueled.devid, NULL, GPIOLED_NAME);
    if (IS_ERR(blueled.device)) {
        rc = PTR_ERR(blueled.device);
        goto fail_device;
    }
    printk("blueled dirver init\r\n");

    /* 获取设备节点 */
    // 1.根据设备树路径获取设备节点

#if 0
    blueled.node = of_find_node_by_path("/blueled");
    if (blueled.node == NULL) {
        rc = -EINVAL;
        goto fail_node;
    }
#else
    /**
     * 驱动与设备匹配成功后,内核自动将设备树信息转换为platform_device结构体,
     * 取代了之前的手动获取设备树信息
     */
    blueled.node = dev->dev.of_node;
#endif

    // 2.获取led所对应的GPIO
    blueled.led_gpio = of_get_named_gpio(blueled.node, "led-gpio", 0);
    if (blueled.led_gpio < 0) {
        printk("led gpio not found\r\n");
        rc = -EINVAL;
        goto fail_led_gpio;
    }
    printk("led gpio= %d\r\n", blueled.led_gpio);

    // 3.申请IO
    rc = gpio_request(blueled.led_gpio, "led-gpio");
    if (rc) {
        pr_err("failed to request the led gpio\r\n");
        rc = -EINVAL;
        goto fail_gpio_req;
    }

    // 4.使用IO,设置为输出
    rc = gpio_direction_output(blueled.led_gpio, 1);
    if (rc) {
        goto fail_gpio_out;
    }

    // 初始化原子变量
    atomic_set(&blueled.value, 0);

    printk("led probe\r\n");
    return rc;

fail_gpio_out:
    gpio_free(blueled.led_gpio);
fail_gpio_req:
fail_led_gpio:
    device_destroy(blueled.class, blueled.devid);
fail_device:
    class_destroy(blueled.class);
fail_class:
    cdev_del(&blueled.cdev);
fail_cdev:
    unregister_chrdev_region(blueled.devid, GPIOLED_COUNT);
fail_devid:
    return rc;
}

/* 驱动卸载函数 */
static int led_remove(struct platform_device* dev)
{
    // 2.释放IO
    gpio_free(blueled.led_gpio);

    // 3.删除字符设备
    cdev_del(&blueled.cdev);

    // 4.注销设备号
    unregister_chrdev_region(blueled.devid, GPIOLED_COUNT);

    // 5.销毁设备节点
    device_destroy(blueled.class, blueled.devid);

    // 6.销毁设备的逻辑类
    class_destroy(blueled.class);

    printk("led remove\r\n");
    return 0;
}

static struct of_device_id led_of_match[] = {
    { .compatible = "atkmini,blueled" },
    { /* Sentinel */ },
};

/* platform driver 结构体 */
static struct platform_driver led_driver = {
    .probe  = led_probe,
    .remove = led_remove,
    .driver = {
        .name = "imx6ull-led2", // 无设备树与设备资源进行匹配,驱动名称
        .of_match_table=led_of_match, // 设备树匹配表
    },
};

/* 驱动注册与卸载宏 */
module_platform_driver(led_driver);

MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("dts platform driver");