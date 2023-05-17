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

#define GPIOLED_NAME  "greenled" // 设备名
#define GPIOLED_COUNT 1        // 设备个数
#define LED_ON        1        // 开灯
#define LED_OFF       0        // 关灯

/* 驱动设备结构体 */
struct greenled_dev {
    dev_t devid;              // 设备号
    int major;                // 主设备号
    int minor;                // 次设备号
    struct cdev cdev;         // 字符设备
    struct class* class;      // 逻辑类
    struct device* device;    // 设备
    struct device_node* node; // 设备节点
    int led_gpio;             // led所使用的GPIO编号
    atomic_t value;           // 状态值
} greenled;
static int greenled_open(struct inode* inode, struct file* filp)
{
    // 设置私有数据
    filp->private_data = &greenled;
    return 0;
}

ssize_t greenled_read(struct file* filp, char __user* buf, size_t count, loff_t* offt)
{
    int rc;
    unsigned char data[1];
    data[0] = atomic_read(&greenled.value);

    rc = copy_to_user(buf, data, sizeof(data));

    return 0;
}

static ssize_t greenled_write(struct file* filp, const char __user* buf, size_t count, loff_t* offt)
{
    int ret        = 0;
    u8 data_buf[1] = { 0 };

    // 获取私有数据
    struct greenled_dev* dev = (struct greenled_dev*)filp->private_data;

    ret = copy_from_user(&greenled.value, buf, count);
    if (ret < 0) {
        printk("kernel write failed\r\n");
        return -EFAULT;
    }

    // 判断灯的状态
    data_buf[0] = atomic_read(&greenled.value);
    if (data_buf[0] == LED_ON) {
        gpio_set_value(dev->led_gpio, 0); // 打开LED灯
    }
    else if (data_buf[0] == LED_OFF) {
        gpio_set_value(dev->led_gpio, 1); // 关闭LED灯
    }

    return 0;
}

/* 设备文件操作结构体 */
static const struct file_operations greenled_fops = {
    .owner = THIS_MODULE,
    .open  = greenled_open,
    .read  = greenled_read,
    .write = greenled_write,
};

/* 驱动加载函数 */
static int led_probe(struct platform_device* dev)
{
    int rc = 0;

    /* 注册字符设备 */
    // 1.分配设备号
    greenled.major = 0; // 内核指定分配设备号
    if (greenled.major) {
        greenled.devid = MKDEV(greenled.major, 0);
        rc           = register_chrdev_region(greenled.devid, GPIOLED_COUNT, GPIOLED_NAME);
    }
    else {
        rc           = alloc_chrdev_region(&greenled.devid, 0, GPIOLED_COUNT, GPIOLED_NAME);
        greenled.major = MAJOR(greenled.devid);
        greenled.minor = MINOR(greenled.devid);
    }
    if (rc < 0) {
        printk("greenled chrdev region error!\r\n");
        goto fail_devid;
    }
    printk("major=%d,minor=%d\r\n", greenled.major, greenled.minor);

    // 2.注册字符设备
    greenled.cdev.owner = THIS_MODULE;
    cdev_init(&greenled.cdev, &greenled_fops);
    rc = cdev_add(&greenled.cdev, greenled.devid, GPIOLED_COUNT);
    if (rc < 0) {
        rc = -EINVAL;
        goto fail_cdev;
    }

    // 3.动态创建设备的逻辑类
    greenled.class = class_create(THIS_MODULE, GPIOLED_NAME);
    if (IS_ERR(greenled.class)) {
        rc = PTR_ERR(greenled.class);
        goto fail_class;
    }

    // 4.自动创建设备节点
    greenled.device = device_create(greenled.class, NULL, greenled.devid, NULL, GPIOLED_NAME);
    if (IS_ERR(greenled.device)) {
        rc = PTR_ERR(greenled.device);
        goto fail_device;
    }
    printk("greenled dirver init\r\n");

    /* 获取设备节点 */
    // 1.根据设备树路径获取设备节点

#if 0
    greenled.node = of_find_node_by_path("/greenled");
    if (greenled.node == NULL) {
        rc = -EINVAL;
        goto fail_node;
    }
#else
    /**
     * 驱动与设备匹配成功后,内核自动将设备树信息转换为platform_device结构体,
     * 取代了之前的手动获取设备树信息
     */
    greenled.node = dev->dev.of_node;
#endif

    // 2.获取led所对应的GPIO
    greenled.led_gpio = of_get_named_gpio(greenled.node, "led-gpio", 0);
    if (greenled.led_gpio < 0) {
        printk("led gpio not found\r\n");
        rc = -EINVAL;
        goto fail_led_gpio;
    }
    printk("led gpio= %d\r\n", greenled.led_gpio);

    // 3.申请IO
    rc = gpio_request(greenled.led_gpio, "led-gpio");
    if (rc) {
        pr_err("failed to request the led gpio\r\n");
        rc = -EINVAL;
        goto fail_gpio_req;
    }

    // 4.使用IO,设置为输出
    rc = gpio_direction_output(greenled.led_gpio, 1);
    if (rc) {
        goto fail_gpio_out;
    }

    // 初始化原子变量
    atomic_set(&greenled.value, 0);

    printk("led probe\r\n");
    return rc;

fail_gpio_out:
    gpio_free(greenled.led_gpio);
fail_gpio_req:
fail_led_gpio:
    device_destroy(greenled.class, greenled.devid);
fail_device:
    class_destroy(greenled.class);
fail_class:
    cdev_del(&greenled.cdev);
fail_cdev:
    unregister_chrdev_region(greenled.devid, GPIOLED_COUNT);
fail_devid:
    return rc;
}

/* 驱动卸载函数 */
static int led_remove(struct platform_device* dev)
{
    // 2.释放IO
    gpio_free(greenled.led_gpio);

    // 3.删除字符设备
    cdev_del(&greenled.cdev);

    // 4.注销设备号
    unregister_chrdev_region(greenled.devid, GPIOLED_COUNT);

    // 5.销毁设备节点
    device_destroy(greenled.class, greenled.devid);

    // 6.销毁设备的逻辑类
    class_destroy(greenled.class);

    printk("led remove\r\n");
    return 0;
}

static struct of_device_id led_of_match[] = {
    { .compatible = "atkmini,greenled" },
    { /* Sentinel */ },
};

/* platform driver 结构体 */
static struct platform_driver led_driver = {
    .probe  = led_probe,
    .remove = led_remove,
    .driver = {
        .name = "imx6ull-led1", // 无设备树与设备资源进行匹配,驱动名称
        .of_match_table=led_of_match, // 设备树匹配表
    },
};

/* 驱动注册与卸载宏 */
module_platform_driver(led_driver);

MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("dts platform driver");