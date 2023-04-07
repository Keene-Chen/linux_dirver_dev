/**
 * @file    : led_driver.c
 * @author  : KeeneChen
 * @date    : 2023.04.05-13:38:57
 * @details : led_driver
 */

/**
 * platform_driver 结构体
 * @note #include <linux/platform_device.h>
 ** struct platform_driver {
 **     int (*probe)(struct platform_device *);
 **     int (*remove)(struct platform_device *);
 **     void (*shutdown)(struct platform_device *);
 **     int (*suspend)(struct platform_device *, pm_message_t state);
 **     int (*resume)(struct platform_device *);
 **     struct device_driver driver;
 **     const struct platform_device_id *id_table;
 **     bool prevent_deferred_probe;
 ** };
 *
 ** int platform_driver_register (struct platform_driver *driver)
 * @brief 注册 platform 驱动
 * @param driver 要注册的 platform 驱动
 * @return 负数,失败 0,成功
 *
 ** void platform_driver_unregister(struct platform_driver *drv)
 * @brief 卸载 platform 驱动
 * @param driver 要卸载的 platform 驱动
 *
 */

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

#define GPIOLED_NAME  "gpioled" // 设备名
#define GPIOLED_COUNT 1         // 设备个数
#define LED_ON        1         // 开灯
#define LED_OFF       0         // 关灯

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
} gpioled;

static char strshow[2] = { 0 }; // 需要echo多个字节的话记得设置大一点，我这里只装一个字节

static ssize_t gpioled_show(struct device* dev, struct device_attribute* attr, char* buf)
{
    return scnprintf(buf, sizeof(strshow), "%s\n", strshow);
    // return buf;
}

static ssize_t gpioled_store(struct device* dev, struct device_attribute* attr, const char* buf,
                             size_t count)
{

    printk("buf:%s\r\n", buf);

    sprintf(strshow, "%s", buf);
    if (buf[0] == '0') {
        gpio_set_value(gpioled.led_gpio, LED_ON);
        printk("LED ON\r\n");
    }
    else if (buf[0] == '1') {
        gpio_set_value(gpioled.led_gpio, LED_OFF);
        printk("LED OFF\r\n");
    }
    return count;
}

static DEVICE_ATTR(gpioled, S_IRUGO | S_IWUSR, gpioled_show, gpioled_store);

static int gpioled_open(struct inode* inode, struct file* filp)
{
    // 设置私有数据
    filp->private_data = &gpioled;
    return 0;
}

static ssize_t gpioled_write(struct file* filp, const char __user* buf, size_t count, loff_t* offt)
{
    int ret        = 0;
    u8 data_buf[1] = { 0 };

    // 获取私有数据
    struct gpioled_dev* dev = (struct gpioled_dev*)filp->private_data;

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

/* 设备文件操作结构体 */
static const struct file_operations gpioled_fops = {
    .owner = THIS_MODULE,
    .open  = gpioled_open,
    .write = gpioled_write,
};

/* 驱动加载函数 */
static int led_probe(struct platform_device* dev)
{
    int rc = 0;

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

#if 0
    gpioled.node = of_find_node_by_path("/gpioled");
    if (gpioled.node == NULL) {
        rc = -EINVAL;
        goto fail_node;
    }
#else
    /**
     * 驱动与设备匹配成功后,内核自动将设备树信息转换为platform_device结构体,
     * 取代了之前的手动获取设备树信息
     */
    gpioled.node = dev->dev.of_node;
#endif

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

    // 6.创建驱动接口
    if (sysfs_create_file((&gpioled.device->kobj), &dev_attr_gpioled.attr)) {
        goto fail_node;
    }

    printk("led probe\r\n");
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

/* 驱动卸载函数 */
static int led_remove(struct platform_device* dev)
{
    // 1.关灯
    gpio_set_value(gpioled.led_gpio, 1);

    // 2.释放IO
    gpio_free(gpioled.led_gpio);

    // 3.删除字符设备
    cdev_del(&gpioled.cdev);

    // 4.注销设备号
    unregister_chrdev_region(gpioled.devid, GPIOLED_COUNT);

    // 5.销毁设备节点
    device_destroy(gpioled.class, gpioled.devid);

    // 6.销毁设备的逻辑类
    class_destroy(gpioled.class);

    printk("led remove\r\n");
    return 0;
}

static struct of_device_id led_of_match[] = {
    { .compatible = "atkmini,gpioled" },
    { /* Sentinel */ },
};

/* platform driver 结构体 */
static struct platform_driver led_driver = {
    .probe  = led_probe,
    .remove = led_remove,
    .driver = {
        .name = "imx6ull-led", // 无设备树与设备资源进行匹配,驱动名称
        .of_match_table=led_of_match, // 设备树匹配表
    },
};

#if 0
/* 驱动入口函数 */
static int __init led_driver_init(void)
{
    int rc = 0;
    printk("led driver init\r\n");
    // 注册platform设备
    rc = platform_driver_register(&led_driver);

    return rc;
}

/* 驱动出口函数 */
static void __exit led_driver_exit(void)
{
    // 卸载platform设备
    platform_driver_unregister(&led_driver);

    printk("led driver exit\r\n");
}

/* 驱动注册与卸载 */
module_init(led_driver_init);
module_exit(led_driver_exit);

#else
/* 驱动注册与卸载宏 */
module_platform_driver(led_driver);
#endif

MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("dts platform driver");