/**
 * @file    : beep_driver.c
 * @author  : KeeneChen
 * @date    : 2023.04.07-22:54:41
 * @details : beep misc driver
 */

/**
 * 杂项设备结构体
 ** struct miscdevice {
 **     int minor; // 子设备号
 **     const char *name; // 设备名字
 **     const struct file_operations *fops; // 设备操作集
 **     struct list_head list;
 **     struct device *parent;
 **     struct device *this_device;
 **     const struct attribute_group **groups;
 **     const char *nodename;
 **     umode_t mode;
 ** };
 *
 ** int misc_register(struct miscdevice * misc)
 * @brief 注册 misc 设备
 * @param misc 要注册的 MISC 设备
 * @return 负数,失败; 0,成功
 *
 ** int misc_deregister(struct miscdevice *misc)
 * @brief 注销 misc 设备
 * @param misc 要注销的 MISC 设备
 * @return 负数,失败; 0,成功
 */

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define BEEP_MISC_NAME  "beep_misc" // 名字
#define BEEP_MISC_MINOR 144         // 子设备号
#define BEEP_OFF        0           // 关蜂鸣器
#define BEEP_ON         1           // 开蜂鸣器

struct beep_misc_dev {
    dev_t devid;              // 设备号
    int major;                // 主设备号
    int minor;                // 次设备号
    struct cdev cdev;         // 字符设备
    struct class* class;      // 逻辑类
    struct device* device;    // 设备
    struct device_node* node; // 设备节点
    int beep_gpio;            // beep所使用的GPIO编号
    atomic_t value;           // 状态值
} beep_misc;

int beep_misc_open(struct inode* inode, struct file* filp)
{
    filp->private_data = &beep_misc; // 设置私有数据
    return 0;
}

int beep_misc_release(struct inode* inode, struct file* filp)
{
    // struct beep_misc_dev* dev = filp->private_data; // 取出私有数据

    // printk("MAJOR: %d", dev->major);
    // printk("MINOR: %d", dev->minor);
    // printk("BEEP-GPIO: %d", dev->beep_gpio);

    return 0;
}

ssize_t beep_misc_read(struct file* filp, char __user* buf, size_t count, loff_t* offt)
{
    int rc;
    unsigned char data[1];
    data[0] = atomic_read(&beep_misc.value);

    rc = copy_to_user(buf, data, sizeof(data));

    return 0;
}

ssize_t beep_misc_write(struct file* filp, const char __user* buf, size_t count, loff_t* offt)
{
    int rc;
    unsigned char beepstat;
    struct beep_misc_dev* dev = filp->private_data;

    rc = copy_from_user(&dev->value, buf, count);
    if (rc < 0) {
        printk("kernel write failed!\r\n");
        return -EFAULT;
    }

    beepstat = atomic_read(&dev->value); // 获取状态值
    if (beepstat == BEEP_ON) {
        gpio_set_value(dev->beep_gpio, 0); // 打开蜂鸣器
    }
    else if (beepstat == BEEP_OFF) {
        gpio_set_value(dev->beep_gpio, 1); // 关闭蜂鸣器
    }

    return rc;
}

/* 设备文件操作集合结构体 */
static const struct file_operations beep_misc_fops = {
    .owner   = THIS_MODULE,
    .open    = beep_misc_open,
    .release = beep_misc_release,
    .read    = beep_misc_read,
    .write   = beep_misc_write,
};

/* misc 设备结构体 */
static struct miscdevice beep_misc_cdev = {
    .minor = BEEP_MISC_MINOR,
    .name  = BEEP_MISC_NAME,
    .fops  = &beep_misc_fops,
};

/* 驱动入口 */
static int beep_misc_probe(struct platform_device* pdev)
{
    int rc = 0;

    /**
     * 1.注册字符设备
     * 一般情况下会注册对应的字符设备,但是这里我们使用MISC设备
     * 所以我们不需要自己注册字符设备驱动,只需要注册misc设备驱动即可
     */
    rc = misc_register(&beep_misc_cdev);
    if (rc < 0) {
        printk("misc device register failed!\r\n");
        return -EFAULT;
    }

    /* 2.获取设备节点 */
    // 获取设备节点: beep
    beep_misc.node = pdev->dev.of_node;
    // beep_misc.node = of_find_node_by_path("/beep");
    if (beep_misc.node == NULL) {
        printk("beep node not find!\r\n");
        return -EINVAL;
    }

    // 获取设备树中的 gpio 属性,得到 beep 所使用的 beep 编号
    beep_misc.beep_gpio = of_get_named_gpio(beep_misc.node, "beep-gpio", 0);
    if (beep_misc.beep_gpio < 0) {
        printk("can't get beep-gpio");
        return -EINVAL;
    }

    // 设置 GPIO5_IO01 为输出,并且输出高电平,默认关闭 beep
    rc = gpio_direction_output(beep_misc.beep_gpio, 1);
    if (rc < 0) {
        printk("can't set gpio!\r\n");
    }

    // 初始化原子变量
    atomic_set(&beep_misc.value, 0);

    return 0;
}

/* 驱动出口 */
static int beep_misc_remove(struct platform_device* pdev)
{
    // 注销设备的时候关闭 beep
    gpio_set_value(beep_misc.beep_gpio, 1);

    // 注销 misc 设备
    misc_deregister(&beep_misc_cdev);

    return 0;
}

/* 设备匹配列表与设备树匹配 */
static const struct of_device_id beep_of_match[] = {
    { .compatible = "atkmini,beep" },
    /* Sentinel */
};

/* platform 平台驱动结构体 */
static struct platform_driver beep_misc_driver={
    .driver = {
        .name = "beep_misc",
        .of_match_table = beep_of_match,
    },
    .probe = beep_misc_probe,
    .remove = beep_misc_remove,
};

/* 使用辅助宏进行驱动模块加载与卸载 */
module_platform_driver(beep_misc_driver);

/* 驱动模块信息 */
MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("beep misc driver");