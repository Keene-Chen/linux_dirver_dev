/**
 * @file    : key.c
 * @author  : KeeneChen
 * @date    : 2023.03.15-13:52:16
 * @details : key driver module
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
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define KEYIO_NAME  "keyio" // 设备名
#define KEYIO_COUNT 1       // 设备个数
#define KEY_VALUE   0xF0    // 按键有效值
#define KEY_INVALID 0x00    // 按键无效值

/* 驱动设备结构体 */
struct keyio_dev {
    dev_t devid;              // 设备号
    int major;                // 主设备号
    int minor;                // 次设备号
    struct cdev cdev;         // 字符设备
    struct class* class;      // 逻辑类
    struct device* device;    // 设备
    struct device_node* node; // 设备节点
    int key_gpio;             // key所使用的GPIO编号
    atomic_t key_val;         // 按键值
} keyio;

static int keyio_open(struct inode* inode, struct file* filp)
{
    // 设置私有数据
    filp->private_data = &keyio;
    return 0;
}

static int keyio_release(struct inode* inode, struct file* filp)
{
    // 取出私有数据
    struct keyio_dev* dev = (struct keyio_dev*)filp->private_data;
    printk("major=%d\r\n", dev->major);
    printk("minor=%d\r\n", dev->minor);
    printk("devid=%d\r\n", dev->devid);

    return 0;
}

static ssize_t keyio_read(struct file* filp, char __user* buf, size_t count, loff_t* offt)
{
    int value, rc;

    struct keyio_dev* dev = (struct keyio_dev*)filp->private_data;

    if (gpio_get_value(dev->key_gpio) == 0) {
        while (!gpio_get_value(dev->key_gpio))
            ;
        atomic_set(&dev->key_val, KEY_VALUE);
    }
    else {
        atomic_set(&dev->key_val, KEY_INVALID);
    }

    value = atomic_read(&dev->key_val); // 将原子变量转为正常变量后返回给用户
    rc    = copy_to_user(buf, &value, count);

    return rc;
}

static ssize_t keyio_write(struct file* filp, const char __user* buf, size_t count, loff_t* offt)
{
    return 0;
}

static const struct file_operations keyio_fops = {
    .owner   = THIS_MODULE,
    .open    = keyio_open,
    .release = keyio_release,
    .read    = keyio_read,
    .write   = keyio_write,
};

/* keyio 初始化 */
static int mykey_init(struct keyio_dev* dev)
{
    int rc = 0;

    /* 初始化原子变量 */
    atomic_set(&keyio.key_val, KEY_INVALID);

    /* 获取设备节点 */
    // 1.根据设备树路径获取设备节点
    dev->node = of_find_node_by_path("/key");
    if (dev->node == NULL) {
        rc = -EINVAL;
        goto fail_node;
    }

    // 2.获取key所对应的GPIO
    dev->key_gpio = of_get_named_gpio(dev->node, "key-gpio", 0);
    if (dev->key_gpio < 0) {
        rc = -EINVAL;
        printk("key gpio not found\r\n");
        goto fail_key_gpio;
    }
    printk("key gpio= %d\r\n", dev->key_gpio);

    // 3.申请IO
    rc = gpio_request(dev->key_gpio, "key-gpio");
    if (rc) {
        rc = -EBUSY;
        pr_err("failed to request the key gpio\r\n");
        goto fail_gpio_req;
    }

    // 4.使用IO,设置为输入
    rc = gpio_direction_input(dev->key_gpio);
    if (rc) {
        rc = -EINVAL;
        pr_err("failed to request the key gpio\r\n");
        goto fail_gpio_input;
    }

    return 0;

fail_gpio_input:
    gpio_free(dev->key_gpio);
fail_gpio_req:
fail_key_gpio:
fail_node:
    return rc;
}

/* 驱动入口函数 */
static int __init keyio_init(void)
{
    int rc = 0;

    /* 注册字符设备 */
    // 1.分配设备号
    keyio.major = 0; // 内核指定分配设备号
    if (keyio.major) {
        keyio.devid = MKDEV(keyio.major, 0);
        rc          = register_chrdev_region(keyio.devid, KEYIO_COUNT, KEYIO_NAME);
    }
    else {
        rc          = alloc_chrdev_region(&keyio.devid, 0, KEYIO_COUNT, KEYIO_NAME);
        keyio.major = MAJOR(keyio.devid);
        keyio.minor = MINOR(keyio.devid);
    }
    if (rc < 0) {
        printk("keyio chrdev region error!\r\n");
        goto fail_devid;
    }
    printk("major=%d,minor=%d\r\n", keyio.major, keyio.minor);

    // 2.注册字符设备
    keyio.cdev.owner = THIS_MODULE;
    cdev_init(&keyio.cdev, &keyio_fops);
    rc = cdev_add(&keyio.cdev, keyio.devid, KEYIO_COUNT);
    if (rc < 0) {
        rc = -EINVAL;
        goto fail_cdev;
    }

    // 3.动态创建设备的逻辑类
    keyio.class = class_create(THIS_MODULE, KEYIO_NAME);
    if (IS_ERR(keyio.class)) {
        rc = PTR_ERR(keyio.class);
        goto fail_class;
    }

    // 4.自动创建设备节点
    keyio.device = device_create(keyio.class, NULL, keyio.devid, NULL, KEYIO_NAME);
    if (IS_ERR(keyio.device)) {
        rc = PTR_ERR(keyio.device);
        goto fail_device;
    }
    printk("keyio dirver init\r\n");

    // 5.keyio 初始化
    rc = mykey_init(&keyio);
    if (rc < 0) {
        rc = -EINVAL;
        printk("key init failed\r\n");
        goto fail_gpio;
    }

    return 0;

fail_gpio:
    device_destroy(keyio.class, keyio.devid);
fail_device:
    class_destroy(keyio.class);
fail_class:
    cdev_del(&keyio.cdev);
fail_cdev:
    unregister_chrdev_region(keyio.devid, KEYIO_COUNT);
fail_devid:
    return rc;
}

/* 驱动出口函数 */
static void __exit keyio_exit(void)
{
    // 1.释放IO
    gpio_free(keyio.key_gpio);
    // 2.删除字符设备
    cdev_del(&keyio.cdev);
    // 3.注销设备号
    unregister_chrdev_region(keyio.devid, KEYIO_COUNT);
    // 4.销毁设备节点
    device_destroy(keyio.class, keyio.devid);
    // 5.销毁设备的逻辑类
    class_destroy(keyio.class);

    printk("keyio dirver exit\r\n");
}

/* 驱动注册与卸载 */
module_init(keyio_init);
module_exit(keyio_exit);
MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("key driver module");