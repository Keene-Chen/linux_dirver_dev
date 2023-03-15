/**
 * @file    : atomic.c
 * @author  : KeeneChen
 * @date    : 2023.03.14-14:11:06
 * @details : atomic
 */

/**
 * 原子整形操作 API 函数
 ** ATOMIC_INIT(int i)
 * @brief 定义原子变量的时候对其初始化。
 ** int atomic_read(atomic_t *v)
 * @brief 读取 v 的值，并且返回。
 ** void atomic_set(atomic_t *v, int i)
 * @brief 向 v 写入 i 值。
 ** void atomic_add(int i, atomic_t *v)
 * @brief 给 v 加上 i 值。
 ** void atomic_sub(int i, atomic_t *v)
 * @brief 从 v 减去 i 值。
 ** void atomic_inc(atomic_t *v)
 * @brief 给 v 加 1，也就是自增。
 ** void atomic_dec(atomic_t *v)
 * @brief 从 v 减 1，也就是自减
 ** int atomic_dec_return(atomic_t *v)
 * @brief 从 v 减 1，并且返回 v 的值。
 ** int atomic_inc_return(atomic_t *v)
 * @brief 给 v 加 1，并且返回 v 的值。
 ** int atomic_sub_and_test(int i, atomic_t *v)
 * @brief 从 v 减 i，如果结果为 0 就返回真，否则返回假
 ** int atomic_dec_and_test(atomic_t *v)
 * @brief 从 v 减 1，如果结果为 0 就返回真，否则返回假
 ** int atomic_inc_and_test(atomic_t *v)
 * @brief 给 v 加 1，如果结果为 0 就返回真，否则返回假
 ** int atomic_add_negative(int i, atomic_t *v)
 * @brief 给 v 加 i，如果结果为负就返回真，否则返回假
 *
 * 原子位操作 API 函数
 ** void set_bit(int nr, void *p)
 * @brief 将 p 地址的第 nr 位置 1。
 ** void clear_bit(int nr,void *p)
 * @brief 将 p 地址的第 nr 位清零。
 ** void change_bit(int nr, void *p)
 * @brief 将 p 地址的第 nr 位进行翻转。
 ** int test_bit(int nr, void *p)
 * @brief 获取 p 地址的第 nr 位的值。
 ** int test_and_set_bit(int nr, void *p)
 * @brief 将 p 地址的第 nr 位置 1，并且返回 nr 位原来的值。
 ** int test_and_clear_bit(int nr, void *p)
 * @brief 将 p 地址的第 nr 位清零，并且返回 nr 位原来的值。
 ** int test_and_change_bit(int nr, void *p)
 * @brief 将 p 地址的第 nr 位翻转，并且返回 nr 位原来的值。
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

#define GPIOLED_NAME  "atomic" // 设备名
#define GPIOLED_COUNT 1        // 设备个数
#define LED_ON        1        // 开灯
#define LED_OFF       0        // 关灯

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
    atomic_t lock;            // 原子操作
} gpioled;

static int gpioled_open(struct inode* inode, struct file* filp)
{
    // 设置私有数据
    filp->private_data = &gpioled;

    /* 判断lock */

#if 0
    if (atomic_read(&gpioled.lock) <= 0) {
        return -EBUSY;
    }
    else {
        atomic_dec(&gpioled.lock); // 减一,加载驱动
    }
#endif

    if (!atomic_dec_and_test(&gpioled.lock)) {
        // 不能使用驱动
        atomic_inc(&gpioled.lock);
        return -EBUSY;
    }

    return 0;
}

static int gpioled_release(struct inode* inode, struct file* filp)
{
    // 取出私有数据
    struct gpioled_dev* dev = (struct gpioled_dev*)filp->private_data;

    atomic_inc(&dev->lock); // 加一,释放驱动

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

    /* 原子操作 */
    // 初始化原子变量
    atomic_set(&gpioled.lock, 1);

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
MODULE_DESCRIPTION("atomic");