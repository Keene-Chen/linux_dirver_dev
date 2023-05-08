/**
 * @file    : dht11_driver.c
 * @author  : KeeneChen
 * @date    : 2023.04.09-12:19:50
 * @details : dht11 misc driver
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/ide.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#define DHT11_PIN        dht11_misc.dht11_gpio
#define DHT11_IO_OUT()   gpio_direction_output(DHT11_PIN, 1);
#define DHT11_IO_IN()    gpio_direction_input(DHT11_PIN)
#define DHT11_WRITE(bit) gpio_set_value(DHT11_PIN, bit)
#define DHT11_READ()     gpio_get_value(DHT11_PIN)

#define DEV_DTS_NODE_PATH  "/dht11"             // 设备树节点的路径，在根节点下
#define DEV_PIN_DTS_NAME   "dht11-gpio"         // GPIO引脚的属性名
#define DEV_NAME           "dht11"              // 设备名  /dev/dht11
#define DEV_DTS_COMPATIBLE "atkmini,dht11-gpio" // 设备匹配属性 compatible
#define DEV_MINOR          145                  // 次设备号

struct dht11_misc_dev {
    struct miscdevice cdev;   // 杂项设备
    struct class* class;      // 逻辑类
    struct device* device;    // 设备
    struct device_node* node; // 设备节点
    dev_t devid;              // 设备号
    int dht11_gpio;           // dht11所使用的GPIO编号
    spinlock_t lock;          // 自旋锁
} dht11_misc;

static int dht11_wait_for_ready(void)
{
    int timeout;

    /* 等待低电平到来 */
    timeout = 400;
    while (DHT11_READ() && timeout) {
        udelay(1);
        --timeout;
    }
    if (!timeout) {
        printk("timeout %d\n", __LINE__);
        return -1;
    }

    /* 等待高电平到来    */
    timeout = 1000;
    while (!DHT11_READ() && timeout) {
        udelay(1);
        --timeout;
    }
    if (!timeout) {
        printk("timeout %d\n", __LINE__);
        return -1;
    }

    /* 等待高电平结束 */
    timeout = 1000;
    while (DHT11_READ() && timeout) {
        udelay(1);
        --timeout;
    }
    if (!timeout) {
        printk("timeout %d\n", __LINE__);
        return -1;
    }

    return 0;
}

static int dht11_start(void)
{
    DHT11_IO_OUT();
    DHT11_WRITE(0);
    mdelay(20);
    DHT11_WRITE(1);
    udelay(30);
    DHT11_IO_IN();
    udelay(2);

    if (dht11_wait_for_ready())
        return -1;
    return 0;
}

static int dht11_read_byte(unsigned char* byte)
{
    unsigned char i;
    unsigned char bit  = 0;
    unsigned char data = 0;
    int timeout        = 0;

    for (i = 0; i < 8; i++) {
        /* 等待变为低电平 */
        timeout = 1000;
        while (DHT11_READ() && timeout) {
            udelay(1);
            --timeout;
        }
        if (!timeout) {
            printk("timeout %d\n", __LINE__);
            return -1;
        }

        /* 等待变为高电平 */
        timeout = 1000;
        while (!DHT11_READ() && timeout) {
            udelay(1);
            --timeout;
        }
        if (!timeout) {
            printk("timeout %d\n", __LINE__);
            return -1;
        }
        udelay(40);

        bit = DHT11_READ();

        data <<= 1;
        if (bit) {
            data |= 0x01;
        }

        *byte = data;
    }
    return 0;
}

static int dht11_read_data(struct dht11_misc_dev* dev, unsigned char* data)
{
    int i   = 0;
    int ret = 0;
    unsigned long flags;

    // 关闭中断,防止时序被中断破坏
    spin_lock_irqsave(&dht11_misc.lock, flags);

    // 启动信号
    if (dht11_start() != 0) {
        printk("dht11 start failed\n");
        ret = -EFAULT;
        goto failed;
    }

    // 读出5字节数据
    for (i = 0; i < 5; i++) {
        if (dht11_read_byte(&data[i])) {
            printk("read data err\n");
            ret = -EAGAIN;
            goto failed;
        }
    }

    // 打开中断
    spin_unlock_irqrestore(&dht11_misc.lock, flags);

    // 校验数据
    if (data[4] != (data[0] + data[1] + data[2] + data[3])) {
        printk("check data failed\n");
        ret = -EAGAIN;
        goto failed;
    }

    return 0;

failed:
    spin_unlock_irqrestore(&dht11_misc.lock, flags);
    return ret;
}

static ssize_t dht11_show_data(struct device* dev, struct device_attribute* attr, char* buf)
{
    int rc                = 0;
    unsigned char data[5] = { 0 };

    rc = dht11_read_data(&dht11_misc, data);
    if (rc < 0) {
        return -EINVAL;
    }

    return sprintf(buf, "%d.%d,%d.%d\n", data[0], data[1], data[2], data[3]);
}

static DEVICE_ATTR(value, S_IRUGO, dht11_show_data, NULL);
static struct attribute* dht11_attrs[]                = { &dev_attr_value.attr, NULL };
static const struct attribute_group dht11_attrs_group = {
    .attrs = dht11_attrs,
};
static const struct attribute_group* dht11_attr_groups[] = { &dht11_attrs_group, NULL };

static int dht11_misc_open(struct inode* inode, struct file* filp)
{
    filp->private_data = &dht11_misc; // 设置私有数据
    return 0;
}

static ssize_t dht11_misc_read(struct file* filp, char __user* buf, size_t count, loff_t* offt)
{
    int rc                     = 0;
    unsigned char data[5]      = { 0 };
    struct dht11_misc_dev* dev = filp->private_data;

    if (count != 5)
        return -EINVAL;

    // 读取dht11数据并保存到data数组中
    rc = dht11_read_data(dev, data);
    if (rc < 0) {
        return -EINVAL;
    }

    // 将数据拷贝回用户空间
    rc = copy_to_user(buf, data, 5);
    if (rc < 0) {
        return -EINVAL;
    }

    return rc;
}

static struct file_operations dht11_misc_fops = {
    .owner = THIS_MODULE,
    .open  = dht11_misc_open,
    .read  = dht11_misc_read,
};

/* 驱动入口函数 */
static int dht11_misc_probe(struct platform_device* pdev)
{
    int rc = 0;

    // 注册杂项设备
    dht11_misc.cdev.name   = DEV_NAME;
    dht11_misc.cdev.minor  = DEV_MINOR;
    dht11_misc.cdev.groups = dht11_attr_groups;
    dht11_misc.cdev.fops   = &dht11_misc_fops;

    rc = misc_register(&dht11_misc.cdev);
    if (rc < 0) {
        printk("dht11 misc_register failed\r\n");
        rc = -EINVAL;
        return rc;
    }
    else {
        printk("dht11 misc_register succeed\r\n");
    }

    // 获取设备节点
    dht11_misc.node = pdev->dev.of_node;
    // beep_misc.node = of_find_node_by_path("/dht11");
    if (dht11_misc.node == NULL) {
        printk("dht11 node not find!\r\n");
        return -EINVAL;
    }

    // 获取设备树中的 gpio 属性,得到 dht11 所使用的 gpio 编号
    dht11_misc.dht11_gpio = of_get_named_gpio(dht11_misc.node, DEV_PIN_DTS_NAME, 0);
    if (dht11_misc.dht11_gpio < 0) {
        printk("can't get dht11-gpio");
        return -EINVAL;
    }

    return rc;
}

/* 驱动出口函数 */
static int dht11_misc_remove(struct platform_device* pdev)
{
    int rc = 0;

    // 卸载杂项设备
    rc = misc_deregister(&dht11_misc.cdev);
    if (rc < 0) {
        printk("dht11 misc_deregister failed\r\n");
        rc = -EINVAL;
        return rc;
    }
    else {
        printk("dht11 misc_deregister succeed\r\n");
    }

    return rc;
}

static struct of_device_id dht11_misc_of_match[] = {
    { .compatible = DEV_DTS_COMPATIBLE },
    { /* Sentinel */ },
};

/* platform 结构体 */
static struct platform_driver dht11_misc_dev = {
    .driver = {
        .name = DEV_NAME,
        .of_match_table = dht11_misc_of_match,
    },
    .probe  = dht11_misc_probe,
    .remove = dht11_misc_remove,
};

/* platform 驱动加载与卸载辅助宏 */
module_platform_driver(dht11_misc_dev);

/* 驱动模块信息 */
MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("dht11 misc driver");