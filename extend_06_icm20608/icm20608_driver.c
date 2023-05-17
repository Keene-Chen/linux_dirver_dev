#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>

#include "icm20608_reg.h"
#include <asm/io.h>
#include <linux/atomic.h>
#include <linux/fcntl.h>
#include <linux/i2c.h>
#include <linux/ide.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>

#define ICM20608_COUNT 1
#define ICM20608_NAME  "icm20608"
struct icm20608_dev {
    dev_t devid;            /* 设备号 */
    struct cdev cdev;       /* 字符设备 */
    struct class* class;    /* 类 */
    struct device* device;  /* 设备 */
    struct device_node* nd; /* 设备树节点 */
    int major;              /* 主设备号 */
    int minor;
    void* private_data;     /* 私有数据 */
    int cs_gpio;            /* 片选所使用的 GPIO 编号*/
    signed int gyro_x_adc;  /* 陀螺仪 X 轴原始值*/
    signed int gyro_y_adc;  /* 陀螺仪 Y 轴原始值*/
    signed int gyro_z_adc;  /* 陀螺仪 Z 轴原始值*/
    signed int accel_x_adc; /* 加速度计 X 轴原始值*/
    signed int accel_y_adc; /* 加速度计 Y 轴原始值*/
    signed int accel_z_adc; /* 加速度计 Z 轴原始值*/
    signed int temp_adc;    /* 温度原始值*/
};

static struct icm20608_dev icm20608dev;

/* SPI 读寄存器 */
static int icm20608_read_regs(struct icm20608_dev* dev, u8 reg, u8* buf, u8 len)
{
#if 1
    int ret = -1;
    unsigned char txdata[1];
    unsigned char* rxdata;
    struct spi_message m;
    struct spi_transfer* t;
    struct spi_device* spi = (struct spi_device*)dev->private_data;

    t = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL); /* 申请内存 */
    if (!t) {
        return -ENOMEM;
    }

    rxdata = kzalloc(sizeof(char) * len, GFP_KERNEL); /* 申请内存 */
    if (!rxdata) {
        goto out1;
    }

    /* 一共发送len+1个字节的数据，第一个字节为
    寄存器首地址，一共要读取len个字节长度的数据，*/
    txdata[0] = reg | 0x80;      /* 写数据的时候首寄存器地址bit8要置1 */
    t->tx_buf = txdata;          /* 要发送的数据 */
    t->rx_buf = rxdata;          /* 要读取的数据 */
    t->len    = len + 1;         /* t->len=发送的长度+读取的长度 */
    spi_message_init(&m);        /* 初始化spi_message */
    spi_message_add_tail(t, &m); /* 将spi_transfer添加到spi_message队列 */
    ret = spi_sync(spi, &m);     /* 同步发送 */
    if (ret) {
        goto out2;
    }

    memcpy(buf, rxdata + 1, len); /* 只需要读取的数据 */

out2:
    kfree(rxdata); /* 释放内存 */
out1:
    kfree(t); /* 释放内存 */

    return ret;
#else
    struct spi_device* spi = (struct spi_device*)dev->private_data;
    int ret                = 0;
    u8 data                = 0;
    data                   = reg | 0x80;
    spi_write_then_read(spi, &data, 1, buf, len);
    return ret;
#endif
}
/* SPI 写寄存器 */
static int icm20608_write_regs(struct icm20608_dev* dev, u8 reg, void* buf, u8 len)
{
#if 1
    int ret = -1;
    unsigned char* txdata;
    struct spi_message m;
    struct spi_transfer* t;
    struct spi_device* spi = (struct spi_device*)dev->private_data;

    t = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL); /* 申请内存 */
    if (!t) {
        return -ENOMEM;
    }

    txdata = kzalloc(sizeof(char) + len, GFP_KERNEL);
    if (!txdata) {
        goto out1;
    }

    /* 一共发送len+1个字节的数据，第一个字节为
    寄存器首地址，len为要写入的寄存器的集合，*/
    *txdata = reg & ~0x80;        /* 写数据的时候首寄存器地址bit8要清零 */
    memcpy(txdata + 1, buf, len); /* 把len个寄存器拷贝到txdata里，等待发送 */
    t->tx_buf = txdata;           /* 要发送的数据 */
    t->len    = len + 1;          /* t->len=发送的长度+读取的长度 */
    spi_message_init(&m);         /* 初始化spi_message */
    spi_message_add_tail(t, &m);  /* 将spi_transfer添加到spi_message队列 */
    ret = spi_sync(spi, &m);      /* 同步发送 */
    if (ret) {
        goto out2;
    }

out2:
    kfree(txdata); /* 释放内存 */
out1:
    kfree(t); /* 释放内存 */
    return ret;
#else
    struct spi_device* spi = (struct spi_device*)dev->private_data;
    u8 data                = 0;
    u8* txdata;
    int ret = 0;
    data    = reg & ~0x80;
    txdata  = kzalloc(len + 1, GFP_KERNEL);
    txdata  = kzalloc(sizeof(char) + len, GFP_KERNEL);
    if (!txdata) {
        printk("txdata kzalloc failed\r\n");
        return -1;
    }
    txdata[0] = data;
    memcpy(txdata + 1, buf, len);

    ret = spi_write(spi, txdata, len + 1); // 发送数据

    kfree(txdata);

    return 0;
#endif
}

/* 读取ICM20608 单个寄存器的值 */
static unsigned char icm20608_read_onereg(struct icm20608_dev* dev, u8 reg)
{
    unsigned char value = 0;
    icm20608_read_regs(dev, reg, &value, 1);
    return value;
}

/* 写入ICM20608 单个寄存器的值 */
static int icm20608_write_onereg(struct icm20608_dev* dev, u8 reg, u8 value)
{

    u8 buf = value;
    return icm20608_write_regs(dev, reg, &buf, 1);
}

/*
 * @description	: 读取ICM20608的数据，读取原始数据，包括三轴陀螺仪、
 * 				: 三轴加速度计和内部温度。
 * @param - dev	: ICM20608设备
 * @return 		: 无。
 */
void icm20608_readdata(struct icm20608_dev* dev)
{
    unsigned char data[14] = { 0 };
    icm20608_read_regs(dev, ICM20_ACCEL_XOUT_H, data, 14);

    dev->accel_x_adc = (signed short)((data[0] << 8) | data[1]);
    dev->accel_y_adc = (signed short)((data[2] << 8) | data[3]);
    dev->accel_z_adc = (signed short)((data[4] << 8) | data[5]);
    dev->temp_adc    = (signed short)((data[6] << 8) | data[7]);
    dev->gyro_x_adc  = (signed short)((data[8] << 8) | data[9]);
    dev->gyro_y_adc  = (signed short)((data[10] << 8) | data[11]);
    dev->gyro_z_adc  = (signed short)((data[12] << 8) | data[13]);
}

/* 初始化20608 */
static void icm20608reg_init(struct icm20608_dev* dev)
{
    u8 value = 0;
    icm20608_write_onereg(dev, ICM20_PWR_MGMT_1, 0x80); /* 复位，复位后位0x40,睡眠模式 */
    mdelay(50);
    icm20608_write_onereg(dev, ICM20_PWR_MGMT_1, 0x01); /* 关闭睡眠模式，自动选择时钟 */
    mdelay(50);

    value = icm20608_read_onereg(dev, ICM20_WHO_AM_I);

    printk("ICM20608_ID=%#X", value);

    icm20608_write_onereg(dev, ICM20_SMPLRT_DIV, 0x00);    /* 输出速率是内部采样率					*/
    icm20608_write_onereg(dev, ICM20_GYRO_CONFIG, 0x18);   /* 陀螺仪±2000dps量程 				*/
    icm20608_write_onereg(dev, ICM20_ACCEL_CONFIG, 0x18);  /* 加速度计±16G量程 					*/
    icm20608_write_onereg(dev, ICM20_CONFIG, 0x04);        /* 陀螺仪低通滤波BW=20Hz 				*/
    icm20608_write_onereg(dev, ICM20_ACCEL_CONFIG2, 0x04); /* 加速度计低通滤波BW=21.2Hz */
    icm20608_write_onereg(dev, ICM20_PWR_MGMT_2, 0x00); /* 打开加速度计和陀螺仪所有轴 */
    icm20608_write_onereg(dev, ICM20_LP_MODE_CFG, 0x00); /* 关闭低功耗 						*/
    icm20608_write_onereg(dev, ICM20_FIFO_EN, 0x00);     /* 关闭FIFO	*/
}

static int icm20608_open(struct inode* inode, struct file* filp)
{
    filp->private_data = &icm20608dev;

    return 0;
}

static int icm20608_close(struct inode* inode, struct file* filp)
{

    return 0;
}

/* 读数据 */
static ssize_t icm20608_read(struct file* filp, char __user* buf, size_t cnt, loff_t* off)
{
    signed int data[7];
    long err                 = 0;
    struct icm20608_dev* dev = (struct icm20608_dev*)filp->private_data;

    icm20608_readdata(dev);
    data[0] = dev->gyro_x_adc;
    data[1] = dev->gyro_y_adc;
    data[2] = dev->gyro_z_adc;
    data[3] = dev->accel_x_adc;
    data[4] = dev->accel_y_adc;
    data[5] = dev->accel_z_adc;
    data[6] = dev->temp_adc;
    err     = copy_to_user(buf, data, sizeof(data));

    return 0;
}

/* 驱动结构体 */
static struct file_operations icm20608_fops = {
    .owner   = THIS_MODULE,
    .open    = icm20608_open,
    .release = icm20608_close,
    .read    = icm20608_read,
};

static int icm20608_probe(struct spi_device* spi)
{
    int ret = 0;
    /* 1、分配设备号 */
    if (icm20608dev.major) /* 有主设备号 */
    {
        icm20608dev.devid = MKDEV(icm20608dev.major, 0);
        ret = register_chrdev_region(icm20608dev.devid, ICM20608_COUNT, ICM20608_NAME);
    }
    else /* 没有主设备号 */
    {
        ret = alloc_chrdev_region(&icm20608dev.devid, 0, ICM20608_COUNT, ICM20608_NAME);
        icm20608dev.major = MAJOR(icm20608dev.devid);
        icm20608dev.minor = MINOR(icm20608dev.devid);
    }
    if (ret < 0) /* 设备号分配失败 */
    {
        printk("fail_devid\r\n");
        ret = -1;
        goto fail_devid;
    }
    printk("major=%d,minor=%d\r\n", icm20608dev.major, icm20608dev.minor);

    /* 2、注册字符驱动 ledcdev*/
    icm20608dev.cdev.owner = THIS_MODULE;
    cdev_init(&icm20608dev.cdev, &icm20608_fops);
    ret = cdev_add(&icm20608dev.cdev, icm20608dev.devid, ICM20608_COUNT);
    if (ret < 0) {
        printk("fail_cdev\r\n");
        ret = -EINVAL;
        goto fail_cdev;
    }

    /* 3、创建类 */
    icm20608dev.class = class_create(THIS_MODULE, ICM20608_NAME);
    if (IS_ERR(icm20608dev.class)) {
        printk("fail_class\r\n");
        ret = PTR_ERR(icm20608dev.class);
        goto fail_class;
    }

    /* 4、创建设备 */
    icm20608dev.device =
        device_create(icm20608dev.class, NULL, icm20608dev.devid, NULL, ICM20608_NAME);
    if (IS_ERR(icm20608dev.device)) {
        printk("fail_device\r\n");
        ret = PTR_ERR(icm20608dev.device);
        goto fail_device;
    }

    // /* 设置为输出高电平 */
    // ret = gpio_direction_output(icm20608dev.cs_gpio, 1);

    /* 初始化 spi_device*/
    spi->mode = SPI_MODE_0;
    spi_setup(spi);

    /* 设置私有数据 */
    icm20608dev.private_data = spi;

    /* 初始化icm20608 */
    icm20608reg_init(&icm20608dev);

    return 0;
// gpioreq_fail:
// fail_cs_gpio:
//     device_destroy(icm20608dev.class, icm20608dev.devid); /* 删除device */
fail_device:
    class_destroy(icm20608dev.class); /* 删除class  ，先删除device */
fail_class:
    cdev_del(&icm20608dev.cdev); /* 删除字符设备 */
fail_cdev:
    unregister_chrdev_region(icm20608dev.devid, ICM20608_COUNT); /* 注销设备号 */
fail_devid:

    return ret;
}

/*
 * spi 驱动的 remove 函数，移除 spi 驱动的时候此函数会执行
 * @description
 * @param – client : spi 设备
 * @return
 */
static int icm20608_remove(struct spi_device* spi)
{
    /* 注销设备号 */
    unregister_chrdev_region(icm20608dev.devid, ICM20608_COUNT);

    /* 删除字符设备 */
    cdev_del(&icm20608dev.cdev);

    /* 删除device */
    device_destroy(icm20608dev.class, icm20608dev.devid);

    /* 删除class  ，先删除device */
    class_destroy(icm20608dev.class);

    /* 释放gpio */
    gpio_free(icm20608dev.cs_gpio);
    return 0;
}
/* 传统匹配方式 ID 列表 */
static const struct spi_device_id icm20608_id[] = { { "atkmini,icm20608", 0 }, {} };

/* 设备树匹配列表 */
static const struct of_device_id icm20608_of_match[] = { { .compatible = "atkmini,icm20608" },
                                                         { /* Sentinel */ } };

/* SPI 驱动结构体 */
static struct spi_driver icm20608_driver = {
    .probe    = icm20608_probe,
    .remove   = icm20608_remove,
    .driver   = { .name = "icm20608", .owner = THIS_MODULE, .of_match_table = icm20608_of_match },
    .id_table = icm20608_id
};

/* 驱动入口 */
static int __init icm20608_init(void)
{

    return spi_register_driver(&icm20608_driver);
}
/* 驱动卸载 */
static void __exit icm20608_exit(void)
{
    spi_unregister_driver(&icm20608_driver);
}

module_init(icm20608_init);
module_exit(icm20608_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xiaotang");