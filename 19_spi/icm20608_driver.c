/**
 * @file    : icm20608_driver.c
 * @author  : KeeneChen
 * @date    : 2023.04.20-17:08:37
 * @details : icm20608_driver
 */

/**
 ** void spi_message_init(struct spi_message *m)
 * @brief spi_message 初始化
 * @param m 要初始化的 spi_message
 *
 ** void spi_message_add_tail(struct spi_transfer *t, struct spi_message *m)
 * @brief 完成以后需要将 spi_transfer 添加到 spi_message 队列中
 * @param t 要添加到队列中的 spi_transfer
 * @param m spi_transfer 要加入的 spi_message
 *
 ** int spi_sync(struct spi_device *spi, struct spi_message *message)
 * @brief spi_message 准备好以后既可以进行数据传输,同步传输会阻塞的等待 SPI 数据传输完成
 * @param spi 要进行数据传输的 spi_device
 * @param message要传输的 spi_message
 *
 ** int spi_async(struct spi_device *spi, struct spi_message *message)
 * @brief 异步传输不会阻塞的等到 SPI 数据传输完成,异步传输需要设置 spi_message 中的 complete成员变量
 * @param spi 要进行数据传输的 spi_device
 * @param message要传输的 spi_message
 */

#include "icm20608_reg.h"
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/input.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>

#define DEV_NAME  "icm20608" // 设备名
#define DEV_COUNT 1          // 设备个数

struct icm20608_dev {
    dev_t devid;              // 设备号
    struct cdev cdev;         // cdev
    struct class* class;      // 类
    struct device* device;    // 设备
    struct device_node* node; // 设备节点
    int major;                // 主设备号
    int minor;                // 次设备号
    void* private_data;       // 私有数据
    // int cs_gpios;          // 片选引脚
    struct icm20608_data data;
} icm20608;

#if 0
/* 读取 icm20608 的 N 个寄存器值 */
static int icm20608_read_regs(struct icm20608_dev* dev, u8 reg, void* val, int len)
{
    int rc                 = 0;
    struct spi_device* spi = (struct spi_device*)dev->private_data;
    struct spi_message msg;
    struct spi_transfer* tfer;
    u8 tx_data[len];

    /* 构建 spi_transfer */
    tfer = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);

    /* 1.发送待读取的寄存器地址 */
    tx_data[0]   = reg | 0x80;
    tfer->tx_buf = tx_data;
    tfer->len    = 1;

    spi_message_init(&msg);
    spi_message_add_tail(tfer, &msg);
    spi_sync(spi, &msg);

    /* 2.读取数据并返回 */
    tx_data[0]   = 0xff;
    tfer->rx_buf = val;
    tfer->len    = len;
    spi_message_init(&msg);
    spi_message_add_tail(tfer, &msg);
    rc = spi_sync(spi, &msg);

    /* 释放 spi_transfer */
    kfree(tx_data);

    return rc;
}

/* 写入 icm20608 的 N 个寄存器值 */
static int icm20608_write_regs(struct icm20608_dev* dev, u8 reg, void* buf, int len)
{
    int rc                 = 0;
    struct spi_device* spi = (struct spi_device*)dev->private_data;
    struct spi_message msg;
    struct spi_transfer* tfer;
    u8 tx_data[len];

    /* 构建 spi_transfer */
    tfer = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);

    /* 1.发送待发送的寄存器地址 */
    tx_data[0]   = reg & ~0x80;
    tfer->tx_buf = tx_data;
    tfer->len    = 1;

    spi_message_init(&msg);
    spi_message_add_tail(tfer, &msg);
    spi_sync(spi, &msg);

    /* 2.写入数据 */
    tfer->tx_buf = buf;
    tfer->len    = len;
    spi_message_init(&msg);
    spi_message_add_tail(tfer, &msg);
    rc = spi_sync(spi, &msg);

    /* 释放 spi_transfer */
    kfree(tx_data);

    return rc;
}
#else
/* 读取 icm20608 的 N 个寄存器值 */
static int icm20608_read_regs(struct icm20608_dev* dev, u8 reg, void* val, int len)
{
    int rc                 = 0;
    struct spi_device* spi = (struct spi_device*)dev->private_data;
    u8 data;

    /* 1.发送待读取的寄存器地址  */
    data = reg | 0x80;
    spi_write(spi, &data, 1);

    /* 2.读取数据 */
    spi_read(spi, val, len);

    return rc;
}

/* 写入 icm20608 的 N 个寄存器值 */
static int icm20608_write_regs(struct icm20608_dev* dev, u8 reg, void* buf, int len)
{
    int rc                 = 0;
    struct spi_device* spi = (struct spi_device*)dev->private_data;
    u8 data;

    /* 1.发送待写入的寄存器地址  */
    data = reg & ~0x80;
    spi_write(spi, &data, 1);

    /* 2.写入数据 */
    spi_write(spi, buf, len);

    return rc;
}
#endif

/* 读取 icm20608 的一个寄存器值 */
static u8 icm20608_read_reg(struct icm20608_dev* dev, u8 reg)
{
    u8 data = 0;
    icm20608_read_regs(dev, reg, &data, 1);

    return data;
}

/* 写入 icm20608 的一个寄存器值 */
static void icm20608_write_reg(struct icm20608_dev* dev, u8 reg, u8 data)
{
    icm20608_write_regs(dev, reg, &data, 1);
}

/* icm20608 数据读取 */
static void icm20608_read_data(struct icm20608_dev* dev)
{
    u8 data[14] = { 0 };
    icm20608_read_regs(dev, ICM20_ACCEL_XOUT_H, data, 14);

    dev->data.acc.x  = (int)((data[0] << 8) | data[1]);
    dev->data.acc.y  = (int)((data[2] << 8) | data[3]);
    dev->data.acc.z  = (int)((data[4] << 8) | data[5]);
    dev->data.temp   = (int)((data[6] << 8) | data[7]);
    dev->data.gyro.x = (int)((data[8] << 8) | data[9]);
    dev->data.gyro.y = (int)((data[10] << 8) | data[11]);
    dev->data.gyro.z = (int)((data[12] << 8) | data[13]);
}

void icm20608_reginit(void)
{
    u8 value = 0;

    icm20608_write_reg(&icm20608, ICM20_PWR_MGMT_1, 0x80);
    mdelay(50);
    icm20608_write_reg(&icm20608, ICM20_PWR_MGMT_1, 0x01);
    mdelay(50);

    value = icm20608_read_reg(&icm20608, ICM20_WHO_AM_I);
    printk("ICM20608 ID = %#X\r\n", value);

    icm20608_write_reg(&icm20608, ICM20_SMPLRT_DIV, 0x00);    // 输出速率是内部采样率
    icm20608_write_reg(&icm20608, ICM20_GYRO_CONFIG, 0x18);   // 陀螺仪±2000dps量程
    icm20608_write_reg(&icm20608, ICM20_ACCEL_CONFIG, 0x18);  // 加速度计±16G量程
    icm20608_write_reg(&icm20608, ICM20_CONFIG, 0x04);        // 陀螺仪低通滤波BW=20Hz
    icm20608_write_reg(&icm20608, ICM20_ACCEL_CONFIG2, 0x04); // 加速度计低通滤波BW=21.2Hz
    icm20608_write_reg(&icm20608, ICM20_PWR_MGMT_2, 0x00); // 打开加速度计和陀螺仪所有轴
    icm20608_write_reg(&icm20608, ICM20_LP_MODE_CFG, 0x00); // 关闭低功耗
    icm20608_write_reg(&icm20608, ICM20_FIFO_EN, 0x00);     // 关闭FIFO
}

int icm20608_open(struct inode* inode, struct file* filp)
{
    filp->private_data = &icm20608; // 设置私有数据

    return 0;
}

ssize_t icm20608_read(struct file* filp, char __user* buf, size_t cnt, loff_t* offt)
{
    signed int data[7];
    long err                 = 0;
    struct icm20608_dev* dev = (struct icm20608_dev*)filp->private_data;

    icm20608_read_data(dev);
    data[0] = dev->data.acc.x;
    data[1] = dev->data.acc.y;
    data[2] = dev->data.acc.z;
    data[3] = dev->data.gyro.x;
    data[4] = dev->data.gyro.y;
    data[5] = dev->data.gyro.z;
    data[6] = dev->data.temp;
    err     = copy_to_user(buf, data, sizeof(data));

    return 0;
}

/* AP3216C 操作函数 */
static const struct file_operations icm20608_fops = {
    .owner = THIS_MODULE,
    .open  = icm20608_open,
    .read  = icm20608_read,
};

static int icm20608_probe(struct spi_device* spi)
{
    int rc = 0;

    /* 注册字符设备 */
    // 1.分配设备号
    icm20608.major = 0; // 内核指定分配设备号
    if (icm20608.major) {
        icm20608.devid = MKDEV(icm20608.major, 0);
        rc             = register_chrdev_region(icm20608.devid, DEV_COUNT, DEV_NAME);
    }
    else {
        rc             = alloc_chrdev_region(&icm20608.devid, 0, DEV_COUNT, DEV_NAME);
        icm20608.major = MAJOR(icm20608.devid);
        icm20608.minor = MINOR(icm20608.devid);
    }
    if (rc < 0) {
        printk("icm20608 chrdev region error!\r\n");
        goto fail_devid;
    }
    printk("major=%d,minor=%d\r\n", icm20608.major, icm20608.minor);

    // 2.注册字符设备
    icm20608.cdev.owner = THIS_MODULE;
    cdev_init(&icm20608.cdev, &icm20608_fops);
    rc = cdev_add(&icm20608.cdev, icm20608.devid, DEV_COUNT);
    if (rc < 0) {
        rc = -EINVAL;
        goto fail_cdev;
    }

    // 3.动态创建设备的逻辑类
    icm20608.class = class_create(THIS_MODULE, DEV_NAME);
    if (IS_ERR(icm20608.class)) {
        rc = PTR_ERR(icm20608.class);
        goto fail_class;
    }

    // 4.自动创建设备节点
    icm20608.device = device_create(icm20608.class, NULL, icm20608.devid, NULL, DEV_NAME);
    if (IS_ERR(icm20608.device)) {
        rc = PTR_ERR(icm20608.device);
        goto fail_device;
    }

    // 初始化spi_device
    spi->mode = SPI_MODE_0; // MODE0，CPOL=0，CPHA=0
    spi_setup(spi);
    icm20608.private_data = spi; // 设置私有数据

    // 初始化icm20608内部寄存器
    icm20608_reginit();

    printk("icm20608 dirver init\r\n");

    return rc;

fail_device:
    device_destroy(icm20608.class, icm20608.devid);
    class_destroy(icm20608.class);
fail_class:
    cdev_del(&icm20608.cdev);
fail_cdev:
    unregister_chrdev_region(icm20608.devid, DEV_COUNT);
fail_devid:
    return rc;
}

static int icm20608_remove(struct spi_device* spi)
{
    // 1.删除字符设备
    cdev_del(&icm20608.cdev);

    // 2.注销设备号
    unregister_chrdev_region(icm20608.devid, DEV_COUNT);

    // 3.销毁设备节点
    device_destroy(icm20608.class, icm20608.devid);

    // 4.销毁设备的逻辑类
    class_destroy(icm20608.class);

    printk("icm20608 remove\r\n");

    return 0;
}

/* 传统匹配方式ID列表 */
static const struct spi_device_id icm20608_id[] = {
    { "atkmini,icm20608", 0 },
    { /* Sentinel */ },
};

/* 设备匹配列表与设备树匹配 */
static const struct of_device_id icm20608_of_match[] = {
    { .compatible = "atkmini,icm20608" },
    { /* Sentinel */ },
};

/* i2c 驱动结构体 */
static struct spi_driver icm20608_driver = {
    .probe  = icm20608_probe,
    .remove = icm20608_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "icm20608",
        .of_match_table = icm20608_of_match,
    },
    .id_table = icm20608_id,
};

/* spi 驱动注册与卸载辅助宏 */
module_spi_driver(icm20608_driver);

/* 驱动模块信息 */
MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("icm20608 driver");