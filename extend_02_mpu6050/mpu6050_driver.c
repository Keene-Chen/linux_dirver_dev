#include "mpu6050.h"
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/ide.h>
#include <linux/input.h>
#include <linux/of_gpio.h>

#define DEV_COUNT 1
#define DEV_NAME  "mpu6050"

struct mpu6050_dev {
    dev_t devid;              // 设备号
    struct cdev cdev;         // cdev
    struct class* class;      // 类
    struct device* device;    // 设备
    struct device_node* node; // 设备节点
    int major;                // 主设备号
    int minor;                // 次设备号
    void* private_data;       // 私有数据
    struct mpu6050_data data; // mpu6050数据
} mpu6050;

/* 读取 mpu6050 的 N 个寄存器值 */
static int mpu6050_read_regs(struct mpu6050_dev* dev, u8 reg, void* val, int len)
{
    /* 设备树匹配成功会自动分配i2c_client结构体,这里使用private_data去保存client */
    struct i2c_client* client = (struct i2c_client*)dev->private_data;
    struct i2c_msg msg[2];
    int rc = 0;

    /* 1.msg[0] 发送要读取的从机地址 */
    msg[0].addr  = client->addr; // i2c从机地址
    msg[0].flags = 0;            // 0,发送数据
    msg[0].buf   = &reg;         // 待发送数据
    msg[0].len   = 1;            // 待发送数据长度

    /* 2.msg[1] 读取数据 */
    msg[1].addr  = client->addr; // i2c从机地址
    msg[1].flags = I2C_M_RD;     // 读取数据
    msg[1].buf   = val;          // 待发送数据
    msg[1].len   = len;          // 待发送数据长度

    rc = i2c_transfer(client->adapter, msg, 2);
    if (rc != 2) {
        rc = -EREMOTEIO;
        printk("i2c rd failed=%d reg=%06x len=%d\n", rc, reg, len);
    }
    return rc;
}

/* 写入 mpu6050 的 N 个寄存器值 */
static int mpu6050_write_regs(struct mpu6050_dev* dev, u8 reg, void* buf, int len)
{
    struct i2c_client* client = (struct i2c_client*)dev->private_data;
    struct i2c_msg msg;
    u8 data[256];
    int rc = 0;

    /* 构建发送数据,寄存器地址+实际数据 */
    data[0] = reg;
    memcpy(&data[1], buf, len);

    /* msg 发送数据 */
    msg.addr  = client->addr; // i2c从机地址
    msg.flags = 0;            // 发送数据
    msg.buf   = data;         // 待发送数据
    msg.len   = len + 1;      // 待发送数据长度:寄存器地址+实际数据

    rc = i2c_transfer(client->adapter, &msg, len);
    if (rc) {
        printk("i2c write failed");
    }
    return rc;
}

/* 读取 mpu6050 的一个寄存器值 */
static u_char mpu6050_read_reg(struct mpu6050_dev* dev, u8 reg)
{
    u8 data = 0;
    mpu6050_read_regs(dev, reg, &data, 1);

    return data;
}

/* 写入 mpu6050 的一个寄存器值 */
static void mpu6050_write_reg(struct mpu6050_dev* dev, u8 reg, u8 data)
{
    mpu6050_write_regs(dev, reg, &data, 1);
}

/**
 * @brief 读取 mpu6050 的数据,读取原始数据
 * @param dev mpu6050 设备
 */
void mpu6050_read_data(struct mpu6050_dev* dev)
{
    unsigned char buf[14], name[1];
    mpu6050_read_regs(dev, MPU6050_ACCEL_XOUT_H, buf, 14);

    dev->data.acc.x = (short)(buf[0] << 8 | buf[1]);
    dev->data.acc.y = (short)(buf[2] << 8 | buf[3]);
    dev->data.acc.z = (short)(buf[4] << 8 | buf[5]);
    dev->data.temp  = (short)(buf[6] << 8 | buf[7]);
    dev->data.anv.x = (short)(buf[8] << 8 | buf[9]);
    dev->data.anv.y = (short)(buf[10] << 8 | buf[11]);
    dev->data.anv.z = (short)(buf[12] << 8 | buf[13]);
    mpu6050_read_regs(dev, MPU6050_WHO_AM_I, name, 1);
    dev->data.name = name[0];
}

int mpu6050_open(struct inode* inode, struct file* filp)
{
    u8 value;
    filp->private_data = &mpu6050;

    /* 初始化 mpu6050 */
    mpu6050_write_reg(&mpu6050, MPU6050_PWR_MGMT_1, 0x80);
    mdelay(100);
    mpu6050_write_reg(&mpu6050, MPU6050_PWR_MGMT_1, 0x00);
    mpu6050_write_reg(&mpu6050, MPU6050_SMPLRT_DIV, 0x07);
    mpu6050_write_reg(&mpu6050, MPU6050_CONFIG, 0x06);
    mpu6050_write_reg(&mpu6050, MPU6050_ACCEL_CONFIG, 0x08); //+-4g
    mpu6050_write_reg(&mpu6050, MPU6050_GYRO_CONFIG, 0x08);  //+-500/s
    mpu6050_write_reg(&mpu6050, MPU_FIFO_EN_REG, 0x00);
    mpu6050_write_reg(&mpu6050, MPU6050_INT_ENABLE, 0x00);
    mpu6050_write_reg(&mpu6050, MPU6050_PWR_MGMT_1, 0x01);
    mpu6050_write_reg(&mpu6050, MPU6050_PWR_MGMT_2, 0x00);

    value = mpu6050_read_reg(&mpu6050, MPU6050_WHO_AM_I);
    printk("mpu6050 ID = %#X\r\n", value);

    return 0;
}

ssize_t mpu6050_read(struct file* filp, char __user* buf, size_t cnt, loff_t* offt)
{
    short data[7];
    long err                = 0;
    struct mpu6050_dev* dev = (struct mpu6050_dev*)filp->private_data;

    mpu6050_read_data(dev);
    data[0] = dev->data.acc.x;
    data[1] = dev->data.acc.y;
    data[2] = dev->data.acc.z;
    data[3] = dev->data.temp;
    data[4] = dev->data.acc.x;
    data[5] = dev->data.acc.y;
    data[6] = dev->data.acc.z;
    err     = copy_to_user(buf, data, sizeof(data));

    return 0;
}

/* mpu6050 操作函数 */
static const struct file_operations mpu6050_fops = {
    .owner = THIS_MODULE,
    .open  = mpu6050_open,
    .read  = mpu6050_read,
};

static int mpu6050_probe(struct i2c_client* client, const struct i2c_device_id* dev)
{
    int rc = 0;

    /* 注册字符设备 */
    // 1.分配设备号
    mpu6050.major = 0; // 内核指定分配设备号
    if (mpu6050.major) {
        mpu6050.devid = MKDEV(mpu6050.major, 0);
        rc            = register_chrdev_region(mpu6050.devid, DEV_COUNT, DEV_NAME);
    }
    else {
        rc            = alloc_chrdev_region(&mpu6050.devid, 0, DEV_COUNT, DEV_NAME);
        mpu6050.major = MAJOR(mpu6050.devid);
        mpu6050.minor = MINOR(mpu6050.devid);
    }
    if (rc < 0) {
        printk("mpu6050 chrdev region error!\r\n");
        goto fail_devid;
    }
    printk("major=%d,minor=%d\r\n", mpu6050.major, mpu6050.minor);

    // 2.注册字符设备
    mpu6050.cdev.owner = THIS_MODULE;
    cdev_init(&mpu6050.cdev, &mpu6050_fops);
    rc = cdev_add(&mpu6050.cdev, mpu6050.devid, DEV_COUNT);
    if (rc < 0) {
        rc = -EINVAL;
        goto fail_cdev;
    }

    // 3.动态创建设备的逻辑类
    mpu6050.class = class_create(THIS_MODULE, DEV_NAME);
    if (IS_ERR(mpu6050.class)) {
        rc = PTR_ERR(mpu6050.class);
        goto fail_class;
    }

    // 4.自动创建设备节点
    mpu6050.device = device_create(mpu6050.class, NULL, mpu6050.devid, NULL, DEV_NAME);
    if (IS_ERR(mpu6050.device)) {
        rc = PTR_ERR(mpu6050.device);
        goto fail_device;
    }
    printk("mpu6050 dirver init\r\n");

    mpu6050.private_data = client;

    return rc;

fail_device:
    device_destroy(mpu6050.class, mpu6050.devid);
    class_destroy(mpu6050.class);
fail_class:
    cdev_del(&mpu6050.cdev);
fail_cdev:
    unregister_chrdev_region(mpu6050.devid, DEV_COUNT);
fail_devid:
    return rc;
}

static int mpu6050_remove(struct i2c_client* client)
{
    // 1.删除字符设备
    cdev_del(&mpu6050.cdev);

    // 2.注销设备号
    unregister_chrdev_region(mpu6050.devid, DEV_COUNT);

    // 3.销毁设备节点
    device_destroy(mpu6050.class, mpu6050.devid);

    // 4.销毁设备的逻辑类
    class_destroy(mpu6050.class);

    printk("mpu6050 remove\r\n");

    return 0;
}

/* 传统匹配方式ID列表 */
static const struct i2c_device_id mpu6050_id[] = {
    { "atkmini,mpu6050", 0 },
    { /* Sentinel */ },
};

/* 设备树匹配列表 */
static const struct of_device_id mpu6050_of_match[] = {
    { .compatible = "atkmini,mpu6050" },
    { /* Sentinel */ },
};

/* i2c驱动结构体 */
static struct i2c_driver mpu6050_driver = {
    .probe = mpu6050_probe,
    .remove = mpu6050_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "mpu6050",
        .of_match_table = mpu6050_of_match, 
    },
    .id_table = mpu6050_id,
};

/* i2c 驱动注册与卸载辅助宏 */
module_i2c_driver(mpu6050_driver);

/* 驱动模块信息 */
MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("mpu6050 driver");
