/**
 * @file    : ap3216c_driver.c
 * @author  : KeeneChen
 * @date    : 2023.04.17-19:00:35
 * @details : ap3216c_driver
 */

/**
 * i2c1
 * MX6UL_PAD_UART4_TX_DATA__I2C1_SCL 43
 * MX6UL_PAD_UART4_RX_DATA__I2C1_SDA 42
 *
 ** int i2c_transfer(struct i2c_adapter *adap,struct i2c_msg *msgs,int num)
 * @brief I2C 设备数据收发函数
 * @param adap 所使用的 I2C 适配器, i2c_client 会保存其对应的 i2c_adapter
 * @param msgs I2C 要发送的一个或多个消息
 * @param num 消息数量,也就是 msgs 的数量
 * @return 负值,失败,其他非负值,发送的 msgs 数量
 *
 ** int i2c_master_send(const struct i2c_client *client,const char *buf,int count)
 * @brief I2C 设备数据收发函数
 * @param client I2C 设备对应的 i2c_client
 * @param buf 要发送的数据
 * @param count 要发送的数据字节数,要小于 64KB,以为 i2c_msg 的 len
 * 成员变量是一个 u16(无符号 16位)类型的数据
 * @return 负值,失败,其他非负值,发送的字节数
 *
 ** int i2c_master_recv(const struct i2c_client *client,char *buf,int count)
 * @brief I2C 设备数据收发函数
 * @param client I2C 设备对应的 i2c_client
 * @param buf 要接收的数据
 * @param count 要接收的数据字节数,要小于 64KB,以为 i2c_msg 的 len
 * 成员变量是一个 u16(无符号 16位)类型的数据
 * @return 负值,失败,其他非负值,发送的字节数
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/ide.h>
#include <linux/input.h>
#include <linux/of_gpio.h>

/* AP3316C寄存器 */
#define AP3216C_ADDR        0X1E // AP3216C器件地址
#define AP3216C_SYSTEMCONG  0x00 // 配置寄存器
#define AP3216C_INTSTATUS   0X01 // 中断状态寄存器
#define AP3216C_INTCLEAR    0X02 // 中断清除寄存器
#define AP3216C_IRDATALOW   0x0A // IR数据低字节
#define AP3216C_IRDATAHIGH  0x0B // IR数据高字节
#define AP3216C_ALSDATALOW  0x0C // ALS数据低字节
#define AP3216C_ALSDATAHIGH 0X0D // ALS数据高字节
#define AP3216C_PSDATALOW   0X0E // PS数据低字节
#define AP3216C_PSDATAHIGH  0X0F // PS数据高字节

#define DEV_NAME  "ap3216c" // 设备名
#define DEV_COUNT 1         // 设备个数

struct ap3216c_dev {
    dev_t devid;                // 设备号
    struct cdev cdev;           // cdev
    struct class* class;        // 类
    struct device* device;      // 设备
    struct device_node* node;   // 设备节点
    int major;                  // 主设备号
    int minor;                  // 次设备号
    void* private_data;         // 私有数据
    unsigned short ir, als, ps; // 三个光传感器数据
} ap3216c;

/* 读取 AP3216C 的 N 个寄存器值 */
static int ap3216c_read_regs(struct ap3216c_dev* dev, u8 reg, void* val, int len)
{
    /* 设备树匹配成功会自动分配i2c_client结构体,这里使用private_data去保存client */
    struct i2c_client* client = (struct i2c_client*)dev->private_data;
    struct i2c_msg msg[2];

    /* 1.msg[0] 发送要读取的从机地址 */
    msg[0].addr  = client->addr; // i2c从机地址
    msg[0].flags = 0;            // 0,发送数据
    msg[0].buf   = &reg;         // 待发送数据
    msg[0].len   = 1;            // 待发送数据长度

    /* 2.msg[1] 读取数据 */
    msg[1].addr  = client->addr; // i2c从机地址
    msg[1].flags = I2C_M_RD;     // 读取数据
    msg[1].buf   = val;          // 待发送数据
    msg[1].len   = len;            // 待发送数据长度

    return i2c_transfer(client->adapter, msg, 2);
}

/* 写入 AP3216C 的 N 个寄存器值 */
static int ap3216c_write_regs(struct ap3216c_dev* dev, u8 reg, void* buf, int len)
{
    struct i2c_client* client = (struct i2c_client*)dev->private_data;
    struct i2c_msg msg;
    u8 data[256];

    /* 构建发送数据,寄存器地址+实际数据 */
    data[0] = reg;
    memcpy(&data[1], buf, len);

    /* msg 发送数据 */
    msg.addr  = client->addr; // i2c从机地址
    msg.flags = 0;            // 发送数据
    msg.buf   = data;         // 待发送数据
    msg.len   = len + 1;      // 待发送数据长度:寄存器地址+实际数据

    return i2c_transfer(client->adapter, &msg, 1);
}

/* 读取 AP3216C 的一个寄存器值 */
static u_char ap3216c_read_reg(struct ap3216c_dev* dev, u8 reg)
{
    u8 data = 0;
    ap3216c_read_regs(dev, reg, &data, 1);

    return data;
}

/* 写入 AP3216C 的一个寄存器值 */
static void ap3216c_write_reg(struct ap3216c_dev* dev, u8 reg, u8 data)
{
    ap3216c_write_regs(dev, reg, &data, 1);
}

/* AP3216C数据读取 */
static void ap3216c_read_data(struct ap3216c_dev* dev)
{
    unsigned char buf[6];
    unsigned char i = 0;

    /* 循环的读取数据 */
    for (i = 0; i < 6; i++) {
        buf[i] = ap3216c_read_reg(dev, AP3216C_IRDATALOW + i);
    }

    if (buf[0] & 0x80) { /* 为真表示IR和PS数据无效 */
        dev->ir = 0;
        dev->ps = 0;
    }
    else {
        dev->ir = ((unsigned short)buf[1] << 2) | (buf[0] & 0x03);
        dev->ps = (((unsigned short)buf[5] & 0x3F) << 4) | (buf[4] & 0x0F);
    }

    dev->als = ((unsigned short)buf[3] << 8) | buf[2];

    // printk("ir = %d, als = %d, ps = %d\r\n", dev->ir, dev->als, dev->ps);
}

int ap3216c_open(struct inode* inode, struct file* filp)
{
    u_char value       = 0;
    filp->private_data = &ap3216c; // 设置私有数据

    /* 初始化AP3216C */
    ap3216c_write_reg(&ap3216c, AP3216C_SYSTEMCONG, 0X4); /* 复位 */
    mdelay(50);
    ap3216c_write_reg(&ap3216c, AP3216C_SYSTEMCONG, 0X3); /* 复位 */

    value = ap3216c_read_reg(&ap3216c, AP3216C_SYSTEMCONG);
    // printk("AP3216C_SYSTEMCONG = %#x\r\n", value);

    return 0;
}

ssize_t ap3216c_read(struct file* filp, char __user* buf, size_t cnt, loff_t* offt)
{
    long err = 0;
    short data[3];

    struct ap3216c_dev* dev = (struct ap3216c_dev*)filp->private_data;

    /* 向应用返回AP3216C的原始数据 */
    ap3216c_read_data(dev);

    data[0] = dev->ir;
    data[1] = dev->als;
    data[2] = dev->ps;

    err = copy_to_user(buf, data, sizeof(data));

    return 0;
}

/* AP3216C 操作函数 */
static const struct file_operations ap3216c_fops = {
    .owner = THIS_MODULE,
    .open  = ap3216c_open,
    .read  = ap3216c_read,
};

static int ap3216c_probe(struct i2c_client* client, const struct i2c_device_id* dev)
{
    int rc = 0;

    /* 注册字符设备 */
    // 1.分配设备号
    ap3216c.major = 0; // 内核指定分配设备号
    if (ap3216c.major) {
        ap3216c.devid = MKDEV(ap3216c.major, 0);
        rc            = register_chrdev_region(ap3216c.devid, DEV_COUNT, DEV_NAME);
    }
    else {
        rc            = alloc_chrdev_region(&ap3216c.devid, 0, DEV_COUNT, DEV_NAME);
        ap3216c.major = MAJOR(ap3216c.devid);
        ap3216c.minor = MINOR(ap3216c.devid);
    }
    if (rc < 0) {
        printk("ap3216c chrdev region error!\r\n");
        goto fail_devid;
    }
    printk("major=%d,minor=%d\r\n", ap3216c.major, ap3216c.minor);

    // 2.注册字符设备
    ap3216c.cdev.owner = THIS_MODULE;
    cdev_init(&ap3216c.cdev, &ap3216c_fops);
    rc = cdev_add(&ap3216c.cdev, ap3216c.devid, DEV_COUNT);
    if (rc < 0) {
        rc = -EINVAL;
        goto fail_cdev;
    }

    // 3.动态创建设备的逻辑类
    ap3216c.class = class_create(THIS_MODULE, DEV_NAME);
    if (IS_ERR(ap3216c.class)) {
        rc = PTR_ERR(ap3216c.class);
        goto fail_class;
    }

    // 4.自动创建设备节点
    ap3216c.device = device_create(ap3216c.class, NULL, ap3216c.devid, NULL, DEV_NAME);
    if (IS_ERR(ap3216c.device)) {
        rc = PTR_ERR(ap3216c.device);
        goto fail_device;
    }
    printk("ap3216c dirver init\r\n");

    ap3216c.private_data = client;

    return rc;

fail_device:
    device_destroy(ap3216c.class, ap3216c.devid);
    class_destroy(ap3216c.class);
fail_class:
    cdev_del(&ap3216c.cdev);
fail_cdev:
    unregister_chrdev_region(ap3216c.devid, DEV_COUNT);
fail_devid:
    return rc;
}

static int ap3216c_remove(struct i2c_client* client)
{
    // 1.删除字符设备
    cdev_del(&ap3216c.cdev);

    // 2.注销设备号
    unregister_chrdev_region(ap3216c.devid, DEV_COUNT);

    // 3.销毁设备节点
    device_destroy(ap3216c.class, ap3216c.devid);

    // 4.销毁设备的逻辑类
    class_destroy(ap3216c.class);

    printk("ap3216c remove\r\n");

    return 0;
}

/* 传统匹配方式ID列表 */
static const struct i2c_device_id ap3216c_id[] = {
    { "atkmini,ap3216c", 0 },
    { /* Sentinel */ },
};

/* 设备匹配列表与设备树匹配 */
static const struct of_device_id ap3216c_of_match[] = {
    { .compatible = "atkmini,ap3216c" },
    { /* Sentinel */ },
};

/* i2c 驱动结构体 */
static struct i2c_driver ap3216c_driver = {
    .probe  = ap3216c_probe,
    .remove = ap3216c_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "ap3216c",
        .of_match_table = of_match_ptr(ap3216c_of_match),
    },
    .id_table = ap3216c_id,
};

/* i2c 驱动注册与卸载辅助宏 */
module_i2c_driver(ap3216c_driver);

/* 驱动模块信息 */
MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("ap3216c driver");