/**
 * @file    : newchrled.c
 * @author  : KeeneChen
 * @date    : 2023.03.09-13:50:55
 * @details : new character led driver module
 */

/**
 * 常用函数API
 * 1.分配设备号
 ** int register_chrdev_region(dev_t from, unsigned count, const char *name)
 * @brief 注册一系列设备编号
 * @param from 所需设备编号范围内的第一个;必须包括主设备号
 * @param count 所需的连续设备编号数
 * @param name  设备或驱动程序的名称
 * @return 成功时返回值为零,失败时返回错误代码为负数
 *
 ** int alloc_chrdev_region(dev_t *dev, unsigned baseminor, unsigned count,const char *name)
 * @brief 注册一系列设备编号
 * @param dev 第一个号码分配的输出参数
 * @param baseminor 请求的次要号码范围中的第一个
 * @param count 所需的次要号码数
 * @param name  关联设备或驱动程序的名称
 * @return 成功时返回值为零,失败时返回错误代码为负数
 *
 ** void unregister_chrdev_region(dev_t from, unsigned count)
 * @brief 注销一系列设备编号
 * @param from 取消注册号码范围内的第一个
 * @param count 要注销的设备号码数量
 *
 * 2.注册字符设备
 ** void cdev_init(struct cdev *cdev, const struct file_operations *fops)
 * @brief 初始化 cdev 结构
 * @param cdev 要初始化的结构
 * @param fops 此设备的file_operations
 *
 ** int cdev_add(struct cdev *p, dev_t dev, unsigned count)
 * @brief 向系统添加字符设备
 * @param p 设备的 CDV 结构
 * @param dev 此设备负责的第一个设备编号
 * @param count 与此对应的连续次要数字的数量
 * @return 成功时返回值为零,失败时返回错误代码为负数
 *
 ** void cdev_del(struct cdev *p)
 * @brief 向系统删除字符设备
 * @param p 要删除的 cdev 结构
 *
 * 3.动态创建设备的逻辑类
 ** #define class_create(owner, name)		\
 ** ({						\
 **	static struct lock_class_key __key;	\
 **	__class_create(owner, name, &__key);	\
 ** })
 *
 ** void class_destroy(struct class *cls)
 * @brief 销毁一个struct类结构
 * @param cls 指向要销毁的结构类的指针
 *
 * 4.自动创建设备节点
 ** struct device *device_create(struct class *class, struct device *parent,dev_t devt, void
 * drvdata, const char *fmt, ...)
 * @brief 创建一个设备并将其注册到 sysfs
 * @param class 指向该设备应该注册到的结构类的指针
 * @param parent 指向这个新设备的父结构设备的指针,如果有的话
 * @param devt 要添加的字符设备的dev_t
 * @param drvdata 要添加到设备以供回调的数据
 * @param fmt 设备名称的字符串
 * @return 成功时返回 &struct 设备指针,错误时返回 ERR_PTR()
 *
 ** void device_destroy(struct class *class, dev_t devt)
 * @brief 删除使用 device_create()创建的设备
 * @param class 指向此设备注册的结构类的指针
 * @param devt  之前注册的设备dev_t
 */

#include <asm/io.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>

#define NEWCHRLED_NAME  "newchrled"
#define NEWCHRLED_COUNT 1
#define LED_ON          1 // 开灯
#define LED_OFF         0 // 关灯

/* 寄存器物理地址 */
#define CCM_CCGR1_BASE         (0x020C406C)
#define SW_MUX_GPIO1_IO03_BASE (0x020E0068)
#define SW_PAD_GPIO1_IO03_BASE (0x020E02F4)
#define GPIO1_DR_BASE          (0x0209C000)
#define GPIO1_GDIR_BASE        (0x0209C004)

/* 映射后的寄存器虚拟地址指针 */
static void __iomem* IMX6U_CCM_CCGR1;
static void __iomem* SW_MUX_GPIO1_IO03;
static void __iomem* SW_PAD_GPIO1_IO03;
static void __iomem* GPIO1_DR;
static void __iomem* GPIO1_GDIR;

/* LED设备结构体 */
typedef struct newchrled_dev {
    struct cdev cdev;      // 字符设备
    struct class* class;   // 逻辑类指针
    struct device* device; // 设备指针
    dev_t devid;           // 设备号
    int major;             // 主设备号
    int minor;             // 次设备号
} newchrled_dev_t;
newchrled_dev_t newchrled;

/**
 * @brief 打开设备
 * @param inode 传递给驱动的inode
 * @param filp  设备文件,file结构体有个叫做private_data的成员变量
 *              一般在open的时候将private_data指向设备结构体。
 * @return 函数调用状态
 * @retval 0     成功
 * @retval other 失败
 */
static int newchrled_open(struct inode* inode, struct file* filp)
{
    // 设置私有数据
    filp->private_data = &newchrled;
    return 0;
}

/**
 * @brief 关闭/释放设备
 * @param filp 要关闭的设备文件(文件描述符)
 * @return 函数调用状态
 * @retval 0     成功
 * @retval other 失败
 */
static int newchrled_release(struct inode* inode, struct file* filp)
{
    return 0;
}

/**
 * @brief 从设备读取数据
 * @param filp 要打开的设备文件(文件描述符)
 * @param buf  返回给用户空间的数据缓冲区
 * @param cnt  要读取的数据长度
 * @param offt 相对于文件首地址的偏移
 * @return 读取的字节数
 * @retval 负值表示读取失败
 */
static ssize_t newchrled_read(struct file* filp, char __user* buf, size_t count, loff_t* offt)
{
    newchrled_dev_t* dev = filp->private_data;
    printk("major=%d\r\n", dev->major);
    printk("minor=%d\r\n", dev->minor);
    printk("devid=%d\r\n", dev->devid);
    return 0;
}

/**
 * @brief 向设备写数据
 * @param filp 设备文件,表示打开的文件描述符
 * @param buf  要写给设备写入的数据
 * @param cnt  要写入的数据长度
 * @param offt 相对于文件首地址的偏移
 * @return 写入的字节数
 * @retval 负值表示写入失败
 */
static ssize_t newchrled_write(struct file* filp, const char __user* buf, size_t count,
                               loff_t* offt)
{
    int ret        = 0;
    int val        = 0;
    u8 data_buf[1] = { 0 };

    ret = copy_from_user(data_buf, buf, count);
    if (ret < 0) {
        printk("kernel write failed");
        return -EFAULT;
    }

    // 判断灯的状态
    if (data_buf[0] == LED_ON) {
        val = readl(GPIO1_DR);
        val &= ~(1 << 3);
        writel(val, GPIO1_DR);
    }
    else if (data_buf[0] == LED_OFF) {
        val = readl(GPIO1_DR);
        val |= (1 << 3);
        writel(val, GPIO1_DR);
    }

    return 0;
}

/* 字符设备操作集合 */
static const struct file_operations newchrled_fops = {
    .owner   = THIS_MODULE,
    .open    = newchrled_open,
    .release = newchrled_release,
    .read    = newchrled_read,
    .write   = newchrled_write,
};

static void led_init(void)
{
    int val = 0;

    /* 1.寄存器地址映射 */
    IMX6U_CCM_CCGR1   = ioremap(CCM_CCGR1_BASE, 4);
    SW_MUX_GPIO1_IO03 = ioremap(SW_MUX_GPIO1_IO03_BASE, 4);
    SW_PAD_GPIO1_IO03 = ioremap(SW_PAD_GPIO1_IO03_BASE, 4);
    GPIO1_DR          = ioremap(GPIO1_DR_BASE, 4);
    GPIO1_GDIR        = ioremap(GPIO1_GDIR_BASE, 4);

    /* 2.初始化 */
    /* 初始化时钟 */
    val = readl(IMX6U_CCM_CCGR1);
    val &= ~(3 << 26);            // 清除bit[26-27]
    val |= (3 << 26);             // bit[26-27]置1
    writel(val, IMX6U_CCM_CCGR1); // 写入CCM_CCGR1寄存器

    /**
     * 初始化GPIO寄存器
     * SW_PAD_GPIO1_IO03设置IO属性
     * bit [16:0] : HYS关闭
     * bit [15:14]: 00  默认下拉
     * bit [13]   : 0   kepper功能
     * bit [12]   : 1   pull/keeper使能
     * bit [11]   : 0   关闭开路输出
     * bit [7:6]  : 10  速度100Mhz
     * bit [5:3]  : 110 R0/6驱动能力
     * bit [0]    : 0   低转换率
     */
    writel(0x05, SW_MUX_GPIO1_IO03);   // 设置IO复用
    writel(0x10B0, SW_PAD_GPIO1_IO03); // 设置IO电气属性

    val = readl(GPIO1_GDIR);
    val |= (1 << 3);         // bit[3]置1,设置为输出
    writel(val, GPIO1_GDIR); // 写入GPIO1_GDIR寄存器

    val = readl(GPIO1_DR);
    val &= ~(1 << 3);      // bit[3]清零,上电默认输出低电平即打开LED灯
    writel(val, GPIO1_DR); // 写入GPIO1_DR寄存器
}

/* 驱动入口函数 */
static int __init newchrled_init(void)
{
    int rc = 0;

    // 1.初始化led
    led_init();

    // 2.分配设备号
    if (newchrled.major) { // 是否给定主设备号
        newchrled.devid = MKDEV(newchrled.major, 0);
        rc              = register_chrdev_region(newchrled.devid, NEWCHRLED_COUNT, NEWCHRLED_NAME);
    }
    else {
        rc              = alloc_chrdev_region(&newchrled.devid, 0, NEWCHRLED_COUNT, NEWCHRLED_NAME);
        newchrled.major = MAJOR(newchrled.devid);
        newchrled.minor = MINOR(newchrled.minor);
    }

    if (rc < 0) {
        printk("newchrled chrdev region error!\r\n");
        goto out_chrdev;
    }
    printk("major=%d,minor=%d\r\n", newchrled.major, newchrled.minor);

    // 3.注册字符设备
    newchrled.cdev.owner = THIS_MODULE;
    cdev_init(&newchrled.cdev, &newchrled_fops);
    rc = cdev_add(&newchrled.cdev, newchrled.devid, NEWCHRLED_COUNT);
    if (rc < 0) {
        goto out_cdev;
    }

    // 4.动态创建设备的逻辑类
    newchrled.class = class_create(THIS_MODULE, NEWCHRLED_NAME);
    if (IS_ERR(newchrled.class)) {
        return PTR_ERR(newchrled.class);
    }

    // 5.自动创建设备节点
    newchrled.device = device_create(newchrled.class, NULL, newchrled.devid, NULL, NEWCHRLED_NAME);
    if (IS_ERR(newchrled.device)) {
        return PTR_ERR(newchrled.device);
    }
    printk("newchrled dirver init\r\n");

out_chrdev:
    unregister_chrdev_region(newchrled.devid, NEWCHRLED_COUNT);

out_cdev:
    return rc;
}

/* 驱动出口函数 */
static void __exit newchrled_exit(void)
{
    // 1.取消led地址映射
    iounmap(IMX6U_CCM_CCGR1);
    iounmap(SW_MUX_GPIO1_IO03);
    iounmap(SW_PAD_GPIO1_IO03);
    iounmap(GPIO1_DR);
    iounmap(GPIO1_GDIR);

    // 2.删除字符设备
    cdev_del(&newchrled.cdev);

    // 3.注销字符设备
    unregister_chrdev_region(newchrled.devid, NEWCHRLED_COUNT);

    // 4.销毁设备节点
    device_destroy(newchrled.class, newchrled.devid);

    // 5.销毁设备的逻辑类
    class_destroy(newchrled.class);

    printk("newchrled dirver exit\r\n");
}

/* 驱动注册与卸载 */
module_init(newchrled_init);
module_exit(newchrled_exit);
MODULE_DESCRIPTION("new character led driver module");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("KeeneChen");
