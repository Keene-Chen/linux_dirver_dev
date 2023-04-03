/**
 * @file    : led_driver.c
 * @author  : KeeneChen
 * @date    : 2023.04.03-16:10:57
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
#include <linux/platform_device.h>
#include <linux/types.h>

#define LED_NAME  "platform_led"
#define LED_COUNT 1
#define LED_ON    1 // 开灯
#define LED_OFF   0 // 关灯

/* 映射后的寄存器虚拟地址指针 */
static void __iomem* IMX6U_CCM_CCGR1;
static void __iomem* SW_MUX_GPIO1_IO03;
static void __iomem* SW_PAD_GPIO1_IO03;
static void __iomem* GPIO1_DR;
static void __iomem* GPIO1_GDIR;

/* LED设备结构体 */
typedef struct led_dev {
    struct cdev cdev;      // 字符设备
    struct class* class;   // 逻辑类指针
    struct device* device; // 设备指针
    dev_t devid;           // 设备号
    int major;             // 主设备号
    int minor;             // 次设备号
} led_dev_t;
led_dev_t led;

/**
 * @brief 打开设备
 * @param inode 传递给驱动的inode
 * @param filp  设备文件,file结构体有个叫做private_data的成员变量
 *              一般在open的时候将private_data指向设备结构体。
 * @return 函数调用状态
 * @retval 0     成功
 * @retval other 失败
 */
static int led_open(struct inode* inode, struct file* filp)
{
    // 设置私有数据
    filp->private_data = &led;
    return 0;
}

/**
 * @brief 关闭/释放设备
 * @param filp 要关闭的设备文件(文件描述符)
 * @return 函数调用状态
 * @retval 0     成功
 * @retval other 失败
 */
static int led_release(struct inode* inode, struct file* filp)
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
static ssize_t led_read(struct file* filp, char __user* buf, size_t count, loff_t* offt)
{
    led_dev_t* dev = filp->private_data;
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
static ssize_t led_write(struct file* filp, const char __user* buf, size_t count, loff_t* offt)
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
static const struct file_operations led_fops = {
    .owner   = THIS_MODULE,
    .open    = led_open,
    .release = led_release,
    .read    = led_read,
    .write   = led_write,
};

static int led_probe(struct platform_device* dev)
{
    int i   = 0;
    int val = 0;
    int rc  = 0;
    struct resource* led_source[5];
    int led_source_size[5];
    printk("led dirver probe\r\n");

    /* 1.获取platform_device中的寄存器资源 */
    for (i = 0; i < 5; i++) {
        led_source[i] = platform_get_resource(dev, IORESOURCE_MEM, i); // 依次访问MEM类型资源
        if (!led_source[i]) {
            dev_err(&dev->dev, "No MEM resource for always on\n");
            return -ENXIO;
        }
        led_source_size[i] = resource_size(led_source[i]);
    }

    /* 2.初始化LED */
    /* 寄存器地址映射 */
    IMX6U_CCM_CCGR1   = ioremap(led_source[0]->start, led_source_size[0]);
    SW_MUX_GPIO1_IO03 = ioremap(led_source[1]->start, led_source_size[1]);
    SW_PAD_GPIO1_IO03 = ioremap(led_source[2]->start, led_source_size[2]);
    GPIO1_DR          = ioremap(led_source[3]->start, led_source_size[3]);
    GPIO1_GDIR        = ioremap(led_source[4]->start, led_source_size[4]);

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

    /* 3.分配设备号 */
    if (led.major) { // 是否给定主设备号
        led.devid = MKDEV(led.major, 0);
        rc        = register_chrdev_region(led.devid, LED_COUNT, LED_NAME);
    }
    else {
        rc        = alloc_chrdev_region(&led.devid, 0, LED_COUNT, LED_NAME);
        led.major = MAJOR(led.devid);
        led.minor = MINOR(led.minor);
    }

    if (rc < 0) {
        printk("led chrdev region error!\r\n");
        goto out_chrdev;
    }
    printk("major=%d,minor=%d\r\n", led.major, led.minor);

    /* 4.注册字符设备 */
    led.cdev.owner = THIS_MODULE;
    cdev_init(&led.cdev, &led_fops);
    rc = cdev_add(&led.cdev, led.devid, LED_COUNT);
    if (rc < 0) {
        goto out_cdev;
    }

    /* 5.动态创建设备的逻辑类 */
    led.class = class_create(THIS_MODULE, LED_NAME);
    if (IS_ERR(led.class)) {
        return PTR_ERR(led.class);
    }

    /* 6.自动创建设备节点 */
    led.device = device_create(led.class, NULL, led.devid, NULL, LED_NAME);
    if (IS_ERR(led.device)) {
        return PTR_ERR(led.device);
    }

out_chrdev:
    unregister_chrdev_region(led.devid, LED_COUNT);

out_cdev:
    return rc;
}

static int led_remove(struct platform_device* dev)
{
    /* 1.取消led地址映射 */
    iounmap(IMX6U_CCM_CCGR1);
    iounmap(SW_MUX_GPIO1_IO03);
    iounmap(SW_PAD_GPIO1_IO03);
    iounmap(GPIO1_DR);
    iounmap(GPIO1_GDIR);

    /* 2.删除字符设备 */
    cdev_del(&led.cdev);

    /* 3.注销字符设备 */
    unregister_chrdev_region(led.devid, LED_COUNT);

    /* 4.销毁设备节点 */
    device_destroy(led.class, led.devid);

    /* 5.销毁设备的逻辑类 */
    class_destroy(led.class);

    printk("led dirver remove\r\n");

    return 0;
}

/* platform 驱动结构体 */
static struct platform_driver led_driver = {
    .driver = { .name = "imx6ull-led" },
    .probe  = led_probe,
    .remove = led_remove,
};

/* 驱动入口函数 */
static int __init led_driver_init(void)
{

    printk("led driver init\r\n");
    // 注册platform设备
    return platform_driver_register(&led_driver);
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
MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("led_driver");