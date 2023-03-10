/**
 * @file    : led.c
 * @author  : KeeneChen
 * @date    : 2022.12.21-20:23:03
 * @details : led driver module
 */

#include <asm/io.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>

#define LED_MOJAR 200
#define LED_NAME  "led"
#define LED_ON    1 // 开灯
#define LED_OFF   0 // 关灯

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

/**
 * @brief 打开设备
 * @param inode 传递给驱动的inode
 * @param filp  设备文件，file结构体有个叫做private_data的成员变量
 *              一般在open的时候将private_data指向设备结构体。
 * @return 函数调用状态
 * @retval 0     成功
 * @retval other 失败
 */
static int led_open(struct inode* inode, struct file* filp)
{
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
    return 0;
}

/**
 * @brief 向设备写数据
 * @param filp 设备文件，表示打开的文件描述符
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

/* 设备操作集合 */
static struct file_operations led_fops = {
    .owner   = THIS_MODULE,
    .open    = led_open,
    .release = led_release,
    .read    = led_read,
    .write   = led_write,
};

/**
 * @brief  驱动入口函数
 * @return 函数调用状态
 * @retval 0     成功
 * @retval other 失败
 */
static int __init led_init(void)
{
    int ret = 0;
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

    // 注册字符设备,占用主设备号与次设备号,浪费资源,需要手动注册
    ret = register_chrdev(LED_MOJAR, LED_NAME, &led_fops);
    if (ret < 0) {
        printk("register_chrdev failed");
        return -EIO;
    }
    printk("led_init\r\n");

    return 0;
}

/**
 * @brief  驱动出口函数
 * @return void
 */
static void __exit led_exit(void)
{
    /* 取消寄存器地址映射 */
    iounmap(IMX6U_CCM_CCGR1);
    iounmap(SW_MUX_GPIO1_IO03);
    iounmap(SW_PAD_GPIO1_IO03);
    iounmap(GPIO1_DR);
    iounmap(GPIO1_GDIR);

    unregister_chrdev(LED_MOJAR, LED_NAME);
    printk("led_exit\r\n");
}

/* 内核驱动模块加载与卸载 */
module_init(led_init);
module_exit(led_exit);

/* 内核驱动模块信息 */
MODULE_DESCRIPTION("led driver module");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("KeeneChen");