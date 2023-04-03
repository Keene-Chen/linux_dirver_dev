/**
 * @file    : led_device.c
 * @author  : KeeneChen
 * @date    : 2023.04.03-15:20:41
 * @details : led_device
 */

/**
 * platform_device 结构体
 * @note #include <linux/platform_device.h>
 ** struct platform_device {
 **    const char *name;
 **    int id;
 **    bool id_auto;
 **    struct device dev;
 **    u32 num_resources;
 **    struct resource *resource;
 **
 **    const struct platform_device_id *id_entry;
 **    char *driver_override; // Driver name to force a match 
 **
 **    // MFD cell pointer 
 **    struct mfd_cell *mfd_cell;
 **
 **    // arch specific additions 
 **    struct pdev_archdata archdata;
 ** };
 *
 * resource 结构体
 ** struct resource {
 **     resource_size_t start;
 **     resource_size_t end;
 **     const char *name;
 **     unsigned long flags;
 **     struct resource *parent, *sibling, *child;
 ** };
 *
 *
 *
 ** int platform_device_register(struct platform_device *pdev)
 * @brief 注册 platform 设备
 * @param pdev 要注册的 platform 设备
 * @return 负数,失败 0,成功
 *
 ** void platform_driver_unregister(struct platform_driver *drv)
 * @brief 卸载 platform 设备
 * @param driver 要卸载的 platform 设备
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

/* 寄存器物理地址 */
#define CCM_CCGR1_BASE         (0x020C406C)
#define SW_MUX_GPIO1_IO03_BASE (0x020E0068)
#define SW_PAD_GPIO1_IO03_BASE (0x020E02F4)
#define GPIO1_DR_BASE          (0x0209C000)
#define GPIO1_GDIR_BASE        (0x0209C004)
#define REGISTER_LENGTH        4

static void led_device_release(struct device* dev)
{
    printk("led_device_release\r\n");
}

/* 设备资源结构体 */
static struct resource led_device_resource[] = {
    [0] = { .start = CCM_CCGR1_BASE,
            .end   = CCM_CCGR1_BASE + REGISTER_LENGTH - 1,
            .flags = IORESOURCE_MEM },
    [1] = { .start = SW_MUX_GPIO1_IO03_BASE,
            .end   = SW_MUX_GPIO1_IO03_BASE + REGISTER_LENGTH - 1,
            .flags = IORESOURCE_MEM },
    [2] = { .start = SW_PAD_GPIO1_IO03_BASE,
            .end   = SW_PAD_GPIO1_IO03_BASE + REGISTER_LENGTH - 1,
            .flags = IORESOURCE_MEM },
    [3] = { .start = GPIO1_DR_BASE,
            .end   = GPIO1_DR_BASE + REGISTER_LENGTH - 1,
            .flags = IORESOURCE_MEM },
    [4] = { .start = GPIO1_GDIR_BASE,
            .end   = GPIO1_GDIR_BASE + REGISTER_LENGTH - 1,
            .flags = IORESOURCE_MEM },
};

/* platform 设备结构体 */
static struct platform_device led_device = {
    .name          = "imx6ull-led",
    .id            = -1,
    .dev           = { .release = led_device_release },
    .num_resources = ARRAY_SIZE(led_device_resource),
    .resource      = led_device_resource,
};

/* 驱动入口函数 */
static int __init led_device_init(void)
{

    printk("led device init\r\n");
    // 注册platform设备
    return platform_device_register(&led_device);
}

/* 驱动出口函数 */
static void __exit led_device_exit(void)
{
    // 卸载platform设备
    platform_device_unregister(&led_device);
    printk("led device exit\r\n");
}

/* 驱动注册与卸载 */
module_init(led_device_init);
module_exit(led_device_exit);
MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("led_device");