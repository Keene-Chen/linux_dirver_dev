/**
 * @file    : dtsled.c
 * @author  : KeeneChen
 * @date    : 2023.03.10-15:25:44
 * @details : dts led driver module
 */

/**
 * 设备树常用of函数
 *? 1.查找节点有关的of函数
 ** struct device_node *of_find_node_by_name(struct device_node *from,const char *name);
 * @brief 通过节点名字查找指定的节点
 * @param from 开始查找的节点,如果为 NULL 表示从根节点开始查找整个设备树
 * @param name 要查找的节点名字
 * @return 找到的节点,如果为 NULL 表示查找失败
 *
 ** struct device_node *of_find_node_by_type(struct device_node *from, const char *type)
 * @brief 通过 device_type 属性查找指定的节点
 * @param from 开始查找的节点,如果为 NULL 表示从根节点开始查找整个设备树
 * @param name 要查找的节点对应的 type 字符串,也就是 device_type 属性值
 * @return 找到的节点,如果为 NULL 表示查找失败
 *
 ** struct device_node *of_find_compatible_node(struct device_node *from,
 ** const char *type,
 ** const char *compatible)
 * @brief 根据 device_type 和 compatible 这两个属性查找指定的节点
 * @param from 开始查找的节点,如果为 NULL 表示从根节点开始查找整个设备树
 * @param name 要查找的节点对应的 type 字符串,也就是 device_type 属性值,可以为
 * NULL,表示忽略掉device_type 属性
 * @return 找到的节点,如果为 NULL 表示查找失败
 *
 ** struct device_node *of_find_matching_node_and_match(struct device_node *from,
 ** const struct of_device_id *matches,const struct of_device_id **match)
 * @brief 通过 of_device_id 匹配表来查找指定的节点
 * @param from 开始查找的节点,如果为 NULL 表示从根节点开始查找整个设备树
 * @param matches of_device_id 匹配表,也就是在此匹配表里面查找节点
 * @param match 找到的匹配的 of_device_id
 * @return 找到的节点,如果为 NULL 表示查找失败
 *
 ** inline struct device_node *of_find_node_by_path(const char *path)
 * @brief 通过路径来查找指定的节点
 * @param path 带有全路径的节点名,可以使用节点的别名,比如“/backlight”就是backlight这个节点的全路径
 * @return 找到的节点,如果为 NULL 表示查找失败
 *
 *? 2.查找父/子节点的of函数
 ** struct device_node *of_get_parent(const struct device_node *node)
 * @brief 获取指定节点的父节点(如果有父节点的话)
 * @param node 要查找的父节点的节点
 * @return 找到的父节点
 *
 ** struct device_node *of_get_next_child(const struct device_node *node,struct device_node *prev)
 * @brief 用迭代的方式查找子节点
 * @param node 要查找的父节点的节点
 * @param prev 前一个子节点,可以设置为NULL,表示从第一个子节点开始
 * @return 找到的父节点
 *
 *? 3.提取属性值的of函数
 ** struct property {
 **     char *name;            // 属性名字
 **     int length;            // 属性长度
 **     void *value;           // 属性值
 **     struct property *next; // 下一个属性
 **     unsigned long _flags;
 **     unsigned int unique_id;
 **     struct bin_attribute attr;
 ** };
 *
 ** property *of_find_property(const struct device_node *np,const char *name,int *lenp)
 * @brief 用于查找指定的属性
 * @param np 设备节点
 * @param name 属性名字
 * @param lenp 属性值的字节数
 * @return 找到的属性
 *
 ** int of_property_count_elems_of_size(const struct device_node *np,
 ** const char *propname,int elem_size)
 * @brief 用于获取属性中元素的数量
 * @param np 设备节点
 * @param proname 需要统计元素数量的属性名字
 * @param elem_size 元素长度
 * @return 得到的属性元素数量
 *
 ** int of_property_read_u32_index(const struct device_node *np,const char *propname,
 ** u32 index,u32 out_value)
 * @brief 用于从属性中获取指定标号的 u32 类型数据值
 * @param np 设备节点
 * @param proname 要读取的属性名字
 * @param index 要读取的值标号
 * @param out_value 读取到的值
 * @return 0 读取成功,负值,读取失败,-EINVAL 表示属性不存在,-ENODATA 表示没有要读取的数据,
 * -EOVERFLOW表示属性值列表太小
 *
 ** int of_property_read_u8_array(const struct device_node *np,const char *propname,
 ** u8 out_values,size_t sz)
 ** int of_property_read_u16_array(const struct device_node *np,const char *propname,
 ** u16 out_values,size_t sz)
 ** int of_property_read_u32_array(const struct device_node *np,const char *propname,
 ** u32 out_values,size_t sz)
 ** int of_property_read_u64_array(const struct device_node *np,const char *propname,
 ** u64 out_values,size_t sz)
 * @brief 用于读取属性中 u8、 u16、 u32 和 u64 类型的数组数据
 * @param np 设备节点
 * @param proname 要读取的属性名字
 * @param out_value 读取到的数组值,分别为 u8、 u16、 u32 和 u64
 * @param sz 要读取的数组元素数量
 * @return 0,读取成功,负值,读取失败, -EINVAL 表示属性不存在, -ENODATA 表示没有要读取的数据,
 * -EOVERFLOW 表示属性值列表太小
 *
 ** int of_property_read_u8(const struct device_node *np,
 ** const char *propname,u8 *out_value)
 ** int of_property_read_u16(const struct device_node *np,
 ** const char *propname,u16 *out_value)
 ** int of_property_read_u32(const struct device_node *np,
 ** const char *propname,u32 *out_value)
 ** int of_property_read_u64(const struct device_node *np,
 ** const char *propname,u64 *out_value)
 * @brief 用于读取这种只有一个整形值的属性
 * @param np 设备节点
 * @param proname 要读取的属性名字
 * @param out_value 读取到的数组值
 * @return 0,读取成功,负值,读取失败, -EINVAL 表示属性不存在, -ENODATA 表示没有要读取的数据,
 * -EOVERFLOW 表示属性值列表太小
 *
 ** int of_property_read_string(struct device_node *np,
 ** const char *propname,const char **out_string)
 * @brief 用于读取这种只有一个整形值的属性
 * @param np 设备节点
 * @param proname 要读取的属性名字
 * @param out_string 读取到的字符串值
 * @return 0,读取成功,负值,读取失败
 *
 ** int of_n_addr_cells(struct device_node *np)
 * @brief 用于获取#address-cells属性值
 * @param np 设备节点
 * @return 获取到的#address-cells属性值
 *
 ** int of_n_size_cells(struct device_node *np)
 * @brief 用于获取#size_cells属性值
 * @param np 设备节点
 * @return 获取到的#size_cells属性值
 *
 *? 4.其他常用的of函数
 ** int of_device_is_compatible(const struct device_node *device,const char *compat)
 * @brief 用于查看节点的 compatible 属性是否有包含 compat 指定的字
符串
 * @param device 设备节点
 * @param compat 要查看的字符串
 * @return 0,节点的 compatible 属性中不包含 compat 指定的字符串； 正数,节点的 compatible属性中包含
compat 指定的字符串
 *
 ** const __be32 *of_get_address(struct device_node *dev,
 ** int index,u64 *size,unsigned int *flags)
 * @brief 用于获取地址相关属性,主要是“reg”或者assigned-addresses”属性值
 * @param dev 设备节点
 * @param index 要读取的地址标号
 * @param size 地址长度
 * @param flags 参数,比如 IORESOURCE_IO、 IORESOURCE_MEM 等
 * @return 读取到的地址数据首地址,为 NULL 的话表示读取失败
 *
 ** u64 of_translate_address(struct device_node *dev,const __be32 *in_addr)
 * @brief 从设备树读取到的地址转换为物理地址
 * @param dev 设备节点
 * @param in_addr 要转换的地址
 * @return 得到的物理地址,如果为 OF_BAD_ADDR 的话表示转换失败
 *
 ** void __iomem *of_iomap(struct device_node *np,int index)
 * @brief 用于直接内存映射
 * @param np 设备节点
 * @param index reg 属性中要完成内存映射的段,如果 reg 属性只有一段的话 index 就设置为 0
 * @return 经过内存映射后的虚拟内存首地址,如果为 NULL 的话表示内存映射失败
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define DTSLED_NAME  "dtsled" // 设备名
#define DTSLED_COUNT 1        // 设备个数
#define LED_ON       1        // 开灯
#define LED_OFF      0        // 关灯

/* 映射后的寄存器虚拟地址指针 */
static void __iomem* IMX6U_CCM_CCGR1;
static void __iomem* SW_MUX_GPIO1_IO03;
static void __iomem* SW_PAD_GPIO1_IO03;
static void __iomem* GPIO1_DR;
static void __iomem* GPIO1_GDIR;

/* 字符设备结构体 */
struct dtsled_dev {
    dev_t devid;              // 设备号
    int major;                // 主设备号
    int minor;                // 次设备号
    struct cdev cdev;         // 字符设备
    struct class* class;      // 逻辑类
    struct device* device;    // 设备
    struct device_node* node; // 设备节点
} dtsled;

static int dtsled_open(struct inode* inode, struct file* filp)
{
    // 设置私有数据
    filp->private_data = &dtsled;
    return 0;
}

static int dtsled_release(struct inode* inode, struct file* filp)
{
    // 取出私有数据
    struct dtsled_dev* dev = (struct dtsled_dev*)filp->private_data;
    printk("major=%d\r\n", dev->major);
    printk("minor=%d\r\n", dev->minor);
    printk("devid=%d\r\n", dev->devid);

    return 0;
}

static ssize_t dtsled_write(struct file* filp, const char __user* buf, size_t count, loff_t* offt)
{
    int ret        = 0;
    int val        = 0;
    u8 data_buf[1] = { 0 };

    // 获取私有数据
    struct dtsled_dev* dev = (struct dtsled_dev*)filp->private_data;
    printk("major=%d\r\n", dev->major);
    printk("minor=%d\r\n", dev->minor);
    printk("devid=%d\r\n", dev->devid);

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
static const struct file_operations dtsled_fops = {
    .owner   = THIS_MODULE,
    .open    = dtsled_open,
    .release = dtsled_release,
    .write   = dtsled_write,
};

/* 驱动入口函数 */
static int __init dtsled_init(void)
{
    int rc          = 0;
    const char* str = NULL;
    u32 regdata[10] = { 0 };
    int i           = 0;
    int val         = 0;

    /* 获取设备树属性 */
    // 1.获取设备树节点地址
    dtsled.node = of_find_node_by_path("/miniled");
    if (dtsled.node == NULL) {
        rc = -EINVAL;
        goto fail_find_node;
    }

    // 2.获取属性
    // /->miniled->status
    rc = of_property_read_string(dtsled.node, "status", &str);
    if (rc < 0) {
        goto fail_rs;
    }
    else {
        printk("status = %s\r\n", str);
    }

    // /->miniled->compatible
    rc = of_property_read_string(dtsled.node, "compatible", &str);
    if (rc < 0) {
        goto fail_rs;
    }
    else {
        printk("compatible = %s\r\n", str);
    }

    // /->miniled->reg
    rc = of_property_read_u32_array(dtsled.node, "reg", regdata, 10);
    if (rc < 0) {
        goto fail_rs;
    }
    else {
        printk("regdata:\r\n");
        for (i = 0; i < 10; i++) {
            printk("%#x ", regdata[i]);
        }
        printk("\r\n");
    }

    /* 初始化LED */

    // 1.寄存器地址映射

#if 0
    IMX6U_CCM_CCGR1   = ioremap(regdata[0], regdata[1]);
    SW_MUX_GPIO1_IO03 = ioremap(regdata[2], regdata[3]);
    SW_PAD_GPIO1_IO03 = ioremap(regdata[4], regdata[5]);
    GPIO1_DR          = ioremap(regdata[6], regdata[7]);
    GPIO1_GDIR        = ioremap(regdata[8], regdata[9]);
#endif
    IMX6U_CCM_CCGR1   = of_iomap(dtsled.node, 0);
    SW_MUX_GPIO1_IO03 = of_iomap(dtsled.node, 1);
    SW_PAD_GPIO1_IO03 = of_iomap(dtsled.node, 2);
    GPIO1_DR          = of_iomap(dtsled.node, 3);
    GPIO1_GDIR        = of_iomap(dtsled.node, 4);

    // 2.初始化
    val = readl(IMX6U_CCM_CCGR1);
    val &= ~(3 << 26);                 // 清除bit[26-27]
    val |= (3 << 26);                  // bit[26-27]置1
    writel(val, IMX6U_CCM_CCGR1);      // 写入CCM_CCGR1寄存器
    writel(0x05, SW_MUX_GPIO1_IO03);   // 设置IO复用
    writel(0x10B0, SW_PAD_GPIO1_IO03); // 设置IO电气属性

    val = readl(GPIO1_GDIR);
    val |= (1 << 3);         // bit[3]置1,设置为输出
    writel(val, GPIO1_GDIR); // 写入GPIO1_GDIR寄存器

    /* 注册字符设备 */
    // 1.分配设备号
    dtsled.major = 0;   // 由内核分配
    if (dtsled.major) { // 是否给定主设备号
        dtsled.devid = MKDEV(dtsled.major, 0);
        rc           = register_chrdev_region(dtsled.devid, DTSLED_COUNT, DTSLED_NAME);
    }
    else {
        rc           = alloc_chrdev_region(&dtsled.devid, 0, DTSLED_COUNT, DTSLED_NAME);
        dtsled.major = MAJOR(dtsled.devid);
        dtsled.minor = MINOR(dtsled.minor);
    }
    if (rc < 0) {
        printk("dtsled chrdev region error!\r\n");
        goto fail_devid;
    }
    printk("major=%d,minor=%d\r\n", dtsled.major, dtsled.minor);

    // 2.注册字符设备
    dtsled.cdev.owner = THIS_MODULE;
    cdev_init(&dtsled.cdev, &dtsled_fops);
    rc = cdev_add(&dtsled.cdev, dtsled.devid, DTSLED_COUNT);
    if (rc < 0) {
        goto fail_cdev;
    }

    // 3.动态创建设备的逻辑类
    dtsled.class = class_create(THIS_MODULE, DTSLED_NAME);
    if (IS_ERR(dtsled.class)) {
        rc = PTR_ERR(dtsled.class);
        goto fail_class;
    }

    // 4.自动创建设备节点
    dtsled.device = device_create(dtsled.class, NULL, dtsled.devid, NULL, DTSLED_NAME);
    if (IS_ERR(dtsled.device)) {
        rc = PTR_ERR(dtsled.device);
        goto fail_device;
    }
    printk("dtsled dirver init\r\n");
    return 0;

fail_rs:
    return 0;
fail_find_node:
    device_destroy(dtsled.class, dtsled.devid);
fail_device:
    class_destroy(dtsled.class);
fail_class:
    cdev_del(&dtsled.cdev);
fail_cdev:
    unregister_chrdev_region(dtsled.devid, DTSLED_COUNT);
fail_devid:
    return rc;
}

/* 驱动出口函数 */
static void __exit dtsled_exit(void)
{
    // 1.取消led地址映射
    iounmap(IMX6U_CCM_CCGR1);
    iounmap(SW_MUX_GPIO1_IO03);
    iounmap(SW_PAD_GPIO1_IO03);
    iounmap(GPIO1_DR);
    iounmap(GPIO1_GDIR);

    // 2.删除字符设备
    cdev_del(&dtsled.cdev);

    // 3.注销设备号
    unregister_chrdev_region(dtsled.devid, DTSLED_COUNT);

    // 4.销毁设备节点
    device_destroy(dtsled.class, dtsled.devid);

    // 5.销毁设备的逻辑类
    class_destroy(dtsled.class);

    printk("dtsled dirver exit\r\n");
}

/* 驱动注册与卸载 */
module_init(dtsled_init);
module_exit(dtsled_exit);
MODULE_DESCRIPTION("dts led driver module");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("KeeneChen");