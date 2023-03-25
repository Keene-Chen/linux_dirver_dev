/**
 * @file    : work.c
 * @author  : KeeneChen
 * @date    : 2023.03.24-16:42:43
 * @details : work
 */

/**
 * Linux 内核使用 work_struct 结构体表示一个工作
 ** struct work_struct {
 **     atomic_long_t data;
 **     struct list_head entry;
 **     work_func_t func; // 工作队列处理函数
 ** };
 *
 ** #define INIT_WORK(_work, _func)
 * @param _work 表示要初始化的工作
 * @param _func 是工作对应的处理函数
 *
 ** #define DECLARE_WORK(n, f)
 * @param n 表示定义的工作(work_struct)
 * @param f 表示工作对应的处理函数
 *
 ** bool schedule_work(struct work_struct *work)
 * @param work 要调度的工作
 * @return 0 成功,其他值 失败
 */

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define INTERRUPT_NAME  "interrupt" // 设备名
#define INTERRUPT_COUNT 1           // 设备个数
#define KEY_NUM         1           // 按键个数
#define KEY_VALUE       0x01        // 按键有效值
#define KEY_INVALID     0xFF        // 按键无效值

/* key 结构体 */
struct key_irq_desc {
    int gpio;                           // IO编号
    int irq_num;                        // 中断号
    unsigned char value;                // 按键值
    char name[10];                      // 名称
    irqreturn_t (*handler)(int, void*); // 中断处理函数
};

/* 驱动设备结构体 */
struct interrupt_dev {
    dev_t devid;                          // 设备号
    int major;                            // 主设备号
    int minor;                            // 次设备号
    struct cdev cdev;                     // 字符设备
    struct class* class;                  // 逻辑类
    struct device* device;                // 设备
    struct device_node* node;             // 设备节点
    struct key_irq_desc key_irq[KEY_NUM]; // 按键结构体
    struct timer_list timer;              // 定时器
    atomic_t key_value;                   // 有效的按键键值
    atomic_t key_release;    // 标记是否完成一次完成的按键，包括按下和释放
    struct work_struct work; // 使用 work 实现下半部机制
} interrupt;

static int interrupt_open(struct inode* inode, struct file* filp)
{
    // 设置私有数据
    filp->private_data = &interrupt;
    return 0;
}

static int interrupt_release(struct inode* inode, struct file* filp)
{
    // 取出私有数据
    struct interrupt_dev* dev = (struct interrupt_dev*)filp->private_data;
    printk("major=%d\r\n", dev->major);
    printk("minor=%d\r\n", dev->minor);
    printk("devid=%d\r\n", dev->devid);

    return 0;
}

static ssize_t interrupt_read(struct file* filp, char __user* buf, size_t count, loff_t* offt)
{
    int rc = 0;
    unchar key_value;
    unchar key_release;
    struct interrupt_dev* dev = (struct interrupt_dev*)filp->private_data;

    key_value   = atomic_read(&dev->key_value);
    key_release = atomic_read(&dev->key_release);
    if (key_release) {
        if (key_value & 0x80) {
            key_value &= ~0x80;
            rc = copy_to_user(buf, &key_value, sizeof(key_value));
        }
        else {
            goto data_err;
        }
        atomic_set(&dev->key_release, 0); // 按键按下标志清零
    }
    else {
        goto data_err;
    }

    return rc;
data_err:
    return -EINVAL;
}

static const struct file_operations interrupt_fops = {
    .owner   = THIS_MODULE,
    .open    = interrupt_open,
    .release = interrupt_release,
    .read    = interrupt_read,
};

/* 按键中断函数 */
static irqreturn_t key_handler(int irq_num, void* dev_id)
{
    struct interrupt_dev* dev = dev_id;

    // 调度 work
    schedule_work(&dev->work);

    return IRQ_HANDLED;
}

/* work 回调函数 */
static void key_work(struct work_struct* data)
{
    struct interrupt_dev* dev = container_of(data, struct interrupt_dev, work);

    printk("key_work\r\n");

    // 设置参数
    dev->timer.data = (volatile unsigned long)dev;

    // 启动定时器 20ms
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(20));
}

/* 定时器回调函数 */
static void timer_cb(unsigned long arg)
{
    struct interrupt_dev* dev = (struct interrupt_dev*)arg;
    int value                 = 0;

    value = gpio_get_value(dev->key_irq[0].gpio);
    if (value == 0) {
        atomic_set(&dev->key_value, dev->key_irq[0].value); // 按键按下就保存按键值
    }
    else if (value == 1) {

        atomic_set(&dev->key_value, 0x80 | (dev->key_irq[0].value)); // 按键释放时将按键值最高位置一
        atomic_set(&dev->key_release, 1);
    }
}

/* 按键初始化 */
static int keyio_init(struct interrupt_dev* dev)
{
    int rc = 0;
    int i  = 0;

    // 1.根据设备树路径获取设备节点
    dev->node = of_find_node_by_path("/key");
    if (dev->node == NULL) {
        rc = -EINVAL;
        printk("find node failed\r\n");
        goto fail_node;
    }

    for (i = 0; i < KEY_NUM; i++) {
        // 2.获取key所对应的GPIO
        dev->key_irq[i].gpio = of_get_named_gpio(dev->node, "key-gpio", i);
        if (dev->key_irq[i].gpio < 0) {
            rc = -EINVAL;
            printk("key %d gpio not found\r\n", i);
            goto fail_led_gpio;
        }

        // 初始化按键结构体
        memset(dev->key_irq[i].name, 0, sizeof(dev->key_irq[i].name));
        sprintf(dev->key_irq[i].name, "key-%d", i);
        dev->key_irq[0].handler = key_handler;
        dev->key_irq[0].value   = KEY_VALUE;

        // 3.申请IO
        rc = gpio_request(dev->key_irq[i].gpio, dev->key_irq[i].name);
        if (rc) {
            rc = -EINVAL;
            pr_err("failed to request the key gpio\r\n");
            goto fail_gpio_req;
        }

        // 4.使用IO,设置为输入
        rc = gpio_direction_input(dev->key_irq[i].gpio);
        if (rc) {
            rc = -EINVAL;
            printk("IO input settings are incorrect\r\n");
            goto fail_gpio_input;
        }

        // 5.获取中断号
        dev->key_irq[i].irq_num = gpio_to_irq(dev->key_irq[i].gpio);
#if 0
            dev->key_irq[i].irq_num = irq_of_parse_and_map(dev->node,i);
#endif
        printk("key%d,gpio=%d, irqnum=%d \r\n", i, dev->key_irq[i].gpio, dev->key_irq[i].irq_num);

        // 6.中断初始化
        rc = request_irq(dev->key_irq[i].irq_num, dev->key_irq[i].handler,
                         IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, dev->key_irq->name, dev);
        if (rc) {
            rc = -EINVAL;
            pr_err("request irq %d failed\r\n", dev->key_irq[i].irq_num);
            goto fail_request_irq;
        }
    }
    INIT_WORK(&dev->work, key_work);

fail_request_irq:
fail_gpio_input:
    for (i = 0; i < KEY_NUM; i++)
        gpio_free(interrupt.key_irq[i].gpio);

fail_gpio_req:
fail_led_gpio:
fail_node:
    return rc;
}

/* 驱动入口函数 */
static int __init interrupt_init(void)
{
    int rc = 0;

    /* 注册字符设备 */
    // 1.分配设备号
    interrupt.major = 0; // 内核指定分配设备号
    if (interrupt.major) {
        interrupt.devid = MKDEV(interrupt.major, 0);
        rc              = register_chrdev_region(interrupt.devid, INTERRUPT_COUNT, INTERRUPT_NAME);
    }
    else {
        rc              = alloc_chrdev_region(&interrupt.devid, 0, INTERRUPT_COUNT, INTERRUPT_NAME);
        interrupt.major = MAJOR(interrupt.devid);
        interrupt.minor = MINOR(interrupt.devid);
    }
    if (rc < 0) {
        printk("interrupt chrdev region error\r\n");
        goto fail_devid;
    }
    printk("major=%d,minor=%d\r\n", interrupt.major, interrupt.minor);

    // 2.注册字符设备
    interrupt.cdev.owner = THIS_MODULE;
    cdev_init(&interrupt.cdev, &interrupt_fops);
    rc = cdev_add(&interrupt.cdev, interrupt.devid, INTERRUPT_COUNT);
    if (rc < 0) {
        rc = -EINVAL;
        printk("interrupt cdev failed\r\n");
        goto fail_cdev;
    }

    // 3.动态创建设备的逻辑类
    interrupt.class = class_create(THIS_MODULE, INTERRUPT_NAME);
    if (IS_ERR(interrupt.class)) {
        rc = PTR_ERR(interrupt.class);
        printk("interrupt class create failed\r\n");
        goto fail_class;
    }

    // 4.自动创建设备节点
    interrupt.device = device_create(interrupt.class, NULL, interrupt.devid, NULL, INTERRUPT_NAME);
    if (IS_ERR(interrupt.device)) {
        rc = PTR_ERR(interrupt.device);
        printk("interrupt device create failed\r\n");
        goto fail_device;
    }

    // 5.按键初始化
    rc = keyio_init(&interrupt);
    if (rc < 0) {
        printk("key init failed\r\n");
        goto fail_key;
    }

    // 初始化定时器
    init_timer(&interrupt.timer);
    interrupt.timer.function = timer_cb; // 设置定时器回调函数

    // 初始化原子变量
    atomic_set(&interrupt.key_value, KEY_INVALID);
    atomic_set(&interrupt.key_release, 0);

    printk("interrupt dirver init\r\n");

    return rc;

fail_key:
    device_destroy(interrupt.class, interrupt.devid);
fail_device:
    class_destroy(interrupt.class);
fail_class:
    cdev_del(&interrupt.cdev);
fail_cdev:
    unregister_chrdev_region(interrupt.devid, INTERRUPT_COUNT);
fail_devid:
    return rc;
}

/* 驱动出口函数 */
static void __exit interrupt_exit(void)
{
    int i = 0;

    // 2.释放中断和IO
    for (i = 0; i < KEY_NUM; i++) {
        free_irq(interrupt.key_irq[i].irq_num, &interrupt);
        gpio_free(interrupt.key_irq[i].gpio);
    }

    // 删除定时器
    del_timer_sync(&interrupt.timer);

    // 3.删除字符设备
    cdev_del(&interrupt.cdev);
    // 4.注销设备号
    unregister_chrdev_region(interrupt.devid, INTERRUPT_COUNT);
    // 5.销毁设备节点
    device_destroy(interrupt.class, interrupt.devid);
    // 6.销毁设备的逻辑类
    class_destroy(interrupt.class);

    printk("interrupt dirver exit\r\n");
}

/* 驱动注册与卸载 */
module_init(interrupt_init);
module_exit(interrupt_exit);
MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("key interrupt driver");