/**
 * @file    : bolock_io.c
 * @author  : KeeneChen
 * @date    : 2023.04.02-19:38:44
 * @details : key interrupt driver
 */

/**
 * 等待头结构体
 ** struct __wait_queue_head {
 **     spinlock_t lock;
 **     struct list_head task_list;
 ** };
 ** typedef struct __wait_queue_head wait_queue_head_t;
 *
 ** void init_waitqueue_head(wait_queue_head_t *q)
 * @brief 初始化等待队列头
 * @param q 就是要初始化的等待队列头
 * 也可以使用宏 DECLARE_WAIT_QUEUE_HEAD 来一次性完成等待队列头的定义的初始化
 *
 * 等待队列项
 ** struct __wait_queue {
 **    unsigned int flags;
 **    void *private;
 **    wait_queue_func_t func;
 **    struct list_head task_list;
 ** };
 ** typedef struct __wait_queue wait_queue_t;
 *
 ** DECLARE_WAITQUEUE(name, tsk)
 *
 ** void add_wait_queue(wait_queue_head_t *q,wait_queue_t *wait)
 * @brief 将队列项添加等待队列头
 * @param q 等待队列项要加入的等待队列头
 * @param wait 要加入的等待队列项
 *
 ** void remove_wait_queue(wait_queue_head_t *q,wait_queue_t *wait)
 * @brief 将队列项移除等待队列头
 * @param q 等待队列项要加入的等待队列头
 * @param wait 要加入的等待队列项
 *
 * 等待唤醒
 ** void wake_up(wait_queue_head_t *q)
 ** void wake_up_interruptible(wait_queue_head_t *q)
 * @brief 这两个函数会将这个等待队列头中的所有进程都唤醒wake_up 函数可以唤醒处于
 * TASK_INTERRUPTIBLE 和 TASK_UNINTERRUPTIBLE 状态的进程,而 wake_up_interruptible 函数只能唤醒处于
 * TASK_INTERRUPTIBLE 状态的进程
 * @param q 就是要唤醒的等待队列头
 *
 * 等待事件
 ** wait_event(wq, condition)
 * @brief 等待以 wq 为等待队列头的等待队列被唤醒,前提是 condition 条件必须满足(为真),
 * 否则一直阻塞此函数会将进程设置为TASK_UNINTERRUPTIBLE 状态
 ** wait_event_timeout(wq, condition, timeout)
 * @brief 功能和 wait_event 类似,但是此函数可以添加超时时间,以 jiffies 为单位
 * 此函数有返回果返回 0 的话表示超时时间到,而且 condition为假
 * 为 1 的话表示 condition为真,也就是条件满足了
 ** wait_event_interruptible(wq, condition)
 * @brief 与 wait_event 函数类似,但是此函数将进程设置为 TASK_INTERRUPTIBLE,就是可以被信号打断
 ** wait_event_interruptible_timeout(wq,condition, timeout)
 * @brief 与 wait_event_timeout 函数类似,此函数也将进程设置为 TASK_INTERRUPTIBLE,可以被信号打断
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
#include <linux/wait.h>

#define INTERRUPT_NAME  "interrupt" // 设备名
#define INTERRUPT_COUNT 1           // 设备个数
#define KEY_NUM         1           // 按键个数
#define KEY_VALUE       0x01        // 按键有效值
#define KEY_INVALID     0xFF        // 按键无效值
#define WAIT_FLAG       0           // 等待头or等待项

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
    atomic_t key_release;                 // 标记是否完成按键,包括按下和释放
    wait_queue_head_t read_wait;          // 读队列头
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

#if WAIT_FLAG
    // 等待事件
    // wait_event(dev->read_wait, atomic_read(&dev->key_release));
    wait_event_interruptible(dev->read_wait, atomic_read(&dev->key_release)); // 等待按键有效
#else
    DECLARE_WAITQUEUE(wait, current); // 定义等待队列项
    if (atomic_read(&dev->key_release) == 0) {
        add_wait_queue(&dev->read_wait, &wait); // 按键未按下时,将等待队列项加入等待队列头中等待唤醒
        __set_current_state(TASK_INTERRUPTIBLE); // 当前进程设置为可被打断的状态
        schedule();                              // 切换进程状态

        // 唤醒后判断是否由按键有效唤醒,还是被信号唤醒
        if (signal_pending(current)) {
            rc = -ERESTARTSYS;
            goto wait_err;
        }

        __set_current_state(TASK_RUNNING);         // 将进程设置为运行态
        remove_wait_queue(&dev->read_wait, &wait); // 移除等待队列项
    }
#endif

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

#if WAIT_FLAG
#else
wait_err:
    __set_current_state(TASK_RUNNING);         // 将进程设置为运行态
    remove_wait_queue(&dev->read_wait, &wait); // 移除等待队列项
#endif

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

    dev->timer.data = (unsigned long)dev_id;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(10));

    return IRQ_HANDLED;
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

    // 唤醒等待头
    if (atomic_read(&dev->key_release)) {
        // wake_up(&dev->read_wait);
        wake_up_interruptible(&dev->read_wait); // 按键有效就唤醒
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
    interrupt.timer.function = timer_cb;

    // 初始化原子变量
    atomic_set(&interrupt.key_value, KEY_INVALID);
    atomic_set(&interrupt.key_release, 0);

    // 初始化读等待队列头
    init_waitqueue_head(&interrupt.read_wait);

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

    // 1.释放中断和IO
    for (i = 0; i < KEY_NUM; i++) {
        free_irq(interrupt.key_irq[i].irq_num, &interrupt);
        gpio_free(interrupt.key_irq[i].gpio);
    }

    // 2.删除定时器
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