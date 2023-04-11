/**
 * @file    : key_input_driver.c
 * @author  : KeeneChen
 * @date    : 2023.04.09-21:33:58
 * @details : key input driver
 */

/**
 * linux input 子系统 API 函数
 *
 * input_dev 结构体
 ** struct input_dev {
 **     const char *name;
 **     const char *phys;
 **     const char *uniq;
 **     struct input_id id;
 **     unsigned long propbit[BITS_TO_LONGS(INPUT_PROP_CNT)];
 **     unsigned long evbit[BITS_TO_LONGS(EV_CNT)];   // 事件类型的位图
 **     unsigned long keybit[BITS_TO_LONGS(KEY_CNT)]; // 按键值的位图
 **     unsigned long relbit[BITS_TO_LONGS(REL_CNT)]; // 相对坐标的位图
 **     unsigned long absbit[BITS_TO_LONGS(ABS_CNT)]; // 绝对坐标的位图
 **     unsigned long mscbit[BITS_TO_LONGS(MSC_CNT)]; // 杂项事件的位图
 **     unsigned long ledbit[BITS_TO_LONGS(LED_CNT)]; // LED 相关的位图
 **     unsigned long sndbit[BITS_TO_LONGS(SND_CNT)]; // sound 有关的位图
 **     unsigned long ffbit[BITS_TO_LONGS(FF_CNT)];   // 压力反馈的位图
 **     unsigned long swbit[BITS_TO_LONGS(SW_CNT)];   // 开关状态的位图
 **     ......
 **     bool devres_managed;
 ** };
 * @note #include <linux/input.h>
 *
 * 输入事件宏定义
 ** #define EV_SYN 0x00 // 同步事件
 ** #define EV_KEY 0x01 // 按键事件
 ** #define EV_REL 0x02 // 相对坐标事件
 ** #define EV_ABS 0x03 // 绝对坐标事件
 ** #define EV_MSC 0x04 // 杂项(其他)事件
 ** #define EV_SW 0x05  // 开关事件
 ** #define EV_LED 0x11 // LED
 ** #define EV_SND 0x12 // sound(声音)
 ** #define EV_REP 0x14 // 重复事件
 ** #define EV_FF 0x15  // 压力事件
 ** #define EV_PWR 0x16 // 电源事件
 ** #define EV_FF_STATUS 0x17 // 压力状态事件
 * @note #include <include/uapi/linux/input.h>
 *
 ** struct input_dev *input_allocate_device(void)
 * @brief 申请一个 input_dev
 * @return 申请到的 input_dev
 *
 ** void input_free_device(struct input_dev *dev)
 * @brief 释放 input_dev
 * @param dev 要释放的 input_dev
 *
 ** int input_register_device(struct input_dev *dev)
 * @brief 申请好一个 input_dev 以后就需要初始化这个 input_dev
 * @param dev 待注册的 input_dev
 * @return 0,input_dev 注册成功;负值,input_dev 注册失败
 *
 ** void input_unregister_device(struct input_dev *dev)
 * @brief 注销 input_dev
 * @param dev 要注销的 input_dev
 *
 * input_event 结构体
 ** struct input_event {
 **     struct timeval time;
 **     __u16 type;
 **     __u16 code;
 **     __s32 value;
 ** };
 * @note #include <include/uapi/linux/input.h>
 * @param time 时间,也就是此事件发生的时间
 * @param type 事件类型,比如 EV_KEY,表示此次事件为按键事件,此成员变量为 16 位
 * @param code 事件码,比如在 EV_KEY 事件中 code 就表示具体的按键码,此成员变量为 16 位
 * @param value 值,比如 EV_KEY 事件中 value 就是按键值,1为按下,0为释放
 *
 *
 ** void input_event(struct input_dev *dev,unsigned int type,unsigned int code,int value)
 * @brief 用于上报指定的事件以及对应的值
 * @param dev  需要上报的 input_dev
 * @param type 上报的事件类型,比如 EV_KEY
 * @param code 事件码,也就是我们注册的按键值,比如 KEY_0、KEY_1 等等
 * @param value 事件值,比如 1 表示按键按下,0 表示按键松开
 *
 ** void input_sync(struct input_dev *dev)
 * @brief 上报一个同步事件
 * @param dev 需要上报同步事件的 input_dev
 */

#include <linux/cdev.h>
#include <linux/ide.h>
#include <linux/input.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#define DEV_NAME    "key_input" // 设备名
#define DEV_COUNT   1           // 设备个数
#define KEY_NUM     1           // 按键个数
#define KEY_VALUE   0x01        // 按键有效值
#define KEY_INVALID 0xFF        // 按键无效值

/* key 结构体 */
struct key_irq_desc {
    int gpio;                           // IO编号
    int irq_num;                        // 中断号
    unsigned char value;                // 按键值
    char name[10];                      // 名称
    irqreturn_t (*handler)(int, void*); // 中断处理函数
};

/* key_input 驱动设备结构体 */
struct key_input_dev {
    struct device_node* node;             // 设备节点
    struct key_irq_desc key_irq[KEY_NUM]; // 按键结构体
    struct timer_list timer;              // 定时器
    struct input_dev* input_dev;          // 输入设备
} key_input;

/* 按键中断函数 */
static irqreturn_t key_handler(int irq_num, void* dev_id)
{
    struct key_input_dev* dev = dev_id;

    dev->timer.data = (unsigned long)dev_id;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(10));

    return IRQ_HANDLED;
}

/* 定时器回调函数 */
static void timer_cb(unsigned long arg)
{
    struct key_input_dev* dev = (struct key_input_dev*)arg;
    int value                 = 0;

    value = gpio_get_value(dev->key_irq[0].gpio);
    if (value == 0) {
        input_event(dev->input_dev, EV_KEY, KEY_0, 1); // 按键按下,上报按键值
        input_sync(dev->input_dev);                    // 同步
    }
    else if (value == 1) {
        input_event(dev->input_dev, EV_KEY, KEY_0, 0); // 按键释放,上报按键值
        input_sync(dev->input_dev);                    // 同步
    }
}

/* 按键初始化 */
static int keyio_init(struct key_input_dev* dev)
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
            goto fail_key_gpio;
        }

        // 初始化按键结构体
        memset(dev->key_irq[i].name, 0, sizeof(dev->key_irq[i].name));
        sprintf(dev->key_irq[i].name, "key-%d", i);
        dev->key_irq[0].handler = key_handler;
        dev->key_irq[0].value   = KEY_0;

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
                         IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, dev->key_irq[i].name, dev);
        if (rc) {
            rc = -EINVAL;
            pr_err("request irq %d failed\r\n", dev->key_irq[i].irq_num);
            goto fail_request_irq;
        }
    }

    // 7.初始化定时器
    init_timer(&key_input.timer);
    key_input.timer.function = timer_cb;

    return 0;

fail_request_irq:
fail_gpio_input:
    for (i = 0; i < KEY_NUM; i++)
        gpio_free(key_input.key_irq[i].gpio);

fail_gpio_req:
fail_key_gpio:
fail_node:
    return rc;
}

/* 驱动入口函数 */
static int key_input_probe(struct platform_device* pdev)
{
    int rc = 0;

    // 1.按键初始化
    rc = keyio_init(&key_input);
    if (rc < 0) {
        printk("key init failed\r\n");
        goto fail_key_init;
    }

    // 2.注册input_dev输入设备
    key_input.input_dev = input_allocate_device();
    if (IS_ERR(key_input.input_dev)) {
        return PTR_ERR(key_input.input_dev);
    }

    key_input.input_dev->name = DEV_NAME;
    __set_bit(EV_KEY, key_input.input_dev->evbit); // 按键事件
    __set_bit(EV_REP, key_input.input_dev->evbit); // 按键重复事件
    __set_bit(KEY_0, key_input.input_dev->keybit); // 按键值

    rc = input_register_device(key_input.input_dev);
    if (rc) {
        printk("register input device failed!\r\n");
        goto fail_input_register;
    }

    printk("key_input dirver probe\r\n");
    return 0;

fail_input_register:
    input_free_device(key_input.input_dev);
fail_key_init:
    return rc;
}

/* 驱动出口函数 */
static int key_input_remove(struct platform_device* pdev)
{
    int i = 0;

    // 1.释放中断和IO
    for (i = 0; i < KEY_NUM; i++) {
        free_irq(key_input.key_irq[i].irq_num, &key_input);
        gpio_free(key_input.key_irq[i].gpio);
    }

    // 2.删除定时器
    del_timer_sync(&key_input.timer);

    // 3.注销input_dev输入设备
    input_unregister_device(key_input.input_dev);
    input_free_device(key_input.input_dev);

    printk("key_input dirver remove\r\n");

    return 0;
}

/* 设备匹配列表与设备树匹配 */
static const struct of_device_id key_input_of_match[] = {
    { .compatible = "atkmini,key" }
    /* Sentinel */
};

/* platform 驱动结构体 */
static struct platform_driver key_input_dev = {
    .driver = {
        .name = "key_input",
        .of_match_table = key_input_of_match,
    },
    .probe  = key_input_probe,
    .remove = key_input_remove,
};

/* platform 驱动注册与卸载辅助宏 */
module_platform_driver(key_input_dev);

/* 驱动模块信息 */
MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("key input driver");