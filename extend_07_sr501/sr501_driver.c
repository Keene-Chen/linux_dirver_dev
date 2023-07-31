/**
 * @file    : sr501_driver.c
 * @author  : KeeneChen
 * @date    : 2023.07.28-14:45:15
 * @details : sr501_driver
 */

#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#define DEV_NAME "sr501_input"
#define IRQ_NAME "SR501"

/* sr501 驱动设备结构体 */
struct sr501_input_dev {
    struct device_node* node;    // 设备节点
    struct input_dev* input_dev; // 输入设备
    int sr501_gpio;              // sr501所使用的GPIO编号
    int irq_num;                 // 中断号
} sr501_input;

/* sr501 中断处理函数 */
static irqreturn_t sr501_handler(int irq_num, void* dev_id)
{
    struct sr501_input_dev* dev = dev_id;
    int value                   = 0;

    value = gpio_get_value(dev->sr501_gpio);

    // 触发事件
    input_event(sr501_input.input_dev, EV_KEY, KEY_1, value ? 1 : 0);
    input_sync(sr501_input.input_dev);

    return IRQ_HANDLED;
}

/* sr501 初始化函数 */
static int sr501_init(struct sr501_input_dev* dev)
{
    int ret;

    // 获取设备节点
    dev->node = of_find_node_by_path("/sr501");
    if (dev->node == NULL) {
        printk(KERN_ERR "Failed to find sr501 node\n");
        return -EINVAL;
    }

    // 获取GPIO编号
    dev->sr501_gpio = of_get_named_gpio(dev->node, "sr501-gpio", 0);
    if (dev->sr501_gpio < 0) {
        printk(KERN_ERR "Failed to get sr501 gpio\n");
        return -EINVAL;
    }

    // 申请GPIO
    ret = gpio_request(dev->sr501_gpio, DEV_NAME);
    if (ret) {
        printk(KERN_ERR "Failed to request sr501 gpio\n");
        return ret;
    }

    // 设置GPIO为输入模式
    ret = gpio_direction_input(dev->sr501_gpio);
    if (ret) {
        printk(KERN_ERR "Failed to set sr501 gpio direction\n");
        return ret;
    }

    // 申请中断
    dev->irq_num = gpio_to_irq(dev->sr501_gpio);
    printk(KERN_INFO "sr501 irq_num: %d\n", dev->irq_num);

    // 注册中断处理函数
    ret = request_irq(dev->irq_num, sr501_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                      IRQ_NAME, sr501_input.input_dev);
    if (ret) {
        printk(KERN_ERR "Failed to request IRQ: %d\n", ret);
        return ret;
    }

    return 0;
}

/* 驱动入口函数 */
static int sr501_input_probe(struct platform_device* pdev)
{
    int ret;

    printk("sad");

    // 1.sr501 初始化
    ret = sr501_init(&sr501_input);
    if (ret < 0) {
        printk(KERN_ERR "Failed to initialize sr501\n");
        goto fail_sr501_init;
    }

    // 2.初始化输入设备
    sr501_input.input_dev = input_allocate_device();
    if (IS_ERR(sr501_input.input_dev)) {
        printk(KERN_ERR "Failed to allocate input device\n");
        return PTR_ERR(sr501_input.input_dev);
    }

    // 3.设置输入设备属性
    sr501_input.input_dev->name = DEV_NAME;
    set_bit(EV_KEY, sr501_input.input_dev->evbit); // 按键事件
    set_bit(KEY_1, sr501_input.input_dev->keybit); // 按键值

    // 4.注册输入设备
    ret = input_register_device(sr501_input.input_dev);
    if (ret) {
        printk(KERN_ERR "Failed to register input device: %d\n", ret);
        goto fail_input_register;
    }

    printk(KERN_INFO "SR501 Sensor driver initialized\n");
    return 0;

fail_input_register:
    input_free_device(sr501_input.input_dev);
fail_sr501_init:
    return ret;
}

/* 驱动出口函数 */
static int sr501_input_remove(struct platform_device* pdev)
{
    free_irq(sr501_input.irq_num, sr501_input.input_dev);
    gpio_free(sr501_input.sr501_gpio);
    input_unregister_device(sr501_input.input_dev);
    input_free_device(sr501_input.input_dev);
    printk(KERN_INFO "SR501 Sensor Driver Exited\n");

    return 0;
}

/* 设备匹配列表与设备树匹配 */
static const struct of_device_id sr501_input_of_match[] = {
    { .compatible = "atkmini,sr501" }
    /* Sentinel */
};

/* platform 驱动结构体 */
static struct platform_driver sr501_input_dev = {
    .driver = {
        .name = "sr501_input",
        .of_match_table = sr501_input_of_match,
    },
    .probe  = sr501_input_probe,
    .remove = sr501_input_remove,
};

/* platform 驱动注册与卸载辅助宏 */
module_platform_driver(sr501_input_dev);

/* 驱动模块信息 */
MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("SR501 Sensor Driver");