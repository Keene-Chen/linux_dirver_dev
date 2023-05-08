/**
 * @file    : oled_driver.c
 * @author  : KeeneChen
 * @date    : 2023.04.17-16:18:28
 * @details : oled_driver
 */

#include "oled_font.h"

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/types.h>

#define OLED_CNT  1
#define OLED_NAME "oled"
#define OLED_CMD  0x00
#define OLED_DATA 0x40

struct oled_dev {
    dev_t devid;              // 设备号
    struct cdev cdev;         // cdev
    struct class* class;      // 类
    struct device* device;    // 设备
    struct device_node* node; // 设备节点
    int major;                // 主设备号
    void* private_data;       // 私有数据
} oleddev;

/**
 * @brief SSD1306 初始化参数
 * @param 0xAE display off
 * @param 0x20 Set Memory Addressing Mode
 * @param 0x10 00,Horizontal Addressing Mode;
 *             01,Vertical Addressing Mode;
 *             10,PageAddressing Mode (RESET);
 *             11,Invalid
 * @param 0xB0 Set Page Start Address for Page Addressing Mode,0-7
 * @param 0xC8 Set COM Output Scan Direction
 * @param 0x00 set low column address
 * @param 0x10 set high column address
 * @param 0x40 set start line address
 * @param 0x81 set contrast control register
 * @param 0xFF 亮度调节 0x00~0xff
 * @param 0xA1 set segment re-map 0 to 127
 * @param 0xA6 set normal display
 * @param 0xA8 set multiplex ratio(1 to 64)
 * @param 0x3F
 * @param 0xA4 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
 * @param 0xD3 set display offset
 * @param 0x00 not offset
 * @param 0xD5 set display clock divide ratio/oscillator frequency
 * @param 0xF0 set divide ratio
 * @param 0xD9 set pre-charge period
 * @param 0x22
 * @param 0xDA set com pins hardware configuration
 * @param 0x12
 * @param 0xDB set vcomh
 * @param 0x20 0x20,0.77xVcc
 * @param 0x8D set DC-DC enable
 * @param 0x14
 * @param 0xAF turn on oled panel
 */
uint8_t oled_init_data[] = {
    0xAE, 0x20, 0x10, 0xB0, 0xC8, 0x00, 0x10, 0x40, 0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F,
    0xA4, 0xD3, 0x00, 0xD5, 0xF0, 0xD9, 0x22, 0xDA, 0x12, 0xDB, 0x20, 0x8D, 0x14, 0xAF,
};

/**
 * @brief 向oled多个寄存器写入数据
 * @param dev oled设备
 * @param reg 要写入的寄存器首地址
 * @param val 要写入的数据缓冲区
 * @param len 要写入的数据长度
 * @return 操作结果
 */
static s32 oled_write_regs(struct oled_dev* dev, u8 reg, u8* buf, u8 len)
{
    u8 b[256];
    struct i2c_msg msg;
    struct i2c_client* client = (struct i2c_client*)dev->private_data;
    b[0]                      = reg; /* 寄存器首地址 */
    memcpy(&b[1], buf, len);         /* 将要写入的数据拷贝到数组b里面 */
    msg.addr  = client->addr;        /* oled 地址 */
    msg.flags = 0;                   /* 标记为写数据 */
    msg.buf   = b;                   /* 要写入的数据缓冲区 */
    msg.len   = len + 1;             /* 要写入的数据长度 */
    return i2c_transfer(client->adapter, &msg, 1);
}

static void oled_write_cmd(u_char i2c_command)
{
    oled_write_regs(&oleddev, OLED_CMD, &i2c_command, 1);
}

static void oled_write_data(u_char i2c_command)
{
    oled_write_regs(&oleddev, OLED_DATA, &i2c_command, 1);
}

static void oled_set_pos(u_char x, u_char y) // 设置起始点坐标
{
    oled_write_cmd(0xB0 + y);
    oled_write_cmd(((x & 0xF0) >> 4) | 0x10);
    oled_write_cmd((x & 0x0F) | 0x01);
}

static void oled_fill(u_char fill_data) // 全屏填充
{
    u_char i, j;
    for (i = 0; i < 8; i++) {
        oled_write_cmd(0xb0 + i); // page0-page1
        oled_write_cmd(0x00);     // low column start address
        oled_write_cmd(0x10);     // high column start address
        for (j = 0; j < 128; j++) {
            oled_write_data(fill_data);
        }
    }
}

static void oled_clear(void) // 清屏
{
    oled_fill(0x00);
}

static void oled_reg_init(void)
{
    mdelay(100); // 这里的延时很重要
    oled_write_regs(&oleddev, 0x00, oled_init_data, sizeof(oled_init_data));
    oled_clear();
    printk("oled reginit................\r\n");
}

/**
 * @brief 显示codetab.h中的ASCII字符,有6*8和8*16可选择
 * @param x 起始点坐标(x:0~127, y:0~7)
 * @param y 起始点坐标(x:0~127, y:0~7)
 * @param ch 要显示的字符串
 * @param size 字符大小(1:6*8 ; 2:8*16)
 */
static void oled_show_str(u_char x, u_char y, u_char ch[], u_char size)
{
    u_char c = 0, i = 0, j = 0;
    switch (size) {
    case 1: {
        while (ch[j] != '\0') {
            c = ch[j] - 32;
            if (x > 126) {
                x = 0;
                y++;
            }
            oled_set_pos(x, y);
            for (i = 0; i < 6; i++)
                oled_write_data(F6x8[c][i]);
            x += 6;
            j++;
        }
    } break;
    case 2: {
        while (ch[j] != '\0') {
            c = ch[j] - 32;
            if (x > 120) {
                x = 0;
                y++;
            }
            oled_set_pos(x, y);
            for (i = 0; i < 8; i++)
                oled_write_data(F8X16[c * 16 + i]);
            oled_set_pos(x, y + 1);
            for (i = 0; i < 8; i++)
                oled_write_data(F8X16[c * 16 + i + 8]);
            x += 8;
            j++;
        }
    } break;
    }
}

#if 0

/**
 * @brief  开启 OLED
 */
static void oled_on(void)
{
    oled_write_cmd(0X8D); // 设置电荷泵
    oled_write_cmd(0X14); // 开启电荷泵
    oled_write_cmd(0XAF); // OLED唤醒
}

/**
 * @brief  关闭OLED 休眠模式下,OLED功耗不到10uA
 */
static void oled_off(void)
{
    oled_write_cmd(0X8D); // 设置电荷泵
    oled_write_cmd(0X10); // 关闭电荷泵
    oled_write_cmd(0XAE); // OLED休眠
}

/**
 * @brief 显示codetab.h中的汉字,16*16点阵
 * @param x 起始点坐标(x:0~127, y:0~7)
 * @param y 起始点坐标(x:0~127, y:0~7)
 * @param n 汉字在codetab.h中的索引
 */
static void oled_show_cn(u_char x, u_char y, u_char n)
{
    u_char wm          = 0;
    unsigned int adder = 32 * n;
    oled_set_pos(x, y);
    for (wm = 0; wm < 16; wm++) {
        oled_write_data(Hzk[adder]);
        adder += 1;
    }
    oled_set_pos(x, y + 1);
    for (wm = 0; wm < 16; wm++) {
        oled_write_data(Hzk[adder]);
        adder += 1;
    }
}

/**
 * @brief 显示BMP位图
 * @param x0 起始点坐标(x0:0~127,y0:0~7)
 * @param y0 起始点坐标(x0:0~127,y0:0~7)
 * @param x1 起点对角线(结束点)的坐标(x1:1~128,y1:1~8)
 * @param y1 起点对角线(结束点)的坐标(x1:1~128,y1:1~8)
 */
static void oled_draw_bmp(u_char x0, u_char y0, u_char x1, u_char y1, u_char BMP[])
{
    unsigned int j = 0;
    u_char x, y;
    if (y1 % 8 == 0)
        y = y1 / 8;
    else
        y = y1 / 8 + 1;
    for (y = y0; y < y1; y++) {
        oled_set_pos(x0, y);
        for (x = x0; x < x1; x++) {
            oled_write_data(BMP[j++]);
        }
    }
}

/**
 * @brief  开启OLED颜色反转
 * @param  i 0:黑底白字 1:白底黑字
 * @return void
 */
static void oled_color_turn(uint8_t i)
{
    if (i == 0) {
        oled_write_cmd(0xA6); // 正常显示
    }
    if (i == 1) {
        oled_write_cmd(0xA7); // 反色显示
    }
}

/**
 * @brief  OLED 反转显示
 * @param  i 0:正常模式 1:180反 2:镜像翻转 3:镜像翻转
 * @return void
 */
static void oled_display_turn(uint8_t i)
{
    if (i == 0) {
        oled_write_cmd(0xC8); // 从上到下
        oled_write_cmd(0xA1); // 从左到右
    }
    else if (i == 1) {
        oled_write_cmd(0xC0); // 从下到上,180翻转
        oled_write_cmd(0xA0); // 从右到左
    }
    else if (i == 2) {
        oled_write_cmd(0xC8); // 从上到下,镜像翻转
        oled_write_cmd(0xA0); // 从右到左
    }
    else if (i == 3) {
        oled_write_cmd(0xC0); // 从下到上,镜像翻转
        oled_write_cmd(0xA1); // 从左到右
    }
}
#endif

/**
 * @brief 打开设备
 * @param inode 传递给驱动的inode
 * @param filp 设备文件,file结构体有个叫做private_data的成员变量,
 *             一般在open的时候将private_data指向设备结构体。
 * @return 0 成功;其他 失败
 */
static int oled_open(struct inode* inode, struct file* filp)
{
    filp->private_data = &oleddev;
    return 0;
}

/**
 * @brief 向设备写数据
 * @param filp 设备文件,表示打开的文件描述符
 * @param buf 要写给设备写入的数据
 * @param cnt 要写入的数据长度
 * @param offt 相对于文件首地址的偏移
 * @return 写入的字节数,如果为负值,表示写入失败
 */
static int oled_write(struct file* filp, const char __user* buf, size_t cnt, loff_t* offt)
{
    int retvalue;
    u_char databuf[10];

    retvalue = copy_from_user(databuf, buf, cnt);
    if (retvalue < 0) {
        printk("kernel write failed!\r\n");
        return -EFAULT;
    }
    if (databuf[0] == 0) {
        oled_clear();
    }
    else {
        oled_show_str(0, 0, databuf, 2);
    }
    return 0;
}

/* oled操作函数 */
static const struct file_operations oled_ops = {
    .owner = THIS_MODULE,
    .open  = oled_open,
    .write = oled_write,
};

/**
 * @brief i2c驱动的probe函数,当驱动与设备匹配以后此函数就会执行
 * @param client i2c设备
 * @param id i2c设备ID
 * @return 0,成功;其他负值,失败
 */
static int oled_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
    /* 1.构建设备号 */
    if (oleddev.major) {
        oleddev.devid = MKDEV(oleddev.major, 0);
        register_chrdev_region(oleddev.devid, OLED_CNT, OLED_NAME);
    }
    else {
        alloc_chrdev_region(&oleddev.devid, 0, OLED_CNT, OLED_NAME);
        oleddev.major = MAJOR(oleddev.devid);
    }

    /* 2.注册设备 */
    cdev_init(&oleddev.cdev, &oled_ops);
    cdev_add(&oleddev.cdev, oleddev.devid, OLED_CNT);

    /* 3.创建类 */
    oleddev.class = class_create(THIS_MODULE, OLED_NAME);
    if (IS_ERR(oleddev.class)) {
        return PTR_ERR(oleddev.class);
    }

    /* 4.创建设备 */
    oleddev.device = device_create(oleddev.class, NULL, oleddev.devid, NULL, OLED_NAME);
    if (IS_ERR(oleddev.device)) {
        return PTR_ERR(oleddev.device);
    }

    /* 5.OLED 初始化 */
    oleddev.private_data = client;
    oled_reg_init();
    mdelay(1000);

    oled_show_str(0, 0, "1", 1);
    oled_show_str(0, 1, "2", 1);
    oled_show_str(0, 2, "3", 1);
    oled_show_str(0, 3, "4", 1);
    oled_show_str(0, 4, "5", 1);
    oled_show_str(0, 5, "6", 1);
    oled_show_str(0, 6, "7", 1);
    oled_show_str(0, 7, "8", 1);

    return 0;
}

/**
 * @brief i2c驱动的remove函数,移除i2c驱动的时候此函数会执行
 * @param client i2c设备
 * @return 0,成功;其他负值,失败
 */
static int oled_remove(struct i2c_client* client)
{
    /* 删除设备 */
    cdev_del(&oleddev.cdev);
    unregister_chrdev_region(oleddev.devid, OLED_CNT);

    /* 注销掉类和设备 */
    device_destroy(oleddev.class, oleddev.devid);
    class_destroy(oleddev.class);

    return 0;
}

/* 传统匹配方式ID列表 */
static const struct i2c_device_id oled_id[] = {
    { "atkmini,oled", 0 },
    { /* Sentinel */ },
};

/* 设备树匹配列表 */
static const struct of_device_id oled_of_match[] = {
    { .compatible = "atkmini,oled" },
    { /* Sentinel */ },
};

/* i2c驱动结构体 */
static struct i2c_driver oled_driver = {
	.probe = oled_probe,
	.remove = oled_remove,
	.driver = {
			.owner = THIS_MODULE,
		   	.name = "oled",
		   	.of_match_table = oled_of_match, 
		   },
	.id_table = oled_id,
};

/* i2c 驱动注册与卸载辅助宏 */
module_i2c_driver(oled_driver);

/* 驱动模块信息 */
MODULE_AUTHOR("KeeneChen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("oled i2c driver");