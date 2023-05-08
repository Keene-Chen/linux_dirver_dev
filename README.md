# I.MX6ULL Linux 驱动开发项目

## 简介
本项目是用于学习 `NXP i.MX6ULL Linux` 驱动开发，教程来源正点原子 `I.MX6U 嵌入式 Linux 驱动开发指南`。

## 开发环境
### 物理机
- OS: Windows 10 
- CPU: Intel(R) Core(TM) i5-8300H CPU @ 2.30GHz
- VMware® Workstation 16 Pro: v16.2.1
- Visual Studio Code: v1.77.1
    - Remote-SSH
    - clangd
- MobaXterm: v22.1

### 虚拟机
- Ubuntu 22.04_LTS

### 开发板
- 正点原子 I.MX6U-MINI: v1.8
- CPU: MCIMX6Y2CVM08AB
    - 32KB I/DL1-Cache
    - 单核 Cortex-A7
- DDR3: 512MB
- EMMC: 8GB

### Linux
- Kernel: linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek
- Uboot: uboot-imx-rel_imx_4.1.15_2.1.0_ga_alientek
- Busybox: 1.35.0
- Cross Compiler: gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf

## 源码目录
```
./linux_dirver_dev
├── 01_chrdevbase   # 字符设备驱动开发
├── 02_led          # 嵌入式 Linux LED 驱动开发实验
├── 03_newchrled    # 新字符设备驱动实验
├── 04_dtsled       # 设备树下的LED驱动实验
├── 05_gpioled      # pinctrl 和 gpio 子系统实验
├── 06_beep         # Linux 蜂鸣器实验
├── 07_concurrency_competition # Linux 并发与竞争实验
├── 08_chr_dev_temp # 字符设备驱动开发模板
├── 09_key          # Linux 按键输入实验
├── 10_timer        # Linux 内核定时器实验
├── 11_interrupt    # Linux 中断实验
├── 12_block_nonblock_io # Linux 阻塞和非阻塞IO实验
├── 13_async_notice # Linux 异步通知实验
├── 14_platform     # Linux platform 设备驱动实验
├── 15_dts_platform # Linux 设备树下的 platform 驱动实验
├── 16_misc_beep    # Linux MISC 驱动实验
├── 17_input        # Linux INPUT 子系统实验
├── .clang-format   # 代码风格格式化
├── .gitignore      # git 忽略文件
├── .vscode         # vscode 配置文件
└── README.md
```

## Linux 自带驱动(非项目驱动)
```
Linux 自带的 LED 灯驱动实验
Linux LCD 驱动实验
Linux RTC 驱动实验
Linux I2C驱动实验
Linux SPI驱动实验
Linux RS232/485/GPS驱动实验
Linux 多点电容触摸屏实验
Linux 音频驱动实验
Linux CAN驱动实验
Linux USB驱动实验
Linux 块设备驱动实验
Linux 网络驱动实验
Linux WIFI驱动实验
Linux 4G通信实验
RGB转HDM实验
Linux PWM驱动实验
Regmap API3实验
Linux lIO驱动实验
Linux ADC驱动实验
```
