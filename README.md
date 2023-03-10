# 系统名称

基于STM32F103C8T6的循迹小车

# 软件定位

本系统实现了一个智能循迹小车。该系统利用`STM32F103C8T6`作为主控制器。
在循迹方面，采用一块树莓派作为图像处理单元，利用嵌入式Python来作为机器视觉的实现方式。
在此基础上利用人工智能和深度学习技术，使得树莓派可以识别出所要求的各种标识符号。
主控根据从上位机接受的数据来调整两轮的转速，从而达到稳定巡线的效果。
并且可以根据场地上不同标志和红绿灯状态改变巡线模式。

# 开发环境

- 操作系统：Windows 10
- 编程语言：C、Python
- 开发工具
  - STM32CubeMx
  - keilμvision5
  - pycharm
- 环境依赖
  - python 3.9

# 使用说明

## 启动项目

首先确保开发环境中的各项均准备就绪
将本目录克隆到本地文件夹内
使用keil打开`/MDK-ARM/USARTTest.uvprojx`
编译并下载到`stm32F103C8T6`最小系统版上
电路如`Circuit.png`所示
硬件组装完成后即可上电测试。
通过修改`main.c`中的PID参数来稳定巡线

## 视觉部分

1、具体程序见`/Visual/src/main.py`，可根据实际环境对参数进行调整以达到适合的效果。

2、视觉识别部分请参考`/Visual/FaststDet`，根据`README.md`说明重新训练图像集，并生成onnx文件以部署到树莓派上。此处建议在Anaconda上配置依赖环境进行模型训练。

# 目录结构

```
├── Readme.md // 说明文件
├── vision // 树莓派的python程序和模型
├── Core //stm32的主要代码文件
├── Drivers //stm32HAL库文件
├── MDK-ARM //keil工程文件
├── USARTTest.ioc // 
├── .mxproject //
```

此目录可以直接放在一个文件夹内作为keil和stm32CubeMx的工程文件打开

### 常见问题说明

1、若识别模型部署到树莓派上运行时帧率不足，可以通过减小训练图片的大小、间隔一定时间采集图片进行识别等提高帧率。
