# Nitori的工具链搭建

Nitori使用典型的Linux风格交叉编译器-系统构建工具-内核选项配置工具作为工具链

项目支持ARM-MDK和gcc两套部署流程

这里主要介绍基于开源工具链编译并部署Nitori

使用`sudo apt install build-essential libffi-dev libssl-dev`安装基础依赖

使用`sudo apt install arm-none-eabi-gcc`安装交叉编译器

使用`sudo apt install make cmake`安装系统构建工具和内核选项配置工具

使用`sudo apt install dfu-util libusb-1.0-0 openocd`安装烧录工具

