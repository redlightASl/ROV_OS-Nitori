# ROV_OS-Nitori移植

该目录下包含了各个CPU的统一移植文件和一个用于硬件加速外设API编写的底层接口文件（HardwareAccelerate）。同时所有RTOS中用到的数据类型定义也位于该目录下（Define.h）

移植过程中需要完成以下内容的编写

* .S汇编文件
  对应CPU架构的PendSV中断
* Port.h文件
  采用c内嵌汇编实现NVIC/PLIC中断管理函数。将通用寄存器以结构体的形式映射到内存，方便汇编-c融合操作。特殊的硬件控制函数需要单独封装在Port.h中供上层程序调用
* HardwareAccelerate.c、HardwareAccelerate.h文件
  专用于硬件加速的外设寄存器与相关应用程序HardwareAccelerate
* Define.h文件
  使用宏定义形式实现面向stm32-HAL库的函数接口
