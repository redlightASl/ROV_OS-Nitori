# ROV_OS-Nitori移植组件

该目录下包含了各个CPU的统一移植文件和一个用于硬件加速外设API编写的底层接口文件（HardwareAccelerate）。同时所有RTOS中用到的数据类型定义也位于该目录下（Define.h）

移植过程中需要完成以下内容的编写

* .S汇编文件

  直接控制CPU，为Port.h提供汇编等级的底层接口

  | **函数**                                | **描述**                                         |
  | --------------------------------------- | ------------------------------------------------ |
  | rov_float_register_pop();               | FPU寄存器出栈                                    |
  | rov_float_register_push();              | FPU寄存器压栈                                    |
  | rov_context_switch(from, to);           | 从from线程切换到to线程，用于线程之间的上下文切换 |
  | rov_context_switch_to(to);              | 没有来源线程的上下文切换                         |
  | PendSV_Handler                          | PendSV中断服务函数                               |
  | rov_context_switch_interrupt(from, to); | 线程上下文切换，在中断内部调用                   |

* Port.h文件
  采用c内嵌汇编实现NVIC/PLIC中断管理函数。将系统寄存器以结构体的形式映射到内存，方便汇编-c融合操作。**特殊的硬件加速**控制函数**可能**需要**单独封装在Port.h中供上层程序调用**

  | 宏定义               | 函数                    | **描述**     |
  | -------------------- | ----------------------- | ------------ |
  | DISABLE_INTERRUPTS() | rov_interrupt_disable() | 关闭全局中断 |
  | ENABLE_INTERRUPTS()  | rov_interrupt_enable()  | 打开全局中断 |
  | ENTER_CRITICAL()     | rov_enter_critical()    | 进入临界区   |
  | EXIT_CRITICAL()      | rov_exit_critical()     |              |
  |                      |                         |              |
  |                      |                         |              |

  

* HardwareAccelerate.c、HardwareAccelerate.h文件
  专用于硬件加速的外设寄存器与相关应用程序，如果不外挂FPGA/DSP硬件加速设备可以不移植该部分

* Define.h文件
  定义了系统使用的各种基础数据类型，~~在该文件内使用宏定义形式实现面向stm32-HAL库的函数接口~~，上层应用使用的一些状态标志位也在这里进行定义

