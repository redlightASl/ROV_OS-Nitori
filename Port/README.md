# ROV_OS-Nitori移植

该目录下包含了各个CPU的统一移植文件和一个用于硬件加速外设API编写的底层接口文件（HardwareAccelerate）。同时所有RTOS中用到的数据类型定义也位于该目录下（Define.h）

移植过程中需要完成以下内容的编写
* .S汇编文件
    对应CPU架构的PendSV中断和NVIC
    