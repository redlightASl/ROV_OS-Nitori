# 系统架构描述

## 硬件层

Nitori的移植对象是水下机器人的主控MCU。

硬件层是指由MCU中CPU、中断控制器、系统定时器、其他片上外设、片上或片外硬件加速设备构成硬件实体的抽象，Nitori通过控制CPU，并由CPU控制各个寄存器从而对硬件实体进行控制。

Nitori通过Port组件统一管理CPU和中断，并通过系统定时器实现线程调度；通过Sensor组件统一管理其他片上外设和硬件加速设备。

## 移植层

移植层包括Port组件和Sensor组件的底层部分。Port组件用于适配各种不同类型CPU的线程调度、定时器、中断，以汇编形式编写并向上层提供C形式的标准API；Sensor底层组件用于适配硬件的各个外设寄存器，可以使用硬件厂商给出的标准外设寄存器函数库来简化移植，或基于寄存器自行移植外设；同时还拥有一个可针对异构计算移植的硬件运算加速库

移植层中还带有一套系统内定义的标准宏用于上层软件编写和适配各种编译器。

### API列表

```c
static ROV_ALWAYS_INLINE u32 rov_interrupt_disable(void); //关闭全局中断
static ROV_ALWAYS_INLINE void rov_interrupt_enable(u32 basepri); //开启全局中断
void rov_exception_install(u8(*exception_handle)(void* context)); //设置异常处理函数
ROV_WEAK void rov_cpu_shutdown(void); //进入死循环
ROV_WEAK void rov_cpu_reset(void); //软件复位

u8* rov_thread_stack_init(void* thread_entry, void* parameter, u8* stack_addr, void* thread_exit);
//设置线程栈

ROV_WEAK void SysTickInterrupt_Handler(void); //系统时钟中断处理
ROV_WEAK void SysTickInterrupt_Initer(void); //初始化系统时钟
```

硬件加速API

```c
ROV_ALWAYS_INLINE void rov_add_f32(f32* pSrcA, f32* pSrcB, f32* pDst, u32 blockSize); //加
ROV_ALWAYS_INLINE void rov_sub_f32(f32* pSrcA,f32* pSrcB,f32* pDst,u32 blockSize); //减
ROV_ALWAYS_INLINE void rov_mult_f32(f32* pSrcA,f32* pSrcB,f32* pDst,u32 blockSize); //乘
//除
ROV_ALWAYS_INLINE void rov_abs_f32(f32* pSrc, f32* pDst, u32 blockSize); //绝对值
ROV_WEAK ROV_ALWAYS_INLINE u8 CrcCalculate(u8* CacString, u8 CacStringSize); //CRC校验
ROV_WEAK ROV_ALWAYS_INLINE u8 ParityCalculate(u8* CacString, u8 CacStringSize); //奇偶校验
ROV_WEAK ROV_ALWAYS_INLINE u8 SumCalculate(u8* CacString, u8 CacStringSize); //加和校验
ROV_WEAK ROV_ALWAYS_INLINE u8 XorCalculate(u8* CacString, u8 CacStringSize); //异或校验
//sin
//cos
//tan
//ctg
//arcsin
//arccos
//arctan
//arcctg
//乘加运算
```



## 内核层

内核层包括Thread组件、Sys组件和Sensor组件的内核部分。

内核层是Nitori的系统核心部分，通过Thread组件实现了线程对象和线程调度器。其他组件向上层提供系统内核API和外设驱动API。

Thread组件实现了线程对象、线程调度器、内核链表对象；Sys组件提供系统内核对象、信号量对象、互斥量对象、消息队列对象；Sensor组件的内核部分用于控制底层的寄存器函数，向上层提供抽象的外设对象（如UART对象、PWM控制器对象、硬件加速器对象等），可以使用硬件厂商提供的硬件抽象层函数库通过宏定义来简化移植，或基于Sensor组件底层部分自行移植外设对象。

Sensor组件会使用到Sys组件提供的系统内核对象，只有启用了Sys组件才能进一步使用Sensor组件

### 对象列表

```c
//内核链表对象
//基本内核对象
//系统时钟对象
//线程对象
//信号量对象
//互斥量对象
//消息队列对象
//GPIO对象
//UART对象
//IIC对象
//SPI对象
//硬件定时器对象
//网卡对象
//摄像头对象
//硬件加速器对象
```



### API列表

```c
```







## 服务层

服务层包括Algorithm组件、BasicCtrl组件和Sensor组件的服务部分。

服务层是Nitori的系统API实现部分，通过BasicCtrl组件向上层应用提供外设对象的方法，Sensor组件基于BasicCtrl将所有和传感器有关的硬件设备抽象为传感器对象并向用户直接提供一套API，可以文件打开的形式操作传感器设备

BasicCtrl组件实现了CPU对象、PWM对象、总线对象、网络接口对象、摄像头对象、异构计算对象等硬件抽象，上层应用可以直接调用；Sensor组件的服务部分通过底层提供的硬件API，向上层提供用于总体控制传感器的传感器类，可以简单地定义传感器数据传输格式和传输传感器数据；Algorithm组件是一个独立的数学运算库，提供了机器人姿态控制算法（通过水平、垂直的两个值和两个模式控制数据计算八轴和六轴推进器的具体PWM输出值）、卡尔曼滤波算法、PID算法的ADT实现

Sensor组件会使用到BasicCtrl组件提供的抽象方法，只有启用了BasicCtrl组件才能进一步使用Sensor组件

### 对象列表

```c
//IO对象
//CPU对象
//PWM对象
//总线对象
//网络接口对象
//摄像头对象
//异构计算对象
//传感器对象
```



### API列表

```c
```



### 应用层

应用层包括Application组件和Sensor组件的应用API，这是Nitori的顶层封装，向用户应用程序提供系统API，部分兼容POSIX标准。

Application组件提供了软件包移植功能，并通过一个封装向用户提供所有服务层的API，可以通过更改封装适配不同平台的应用程序；Sensor组件的应用API是对Sensor组件服务部分的重定义，可以适配上层的应用程序

