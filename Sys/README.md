# ROV_OS-Nitori系统服务组件

该目录下文件共同为Nitori上层应用提供信号量/互斥量、消息队列和统一的外设驱动模型，其中外设驱动模型可选

信号量/互斥量用于简单取代裸机编程中的全局变量，Nitori采用查找表配合指针而不是抽象类的形式实现这个功能来保证尽可能小的性能损失；消息队列用来传输较大量的数据（比如解析后的控制数据），从Thread组件的内核对象模型类继承而来；外设驱动模型仿照RT-Thread编写，可以实现对底层硬件外设的“文件式”管理，应用程序可以在外设驱动模型基础上编写需要的驱动程序

该组件可以与Thread组件共同协作使用，也可以配合其他RTOS使用，与整体代码解耦，具有较高的灵活性