# Qua_control

A flight Control platform

* Main program in ./Core/Src/

## program struct

    main
    +- head.h
        +- control         -------------   控制器
        +- estimate        -------------   估计器
        +- math            -------------   数学库
        +- sensor_socket   -------------   传感器接口
        +- toolkit         -------------   底层工具
        +- TR              -------------   通信
    +- stm32 sys header