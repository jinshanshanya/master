# RSLidar ROS驱动修改记录

### V1.0.0(2017-09-29)
---
初版提交

### V2.0.0(2017-11-23)
---
* 1.加入距离随温度的补偿功能。
* 2.移除OpenCV相关的代码

### V3.0.0(2018-05-10)
---
* 1.加入RS-LiDAR-32的功能支持。
* 2.将温度补偿更新到30~71℃。
* 3.使用新的代码架构。
* 4.改变反射率的计算方法。
* 5.兼容不同的标定文件格式。
* 6.加入多雷达时间同步功能。

### V3.1.0(2018-07-14)
---
* 1.在反射率计算中加入温度补偿功能。
* 2.移除package.xml中多余的依赖项。

### V4.0.0(2019-04-11)
---
* 1.新增距离参数**max_distance**和**min_distance**到launch文件设置功能。
* 2.加入difop包解析功能。
* 3.兼容适配0.5cm分辨率雷达。
* 4.修复1cm分辨率雷达解析错误问题。
* 5.增加反射率计算mode=3，直接输出反射强度值，不需要计算标定公式计算。
* 6.新增支持解析雷达**双回波模式**数据。
* 7.将多雷达同步功能的获取时间戳代码从input.cc移到rsdriver.cpp。
* 8.增加**resolution**和**intensity**通过launch文件设置，可以适配bag数据中没有记录difop包的情况。
* 9.增加因为坐标变换引起的XYZ补偿代码。
* 10.规范化代码。
* 11.增加根据水平角度分帧的功能。

### V4.1.0(2019-09-11)
---
* 1.修复不同回波模式下packet_rate参数计算错误问题。
* 2.新增从RS32线的difop包中读取角度标定值。
* 3.更改打印信息输出内容，便于识别打印信息来源。
* 4.更改三角函数计算到查表方式，优化代码执行效率。
* 5.更改不同雷达镜头到转体中心的坐标换。
* 6.修改默认分帧方式到角度分帧，且修改包数分帧的报数对应18K点频设备。
* 7.新增RSBPEARL配置。

### V4.1.1(2019-11-29)
---
* 1.修复BPearl的角度解析错误。
* 2.修复当LiDAR被设置了FOV后出现的角度差值计算错误。