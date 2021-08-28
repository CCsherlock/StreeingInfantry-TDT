舵轮步兵
===
# 1. 软件功能介绍
该开源文件中包含两个文件夹

- gimbalCtrl 用于接收遥控器数据控制云台Pitch、Yaw电机、拨弹电机、弹仓盖电机、与miniPC和下控制板通信
- boardCtrl 用于接收上控制板信息以及裁判系统串口信息控制底盘电机（四个舵向电机和四个动力电机）

## **1.1使用说明**

* 硬件配置：STM32F405RGT6主控 * 2
* 编译说明：将代码从gitee上clone下后可直接进行编译
* IDE: Keil
* 编译器版本：ARM Compiler V6.14
* 编译标准： gnu++11 gnu99
* 下载说明：gimbalCtrl工程下载到云台控制板，boardCtrl工程下载到底盘控制板
* **<font color='red'>有关裁判系统数据和摩擦轮以及超级电容方面的功能只需按照代码中留的接口给出即可</font>**

## **1.2电控总功能拓扑**

![总共能拓扑](media\总共能拓扑.png)
# 2. 主要代码文件功能说明
## **2.1 gimbalCtrl**

![allFile](media\all.png)
* TDT_Task 各执行任务


 ![task](media\gimbalCtrl.png)
```
start_task : 新建任务
dbus_task : 遥控器数据处理任务
imu_task : 陀螺仪处理任务
led_task : 板载指示灯任务
ammo_cover_task : 弹仓盖任务
fire_task : 开火任务（包括摩擦轮和拨弹盘）
gimbal_task : 云台电机控制任务
state_task : 状态更新处理任务
steeringTop_task ：上板底盘决策处理任务
get_distance_task : 自动进补给站任务
```
* TDT_Alg 主要算法

 ![task](media\Alg.png)
 ```
 pid : pid计算算法
 crc ： CRC校验算法 （8位和16位）
 filter : 滤波器算法 （低通和卡尔曼滤波）
 imu ：陀螺仪数据处理算法
 my_math : 快速数学计算
 can_calculate : 针对比赛的can发送算法
 KeyProcess ：按键处理算法
  ```
  ## **2.2 boardCtrl**
        boardCtrl与gimbalCtrl使用的是同一套框架，不同的是board只需要一个控制底盘8个电机的任务，基于的算法是TDT_Alg 文件的SteeringWheel 的舵轮解算算法。

# 3. 主要算法说明
## 舵轮结算算法
我们的舵轮结算的大前提是在4个3508电机只有一个转向，即电机只需加减速无需反向，车体整体方向由四个6020电机控制。

首先我们在上板接收到遥控器信息，将其分解为Vx Vy Vw三个向量，分别代表以云台方向为正方向的直行速度，平移速度和自旋速度。

之后是两个假设：
1. 所有运动都可以当做是车辆质心绕任意一点（速度瞬心）做旋转，直线运动则是速度瞬心在垂直于速度的无穷远处。
2. 车辆在运动时，车身上任意一点绕速度瞬心的旋转速度相同，且速度方向垂直于该点到速度瞬心的连线。

 ![task](media\图解.jpg)

公式推导：

以右前轮为例：

 ![task](media\公式.png)

 最后化简约去后得到两条输出式：

![task](media\公式2.png)

代码为：
```
		wheels.RF.speed = my_sqrt((my_pow(steeringPara.W_Cdistance) * my_pow(steeringSpeed.vw)) / 2 + my_pow(basePara.speedV) + (steeringPara.wheelbase * basePara.sinAlpha - steeringPara.wheeltrack * basePara.cosAlpha) * steeringSpeed.vw * basePara.speedV);
		wheels.RF.angle = (PI / 2) - atan2((2 * basePara.speedV * basePara.cosAlpha - steeringPara.wheeltrack * steeringSpeed.vw), (2 * basePara.speedV * basePara.sinAlpha + steeringPara.wheelbase * steeringSpeed.vw));

```
**<font color='red'>其中要注意的是atan2函数的使用方法</font>**

![task](media\arctan2.png)
# 3. 未来的优化方向
    1.该舵轮算法在车体急转向时没有做角度的圆滑过渡处理以至于，急转向时由于重心高车体会侧倾。目前有的优化思路是根据当前速度和需要转向的角度做角度的过渡曲线，牺牲一些对角度的相应速度来让操作手有更好的开车体验。
    2.对于四个轮子的正方向确定没有传感器或机械装置进行校准以至于在车体旋转时可能会有较大内耗。
    3.在舵轮由自旋运动切换平移运动时能量的损耗较大，会使动作不平滑，可以在动能转化之间加上优化函数，提高能量的利用率。
    4.陀螺仪一方面是陀螺仪数据异常保护，另外应该尝试陀螺仪温控，可以有效减小零飘。


