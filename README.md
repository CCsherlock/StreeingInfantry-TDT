���ֲ���
===
# 1. ������ܽ���
�ÿ�Դ�ļ��а��������ļ���

- gimbalCtrl ���ڽ���ң�������ݿ�����̨Pitch��Yaw�����������������ָǵ������miniPC���¿��ư�ͨ��
- boardCtrl ���ڽ����Ͽ��ư���Ϣ�Լ�����ϵͳ������Ϣ���Ƶ��̵�����ĸ����������ĸ����������

## **1.1ʹ��˵��**

* Ӳ�����ã�STM32F405RGT6���� * 2
* ����˵�����������gitee��clone�º��ֱ�ӽ��б���
* IDE: Keil
* �������汾��ARM Compiler V6.14
* �����׼�� gnu++11 gnu99
* ����˵����gimbalCtrl�������ص���̨���ư壬boardCtrl�������ص����̿��ư�
* **<font color='red'>�йز���ϵͳ���ݺ�Ħ�����Լ��������ݷ���Ĺ���ֻ�谴�մ��������Ľӿڸ�������</font>**

## **1.2����ܹ�������**

![�ܹ�������](media\�ܹ�������.png)
# 2. ��Ҫ�����ļ�����˵��
## **2.1 gimbalCtrl**

![allFile](media\all.png)
* TDT_Task ��ִ������


 ![task](media\gimbalCtrl.png)
```
start_task : �½�����
dbus_task : ң�������ݴ�������
imu_task : �����Ǵ�������
led_task : ����ָʾ������
ammo_cover_task : ���ָ�����
fire_task : �������񣨰���Ħ���ֺͲ����̣�
gimbal_task : ��̨�����������
state_task : ״̬���´�������
steeringTop_task ���ϰ���̾��ߴ�������
get_distance_task : �Զ�������վ����
```
* TDT_Alg ��Ҫ�㷨

 ![task](media\Alg.png)
 ```
 pid : pid�����㷨
 crc �� CRCУ���㷨 ��8λ��16λ��
 filter : �˲����㷨 ����ͨ�Ϳ������˲���
 imu �����������ݴ����㷨
 my_math : ������ѧ����
 can_calculate : ��Ա�����can�����㷨
 KeyProcess �����������㷨
  ```
  ## **2.2 boardCtrl**
        boardCtrl��gimbalCtrlʹ�õ���ͬһ�׿�ܣ���ͬ����boardֻ��Ҫһ�����Ƶ���8����������񣬻��ڵ��㷨��TDT_Alg �ļ���SteeringWheel �Ķ��ֽ����㷨��

# 3. ��Ҫ�㷨˵��
## ���ֽ����㷨
���ǵĶ��ֽ���Ĵ�ǰ������4��3508���ֻ��һ��ת�򣬼����ֻ��Ӽ������跴�򣬳������巽�����ĸ�6020������ơ�

�����������ϰ���յ�ң������Ϣ������ֽ�ΪVx Vy Vw�����������ֱ��������̨����Ϊ�������ֱ���ٶȣ�ƽ���ٶȺ������ٶȡ�

֮�����������裺
1. �����˶������Ե����ǳ�������������һ�㣨�ٶ�˲�ģ�����ת��ֱ���˶������ٶ�˲���ڴ�ֱ���ٶȵ�����Զ����
2. �������˶�ʱ������������һ�����ٶ�˲�ĵ���ת�ٶ���ͬ�����ٶȷ���ֱ�ڸõ㵽�ٶ�˲�ĵ����ߡ�

 ![task](media\ͼ��.jpg)

��ʽ�Ƶ���

����ǰ��Ϊ����

 ![task](media\��ʽ.png)

 ��󻯼�Լȥ��õ��������ʽ��

![task](media\��ʽ2.png)

����Ϊ��
```
		wheels.RF.speed = my_sqrt((my_pow(steeringPara.W_Cdistance) * my_pow(steeringSpeed.vw)) / 2 + my_pow(basePara.speedV) + (steeringPara.wheelbase * basePara.sinAlpha - steeringPara.wheeltrack * basePara.cosAlpha) * steeringSpeed.vw * basePara.speedV);
		wheels.RF.angle = (PI / 2) - atan2((2 * basePara.speedV * basePara.cosAlpha - steeringPara.wheeltrack * steeringSpeed.vw), (2 * basePara.speedV * basePara.sinAlpha + steeringPara.wheelbase * steeringSpeed.vw));

```
**<font color='red'>����Ҫע�����atan2������ʹ�÷���</font>**

![task](media\arctan2.png)
# 3. δ�����Ż�����
    1.�ö����㷨�ڳ��弱ת��ʱû�����Ƕȵ�Բ�����ɴ��������ڣ���ת��ʱ�������ĸ߳������㡣Ŀǰ�е��Ż�˼·�Ǹ��ݵ�ǰ�ٶȺ���Ҫת��ĽǶ����ǶȵĹ������ߣ�����һЩ�ԽǶȵ���Ӧ�ٶ����ò������и��õĿ������顣
    2.�����ĸ����ӵ�������ȷ��û�д��������еװ�ý���У׼�������ڳ�����תʱ���ܻ��нϴ��ںġ�
    3.�ڶ����������˶��л�ƽ���˶�ʱ��������Ľϴ󣬻�ʹ������ƽ���������ڶ���ת��֮������Ż���������������������ʡ�
    4.������һ�����������������쳣����������Ӧ�ó����������¿أ�������Ч��С��Ʈ��


