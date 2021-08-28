/*
 * @Author: Sherlock
 * @Date: 2021-04-11 21:22:20
 * @LastEditTime: 2021-04-22 15:40:36
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Projectd:\TDT\TDT-Frame\TDT_Alg\inc\SteeringWheel.h
 */
#include <board.h>
#ifndef __STEERING_WHEEL_H__
#define __STEERING_WHEEL_H__

/* the radius of wheel(mm) */
#define RADIUS 53
/* the perimeter of wheel(mm) 周长*/
#define PERIMETER 333
/* wheel track distance(mm) 轮距*/
#define WHEELTRACK 314
/* wheelbase distance(mm) 轴距*/
#define WHEELBASE 337
#define WHEELNUMBER 4
/* gimbal is relative to chassis center x axis offset(mm) 云台中心补偿*/
#define ROTATE_X_OFFSET 0
/* gimbal is relative to chassis center y axis offset(mm) */
#define ROTATE_Y_OFFSET 0

/* chassis motor use 3508 */
/* the deceleration ratio of chassis motor */
#define MOTOR_DECELE_RATIO_3508 (1.0f / 14.0f)
/* single 3508 motor maximum speed, unit is rpm */
#define MAX_WHEEL_RPM_3508 9600  
/* chassis maximum translation speed, unit is mm/s */
#define MAX_CHASSIS_VX_SPEED 3810
#define MAX_CHASSIS_VY_SPEED 3810
/* chassis maximum rotation speed, unit is	rad/s */
#define MAX_CHASSIS_VW_SPEED 8.97039f

#define MOTOR_ENCODER_ACCURACY 8192.0f

class SteeringWheel
{
private:
    /* data */
    struct steering_structure
    {
        float wheel_perimeter; /* the perimeter(mm) of wheel */
        float wheeltrack;      /* wheel track distance(mm) */
        float wheelbase;       /* wheelbase distance(mm) */
        float rotate_x_offset; /* rotate offset(mm) relative to the x-axis of the chassis center */
        float rotate_y_offset; /* rotate offset(mm) relative to the y-axis of the chassis center */
        float W_Cdistance;
    } steeringPara;

public:
    SteeringWheel();
    struct steering_speed
    {
        float vx; // +forward	-back
        float vy; // +left	-right
        float vw; // +anticlockwise	-clockwise
    } steeringSpeed;
    struct steering
    {
        float angle;     //°   0---360
        float speed_rpm; //rpm
        float speed;
        float angleFdb;
    };
    struct calculate_para
    {
        float radius;
        float alpha;
        float speedV;
        float sinAlpha;
        float cosAlpha;
    } basePara;
    struct outputData
    {
        vec4f speed_rpm;
        vec4f angle;
    } outputData;
    struct steeringWheels
    {
        struct steering RF; //0
        struct steering LF; //1
        struct steering LB; //2
        struct steering RB; //3
    } wheels;
    /**
		  * @brief 舵轮解算.F:forword; B:backword; L:left; R:right
		  * @param input : ccx=+vx(mm/s)  ccy=+vy(mm/s)  ccw=+vw(deg/s) x方向速度(车头方向为正) y方向速度(车身向右为正) 旋转速度(顺时针旋转为正)
		  *        output: every wheel speed(rpm) 各轮轮速
          *                     every wheel angle(°) 各轮轮向
		  * @note  1=FR 2=FL 3=BL 4=BR  轮组命名为逆时针顺序
		  */
    void steeringWheel_calculate(vec3f speedDef, vec4f wheelFeedback);
};

#endif