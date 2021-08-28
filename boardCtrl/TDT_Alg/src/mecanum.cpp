#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <my_math.h>
#include "mecanum.h"
#ifndef RADIAN_COEF
#define RADIAN_COEF 57.3f
#endif

#define NORM1 2000
#define NORM2 4000
float normKptest1,normKptest2,normKptest3,normKptest4,normtest;
Mecanum::Mecanum()
{
    mec.param.wheelbase = WHEELBASE;
    mec.param.wheeltrack = WHEELTRACK;
    mec.param.rotate_x_offset = ROTATE_X_OFFSET;
    mec.param.rotate_y_offset = ROTATE_Y_OFFSET;
    mec.param.wheel_perimeter = PERIMETER;
}

void Mecanum::mecanum_calculate(vec3f speedDef, struct mecanum_motor_fdb wheel_fdb[])
{
    float rotate_ratio_fr;
    float rotate_ratio_fl;
    float rotate_ratio_bl;
    float rotate_ratio_br;
    float wheel_rpm_ratio;

    mec.speed.vx = speedDef.data[0];
    mec.speed.vy = speedDef.data[1];
    mec.speed.vw = speedDef.data[2];

    rotate_ratio_fr = ((mec.param.wheelbase + mec.param.wheeltrack) / 2.0f - mec.param.rotate_x_offset + mec.param.rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_fl = ((mec.param.wheelbase + mec.param.wheeltrack) / 2.0f - mec.param.rotate_x_offset - mec.param.rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_bl = ((mec.param.wheelbase + mec.param.wheeltrack) / 2.0f + mec.param.rotate_x_offset - mec.param.rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_br = ((mec.param.wheelbase + mec.param.wheeltrack) / 2.0f + mec.param.rotate_x_offset + mec.param.rotate_y_offset) / RADIAN_COEF;
    wheel_rpm_ratio = 60.0f / (mec.param.wheel_perimeter * MOTOR_DECELE_RATIO_3508);

    LIMIT(mec.speed.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED); //mm/s
    LIMIT(mec.speed.vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED); //mm/s
    LIMIT(mec.speed.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED); //deg/s

    float wheel_rpm[4];
    float max = 0;

    wheel_rpm[0] = (-mec.speed.vx - mec.speed.vy - mec.speed.vw * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[1] = (mec.speed.vx - mec.speed.vy - mec.speed.vw * rotate_ratio_fl) * wheel_rpm_ratio;
    wheel_rpm[2] = (mec.speed.vx + mec.speed.vy - mec.speed.vw * rotate_ratio_bl) * wheel_rpm_ratio;
    wheel_rpm[3] = (-mec.speed.vx + mec.speed.vy - mec.speed.vw * rotate_ratio_br) * wheel_rpm_ratio;
    for (u8 i = 0; i < 4; i++)
    {
        wheel_fdb[i].speed_rpm = lpf2p_fdpk.Apply(wheel_fdb[i].speed_rpm);
    }
    //差速Kp归一
    wheelSetError[0] = wheel_rpm[0] - wheel_fdb[0].speed_rpm;
    wheelSetError[1] = wheel_rpm[1] - wheel_fdb[1].speed_rpm;
    wheelSetError[2] = wheel_rpm[2] - wheel_fdb[2].speed_rpm;
    wheelSetError[3] = wheel_rpm[3] - wheel_fdb[3].speed_rpm;

    for (u8 i = 0; i < 4; i++)
    {
        wheel_rpm[i] *= wheelSpeedKp[i] * GETDIRECT(wheelSpeedKp[i]) * K_CU + 1 * (1 - K_CU);
    }

    //找到最大轮速
    for (uint8_t i = 0; i < 4; i++)
    {
        if (ABS(wheel_rpm[i]) > max)
        {
            max = ABS(wheel_rpm[i]);
        }
    }

    //最大旋转速度超过设定最大旋转速度，平均降速
    if (max > MAX_WHEEL_RPM)
    {
        float rate = MAX_WHEEL_RPM / max;
        for (uint8_t i = 0; i < 4; i++)
        {
            wheel_rpm[i] *= rate;
        }
    }
    memcpy(mec.wheel_rpm, wheel_rpm, 4 * sizeof(float));
}

void Mecanum::mecanum_position_measure(struct mecanum_motor_fdb wheel_fdb[])
{
    float rotate_ratio_fr;
    float rotate_ratio_fl;
    float rotate_ratio_bl;
    float rotate_ratio_br;
    float rpm_ratio;
    float ecd_ratio;
    double mecanum_angle;
    static double last_d_x, last_d_y, last_d_w;
    double d_x, d_y, d_w, diff_d_x, diff_d_y, diff_d_w;
    double position_x, position_y, angle_w;
    double v_x, v_y, v_w;

    rotate_ratio_fr = ((mec.param.wheelbase + mec.param.wheeltrack) / 2.0f -
                       mec.param.rotate_x_offset + mec.param.rotate_y_offset);
    rotate_ratio_fl = ((mec.param.wheelbase + mec.param.wheeltrack) / 2.0f -
                       mec.param.rotate_x_offset - mec.param.rotate_y_offset);
    rotate_ratio_bl = ((mec.param.wheelbase + mec.param.wheeltrack) / 2.0f +
                       mec.param.rotate_x_offset - mec.param.rotate_y_offset);
    rotate_ratio_br = ((mec.param.wheelbase + mec.param.wheeltrack) / 2.0f +
                       mec.param.rotate_x_offset + mec.param.rotate_y_offset);
    rpm_ratio = mec.param.wheel_perimeter * MOTOR_DECELE_RATIO_2006 / (4 * 60.0f);
    ecd_ratio = mec.param.wheel_perimeter * MOTOR_DECELE_RATIO_2006 / (4 * MOTOR_ENCODER_ACCURACY);

    last_d_x = d_x;
    last_d_y = d_y;
    last_d_w = d_w;
    d_x = ecd_ratio * (-wheel_fdb[0].total_ecd + wheel_fdb[1].total_ecd + wheel_fdb[2].total_ecd - wheel_fdb[3].total_ecd);
    d_y = ecd_ratio * (-wheel_fdb[0].total_ecd - wheel_fdb[1].total_ecd + wheel_fdb[2].total_ecd + wheel_fdb[3].total_ecd);
    d_w = ecd_ratio * (-wheel_fdb[0].total_ecd / rotate_ratio_fr - wheel_fdb[1].total_ecd / rotate_ratio_fl - wheel_fdb[2].total_ecd / rotate_ratio_bl - wheel_fdb[3].total_ecd / rotate_ratio_br);

    diff_d_x = d_x - last_d_x;
    diff_d_y = d_y - last_d_y;
    diff_d_w = d_w - last_d_w;

    /* use glb_chassis gyro angle data */
    mecanum_angle = mec.gyro.yaw_gyro_angle / RADIAN_COEF;

    position_x += diff_d_x * cos(mecanum_angle) - diff_d_y * sin(mecanum_angle);
    position_y += diff_d_x * sin(mecanum_angle) + diff_d_y * cos(mecanum_angle);

    angle_w += diff_d_w;

    mec.position.position_x_mm = position_x;        //mm
    mec.position.position_y_mm = position_y;        //mm
    mec.position.angle_deg = angle_w * RADIAN_COEF; //degree

    v_x = rpm_ratio * (-wheel_fdb[0].speed_rpm + wheel_fdb[1].speed_rpm + wheel_fdb[2].speed_rpm - wheel_fdb[3].speed_rpm);
    v_y = rpm_ratio * (-wheel_fdb[0].speed_rpm - wheel_fdb[1].speed_rpm + wheel_fdb[2].speed_rpm + wheel_fdb[3].speed_rpm);
    v_w = rpm_ratio * (-wheel_fdb[0].speed_rpm / rotate_ratio_fr - wheel_fdb[1].speed_rpm / rotate_ratio_fl - wheel_fdb[2].speed_rpm / rotate_ratio_bl - wheel_fdb[3].speed_rpm / rotate_ratio_br);

    mec.position.v_x_mm = v_x;                 //mm/s
    mec.position.v_y_mm = v_y;                 //mm/s
    mec.position.rate_deg = v_w * RADIAN_COEF; //degree/s
}
