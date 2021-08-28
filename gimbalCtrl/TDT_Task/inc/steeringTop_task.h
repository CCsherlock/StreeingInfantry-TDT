#ifndef __STEERING_TOP_TASK_H
#define __STEERING_TOP_TASK_H
#include "board.h"
#include "SteeringWheel.h"
#include "pid.h"
struct steeringTop
{
private:
    /* data */

    float gimbalAngle;            //云台偏角

    struct swing_Param //摇摆计算参数
    {
        /* data */
        int timeRecord;
        float sinOmiga = 0.031415*1.3;//y = Asin(wt)  w = 2*3.14/T
        float sinMax = 40.0f/180.0f*MY_PPPIII;
        float sinResult;
    } swingParam;

    struct rotate_Param //小陀螺计算参数
    {
        /* data */
        int timeRecord;
        float sinOmiga = 0.031415;
        float sinMax =0.3;
        float sinResult;
    } rotateParam;

    void flySlope(); //飞坡
    float getGimbalAngle(); //获取云台偏角（-pi --- pi）

    int16_t getMotorSpeed();
    float remoteSensitivity = 1;   //遥控器灵敏度
    float keyboardSensitivity = 1; //wasd灵敏度

public:
    /**
 * @brief Construct a new Chassis object
 *
 * @param ChassisMotor   M2006 M3508
 * @param Motor_Type  MOTOR2006 MOTOR 3508
 */
    steeringTop();
	void steeringTopInit();
    vec3f remoteCtrl();
    vec3f keyboardCtrl();
    vec3f followCtrl();
    vec3f rotateCtrl();
    vec3f swingCtrl();
    vec3f customCtrl();
	vec3f rotateOffsetCompute(vec3f AllOutSpeed);
struct RotateOffsetParam //偏心补偿系数
{
	float omigaK = 0;//速度补偿系数
	float offsetSita = 0;//相对于车辆正方向的顺时针偏角
}rotateOffsetParam;
    /**
 * @brief 功率计算
 *
 */
	void powerCtrlOld();
    /**
 * @brief 速度合成
 *
 */
    void speedCompute();
    /**
 * @brief 云台坐标转换
 *
 * @param speedIn
 * @return vec3f
 */
    vec3f vecterTransform(vec3f speedIn);
    vec3f remoteSpeed, keyboardSpeed, followSpeed, rotateSpeed, swingSpeed, customSpeedIn, customSpeedOut, allSpeed,rotateOffsetsSpeed,jscopSpeed; //前正后负左正右负顺负逆正
	vec4f outPutRpm;
    /**
 * @brief 速度输出
 *
 */
    void speedSend();
    /**
 * @brief 更改底盘跟随正方向
 *
 */
    void changeFollowZero();
    /**
 * @brief 功率模块通信
 */
    void powerInfoSend();
	void powerOfflineCheck();
    void powerOverFlowCal();
    Pid *chassisFollow;
    PidParam *followPid;
    bool lockChassis = 0;     //是否锁底盘
    bool customFlag = 0;      //自定义flag
    bool rotateFlag = 0;      //小陀螺
    bool flexRotate = 0;      //是否变速陀螺
    bool swingFlag = 0;       //摇摆
    u8 ifChassisFollow = 1; //是否底盘跟随，默认不跟随
    bool ifTransform = 1;     //是否坐标转换，默认转换
	bool chassisForceOpenLoop = 0; //底盘强制开环
    void autoGotoStation();
    struct AutoIntoStation
    {
#define STRAIGHT_FIRST 1
#define TURN 2
#define STRAIGHT_SECOND 3
        /* data */
        bool startFlag = 0;
        u8 progress = 1;
		u8 progress_last = 0;
        int runCount = 0;
        float speedStraight = 800.0f;
		float speedTurnLinner;
		float turnRadius = 500.0f;
        float speedTurn = 90.0f/57.295779;
        int progressTime[4] = {0,200,300,200};
    }autoIntoStation;

    struct power_Param        //功率计算参数
    {
        /* data */
        bool usingJgmt;
        bool usingBackup = 0;
        bool powerOffline = 0;
        float powerLimitKp=1;
        float remainPower_P;
        float superPower_V;
        float powerLimit;
        float overFlowKp=1;
        bool PowerPath_Switch = 1;
        float jgmtOfflineKp = 0.5;
		float RemainPowerBuffer;
		bool checkMode = 0;
		bool ULTSMode = 0;
		bool overLimitCase = 1;
		float powerMaxForKp;
		float limitPowerCurrunt;
		float limitPowerNext;
		float noUsePowerKp = 0.5f;
		float noUsePowerKpNomal = 0.8f;
    }powerParam;
    struct jgmt_Param //裁判系统需要的数据
    {
        bool jgmtOffline;
        float chassisBuffer;
        float bufferLimit;
    }jgmtParam;
    void getJgmtMsg();
    void getSuperPowerMsg();
	void getMaxPower();
	struct AcceleratePama
	{
		float linnerSpeed;
		float linnerSpeedLast;
		u8 accelerating;
		u8 decelerating;
		float accCnt;
		float accK = 0.25;
		float accKp;
		float deceleRecodeSpeed[2];
	}accPama;
	void usingAcclerateCurve();
	bool judgeIfMoving();
	int recodetime;
	bool sendIfmove = 0;
    float chassisFollowAngle = 0; //底盘跟随角
	struct CanSendStruct
	{
		vec4f datafloat;
		float powerLimitKp;
		u8 ifdeforce;
	}canSendStruct;
	struct CanRecvStruct
	{
		u8 readyFlag;
		u8 ifMoving;
	}canRecvStruct;
	u8 lostMotorNum[4][2]; //0: 3508 1: 6020
	void travelCalibration();
	u8 judgeShiftMode();
	struct JudgeShift
	{
		u8 judgeShiftFirst;
		u8 judgeShiftNest;
		int timeRecode;
		u8 doubleShift = 0;
	}judgeShift;
	struct CanSendStructToJug
	{
		u8 ifChassisFollow;
		u8 doubleShift;
		int16_t chassisAngle;
	}canToJug;
	int canSendCnt;
};
extern steeringTop SteeringTop;
#endif