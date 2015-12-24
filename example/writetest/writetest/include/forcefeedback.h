#ifndef  __FORCEFEEDBACK_H
#define  __FORCEFEEDBACK_H

// 通过宏来控制是导入还是导出
#ifdef __FORCEFEEDBACK
#define FORCEFEEDBACK_API __declspec(dllexport)
#else
#define FORCEFEEDBACK_API __declspec(dllimport)
#endif

#include "include/Serial.h"

#define BUFFERSIZE 150
#define REDUCTION_66	66.00 //1\2
#define REDUCTION_51	51.00 //3
#define REDUCTION_19	19.20 //4\5\6
#define REDUCTION_24	24.00 //7\8\9
#define CODE_500		500.00 //1\2\3
#define CODE_256		500.00 //7\8\9
#define CODE_128		128.00 //4\5\6
#define PI	 3.1415926
#define L1	 330
#define L2	 257

#define  measure_arm_force_x  680.0
#define  measure_arm_force_y  680.0
#define  measure_arm_force_z  150.0

#define  arm_finger1  0.08 //手指1作用力臂
#define  arm_finger2  0.08 //手指2作用力臂
#define  arm_finger3  0.08 //手指3作用力臂

#define  mask_finger1  1
#define  mask_finger2  2
#define  mask_finger3  4

#define  measure_arm_finger  0.04 //手指测量力臂


//p_x，p_y，p_z单位：mm
struct PosInfo{
	float p_x;
	float p_y;
	float p_z;
	float r_x;
	float r_y;
	float r_z;
};

struct ForcePara{
	float f_x;
	float f_y;
	float f_z;
};

struct TorquePara{
	float t_x;
	float t_y;
	float t_z;
};

//原始的关节电机角度值
struct RawData{
	float m_angle[9];
};

//负数代表负方向
struct FingerForce{
	float finger_force[3];
};

struct FingerTorque{
	float finger_torque[3];
};

class FORCEFEEDBACK_API ForceFeedBack{
public:
	ForceFeedBack();
	~ForceFeedBack();
	void DataFresh();
	bool is_update;

private:
	//配置参数，默认波特率115200
	CSerial::EBaudrate m_EBaud;
	CSerial m_serial;
	DWORD mRead;
	DWORD mWrite;
	char* m_comID;
	HANDLE UpdateThread;
	HANDLE Mutex;
	void ClosePort();

protected:
	unsigned char Motor_Data[22];
	int Encoder_Data[9];
	float Angle[9];
	unsigned char USART_Buffer[BUFFERSIZE];
	bool firstflag;
	bool is_thread_running;

	/*****************************************************************************
	******************************对外接口函数**********************************
	*****************************************************************************/

	/*----------------------------打开ForceFeedbacker设备 ------------------------------
	input: 设备所对应的COM口ID
	output: 未成功打开返回FALSE，否则返回TRUE
	-----------------------------------------------------------------------------------------*/
public:
	bool OpenForcebacker(char* comID);

	/*----------------------------关闭ForceFeedbacker设备 ------------------------------
	input: 无
	output: 未成功关闭返回FALSE，否则返回TRUE
	-----------------------------------------------------------------------------------------*/
	void CloseForcebacker();

	/*-------------------------------设置设备通信波特率 ---------------------------------
	input: 通信波特率EBaud
	output: 设置失败返回FALSE，否则返回TRUE
	-----------------------------------------------------------------------------------------*/
	bool SetEbaud(int EBaud);

	/*-----------------------------------重置设备原点 -------------------------------------
	input: 无
	output: 重置失败返回FALSE，否则返回TRUE
	-----------------------------------------------------------------------------------------*/
	bool SetCurOrig();

	/*-----------------------开更新线程，持续更新设备数据-----------------------------
	input: 无
	output: 无
	-----------------------------------------------------------------------------------------*/
	void StartCycUpdate();

	/*-------------------------每次调用更新一次设备数据--------------------------------
	input: 无
	output: 无
	-----------------------------------------------------------------------------------------*/
	void SingleUpdate();

	/*----------------------------------关闭更新线程---------------------------------------
	input: 无
	output: 无
	-----------------------------------------------------------------------------------------*/
	void StopCycUpdate();

	/*--------------------------读取设备结算后的位置信息-------------------------------
	input: 无
	output: 位置信息结构体PosInfo
	-----------------------------------------------------------------------------------------*/
	PosInfo Read();

	/*----------------------读取设备每个关节角度的原始信息---------------------------
	input: 无
	output: 位置信息结构体RawData
	-----------------------------------------------------------------------------------------*/
	RawData ReadRaw();

	/*---------------------------------设置wrist的力----------------------------------------
	input: 力信息结构体ForcePara
	output: 设置失败返回FALSE，否则返回TRUE
	-----------------------------------------------------------------------------------------*/
	bool SetForceWrist(const ForcePara& para);

	/*-------------------------------设置wrist的力矩----------------------------------------
	input: 力矩信息结构体TorquePara
	output: 设置失败返回FALSE，否则返回TRUE
	-----------------------------------------------------------------------------------------*/
	bool SetTorqueWrist(const TorquePara& para);

	/*--------------------------------设置finger的力----------------------------------------
	input: 1. 手指力信息结构体FingerForce
	           2. 罩子，初始三个指关节都不屏蔽(example：如果需要屏蔽finger3，则mask设置为 mask_finger1 | mask_finger2 )
	output: 设置失败返回FALSE，否则返回TRUE
	-----------------------------------------------------------------------------------------*/
	bool SetForceFinger(const FingerForce& finger_force, const int mask=mask_finger1 | mask_finger2 | mask_finger3);

	/*-------------------------------设置finger的力矩-------------------------------------
	input: 1. 手指力信息结构体FingerTorque
	           2. 罩子，初始三个指关节都不屏蔽(example：如果需要屏蔽finger3，则mask设置为 mask_finger1 | mask_finger2 )
	output: 设置失败返回FALSE，否则返回TRUE
	-----------------------------------------------------------------------------------------*/
	bool SetTorqueFinger(const FingerTorque& finger_torque, const int mask=mask_finger1 | mask_finger2 | mask_finger3);

	/*----------------------------------关闭更新线程---------------------------------------
	input: 清除设备上所存在的所有力和力矩
	output: 设置失败返回FALSE，否则返回TRUE
	-----------------------------------------------------------------------------------------*/
    bool ClearForce();
};

#undef FORCEFEEDBACK_API

#endif