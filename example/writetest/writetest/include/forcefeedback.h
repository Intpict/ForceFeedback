#ifndef  __FORCEFEEDBACK_H
#define  __FORCEFEEDBACK_H

// ͨ�����������ǵ��뻹�ǵ���
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

#define  arm_finger1  0.08 //��ָ1��������
#define  arm_finger2  0.08 //��ָ2��������
#define  arm_finger3  0.08 //��ָ3��������

#define  mask_finger1  1
#define  mask_finger2  2
#define  mask_finger3  4

#define  measure_arm_finger  0.04 //��ָ��������


//p_x��p_y��p_z��λ��mm
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

//ԭʼ�Ĺؽڵ���Ƕ�ֵ
struct RawData{
	float m_angle[9];
};

//������������
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
	//���ò�����Ĭ�ϲ�����115200
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
	******************************����ӿں���**********************************
	*****************************************************************************/

	/*----------------------------��ForceFeedbacker�豸 ------------------------------
	input: �豸����Ӧ��COM��ID
	output: δ�ɹ��򿪷���FALSE�����򷵻�TRUE
	-----------------------------------------------------------------------------------------*/
public:
	bool OpenForcebacker(char* comID);

	/*----------------------------�ر�ForceFeedbacker�豸 ------------------------------
	input: ��
	output: δ�ɹ��رշ���FALSE�����򷵻�TRUE
	-----------------------------------------------------------------------------------------*/
	void CloseForcebacker();

	/*-------------------------------�����豸ͨ�Ų����� ---------------------------------
	input: ͨ�Ų�����EBaud
	output: ����ʧ�ܷ���FALSE�����򷵻�TRUE
	-----------------------------------------------------------------------------------------*/
	bool SetEbaud(int EBaud);

	/*-----------------------------------�����豸ԭ�� -------------------------------------
	input: ��
	output: ����ʧ�ܷ���FALSE�����򷵻�TRUE
	-----------------------------------------------------------------------------------------*/
	bool SetCurOrig();

	/*-----------------------�������̣߳����������豸����-----------------------------
	input: ��
	output: ��
	-----------------------------------------------------------------------------------------*/
	void StartCycUpdate();

	/*-------------------------ÿ�ε��ø���һ���豸����--------------------------------
	input: ��
	output: ��
	-----------------------------------------------------------------------------------------*/
	void SingleUpdate();

	/*----------------------------------�رո����߳�---------------------------------------
	input: ��
	output: ��
	-----------------------------------------------------------------------------------------*/
	void StopCycUpdate();

	/*--------------------------��ȡ�豸������λ����Ϣ-------------------------------
	input: ��
	output: λ����Ϣ�ṹ��PosInfo
	-----------------------------------------------------------------------------------------*/
	PosInfo Read();

	/*----------------------��ȡ�豸ÿ���ؽڽǶȵ�ԭʼ��Ϣ---------------------------
	input: ��
	output: λ����Ϣ�ṹ��RawData
	-----------------------------------------------------------------------------------------*/
	RawData ReadRaw();

	/*---------------------------------����wrist����----------------------------------------
	input: ����Ϣ�ṹ��ForcePara
	output: ����ʧ�ܷ���FALSE�����򷵻�TRUE
	-----------------------------------------------------------------------------------------*/
	bool SetForceWrist(const ForcePara& para);

	/*-------------------------------����wrist������----------------------------------------
	input: ������Ϣ�ṹ��TorquePara
	output: ����ʧ�ܷ���FALSE�����򷵻�TRUE
	-----------------------------------------------------------------------------------------*/
	bool SetTorqueWrist(const TorquePara& para);

	/*--------------------------------����finger����----------------------------------------
	input: 1. ��ָ����Ϣ�ṹ��FingerForce
	           2. ���ӣ���ʼ����ָ�ؽڶ�������(example�������Ҫ����finger3����mask����Ϊ mask_finger1 | mask_finger2 )
	output: ����ʧ�ܷ���FALSE�����򷵻�TRUE
	-----------------------------------------------------------------------------------------*/
	bool SetForceFinger(const FingerForce& finger_force, const int mask=mask_finger1 | mask_finger2 | mask_finger3);

	/*-------------------------------����finger������-------------------------------------
	input: 1. ��ָ����Ϣ�ṹ��FingerTorque
	           2. ���ӣ���ʼ����ָ�ؽڶ�������(example�������Ҫ����finger3����mask����Ϊ mask_finger1 | mask_finger2 )
	output: ����ʧ�ܷ���FALSE�����򷵻�TRUE
	-----------------------------------------------------------------------------------------*/
	bool SetTorqueFinger(const FingerTorque& finger_torque, const int mask=mask_finger1 | mask_finger2 | mask_finger3);

	/*----------------------------------�رո����߳�---------------------------------------
	input: ����豸�������ڵ�������������
	output: ����ʧ�ܷ���FALSE�����򷵻�TRUE
	-----------------------------------------------------------------------------------------*/
    bool ClearForce();
};

#undef FORCEFEEDBACK_API

#endif