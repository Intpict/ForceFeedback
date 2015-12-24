/************************************************
编译前：右击"属性" -> 更改"字符集"为"使用多字节字符集"
************************************************/

#define __FORCEFEEDBACK

#ifndef __FORCEFEEDBACK_H
#include "forcefeedback.h"
#endif

#include <string.h>
#include <math.h>

//APIENTRY声明DLL函数入口点
BOOL APIENTRY DllMain(HANDLE hModule, DWORD ul_reason_for_call, LPVOID lpReserved)
{
	switch (ul_reason_for_call){
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	 }
	 return TRUE;
}

DWORD WINAPI CycUpdate(LPVOID lpParameter){
	ForceFeedBack* ptr = (ForceFeedBack*) lpParameter;
	while(ptr->is_update)
		ptr->DataFresh();
	return 0;
}

ForceFeedBack::ForceFeedBack(){
	m_comID = NULL;
	m_EBaud = CSerial::EBaud115200;
	firstflag = true;
	is_update = false;
	is_thread_running = false;
	UpdateThread = NULL;
	Mutex = CreateMutex(NULL,  false, "ForceFeedback Lock");
	memset(USART_Buffer,0,BUFFERSIZE);
	Motor_Data[0] = 0xAA;
	Motor_Data[1] = 0xBB;
	Motor_Data[2] = 0xAA;
	Motor_Data[3] = 0xBB;
	for(int i=4;i<22;i++)
		Motor_Data[i] = 0x00;
	for(int i=0;i<9;i++){
		Encoder_Data[i] = 32767;
		Angle[i] = 0.0;
	}
}

ForceFeedBack::~ForceFeedBack(){
	CloseForcebacker();
	if(NULL != m_comID){
		delete[] m_comID;
		m_comID = NULL;
	}
}

bool ForceFeedBack::OpenForcebacker(char* comID){
	if(NULL == comID || m_EBaud == CSerial::EBaudUnknown)
		return false;

	int len = strlen(comID);
	m_comID = new char[len+1];
	for (int i=0;i<=len;i++)
		m_comID[i] = comID[i];

	if (m_serial.CheckPort(m_comID) != CSerial::EPortAvailable)
		return false;
	if(m_serial.Open(m_comID) != ERROR_SUCCESS)
		return false;
	m_serial.Setup(m_EBaud, CSerial::EData8, CSerial::EParNone, CSerial::EStop1);
	m_serial.SetupReadTimeouts(CSerial::EReadTimeoutNonblocking);
	Sleep(20);
	if(ERROR_SUCCESS != m_serial.Read(USART_Buffer, BUFFERSIZE, &mRead)){
		m_serial.Clear(m_serial.hFile_m);
		m_serial.Close();
		return false;
	}
	for (int i=0;i<(long)mRead-22;i++)
	{
		if(USART_Buffer[i]==0xAA && USART_Buffer[i+1]==0xBB
			&& USART_Buffer[i+2] == 0xAA && USART_Buffer[i+3]==0xBB)
			return true;
	}
	//check失败，关闭串口，返回false
	m_serial.Clear(m_serial.hFile_m);
	m_serial.Close();
	return false;
}

void ForceFeedBack::ClosePort(){
	if(m_serial.IsOpen()){
		m_serial.Clear(m_serial.hFile_m);
		m_serial.Close();
	}
}

void ForceFeedBack::CloseForcebacker(){
	StopCycUpdate();
	Sleep(50);
	ClearForce();
	Sleep(10);
	ClosePort();
}

bool ForceFeedBack::SetEbaud(int EBaud){
	switch(EBaud){
	case 110:
		m_EBaud = CSerial::EBaud110;
		return true;
	case 300:
		m_EBaud = CSerial::EBaud300;
		return true;
	case 600:
		m_EBaud = CSerial::EBaud600;
		return true;
	case 1200:
		m_EBaud = CSerial::EBaud1200;
		return true;
	case 2400:
		m_EBaud = CSerial::EBaud2400;
		return true;
	case 4800:
		m_EBaud = CSerial::EBaud4800;
		return true;
	case 9600:
		m_EBaud = CSerial::EBaud9600;
		return true;
	case 14400:
		m_EBaud = CSerial::EBaud14400;
		return true;
	case 19200:
		m_EBaud = CSerial::EBaud19200;
		return true;
	case 38400:
		m_EBaud = CSerial::EBaud38400;
		return true;
	case 56000:
		m_EBaud = CSerial::EBaud56000;
		return true;
	case 57600:
		m_EBaud = CSerial::EBaud57600;
		return true;
	case 115200:
		m_EBaud = CSerial::EBaud115200;
		return true;
	case 128000:
		m_EBaud = CSerial::EBaud128000;
		return true;
	case 256000:
		m_EBaud = CSerial::EBaud256000;
		return true;
	}
	m_EBaud = CSerial::EBaudUnknown;
	return false;
}

bool ForceFeedBack::SetCurOrig(){
	DWORD pdwWritten;
	unsigned char resetfram[22] = {0xAA,0xBB,0xAA,0xBB,0x00,0xCC,0x00,0xDD,0x00,0xCC,0x00,0xDD,0x00,0xCC,0x00,0xDD,0x00,0xCC,0x00,0xDD,0x00,0xCC};
	if(ERROR_SUCCESS != m_serial.Write(resetfram,22,&pdwWritten)){
		Sleep(20);
		if(ERROR_SUCCESS != m_serial.Write(resetfram,22,&pdwWritten))
			return false;
	}
	Sleep(20);
	bool thread_running_flag = is_thread_running;
	StopCycUpdate();
	Sleep(50);
	ClosePort();
	if(!OpenForcebacker(m_comID))
		return false;
	if(thread_running_flag)
		StartCycUpdate();
	firstflag = true;
	return true;
}

void ForceFeedBack::DataFresh(){
	//清空旧缓冲区，保证每次读的都是最新数据
	m_serial.Clear(m_serial.hFile_m);
	//等待，保证新数据进读缓冲区
	Sleep(15);

	memset(USART_Buffer,0,BUFFERSIZE);
	m_serial.Read(USART_Buffer,BUFFERSIZE,&mRead);
	float Angle_Temp[9];

	for(int i=0;i<(long)mRead-22;i++){
		if(USART_Buffer[i]==0xAA && USART_Buffer[i+1]==0xBB && USART_Buffer[i+2]==0xAA && USART_Buffer[i+3]==0xBB){
			unsigned char sum = 0x00;
			for (int n=0;n<18;n++)
				sum += USART_Buffer[i+4+n];
			//如果数据无效, 进行下一轮循环
			if(sum != USART_Buffer[i+22]){
				i+=23;
				continue;
			}
			Encoder_Data[0] = USART_Buffer[i+4]*256 + USART_Buffer[i+5];
			Encoder_Data[1] = USART_Buffer[i+6]*256 + USART_Buffer[i+7];
			Encoder_Data[2] = USART_Buffer[i+8]*256 + USART_Buffer[i+9];
			Encoder_Data[3] = USART_Buffer[i+10]*256 + USART_Buffer[i+11];
			Encoder_Data[4] = USART_Buffer[i+12]*256 + USART_Buffer[i+13];
			Encoder_Data[5] = USART_Buffer[i+14]*256 + USART_Buffer[i+15];
			Encoder_Data[6] = USART_Buffer[i+16]*256 + USART_Buffer[i+17];
			Encoder_Data[7] = USART_Buffer[i+18]*256 + USART_Buffer[i+19];
			Encoder_Data[8] = USART_Buffer[i+20]*256 + USART_Buffer[i+21];

			Angle_Temp[0] = (Encoder_Data[0] - 32767)/(REDUCTION_66*CODE_500/2)*180.00;
			Angle_Temp[1] = (32767 - Encoder_Data[1])/(REDUCTION_66*CODE_500/2)*180.00;
			Angle_Temp[2] = (Encoder_Data[2] - 32767)/(REDUCTION_51*CODE_500/2)*180.00;
			Angle_Temp[3] = (Encoder_Data[3] - 32767)/(REDUCTION_19*CODE_128/2)*180.00;
			Angle_Temp[4] = (Encoder_Data[4] - 32767)/(REDUCTION_19*CODE_128/2)*180.00;
			Angle_Temp[5] = (Encoder_Data[5] - 32767)/(REDUCTION_19*CODE_128/2)*180.00;
			Angle_Temp[6] = (Encoder_Data[6] - 32767)/(REDUCTION_24*CODE_256/2)*180.00;
			Angle_Temp[7] = (Encoder_Data[7] - 32767)/(REDUCTION_24*CODE_256/2)*180.00;
			Angle_Temp[8] = (Encoder_Data[8] - 32767)/(REDUCTION_24*CODE_256/2)*180.00;

			WaitForSingleObject(Mutex, INFINITE);
			if(firstflag){
				for(int k=0;k<9;k++)
					Angle[k] = Angle_Temp[k];
				firstflag = false;
			}
			else{
				for (int k=0;k<9;k++){
					for (int k=0;k<9;k++){
						if (((Angle_Temp[k] - Angle[k])<15.000) && ((Angle[k] - Angle_Temp[k]) <15.000) || (Angle_Temp[k] == 0)){
							Angle[k] = Angle_Temp[k];
						}
					}
				}
			}
			ReleaseMutex(Mutex);
			break;
		}
	}
}

void ForceFeedBack::SingleUpdate(){
	if(is_update)  
		return;
	DataFresh();
}

void ForceFeedBack::StartCycUpdate(){
	if(is_update)
		return;
	//当前的线程句柄未被关闭,  则将其关闭
	if(NULL != UpdateThread){
		Sleep(5);
		if(NULL != UpdateThread)
			CloseHandle(UpdateThread);
	}
	is_update = true;
	UpdateThread = CreateThread(NULL, 0, CycUpdate, this, 0, NULL);
	is_thread_running = true;
}

void ForceFeedBack::StopCycUpdate(){
	is_update = false;
	if(NULL != UpdateThread){
		CloseHandle(UpdateThread);
		UpdateThread = NULL;
	}
	is_thread_running = false;
}

PosInfo ForceFeedBack::Read(){
	PosInfo res;
	WaitForSingleObject(Mutex, INFINITE);
	if(firstflag){
		res.p_x = res.p_y = res.p_z = 0;
	}else{
		res.p_x = L1*cos(Angle[1]/180*PI)*cos(PI/2-Angle[0]/180*PI)/sqrt(cos(Angle[1]/180*PI)*cos(Angle[1]/180*PI)+sin(Angle[1]/180*PI)*sin(PI/2-Angle[0]/180*PI)*sin(Angle[1]/180*PI)*sin(PI/2-Angle[0]/180*PI));
		res.p_y = L1*sin(Angle[1]/180*PI)*sin(PI/2-Angle[0]/180*PI)/sqrt(cos(Angle[1]/180*PI)*cos(Angle[1]/180*PI)+sin(Angle[1]/180*PI)*sin(PI/2-Angle[0]/180*PI)*sin(Angle[1]/180*PI)*sin(PI/2-Angle[0]/180*PI));
		res.p_z = L1*cos(Angle[1]/180*PI)*sin(PI/2-Angle[0]/180*PI)/sqrt(cos(Angle[1]/180*PI)*cos(Angle[1]/180*PI)+sin(Angle[1]/180*PI)*sin(PI/2-Angle[0]/180*PI)*sin(Angle[1]/180*PI)*sin(PI/2-Angle[0]/180*PI))+2*L2*cos(85.00/180.00*PI-Angle[2]/180.00*PI)-
		(L1*cos(0/180*PI)*sin(PI/2-0 /180*PI)/sqrt(cos(0/180*PI)*cos(0/180*PI)+sin(0/180*PI)*sin(PI/2-0/180*PI)*sin(0/180*PI)*sin(PI/2-0/180*PI))+2*L2*cos(85.00/180.00*PI-0/180.00*PI));
	}
	res.r_x = Angle[4];
	res.r_y = Angle[5];
	res.r_z = Angle[3];
	ReleaseMutex(Mutex);
	return res;
}

RawData ForceFeedBack::ReadRaw(){
	RawData res;
	WaitForSingleObject(Mutex, INFINITE);
	for(int i=0;i<9;i++)
		res.m_angle[i] = Angle[i];
	ReleaseMutex(Mutex);
	return res;
}

bool ForceFeedBack::SetForceFinger(const FingerForce& force, const int mask){
	if(mask_finger1 & mask){
		if(force.finger_force[0] == 0){
			Motor_Data[16] = Motor_Data[17] = 0x00;
		}else{
			float temp = (force.finger_force[0] * arm_finger1 / measure_arm_finger + 0.2135 ) / 0.0227;
			Motor_Data[16] = (temp>0)? 0x01: 0x02;
			Motor_Data[17] = (unsigned char)abs(temp);
		}
	}

	if(mask_finger2 & mask){
		if(force.finger_force[1] == 0){
			Motor_Data[18] = Motor_Data[19] = 0x00;
		}else{
			float temp = (force.finger_force[1] * arm_finger2 / measure_arm_finger + 0.2135 ) / 0.0227;
			Motor_Data[18] = (temp>0)? 0x01: 0x02;
			Motor_Data[19] = (unsigned char)abs(temp);
		}
	}

	if(mask_finger3 & mask){
		if(force.finger_force[2] == 0){
			Motor_Data[20] = Motor_Data[21] = 0x00;
		}else{
			float temp = (force.finger_force[2] * arm_finger3 / measure_arm_finger + 0.2135 ) / 0.0227;
			Motor_Data[20] = (temp>0)? 0x01: 0x02;
			Motor_Data[21] = (unsigned char)abs(temp);
		}
	}
	if(ERROR_SUCCESS != m_serial.Write(Motor_Data, 22, &mWrite))
		return false;
	return true;
}

bool ForceFeedBack::SetForceWrist(const ForcePara& para){
	PosInfo position = Read();
	if(para.f_x == 0){
		Motor_Data[4] = Motor_Data[5] = 0x00;
	}else{
		float tempx = ((para.f_x * (position.p_z + 530.0) / measure_arm_force_x + 2.4401) / 0.5355) * 5.0;
		Motor_Data[4] = (tempx>0)? 0x01: 0x02;
		Motor_Data[5] = (unsigned char)((abs(tempx)<115)?abs(tempx):115);
	}

	if(para.f_y == 0){
		Motor_Data[6] = Motor_Data[7] = 0x00;
	}else{
		float tempy = ((para.f_y * (position.p_z + 530.0) / measure_arm_force_y + 2.8531) / 0.5771) * 5.0;
		Motor_Data[6] = (tempy>0)? 0x01: 0x02;
		Motor_Data[7] = (unsigned char)((abs(tempy)<115)?abs(tempy):115);
	}

	if(para.f_z == 0){
		Motor_Data[8] = Motor_Data[9] = 0x00;
	}else{
		float tempz;
		if(position.p_z<150){
				tempz = (para.f_z + 0.3459) / 0.2139 * 5.0;
			}else if(position.p_z<250){
				tempz = (para.f_z + 0.4166) / 0.2325 * 5.0;
			}else if(position.p_z<350){
				tempz = (para.f_z + 0.4375) / 0.2595 * 5.0;
			}else if(position.p_z<450){
				tempz = (para.f_z + 1.0310) / 0.3587 * 5.0;
			}else if(position.p_z<550){
				tempz = (para.f_z + 2.5128) / 0.6880 * 5.0;
			}else
				tempz = (para.f_z + 2.5128) / 0.6880 * 5.0;

		Motor_Data[8] = (tempz>0)? 0x01: 0x02;
		Motor_Data[9] = (unsigned char)((abs(tempz)<200)?abs(tempz):200);
	}
	if(ERROR_SUCCESS != m_serial.Write(Motor_Data, 22, &mWrite))
		return false;
	return true;
}

bool ForceFeedBack::SetTorqueFinger(const FingerTorque& torque, const int mask){
	if(mask_finger1 & mask){
		if(torque.finger_torque[0] == 0){
			Motor_Data[16] = Motor_Data[17] = 0x00;
		}else{
			float temp = (torque.finger_torque[0] / measure_arm_finger + 0.2135 ) / 0.0227;
			Motor_Data[16] = (temp>0)? 0x01: 0x02;
			Motor_Data[17] = (unsigned char)abs(temp);
		}
	}

	if(mask_finger2 & mask){
		if(torque.finger_torque[1] == 0){
			Motor_Data[18] = Motor_Data[19] = 0x00;
		}else{
			float temp = (torque.finger_torque[1] / measure_arm_finger + 0.2135 ) / 0.0227;
			Motor_Data[18] = (temp>0)? 0x01: 0x02;
			Motor_Data[19] = (unsigned char)abs(temp);
		}
	}

	if(mask_finger3 & mask){
		if(torque.finger_torque[2] == 0){
			Motor_Data[20] = Motor_Data[21] = 0x00;
		}else{
			float temp = (torque.finger_torque[2] / measure_arm_finger + 0.2135 ) / 0.0227;
			Motor_Data[20] = (temp>0)? 0x01: 0x02;
			Motor_Data[21] = (unsigned char)abs(temp);
		}
	}
	if(ERROR_SUCCESS != m_serial.Write(Motor_Data, 22, &mWrite))
		return false;
	return true;
}

bool ForceFeedBack::SetTorqueWrist(const TorquePara& para){
	if(para.t_x == 0){
		Motor_Data[10] = Motor_Data[11] = 0x00;
	}else{
		float tempx = ((para.t_x + 0.3503) / 0.0602) * 2.0;
		Motor_Data[10] = (tempx>0)? 0x01: 0x02;
		Motor_Data[11] = (unsigned char)abs(tempx);
	}

	if(para.t_y == 0){
		Motor_Data[12] = Motor_Data[13] = 0x00;
	}else{
		float tempy = ((para.t_y + 0.3503) / 0.0602) * 2.0;
		Motor_Data[12] = (tempy>0)? 0x01: 0x02;
		Motor_Data[13] = (unsigned char)abs(tempy);
	}

	if(para.t_z == 0){
		Motor_Data[14] = Motor_Data[15] = 0x00;
	}else{
		float tempz = ((para.t_z + 0.3503) / 0.0602) * 2.0;
		Motor_Data[14] = (tempz>0)? 0x01: 0x02;
		Motor_Data[15] = (unsigned char)abs(tempz);
	}

	if(ERROR_SUCCESS != m_serial.Write(Motor_Data, 22, &mWrite))
		return false;
	return true;
}

bool ForceFeedBack::ClearForce(){
	for(int i=4;i<22;i++)
		Motor_Data[i] = 0x00;
	if(ERROR_SUCCESS != m_serial.Write(Motor_Data, 22, &mWrite))
		return false;
	return true;
}