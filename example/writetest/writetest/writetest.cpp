/************************************************
使用时：右击"属性" -> 更改"字符集"为"使用多字节字符集"
************************************************/

#include <iostream>
#include "include/forcefeedback.h"
using namespace std;

#pragma comment(lib, "lib/forcefeedback.lib")

char* m_COM[30] ={"COM1","COM2","COM3","COM4","COM5","COM6","COM7","COM8","COM9","COM10","COM11","COM12","COM13","COM14","COM15",
	"COM16","COM17","COM18","COM19","COM20","COM21","COM22","COM23","COM24","COM25","COM26","COM27","COM28","COM29","COM30"};

int main(){
	ForceFeedBack m_ins;

	//设置波特率
	if(!m_ins.SetEbaud(115200)){
		cout<<"ERROR: Set Ebaud failed !"<<endl;
		return 1;
	};

	//打开设备
	int i;
	for (i=0;i<30;i++)
	{
		if(m_ins.OpenForcebacker(m_COM[i])){
			cout<<"SUCCESS: Open Device With COM_PORT:"<<m_COM[i]<<" Successfully!"<<endl;
			break;
		}
	}
	if(30 == i){
		cout<<"ERROR: Open Device failed !"<<endl;
		return 1;
	}

	int mode = 0;
	cout<<"Please input control mode[0-5]"<<endl;
	cout<<"0: exit"<<endl;
	cout<<"1: set finger force"<<endl;
	cout<<"2: set finger Torque"<<endl;
	cout<<"3: set wrist force"<<endl;
	cout<<"4: set wrist Torque"<<endl;
	cout<<"5: clear"<<endl;
	while(true){
		cout<<"input mode:";
		cin>>mode;
		if(0 == mode)
			break;
		switch(mode){
		case 1:
			FingerForce tmp1;
			cout<<"input finger force: finger1、finger2、finger3:";
			cin>>tmp1.finger_force[0]>>tmp1.finger_force[1]>>tmp1.finger_force[2];
			if(!m_ins.SetForceFinger(tmp1)){
				cout<<"Set finger force failed !"<<endl;
			}else{
				cout<<"Set finger force successfully !"<<endl;
			}
			break;
		case 2:
			FingerTorque tmp2;
			cout<<"input finger torque: Torque1、Torque2、Torque3:";
			cin>>tmp2.finger_torque[0]>>tmp2.finger_torque[1]>>tmp2.finger_torque[2];
			if(!m_ins.SetTorqueFinger(tmp2)){
				cout<<"Set finger Torque failed !"<<endl;
			}else{
				cout<<"Set finger Torque successfully !"<<endl;
			}
			break;
		case 3:
			ForcePara tmp3;
			cout<<"input wrist force: input force f_x、f_y、f_z";
			cin>>tmp3.f_x>>tmp3.f_y>>tmp3.f_z;
			if(!m_ins.SetForceWrist(tmp3)){
				cout<<"Set wrist force failed !"<<endl;
			}else{
				cout<<"Set wrist force successfully !"<<endl;
			}
			break;
		case 4:
			TorquePara tmp4;
			cout<<"input wrist torque: input t_x、t_y、t_z";
			cin>>tmp4.t_x>>tmp4.t_y>>tmp4.t_z;
			if(!m_ins.SetTorqueWrist(tmp4)){
				cout<<"Set wrist torque failed !"<<endl;
			}else{
				cout<<"Set wrist torque successfully !"<<endl;
			}
			break;
		case 5:
			if(!m_ins.ClearForce()){
				cout<<"Clear failed !"<<endl;
			}else{
				cout<<"Clear successfully !"<<endl;
			}
			break;
		default:
			continue;
		}
	}
	m_ins.ClearForce();
	Sleep(10);
	return 0;
}