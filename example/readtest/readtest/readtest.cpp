/************************************************
使用时：右击"属性" -> 更改"字符集"为"使用多字节字符集"
************************************************/
#include <iostream>
#include <conio.h>
#include "include/forcefeedback.h"
using namespace std;

#pragma comment(lib, "lib/forcefeedback.lib")

char* m_COM[30] ={"COM1","COM2","COM3","COM4","COM5","COM6","COM7","COM8","COM9","COM10","COM11","COM12","COM13","COM14","COM15",
	"COM16","COM17","COM18","COM19","COM20","COM21","COM22","COM23","COM24","COM25","COM26","COM27","COM28","COM29","COM30"};

int main(){
	ForceFeedBack m_ins;
	//设置波特率
	if(!m_ins.SetEbaud(115200)){
		cout<<"ERROR: Set Ebaud Failed !"<<endl;
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

	//reset实现复位，需要关闭串口后重新开启才能生效
	if(!m_ins.SetCurOrig()){
		cout<<"Reset failed !"<<endl;
		return 1;
	};

	//开启更新线程
	m_ins.StartCycUpdate();

	while (!kbhit()){
		PosInfo pos = m_ins.Read();
		cout<<pos.p_x<<' '<<pos.p_y<<' '<<pos.p_z<<' '<<pos.r_x<<' '<<pos.r_y<<' '<<pos.r_z<<'\r';
	}
	cout<<endl;

	//关闭更新线程
	m_ins.StopCycUpdate();
	return 0;
}
