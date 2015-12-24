/************************************************
ʹ��ʱ���һ�"����" -> ����"�ַ���"Ϊ"ʹ�ö��ֽ��ַ���"
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
	//���ò�����
	if(!m_ins.SetEbaud(115200)){
		cout<<"ERROR: Set Ebaud Failed !"<<endl;
		return 1;
	};
	
	//���豸
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

	//resetʵ�ָ�λ����Ҫ�رմ��ں����¿���������Ч
	if(!m_ins.SetCurOrig()){
		cout<<"Reset failed !"<<endl;
		return 1;
	};

	while (!kbhit()){
		PosInfo pos = m_ins.Read();
		RawData ang = m_ins.ReadRaw();
		cout<<pos.p_x<<' '<<pos.p_y<<' '<<pos.p_z<<' '<<pos.r_x<<' '<<pos.r_y<<' '<<pos.r_z<<'\r';
	}
	cout<<endl;
	return 0;
}