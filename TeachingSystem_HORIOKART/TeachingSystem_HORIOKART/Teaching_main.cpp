// TeachingSystem_HORIOKART.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"
#include <ypspur.h>
#include <iostream>
#include <math.h>
#include <windows.h>
#include <stdlib.h>

//�R���g���[���pArduino��COM�|�[�g�̎w��
#define COMPORT "\\\\.\\COM15"





//ypspur�Ƃ̒ʐM�̏�����
//ypspur coordinater�Ƃ̒ʐM���J�n����

int initSpur(void){
	// Windows���ŕW���o�͂��o�b�t�@�����O����Ȃ��悤�ɐݒ�
	setvbuf(stdout, 0, _IONBF, 0);

	// ������
	if (Spur_init() < 0)
	{
		fprintf(stderr, "ERROR : cannot open spur.\n");
		return -1;
	}

	return 0;
}



//Arduino�̃n���h�����擾����

int getArduinoHandle(HANDLE& hComm){
	//�V���A���|�[�g���J���ăn���h�����擾

	hComm = CreateFile(_T(COMPORT), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

	if (hComm == INVALID_HANDLE_VALUE){
		printf("�V���A���|�[�g���J�����Ƃ��ł��܂���ł����B");
		char z;
		z = getchar();
		return -1;
	}
	//�|�[�g���J���Ă���ΒʐM�ݒ���s��
	else
	{
		printf("port open\n");
		DCB lpTest;
		GetCommState(hComm, &lpTest);
		lpTest.BaudRate = 9600;
		lpTest.ByteSize = 8;
		lpTest.Parity = NOPARITY;
		lpTest.StopBits = ONESTOPBIT;
		SetCommState(hComm, &lpTest);
	}
	return 0;
}



int main(int argc, _TCHAR* argv[])
{
	double x_GL, y_GL, th_GL;
	double x_LC, y_LC, th_LC;

	int num = 0;
	int ret;

	//�R���g���[���[�̋N��
	/*if (system("C:/Users/user/Desktop/���΃`�������W2016/HRIOKART2016/controller/HORIOKART_Controller/Debug/HORIOKART_Controller.exe")){
		std::cout << "controller open error....\n";
	}*/

	ShellExecute(NULL, NULL, (LPCWSTR)"C:/Users/user/Desktop/���΃`�������W2016/HRIOKART2016/controller/HORIOKART_Controller/Debug/HORIOKART_Controller.exe", NULL, NULL, SW_SHOWNORMAL);

	std::cout << "start\n";
	initSpur();

	//Spur_set_pos_GL(0.0, 0.0, 0.0);
	//Spur_set_pos_LC(0.0, 0.0, 0.0);

	while (1){
		if(getchar()=='q')
			break;

		Spur_get_pos_GL(&x_GL, &y_GL, &th_GL);
		Spur_get_pos_LC(&x_LC, &y_LC, &th_LC);


		std::cout << "num:" << num << "\n";
		std::cout << "GL:" << x_GL << "," << y_GL << "," << th_GL << "\n";
		std::cout << "LC:" << x_LC << "," << y_LC << "," << th_LC << "\n";

		num++;

		//Spur_set_pos_LC(0.0, 0.0, 0.0);
		
	}

	std::cout << "Program End\n";

	return 0;
}

