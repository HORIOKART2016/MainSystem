// TeachingSystem_HORIOKART.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

/*
�o�H�f�[�^�쐬�v���O�����F
���s�o�H�̃f�[�^���e�B�[�`���O�ɂ��쐬���邽�߂̃v���O����

���[�g�t�H�[�}�b�g�Fcsv
pointnum, mode, GL_x, GL_y, GL_th

mode	1:���i
		2:���̏��]
*/

#pragma warning(disable:4996)

#include "stdafx.h"

#include <iostream>
#include <ypspur.h>

#include <math.h>
#include <windows.h>
#include <stdlib.h>

//�R���g���[���pArduino��COM�|�[�g�̎w��
#define COMPORT "\\\\.\\COM15"


const char *filename = "SampleRouteOut2.csv";




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
	double x_GL = 0.0, y_GL = 0.0, th_GL = 0.0;
	double x_LC = 0.0, y_LC = 0.0, th_LC = 0.0;
	double x_GL_b, y_GL_b, th_GL_b;

	int num = 0;
	int ret;

	FILE *rt;

	//�R���g���[���[�̋N��
	if (system("start C:/Users/user/Desktop/���΃`�������W2016/HRIOKART2016/controller/HORIOKART_Controller/Debug/HORIOKART_Controller.exe")){
		std::cout << "controller open error....\n";
	}

	std::cout << "hit key to start\n";
	getchar();
	
	initSpur();

	Spur_set_pos_GL(0.0, 0.0, 0.0);
	Spur_set_pos_LC(0.0, 0.0, 0.0);

	std::cout << "Origin reset";

	fopen_s(&rt, filename, "w");

	while (1){
		if(getchar()=='q')
			break;

		Spur_get_pos_GL(&x_GL, &y_GL, &th_GL);
		Spur_get_pos_LC(&x_LC, &y_LC, &th_LC);


		std::cout << "num:" << num << "\n";
		std::cout << "GL:" << x_GL << "," << y_GL << "," << th_GL << "\n";
		std::cout << "LC:" << x_LC << "," << y_LC << "," << th_LC << "\n";

		//���i�����Ƃ�:20cm�ȏ�ړ�����
		if (sqrt((x_LC*x_LC) + (y_LC*y_LC)) > 0.2){
			fprintf(rt, "%d,%d,%lf,%lf,%lf\n", num, 1, x_GL, y_GL, th_GL);
		}

		//��]�����̂Ƃ�

		num++;

		Spur_set_pos_LC(0.0, 0.0, 0.0);
		x_GL_b = x_GL; y_GL_b = y_GL; th_GL_b = th_GL;

		
	}

	fclose(rt);

	std::cout << "Program End\n";

	return 0;
}

