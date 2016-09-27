// HORIOKART_MainControlSystem.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"
#include <ypspur.h>
#include <iostream>
#include <math.h>
#include <windows.h>
#include <stdlib.h>

//�R���g���[���pArduino��COM�|�[�g�̎w��
#define COMPORT "\\\\.\\COM15"


//���[�g�̃t�@�C����
const char *routefile = "SampleRoute.csv";
FILE *rt;

#define vel 3000		//�ō����x(m/h)
#define acc	1500		//�ő�����x(m/h/s)


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

	std::cout << "Spur initialized\n\n";

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



void RunControl_mainloop(void){

	char buf[512];
	int num, mode;
	double tar_x, tar_y, tar_th;

	double x_GL = 0.0, y_GL = 0.0, th_GL = 0.0;
	double x_LC = 0.0, y_LC = 0.0, th_LC = 0.0;


	//�e���W�n�����_�ɐݒ�
	Spur_set_pos_GL(0.0, 0.0, 0.0);
	Spur_set_pos_LC(0.0, 0.0, 0.0);

	//�ō����x�̐ݒ�(m/s)
	Spur_set_vel(vel / 3600, vel / 3600);
	//�����x�̐ݒ�(m/s�s)
	Spur_set_accel(acc / 3600, acc / 3600);



	//���[�v�̊J�n
	while (fgets(buf, 5412, rt) != NULL){
		//�ڕW�_�̓ǂݍ���
		sscanf(buf, "%d,%d,%lf,%lf,%lf",&num,&mode,&tar_x,&tar_y,&tar_th);

		Spur_line_GL(tar_x, tar_y, tar_th);

		//���B�������̃��[�v
		while (!Spur_over_line_GL(tar_x, tar_y, tar_th)){

			//�����Ɋe�Z���T����̃t�B�[�h�o�b�N������


			//�쓮�w�߂��C������ꍇ�̓���������ɑ}��

			Sleep(10);
		}
		//�ڕW�n�_�t�߂ɓ��B�������~
		Spur_stop();

		Spur_get_pos_GL(&x_GL, &y_GL, &th_GL);
		Spur_get_pos_LC(&x_LC, &y_LC, &th_LC);

		std::cout << "num:" << num << "\n";
		std::cout << "target:" << tar_x << "," << tar_y << "," << tar_th << "\n";
		std::cout << "  GL  :" << x_GL << "," << y_GL << "," << th_GL << "\n";
		std::cout << "  LC  :" << x_LC << "," << y_LC << "," << th_LC << "\n\n";



		//LC�����_�ɂ���
		Spur_set_pos_LC(0.0, 0.0, 0.0);

		//���̌o�H��
	}

	//���ׂĂ̌o�H���I������Ƃ����ɓ��B����
	std::cout << "All Route Complete!!\n";

	Spur_stop();

}


int main(void)
{
		
	//�o�H�f�[�^���J��
	if ((rt = fopen(routefile, "r")) == NULL){
		//�o�����1�x�ǂ݂���ŗL���Ȍo�H�������Ă��邩�ǂ������m�F����i����̉ۑ�j

		printf("Can't open Route file.....\n");
		return -1;
	}


	//�o�b�N�O���E���h�ŃR���g���[���͋N�����Ă���
	/*if (system("start C:/Users/user/Desktop/���΃`�������W2016/HRIOKART2016/controller/HORIOKART_Controller/Debug/HORIOKART_Controller.exe")){
		std::cout << "controller open error....\n";
	}
	else{ std::cout << "cotroller Open\n"; }
	*/
	//�R���g���[���𓮂����Ă�Ɠr���Ŏ~�߂�w�ߓ��ꂿ�Ⴄ�̂Ŏd�l�ύX���K�v

	//�����ɏ�Q�����m�p��URG�̃n���h����������



	//YPSpur�Ƃ̒ʐM���J�n����
	if (initSpur())
	{
		return(-1);
	}

	//�L�[���͂�҂�
	std::cout << "\nHit key to start";
	getchar();

	//���s���䃋�[�v�ɓ˓�
	RunControl_mainloop();

	return 0;
}

