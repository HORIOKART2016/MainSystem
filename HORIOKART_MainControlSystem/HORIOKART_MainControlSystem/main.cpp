// HORIOKART_MainControlSystem.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"
#include <ypspur.h>
#include <iostream>
#include <math.h>
#include <windows.h>
#include <stdlib.h>

#define PI 3.14159265359

//�R���g���[���pArduino��COM�|�[�g�̎w��
#define COMPORT "\\\\.\\COM15"


//���[�g�̃t�@�C����
const char *routefile = "SampleRoute.csv";
FILE *rt;

#define vel 3000		//�ō����x(m/h)
#define acc	1500		//�ő�����x(m/h/s)

//�����Ɏ��Ԍv�����Ă݂�
LARGE_INTEGER freq;
LARGE_INTEGER start, now;


//�����Ƀg���N�L�^���Ă݂�
FILE *trq;
const char *torq_record = "TorqRecord.csv";


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

	Spur_set_vel(vel / 3600);		//���x0.3m/sec
	Spur_set_accel(acc / 3600);	//�����x�im/s/s�j
	Spur_set_angvel(90 * PI / 180);	//�p���x�irad/s)
	Spur_set_angaccel(180 * PI / 180);		//�p�����x�irad/s/s)

	//�g���N�L�^�p�̃t�@�C���I�[�Ղ�
	trq = fopen(torq_record, "w");
	fprintf(trq, "time,fase,right_torque,left_torque,x,y,th\n");

	return 0;
}



//Arduino�̃n���h�����擾����
//�g���ĂȂ�
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



//����~�̔��f����֐�
//(����~�̎w�߂͕ʂ̃v���O��������o�����)
void EmergencyButtonState(double x, double y, double th){
	
	double RightAngVel, LeftAngVel;
	double Avel;
	int emergency_time_count;

	YP_get_wheel_vel(&RightAngVel, &LeftAngVel);
	
	Avel = (RightAngVel + LeftAngVel) / 2;

	//��]�������ɏ������ꍇ�Ƀ��[�v�ɓ���
	while(Avel < 0.001){
		std::cout << "emergency stop?\n";
		emergency_time_count++;
		if (emergency_time_count>10){
			//1�b�Ԏ~�܂����܂܂��Ɣ���~���Ɣ��f����

			std::cout << "I think emergency stop now!!\n";
			std::cout << "If start again, Please hit key!!\n";
			getchar();

			//�ĊJ����ꍇ�T�C�h�w�߂𑗂�
			Spur_line_GL(x, y, th);

			break;
		}
		
		Sleep(100);

	}

}



//�����Ƀg���N���擾���Ă݂�
//�Ƃ肠������csv�ɂ��߂邾��
//�����Ɏg����������Ȃ�(��]���m��)
void RecordTorq(int num){
	
	double R_torq, L_torq,x,y,th;

	YP_get_wheel_torque(&R_torq, &L_torq);
	Spur_get_pos_GL(&x, &y, &th);

	QueryPerformanceCounter(&now);

	fprintf(trq, "%lf,%d,%lf,%lf,%lf,%lf,%lf\n", (double)((now.QuadPart - start.QuadPart) / freq.QuadPart), num, R_torq, L_torq, x, y, th);


}




//���s����p�̃��C�����[�v
void RunControl_mainloop(void){

	char buf[512];
	int num, mode;
	//�ړI�n�̍��W�iGL�j
	double tar_x, tar_y, tar_th;

	double x_GL = 0.0, y_GL = 0.0, th_GL = 0.0;
	double x_LC = 0.0, y_LC = 0.0, th_LC = 0.0;

	//�e���W�n�����_�ɐݒ�
	Spur_set_pos_GL(0.0, 0.0, 0.0);
	Spur_set_pos_LC(0.0, 0.0, 0.0);
	
	std::cout << "runcontrol start\n";

	QueryPerformanceCounter(&start);

	//���[�v�̊J�n
	while (fgets(buf, 5412, rt) != NULL){
		//�ڕW�_�̓ǂݍ���
		sscanf(buf, "%d,%d,%lf,%lf,%lf",&num,&mode,&tar_x,&tar_y,&tar_th);

		std::cout << "num:" << num << "\n";
		std::cout << "target:" << tar_x << "," << tar_y << "," << tar_th << "\n";

		Spur_line_GL(tar_x, tar_y, tar_th);

		//���B�������̃��[�v
		while (!Spur_over_line_GL(tar_x, tar_y, tar_th)){

			//�����Ɋe�Z���T����̃t�B�[�h�o�b�N������
			
			//�ً}��~�̏�Ԏ擾
			EmergencyButtonState(tar_x,tar_y,tar_th);

			//�g���N�̌v���i������)
			RecordTorq(num);

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
	QueryPerformanceFrequency(&freq);

	//�o�H�f�[�^���J��
	if ((rt = fopen(routefile, "r")) == NULL){
		//�o�����1�x�ǂ݂���ŗL���Ȍo�H�������Ă��邩�ǂ������m�F����i����̉ۑ�j

		printf("Can't open Route file.....\n");
		return -1;
	}


	//�o�b�N�O���E���h�ŃR���g���[���͋N�����Ă���
	if (system("start ../../MS_backgroundController/Debug/HORIOKART_Controller.exe")){
		std::cout << "controller open error....\n";
	}
	else{ std::cout << "cotroller Open\n"; }
	
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


	//�f�o�b�O�p
	//�g���N�v���t�@�C���̕ۑ�
	fclose(trq);

	return 0;
}

