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
const char *routefile = "SampleRouteOut.csv";
//const char *routefile = "../../TeachingSystem_HORIOKART/TeachingSystem_HORIOKART/SampleRoute.csv";
FILE *rt;

#define vel 2500		//�ō����x(m/h)
#define acc	1500		//�ő�����x(m/h/s)

//�����Ɏ��Ԍv�����Ă݂�
LARGE_INTEGER freq;
LARGE_INTEGER start, now;


//�����Ƀg���N�L�^���Ă݂�
FILE *trq;
const char *torq_record = "TorqRecord.csv";

extern int init_URG();			//urg��initialize
extern int obstacle_detection();

#define detect 0			//1�ŗL���@0����

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

	/*Spur_set_vel(vel / 3600);		//���x0.3m/sec
	Spur_set_accel(acc / 3600);	//�����x�im/s/s�j
	Spur_set_angvel(90 * PI / 180);	//�p���x�irad/s)
	Spur_set_angaccel(180 * PI / 180);		//�p�����x�irad/s/s)
	*/

	Spur_set_vel(0.3);		//���x0.3m/sec
	Spur_set_accel(1.0);	//�����x�im/s/s�j
	Spur_set_angvel(1.5);	//�p���x�irad/s)
	Spur_set_angaccel(0.5);		//�p�����x�irad/s/s)
	
	
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


int initialize(){

	//�o�H�f�[�^���J��
	if ((rt = fopen(routefile, "r")) == NULL){
		//�o�����1�x�ǂ݂���ŗL���Ȍo�H�������Ă��邩�ǂ������m�F����i����̉ۑ�j

		printf("Can't open Route file.....\n");
		return 1;
	}


	//�o�b�N�O���E���h�ŃR���g���[�����N�����Ă���
	if (system("start ../Debug/MS_Controller.exe")){
		std::cout << "controller open error....\n";
	}
	else{ std::cout << "cotroller Open\n\n"; }



	//��Q�����m�p��URG�Ƃ̒ʐM�̊J�n
	if (init_URG()){
		std::cout << "URG Error...\n";
		return 1;
	}
	else{
		std::cout << "URG for Detect : Open\n\n";
	}



	//YPSpur�Ƃ̒ʐM���J�n����
	if (initSpur())
	{
		return 1;
	}



	//�g���N�L�^�p�̃t�@�C���I�[�v��
	trq = fopen(torq_record, "w");
	fprintf(trq, "TimeStamp,fase,mode,right_torque,left_torque,x,y,th,xLC,y_LC,th_LC\n");




	return 0;



}

/*------�����܂ł�initialize------*/


void finalise(void){

	//�f�o�b�O�p
	//�g���N�v���t�@�C���̕ۑ�
	fclose(trq);

}


//����~�̔��f����֐�
//(����~�̎w�߂͕ʂ̃v���O��������o�����)
int EmergencyButtonState(double x, double y, double th){
	
	double RightAngVel, LeftAngVel;
	double Avel;
	int emergency_time_count=0;

	YP_get_wheel_vel(&RightAngVel, &LeftAngVel);
	
	Avel = (abs(RightAngVel) + abs(LeftAngVel)) / 2;

	//��]�������ɏ������ꍇ�Ƀ��[�v�ɓ���
	while(Avel < 0.001){
		std::cout << Avel;
		std::cout << "emergency stop?\n";
		emergency_time_count++;
		if (emergency_time_count>10){
			//1�b�Ԏ~�܂����܂܂��Ɣ���~���Ɣ��f����

			std::cout << "I think emergency stop now!!\n";
			std::cout << "If start again, Please hit key!!\n";
			
			if (getchar() == 'q')
			{
				finalise();
				std::cout << "file saved";
				return 1;
			}

			//�ĊJ����ꍇ�T�C�h�w�߂𑗂�
			Spur_line_GL(x, y, th);

			break;
		}
		
		Sleep(1000);

	}
	return 0;

}



//���s�\�̏�Q���ɑ΂��郋�[�v
int Unvoidable_Obstacle(){


	Spur_stop();		//���s�\�̏�Q���𔭌������ꍇ�����ɒ�~����

	int obstacle_state=8;

	std::cout << "Detect obstacle!!\n\n";

	while (1){
		//��Q���̊m�F
		obstacle_state = obstacle_detection();

		//obstacle=detectObstacle;
		if (obstacle_state != 8 && obstacle_state != 2 && obstacle_state != 4 && obstacle_state != 6 && obstacle_state != 7){
			break;
		}
		Sleep(100);
	}

	std::cout << "Obstacle is cleared \n\n";
	return 0;

}

//��Q�����m
int run_Obstacledetection(double x, double y, double th){
	
	int obstacle_state;

	obstacle_state = obstacle_detection();	//��Q���̃X�e�[�^�X�̎擾
	//std::cout << obstacle_state << "\n";
	//���s�\�̏�Q�������m�����Ƃ���~���ă��[�v�ɓ˓�����
	if (obstacle_state == 8 || obstacle_state == 2 || obstacle_state == 4 || obstacle_state == 6 || obstacle_state == 7 ){
		Unvoidable_Obstacle(); 
		Spur_line_GL(x, y, th);
	}

	return 0;

}


//�����Ƀg���N���擾���Ă݂�
//�Ƃ肠������csv�ɂ��߂邾��
//�����Ɏg����������Ȃ�(��]���m��)
void RecordTorq(int num,int mode){
	
	double R_torq, L_torq,x,y,th;
	double x_LC, y_LC, th_LC;

	YP_get_wheel_torque(&R_torq, &L_torq);
	Spur_get_pos_GL(&x, &y, &th);
	Spur_get_pos_LC(&x_LC, &y_LC, &th_LC);

	QueryPerformanceCounter(&now);

	fprintf(trq, "%lf,%d,%d,%lf,%lf,%lf,%lf,%lf%lf,%lf,%lf\n", (((double)now.QuadPart - (double)start.QuadPart) / (double)freq.QuadPart),
																							num, mode, R_torq, L_torq, x, y, th, x_LC, y_LC, th_LC);


}


//���s����p�̃��C�����[�v
void RunControl_mainloop(void){

	char buf[512];
	int num, mode;

	int loop_count = 0;

	int Obstacle_state = 0;

	//�ړI�n�̍��W�iGL�j
	double tar_x_GL, tar_y_GL, tar_th_GL;
	//�ړI�n�̍��W�iLC�j
	double tar_x_LC, tar_y_LC, tar_th_LC;
	//�O�̓_�̍��W���L�^
	double before_x_GL = 0.0f, before_y_GL = 0.0f, before_th_GL = 0.0f;

	//���E��LC�̒l(��)�̐��� ���F���i+�j�@�E�F���i-�j
	double PassibleRange_left = 1.0, PassibleRange_right = -1.0;
		
	//�G���R�[�_�l�擾�p�̕ϐ�
	double x_GL = 0.0f, y_GL = 0.0f, th_GL = 0.0f;
	double x_LC = 0.0f, y_LC = 0.0f, th_LC = 0.0f;



	//�e���W�n�����_�ɐݒ�
	Spur_set_pos_GL(0.0, 0.0, 0.0);
	Spur_set_pos_LC(0.0, 0.0, 0.0);
	
	//�ꉞ���Ԍv���F�J�n
	QueryPerformanceCounter(&start);

	std::cout << "runcontrol start\n";


	//���[�v�̊J�n
	while (fgets(buf, 5412, rt) != NULL){
		
		//�ڕW�_�̓ǂݍ���
		sscanf(buf, "%d,%d,%lf,%lf,%lf,%lf,%lf", &num, &mode, &tar_x_GL, &tar_y_GL, &tar_th_GL, &PassibleRange_right, &PassibleRange_left);
		
		std::cout << "num:" << num << "\n";
		std::cout << "target:" << tar_x_GL << "," << tar_y_GL << "," << tar_th_GL << "\n";


		//�쓮�w��
		Spur_line_GL(tar_x_GL, tar_y_GL, tar_th_GL);

		//LC�ł̖ڕW���W���擾���Ă���
		tar_x_LC = tar_x_GL - before_x_GL;
		tar_y_LC = tar_y_GL - before_y_GL;
		tar_th_LC = 0.0;



		//������ɓ��B�������̃��[�v
		while (!Spur_near_ang_GL(tar_th_GL, 0.1)){
			
			//���B����܂ł͏�Q�����m�E�ً}��~�E�g���N�v���݂̂��s��
			if (EmergencyButtonState(tar_x_GL, tar_y_GL, tar_th_GL)){
				//�ً}��~����q�����͂�����1���Ԃ��ꏈ�����I������
				return;
			}

			
			//��Q�����m
			if (detect)
				run_Obstacledetection(tar_x_GL, tar_y_GL, tar_th_GL);

			//�g���N�̌v���i������)
			RecordTorq(num,1);
			
			//����w��

			Sleep(10);
		}
	


		//�p�x�����B������LC���Z�b�g����
		Spur_get_pos_GL(&x_GL, &y_GL, &th_GL);
		Spur_set_pos_LC(x_GL - before_x_GL, y_GL - before_y_GL, tar_th_GL - th_GL);		//�p�x�̌v�Z���H
		std::cout << "LC Reset!!\n";




		//���B�������̃��[�v�@:�@������O�ŋȂ���n�߂�
		while (!Spur_over_line_LC(tar_x_LC - 0.3, tar_y_LC, tar_th_LC)){
		
			//�g���N�̌v���i������)
			RecordTorq(num,2);
			
			//�e�Z���T����̃X�e�[�^�X
			
			//�ً}��~�̏�Ԏ擾�F�ً}��~����q����������1���Ԃ��ꏈ�����I������
			if(EmergencyButtonState(tar_x_GL, tar_y_GL, tar_th_GL))
				return;
			
			//��Q���̈ʒu���m
			if (detect)
				run_Obstacledetection(tar_x_GL, tar_y_GL, tar_th_GL);


			
			//���E�̓����i���~�b�g�j�̎擾�i100���[�v�Ɉ��j
			//detectLoadLimit();
					

			//�쓮�w�߂��C������ꍇ�̓���������ɑ}��

			Sleep(10);
		}





		Spur_get_pos_GL(&x_GL, &y_GL, &th_GL);
		Spur_get_pos_LC(&x_LC, &y_LC, &th_LC);

	
		std::cout << "  GL  :" << x_GL << "," << y_GL << "," << th_GL << "\n";
		std::cout << "  LC  :" << x_LC << "," << y_LC << "," << th_LC << "\n\n";



		//�O�̖ړI���W���L�^���Ă���
		before_x_GL = tar_x_GL;
		before_y_GL = tar_y_GL;
		before_th_GL = tar_th_GL;

		//���̌o�H��
	}

	//���ׂĂ̌o�H���I������Ƃ����ɓ��B����
	std::cout << "All Route Complete!!\n";

	Spur_stop();


	//�I������
	finalise();

}



int main(void)
{
	int ret;

	QueryPerformanceFrequency(&freq);

	ret = initialize();

	if (ret != 0){
		return 1;
	}

	//�L�[���͂�҂�
	std::cout << "\nHit key to start";
	getchar();

	//���s���䃋�[�v�ɓ˓�
	RunControl_mainloop();


	return 0;
}

