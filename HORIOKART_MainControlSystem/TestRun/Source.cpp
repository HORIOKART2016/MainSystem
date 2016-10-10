
#include <stdio.h>
#include <ypspur.h>
#include <iostream>
#include <math.h>
#include <windows.h>
#include <stdlib.h>

#define PI 3.14159265359
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

	Spur_set_vel(vel / 3600);		//���x0.3m/sec
	Spur_set_accel(acc / 3600);	//�����x�im/s/s�j
	Spur_set_angvel(90 * PI / 180);	//�p���x�irad/s)
	Spur_set_angaccel(180 * PI / 180);		//�p�����x�irad/s/s)

	
	return 0;
}

void RunControl(){

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

	Spur_line_LC(1.0, 0.0, 0.0);

	std::cout << "hit enter to stop\n";
		while(1){
		Spur_get_pos_LC(&x_LC, &y_LC, &th_LC);
		std::cout << "  LC  :" << x_LC << "," << y_LC << "," << th_LC << "\n\n";
		Sleep(100);
		}


}

int main(void)
{

	//�o�b�N�O���E���h�ŃR���g���[���͋N�����Ă���
	if (system("start ../Debug/MS_Controller.exe")){
		std::cout << "controller open error....\n";
	}
	else{ std::cout << "cotroller Open\n"; }

	
	//YPSpur�Ƃ̒ʐM���J�n����
	if (initSpur())
	{
		return(-1);
	}

	//�L�[���͂�҂�
	std::cout << "\nHit key to start";
	getchar();

	//���s���䃋�[�v�ɓ˓�
	RunControl();


	

	return 0;
}

