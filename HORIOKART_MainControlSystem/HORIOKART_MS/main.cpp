// HORIOKART_MainControlSystem.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#define _CRT_SECURE_NO_WARININGS

#include <stdio.h>
#include <tchar.h>
#include <ypspur.h>
#include <iostream>
#include <math.h>
#include <windows.h>
#include <stdlib.h>

#define PI 3.14159265359


//���[�g�̃t�@�C����
const char *routefile = "SampleRouteOut2.csv";
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

extern int Euler_state(void);

#define detect 0			//1�ŗL���@0����

extern void Detect_RoadEdge(double *edge);


//�ړI�n�̍��W�iLC�j
double tar_x_LC, tar_y_LC, tar_th_LC;

//���E��LC�̒l(��)�̐��� ���F���i+�j�@�E�F���i-�j
double PassibleRange_left = 1.5, PassibleRange_right = -1.5;
double RoadEdge_buf1[2] = { 0.0, 0.0 }, RoadEdge_buf2[2] = { 0.0, 0.0 };

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
	Spur_set_angvel(0.5);	//�p���x�irad/s)
	Spur_set_angaccel(1.5);		//�p�����x�irad/s/s)
	
	
	return 0;
}



int initialize(){

	//�o�H�f�[�^���J��
	fopen_s(&rt, routefile, "r");
	/*if ((rt = fopen(routefile, "r")) == NULL){
		//�o�����1�x�ǂ݂���ŗL���Ȍo�H�������Ă��邩�ǂ������m�F����i����̉ۑ�j

		printf("Can't open Route file.....\n");
		return 1;
	}*/


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
	fopen_s(&trq, torq_record, "w");
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
int EmergencyButtonState(void){
	
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
			Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);

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
int run_Obstacledetection(void){
	
	int obstacle_state;

	obstacle_state = obstacle_detection();	//��Q���̃X�e�[�^�X�̎擾
	//std::cout << obstacle_state << "\n";
	//���s�\�̏�Q�������m�����Ƃ���~���ă��[�v�ɓ˓�����
	if (obstacle_state == 8 || obstacle_state == 2 || obstacle_state == 4 || obstacle_state == 6 || obstacle_state == 7 ){
		Unvoidable_Obstacle(); 
		Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
	}
	

	switch (obstacle_state){
		case 8:
			Unvoidable_Obstacle();
			Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
			break;
		case 2:
		case 6:
			if (abs(PassibleRange_left) < 0.3){
				Unvoidable_Obstacle();
				Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
			}
			else{
				tar_y_LC = tar_y_LC - 0.5;
				Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
			}
		case 4:
		case 7:
			if (abs(PassibleRange_right) < 0.3){
				Unvoidable_Obstacle();
				Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
			}
			else{
				tar_y_LC = tar_y_LC + 0.5;
				Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
			}

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



void RoadEdge_syori(void){
	
	double RoadEdge[2];

	Detect_RoadEdge(RoadEdge);
	
	//��O�����F�O�Q��̌��ʂ���1���֐����擾�i2�O��0���̎���0.5�j
	//���̒����Ƃ̋����𓱏o��臒l��p���ė�O����
	//buf��1����O�@2������0.5����i���������߂��j


	if ((abs(RoadEdge_buf1[0] - RoadEdge_buf2[1]) > 0.5) && (abs(RoadEdge_buf2[0] - RoadEdge_buf2[1]) > 0.5))
	{
		double a, c;
		a = (RoadEdge_buf2[0] - RoadEdge_buf1[0]) / 0.5;
		c = RoadEdge_buf1[0];
		if ((abs(a*RoadEdge[0] - 1.0 + c) / sqrt(a*a + 1)) < 0.5)
		{
			PassibleRange_left = RoadEdge[0];
		}
		a = (RoadEdge_buf2[1] - RoadEdge_buf1[1]) / 0.5;
		c = RoadEdge_buf1[1];
		if ((abs(a*RoadEdge[1] - 1.0 + c) / sqrt(a*a + 1)) < 0.5)
		{
			PassibleRange_left = RoadEdge[1];
		}
	}

	RoadEdge_buf1[0] = RoadEdge_buf2[0];
	RoadEdge_buf1[1] = RoadEdge_buf2[1];

	RoadEdge_buf2[0] = RoadEdge[0];
	RoadEdge_buf2[1] = RoadEdge[1];

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
	double PassibleRange_left = 1.5, PassibleRange_right = -1.5;
	
		
	//�G���R�[�_�l�擾�p�̕ϐ�
	double x_GL = 0.0f, y_GL = 0.0f, th_GL = 0.0f;
	double x_LC = 0.0f, y_LC = 0.0f, th_LC = 0.0f;

	int over = 0;
	int online = 0;			//�ڕW�_�E�p�x�̂Ȃ�������ɂ��邩�@�@0�Ffalse 1:true

	double border = 0.5;

	int border_count = 0;

	int roadmode;


	//�e���W�n�����_�ɐݒ�
	Spur_set_pos_GL(0.0, 0.0, 0.0);
	Spur_set_pos_LC(0.0, 0.0, 0.0);
	
	//�ꉞ���Ԍv���F�J�n
	QueryPerformanceCounter(&start);

	std::cout << "runcontrol start\n";


	//���[�g�����ǂ郋�[�v�̊J�n
	while (fgets(buf, 5412, rt) != NULL){
		
		//�ڕW�_�̓ǂݍ���
		//���@�F�ԍ��C���C���C�摜�����̗L��
		sscanf_s(buf, "%d,%lf,%lf,%d", &num, &tar_x_GL, &tar_y_GL, &roadmode);
		
		
		tar_th_GL = atan((tar_y_GL - before_y_GL) / (tar_x_GL - before_x_GL));
		
		std::cout << "num:" << num << "\n";
		std::cout << "target:" << tar_x_GL << "," << tar_y_GL << "," << tar_th_GL << "\n";
	
		
		
		//LC�ł̖ڕW���W���擾���Ă���
		tar_x_LC = sqrt((tar_x_GL - before_x_GL)*(tar_x_GL - before_x_GL) + (tar_y_GL - before_y_GL)*(tar_y_GL - before_y_GL));
		tar_y_LC = 0.0;
		tar_th_LC = 0.0;

		//
		
		
		//���Ȉʒu��LC�̃��Z�b�g
		Spur_get_pos_GL(&x_GL,&y_GL,&th_GL);
		Spur_set_pos_LC(x_GL - tar_x_GL, y_GL - tar_y_GL, th_GL - tar_th_GL);

		//����ȉ��ł͂��ׂ�LC�ō��W������

		//�쓮�w��
		Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
					
		while (!Spur_over_line_LC(tar_x_LC - 0.3, tar_y_LC, tar_th_LC)){
			
			//�e�Z���T����̃X�e�[�^�X

			//�ً}��~�̏�Ԏ擾�F�ً}��~����q����������1���Ԃ��ꏈ�����I������
			if (EmergencyButtonState())
				return;

			//��Q���̈ʒu���m
			if (detect)
				run_Obstacledetection();

			//�����󂩂ǂ���
			if (!online){
				online = Spur_near_ang_LC(tar_th_LC, 0.1);
			}
			
			//�X�΁E��]�p�̕␳
			Euler_state();


			//�g���N�̌v���i������)
			RecordTorq(num, 2);

			//�쓮�w�߂��C������ꍇ�̓���������ɑ}��
			//0.5m�i�ޓx�ɉ摜�擾�E2�����Ƃɋ쓮�w�߂����Ȃ���
			if (Spur_over_line_LC(border, tar_y_LC, tar_th_LC)){
				border_count++;
				if (roadmode)
					RoadEdge_syori();

				//�Б��Ɋ���Ă�Ƃ�:�ړI�n�̂����W��ύX���ċ쓮�w�߂����Ȃ���
				if ((abs(PassibleRange_left) > 0.01) && (abs(PassibleRange_left)<0.3)){
					tar_y_LC = tar_y_LC-0.5;
					Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
				}
				
				else if ((abs(PassibleRange_right) > 0.01) && (abs(PassibleRange_right)<0.3)){
					tar_y_LC = tar_y_LC+0.5;
					Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
				}

				//2m���ƂɎw�߂���꒼��
				if (border_count > 3){
					Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
					border_count = 0;
				}

				border = border + 0.5;
			}




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
		over = 0;
		online = 0;
		border = 0.5;
		border_count = 0;

		PassibleRange_left = 0.0;
		PassibleRange_right = 0.0;

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

