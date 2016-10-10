
#include <stdio.h>
#include <ypspur.h>
#include <iostream>
#include <math.h>
#include <windows.h>
#include <stdlib.h>

#define PI 3.14159265359
#define vel 3000		//最高速度(m/h)
#define acc	1500		//最大加速度(m/h/s)

//ypspurとの通信の初期化
//ypspur coordinaterとの通信を開始する

int initSpur(void){
	// Windows環境で標準出力がバッファリングされないように設定
	setvbuf(stdout, 0, _IONBF, 0);

	// 初期化
	if (Spur_init() < 0)
	{
		fprintf(stderr, "ERROR : cannot open spur.\n");
		return -1;
	}

	std::cout << "Spur initialized\n\n";

	Spur_set_vel(vel / 3600);		//速度0.3m/sec
	Spur_set_accel(acc / 3600);	//加速度（m/s/s）
	Spur_set_angvel(90 * PI / 180);	//角速度（rad/s)
	Spur_set_angaccel(180 * PI / 180);		//角加速度（rad/s/s)

	
	return 0;
}

void RunControl(){

	char buf[512];
	int num, mode;

	int loop_count = 0;

	int Obstacle_state = 0;

	//目的地の座標（GL）
	double tar_x_GL, tar_y_GL, tar_th_GL;
	//目的地の座標（LC）
	double tar_x_LC, tar_y_LC, tar_th_LC;
	//前の点の座標を記録
	double before_x_GL = 0.0f, before_y_GL = 0.0f, before_th_GL = 0.0f;

	//左右のLCの値(ｙ)の制限 左：正（+）　右：負（-）
	double PassibleRange_left = 1.0, PassibleRange_right = -1.0;

	//エンコーダ値取得用の変数
	double x_GL = 0.0f, y_GL = 0.0f, th_GL = 0.0f;
	double x_LC = 0.0f, y_LC = 0.0f, th_LC = 0.0f;


	//各座標系を原点に設定
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

	//バックグラウンドでコントローラは起動しておく
	if (system("start ../Debug/MS_Controller.exe")){
		std::cout << "controller open error....\n";
	}
	else{ std::cout << "cotroller Open\n"; }

	
	//YPSpurとの通信を開始する
	if (initSpur())
	{
		return(-1);
	}

	//キー入力を待つ
	std::cout << "\nHit key to start";
	getchar();

	//走行制御ループに突入
	RunControl();


	

	return 0;
}

