// HORIOKART_MainControlSystem.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include <ypspur.h>
#include <iostream>
#include <math.h>
#include <windows.h>
#include <stdlib.h>

//コントローラ用ArduinoのCOMポートの指定
#define COMPORT "\\\\.\\COM15"


//ルートのファイル名
const char *routefile = "SampleRoute.csv";
FILE *rt;

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

	return 0;
}



//Arduinoのハンドルを取得する

int getArduinoHandle(HANDLE& hComm){
	//シリアルポートを開いてハンドルを取得

	hComm = CreateFile(_T(COMPORT), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

	if (hComm == INVALID_HANDLE_VALUE){
		printf("シリアルポートを開くことができませんでした。");
		char z;
		z = getchar();
		return -1;
	}
	//ポートを開けていれば通信設定を行う
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

	//各座標系を原点に設定
	Spur_set_pos_GL(0.0, 0.0, 0.0);
	Spur_set_pos_LC(0.0, 0.0, 0.0);

	//最高速度の設定(m/s)
	//Spur_set_vel(vel / 3600);
	//加速度の設定(m/s･s)
	//Spur_set_accel(acc / 3600);

	Spur_set_vel(0.3);		//速度0.3m/sec
	Spur_set_accel(1.0);	//加速度（m/s/s）
	Spur_set_angvel(1.5);	//角速度（rad/s)
	Spur_set_angaccel(2.0);		//角加速度（rad/s/s)

	std::cout << "runcontrol start\n";

	//ループの開始
	while (fgets(buf, 5412, rt) != NULL){
		//目標点の読み込み
		sscanf(buf, "%d,%d,%lf,%lf,%lf",&num,&mode,&tar_x,&tar_y,&tar_th);

		std::cout << "num:" << num << "\n";
		std::cout << "target:" << tar_x << "," << tar_y << "," << tar_th << "\n";

		Spur_line_GL(tar_x, tar_y, tar_th);

		//到達したかのループ
		while (!Spur_over_line_GL(tar_x, tar_y, tar_th)){

			//ここに各センサからのフィードバックを入れる


			//駆動指令を修正する場合の動作をここに挿入

			Sleep(10);
		}
		//目標地点付近に到達したら停止
		Spur_stop();

		Spur_get_pos_GL(&x_GL, &y_GL, &th_GL);
		Spur_get_pos_LC(&x_LC, &y_LC, &th_LC);

		std::cout << "num:" << num << "\n";
		std::cout << "target:" << tar_x << "," << tar_y << "," << tar_th << "\n";
		std::cout << "  GL  :" << x_GL << "," << y_GL << "," << th_GL << "\n";
		std::cout << "  LC  :" << x_LC << "," << y_LC << "," << th_LC << "\n\n";



		//LCを原点にする
		Spur_set_pos_LC(0.0, 0.0, 0.0);

		//次の経路へ
	}

	//すべての経路が終了するとここに到達する
	std::cout << "All Route Complete!!\n";

	Spur_stop();

}


int main(void)
{
		
	//経路データを開く
	if ((rt = fopen(routefile, "r")) == NULL){
		//出来れば1度読みこんで有効な経路が入っているかどうかを確認する（今後の課題）

		printf("Can't open Route file.....\n");
		return -1;
	}


	//バックグラウンドでコントローラは起動しておく
	/*if (system("start C:/Users/user/Desktop/つくばチャレンジ2016/HRIOKART2016/controller/HORIOKART_Controller/Debug/HORIOKART_Controller.exe")){
		std::cout << "controller open error....\n";
	}
	else{ std::cout << "cotroller Open\n"; }
	*/
	//コントローラを動かしてると途中で止める指令入れちゃうので仕様変更が必要

	//ここに障害物検知用のURGのハンドル等が入る



	//YPSpurとの通信を開始する
	if (initSpur())
	{
		return(-1);
	}

	//キー入力を待つ
	std::cout << "\nHit key to start";
	getchar();

	//走行制御ループに突入
	RunControl_mainloop();

	return 0;
}

