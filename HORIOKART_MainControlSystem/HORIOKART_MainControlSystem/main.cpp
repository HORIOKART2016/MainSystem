// HORIOKART_MainControlSystem.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include <ypspur.h>
#include <iostream>
#include <math.h>
#include <windows.h>
#include <stdlib.h>

#define PI 3.14159265359

//コントローラ用ArduinoのCOMポートの指定
#define COMPORT "\\\\.\\COM15"


//ルートのファイル名
const char *routefile = "SampleRoute.csv";
FILE *rt;

#define vel 3000		//最高速度(m/h)
#define acc	1500		//最大加速度(m/h/s)

//試しに時間計測してみる
LARGE_INTEGER freq;
LARGE_INTEGER start, now;


//試しにトルク記録してみる
FILE *trq;
const char *torq_record = "TorqRecord.csv";


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

	//トルク記録用のファイルオーぷん
	trq = fopen(torq_record, "w");
	fprintf(trq, "time,fase,right_torque,left_torque,x,y,th\n");

	return 0;
}



//Arduinoのハンドルを取得する
//使ってない
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



//非常停止の判断する関数
//(非常停止の指令は別のプログラムから出される)
void EmergencyButtonState(double x, double y, double th){
	
	double RightAngVel, LeftAngVel;
	double Avel;
	int emergency_time_count;

	YP_get_wheel_vel(&RightAngVel, &LeftAngVel);
	
	Avel = (RightAngVel + LeftAngVel) / 2;

	//回転数が非常に小さい場合にループに入る
	while(Avel < 0.001){
		std::cout << "emergency stop?\n";
		emergency_time_count++;
		if (emergency_time_count>10){
			//1秒間止まったままだと非常停止だと判断する

			std::cout << "I think emergency stop now!!\n";
			std::cout << "If start again, Please hit key!!\n";
			getchar();

			//再開する場合サイド指令を送る
			Spur_line_LC(x, y, th);

			break;
		}
		
		Sleep(100);

	}

}



//試しにトルクを取得してみる
//とりあえずはcsvにためるだけ
//何かに使うかもしれない(空転検知等)
void RecordTorq(int num){
	
	double R_torq, L_torq,x,y,th;

	YP_get_wheel_torque(&R_torq, &L_torq);
	Spur_get_pos_GL(&x, &y, &th);

	QueryPerformanceCounter(&now);

	fprintf(trq, "%lf,%d,%lf,%lf,%lf,%lf,%lf\n", (double)((now.QuadPart - start.QuadPart) / freq.QuadPart), num, R_torq, L_torq, x, y, th);


}




//走行制御用のメインループ
void RunControl_mainloop(void){

	char buf[512];
	int num, mode;

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
	
	//一応時間計測：開始
	QueryPerformanceCounter(&start);

	std::cout << "runcontrol start\n";


	//ループの開始
	while (fgets(buf, 5412, rt) != NULL){
		
		//目標点の読み込み
		sscanf(buf, "%d,%d,%lf,%lf,%lf",&num,&mode,&tar_x_GL,&tar_y_GL,&tar_th_GL);


		std::cout << "num:" << num << "\n";
		std::cout << "target:" << tar_x_GL << "," << tar_y_GL << "," << tar_th_GL << "\n";


		//駆動指令
		Spur_line_GL(tar_x_GL, tar_y_GL, tar_th_GL);

		//LCでの目標座標も取得しておく
		tar_x_LC = tar_x_GL - before_x_GL;
		tar_y_LC = tar_y_GL - before_y_GL;
		tar_th_LC = 0.0;


		//直線上に到達したかのループ
		while (!Spur_near_ang_GL(tar_th_GL, 0.1)){
			
			//到達するまでは障害物検知・緊急停止・トルク計測のみを行う
			EmergencyButtonState(tar_x_LC, tar_y_LC, tar_th_LC);
			
			//トルクの計測（お試し)
			RecordTorq(num);
			
			//回避指令

			Sleep(10);
		}
	

		//角度が到達したらLCをセットする
		Spur_get_pos_GL(&x_GL, &y_GL, &th_GL);
		Spur_set_pos_LC(x_GL - before_x_GL, y_GL - before_y_GL, tar_th_GL - th_GL);		//角度の計算が？
		

		//到達したかのループ　:　少し手前で曲がり始める
		while (!Spur_over_line_LC(tar_x_LC - 0.3, tar_y_LC, tar_th_LC)){

			//ここに各センサからのフィードバックを入れる
			
			//緊急停止の状態取得
			EmergencyButtonState(tar_x_GL, tar_y_GL, tar_th_GL);

			//トルクの計測（お試し)
			RecordTorq(num);

			//駆動指令を修正する場合の動作をここに挿入

			Sleep(10);
		}





		Spur_get_pos_GL(&x_GL, &y_GL, &th_GL);
		Spur_get_pos_LC(&x_LC, &y_LC, &th_LC);

	
		std::cout << "  GL  :" << x_GL << "," << y_GL << "," << th_GL << "\n";
		std::cout << "  LC  :" << x_LC << "," << y_LC << "," << th_LC << "\n\n";



		//LCを原点にする
		Spur_set_pos_LC(0.0, 0.0, 0.0);

		//前の目的座標を記録しておく
		before_x_GL = tar_x_GL;
		before_y_GL = tar_y_GL;
		before_th_GL = tar_th_GL;

		//次の経路へ
	}

	//すべての経路が終了するとここに到達する
	std::cout << "All Route Complete!!\n";

	Spur_stop();

}



int main(void)
{
	QueryPerformanceFrequency(&freq);

	//経路データを開く
	if ((rt = fopen(routefile, "r")) == NULL){
		//出来れば1度読みこんで有効な経路が入っているかどうかを確認する（今後の課題）

		printf("Can't open Route file.....\n");
		return -1;
	}


	//バックグラウンドでコントローラは起動しておく
	if (system("start MS_Controller.exe")){
		std::cout << "controller open error....\n";
	}
	else{ std::cout << "cotroller Open\n"; }
	
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


	//デバッグ用
	//トルク計測ファイルの保存
	fclose(trq);

	return 0;
}

