// TeachingSystem_HORIOKART.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

/*
経路データ作成プログラム：
走行経路のデータをティーチングにより作成するためのプログラム

ルートフォーマット：csv
pointnum, mode, GL_x, GL_y, GL_th

mode	1:直進
		2:その場回転
*/

#pragma warning(disable:4996)

#include "stdafx.h"

#include <iostream>
#include <ypspur.h>

#include <math.h>
#include <windows.h>
#include <stdlib.h>

//コントローラ用ArduinoのCOMポートの指定
#define COMPORT "\\\\.\\COM15"


const char *filename = "SampleRoute.csv";




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



int main(int argc, _TCHAR* argv[])
{
	double x_GL = 0.0, y_GL = 0.0, th_GL = 0.0;
	double x_LC = 0.0, y_LC = 0.0, th_LC = 0.0;
	double x_GL_b, y_GL_b, th_GL_b;

	int num = 0;
	int ret;

	FILE *rt;

	//コントローラーの起動
	if (system("start C:/Users/user/Desktop/つくばチャレンジ2016/HRIOKART2016/controller/HORIOKART_Controller/Debug/HORIOKART_Controller.exe")){
		std::cout << "controller open error....\n";
	}

	std::cout << "start\n";
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

		//直進したとき:20cm以上移動した
		if (sqrt((x_LC*x_LC) + (y_LC*y_LC)) > 0.2){
			fprintf(rt, "%d,%d,%lf,%lf,%lf\n", num, 1, x_GL, y_GL, th_GL);
		}

		//回転だけのとき

		num++;

		Spur_set_pos_LC(0.0, 0.0, 0.0);
		x_GL_b = x_GL; y_GL_b = y_GL; th_GL_b = th_GL;

		
	}

	fclose(rt);

	std::cout << "Program End\n";

	return 0;
}

