
#define _CRT_SECURE_NO_WARININGS
#include <stdio.h>
#include <tchar.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <fstream>
#include <Windows.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <ypspur.h>

using namespace std;

HANDLE EulerhComm = NULL;


HANDLE init_Euler_arduino(int arduinoCOM, HANDLE hComm)
{
	string com = "\\\\.\\COM" + to_string(arduinoCOM);
	hComm = CreateFile(com.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hComm == INVALID_HANDLE_VALUE){
		printf("シリアルポートを開くことができませんでした。");
		char z;
		z = getchar();
		return 0;
	}
	//ポートを開けていれば通信設定を行う
	else
	{
		DCB lpTest;
		GetCommState(hComm, &lpTest);
		lpTest.BaudRate = 115200;
		lpTest.ByteSize = 8;
		lpTest.Parity = NOPARITY;
		lpTest.StopBits = ONESTOPBIT;
		SetCommState(hComm, &lpTest);
	}
	printf("opened!!\n");

	unsigned char	sendbuf[1];
	int				ret;
	unsigned long	len;
	unsigned char	receive_data[30] = {};

	//Sleep(3000);

	// バッファクリア
	memset(sendbuf, 0x00, sizeof(sendbuf));
	// パケット作成
	sendbuf[0] = (unsigned char)1;
	// 通信バッファクリア
	PurgeComm(hComm, PURGE_RXCLEAR);
	// 送信
	ret = WriteFile(hComm, &sendbuf, 5, &len, NULL);

	memset(receive_data, 0x00, sizeof(receive_data));
	// 通信バッファクリア
	PurgeComm(hComm, PURGE_RXCLEAR);
	// Arduinoからデータを受信
	ret = ReadFile(hComm, &receive_data, 1, &len, NULL);

	Sleep(3000);
	return hComm;
}



void tile_cal(float Euler[3], double d_t[2])
{
	for (int i = 0; i < 3; i++)
	{
		Euler[i] = Euler[i] * M_PI / 180;
	}


	double x, y, z;
	x = abs(tan((double)Euler[2])) / sqrt(pow(sin((double)Euler[1]), 2) + pow(tan((double)Euler[2]), 2));
	y = abs(x*sin((double)Euler[2]));
	z = y*abs(cos((double)Euler[1])) + abs(sin(acos(x)))*abs(sin((double)Euler[1]));


	if (Euler[1] <= 0 && Euler[2] <= 0)
	{
		d_t[0] = acos(x);
		d_t[1] = asin(z);
	}
	else if (Euler[1] <= 0 && Euler[2] > 0)
	{
		d_t[0] = M_PI - acos(x);
		d_t[1] = asin(z);
	}
	else if (Euler[1] > 0 && Euler[2] > 0)
	{
		d_t[0] = acos(x) - M_PI;
		d_t[1] = asin(z);
	}
	else if (Euler[1] > 0 && Euler[2] <= 0)
	{
		d_t[0] = -acos(x);
		d_t[1] = asin(z);
	}
	printf("%lf", 2 * M_PI - Euler[0]);
	printf("(Φ,θ) = (%lf,%lf)\n", d_t[0], d_t[1]);
}



void receive_euler(HANDLE hComm, float Euler[3])
{

	DWORD start, end;
	start = GetTickCount();
	bool isInitialized = false;
	unsigned char	sendbuf[1];
	unsigned char	receive_data[30] = {};
	int				ret;
	float			DL, DR, DIS, ANG;
	unsigned long	len;
	char *euler_1, *euler_2, *euler_3;
	int i = 0;
	float			droidOrientation[3];


	end = GetTickCount();
	if ((end - start) < 15)
	{
		int sleeptime = 15 - (int)(end - start);
		Sleep(sleeptime);
		end = GetTickCount();
	}

	if (!hComm)	return;
	// バッファクリア
	memset(sendbuf, 0x00, sizeof(sendbuf));
	// パケット作成
	sendbuf[0] = (unsigned char)1;
	// 通信バッファクリア
	PurgeComm(hComm, PURGE_RXCLEAR);
	// 送信
	ret = WriteFile(hComm, &sendbuf, 1, &len, NULL);


	// バッファクリア
	memset(receive_data, 0x00, sizeof(receive_data));
	// 通信バッファクリア
	PurgeComm(hComm, PURGE_RXCLEAR);
	// Arduinoからデータを受信
	ret = ReadFile(hComm, &receive_data, 20, &len, NULL);
	//cout << static_cast<bitset<8>>(receive_data[0]) << "," << static_cast<bitset<8>>(receive_data[1] )<< endl;





	//初期化されていなければ初期化(初めのデータを捨てる)
	if (!isInitialized)
	{
		isInitialized = true;
		//return ;
	}

	printf("%s...\n", receive_data);
	euler_1 = strtok((char*)receive_data, ",");
	euler_2 = strtok(NULL, ",");
	euler_3 = strtok(NULL, ",");


	Euler[0] = strtod(euler_1, NULL);
	Euler[1] = strtod(euler_2, NULL);
	Euler[2] = strtod(euler_3, NULL);
	printf("%f\n", Euler[0]);
	printf("%f\n", Euler[1]);
	printf("%f\n", Euler[2]);
}

HANDLE hComm = NULL;

void init_Euler(void){
	int arduinoCOM = 71;
	
	//シリアルポートを開いてハンドルを取得
	hComm = init_Euler_arduino(arduinoCOM, hComm);
}



float Euler[3];
double d_t[2];


//main
int Euler_state(void)
{
	receive_euler(hComm, Euler);
	

	tile_cal(Euler, d_t);

	double x, y, rad;
	
	//回転角補正
	Spur_get_pos_GL(&x, &y, &rad);
	Spur_adjust_pos_GL(x, y, 2 * M_PI - Euler[0]);
	
	//傾斜補正
	Spur_tilt_FS(d_t[0], d_t[1]);

	return 0;
}



