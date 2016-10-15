

#include "stdafx.h"
#include <ypspur.h>
#include <iostream>
#include <math.h>
#include <windows.h>
#include <stdlib.h>

#define PI 3.14159265359

//コントローラ用ArduinoのCOMポートの指定
#define EULERECOMPORT "\\\\.\\COM15"

HANDLE EulerhComm;


bool isInitialized = false;

//Arduinoのハンドルを取得する
//使ってない
int getEulerArduinoHandle(void){
	//シリアルポートを開いてハンドルを取得

	EulerhComm = CreateFile(_T(EULERECOMPORT), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

	if (EulerhComm == INVALID_HANDLE_VALUE){
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
		GetCommState(EulerhComm, &lpTest);
		lpTest.BaudRate = 9600;
		lpTest.ByteSize = 8;
		lpTest.Parity = NOPARITY;
		lpTest.StopBits = ONESTOPBIT;
		SetCommState(EulerhComm, &lpTest);
	}
	return 0;
}


void getEulerStatus(double *Euler){

	int ret;
	unsigned char	sendbuf[1];
	unsigned char	receive_data[30] = {};
	char *euler_1, *euler_2, *euler_3;
	float Euler[3];
	
	unsigned long	len;

	// ハンドルチェック
	if (!EulerhComm)	return;
	// バッファクリア
	memset(sendbuf, 0x00, sizeof(sendbuf));
	// パケット作成
	sendbuf[0] = (unsigned char)1;
	// 通信バッファクリア
	PurgeComm(EulerhComm, PURGE_RXCLEAR);
	// 送信
	ret = WriteFile(EulerhComm, &sendbuf, 1, &len, NULL);


	// バッファクリア
	memset(receive_data, 0x00, sizeof(receive_data));
	// 通信バッファクリア
	PurgeComm(EulerhComm, PURGE_RXCLEAR);
	// Arduinoからデータを受信
	ret = ReadFile(EulerhComm, &receive_data, 20, &len, NULL);
	//cout << static_cast<bitset<8>>(receive_data[0]) << "," << static_cast<bitset<8>>(receive_data[1] )<< endl;


	//初期化されていなければ初期化(初めのデータを捨てる)
	if (!isInitialized)
	{
		isInitialized = true;
		//return ;
	}
	euler_1 = strtok((char*)receive_data, ",");
	euler_2 = strtok(NULL, ",");
	euler_3 = strtok(NULL, ",");

	Euler[0] = strtod(euler_1, NULL);
	Euler[1] = strtod(euler_2, NULL);
	Euler[2] = strtod(euler_3, NULL);

}