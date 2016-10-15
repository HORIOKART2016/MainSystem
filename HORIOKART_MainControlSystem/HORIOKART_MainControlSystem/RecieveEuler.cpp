

#include "stdafx.h"
#include <ypspur.h>
#include <iostream>
#include <math.h>
#include <windows.h>
#include <stdlib.h>

#define PI 3.14159265359

//�R���g���[���pArduino��COM�|�[�g�̎w��
#define EULERECOMPORT "\\\\.\\COM15"

HANDLE EulerhComm;


bool isInitialized = false;

//Arduino�̃n���h�����擾����
//�g���ĂȂ�
int getEulerArduinoHandle(void){
	//�V���A���|�[�g���J���ăn���h�����擾

	EulerhComm = CreateFile(_T(EULERECOMPORT), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

	if (EulerhComm == INVALID_HANDLE_VALUE){
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

	// �n���h���`�F�b�N
	if (!EulerhComm)	return;
	// �o�b�t�@�N���A
	memset(sendbuf, 0x00, sizeof(sendbuf));
	// �p�P�b�g�쐬
	sendbuf[0] = (unsigned char)1;
	// �ʐM�o�b�t�@�N���A
	PurgeComm(EulerhComm, PURGE_RXCLEAR);
	// ���M
	ret = WriteFile(EulerhComm, &sendbuf, 1, &len, NULL);


	// �o�b�t�@�N���A
	memset(receive_data, 0x00, sizeof(receive_data));
	// �ʐM�o�b�t�@�N���A
	PurgeComm(EulerhComm, PURGE_RXCLEAR);
	// Arduino����f�[�^����M
	ret = ReadFile(EulerhComm, &receive_data, 20, &len, NULL);
	//cout << static_cast<bitset<8>>(receive_data[0]) << "," << static_cast<bitset<8>>(receive_data[1] )<< endl;


	//����������Ă��Ȃ���Ώ�����(���߂̃f�[�^���̂Ă�)
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