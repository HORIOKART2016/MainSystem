#ifndef _INC_URG_UNKO
#define _INC_URG_UNKO

#include "urg_sensor.h"
#include "urg_utils.h"
#include "urg_open.h"
#include "pcimage.h"

#include <Windows.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>

using namespace std;

#define DEBUG_WRITELINE


//接続したURGの個数を自動で判断するようにしたマクロ
#define getDataUNKO(aURGCOM , aURGPOS , ARDUINOCOM) getDataUNKOOrigin( (aURGCOM),(aURGPOS),(ARDUINOCOM),sizeof((aURGCOM))/sizeof(aURGCOM[0])) 

//指定したCOMポートを閉じる
int CommClose(HANDLE hComm);
//Arduinoのハンドルを取得
void getArduinoHandle(int arduinoCOM, HANDLE& hComm);
//urg_unkoのmainループ
void getDataUNKOOrigin(int URG_COM[], float URGPOS[][4], int ARDUINO_COM, int NumOfURG);

class writePCD:ofstream
{
private:
	//std::ofstream ofs;	//ファイルストリームオブジェクト．pcdファイル作成に用いる
	static int pcdnum;			//pcdファイルの番号をカウントする変数
	int pcdcount;		//pcdファイルに書き込む点の数をカウントする変数

	std::string dirname;

public:
	bool isWritePCD;

	writePCD(std::string dirName = "");
	~writePCD();
	//pcdファイルを作成して書き込む準備を行う
	void pcdinit();
	void pcdinit3D();
	//pcdファイルへ点を書き込む
	void pcdWrite(float x, float y);
	void pcdWrite(float x, float y, float z);
	void pcdWrite(float x, float y, float pos_x, float pos_y);
	void pcdWrite(float x, float y, float pos_x, float pos_y, float dist, float rad);
	void pcdWrite3D(float x, float y, float pos_x, float pos_y, float dist, float rad);
	void pcdWrite(float x, float y, float pos_x, float pos_y, float droidAngle[], float droidGPS[]);
	//pcdファイルへの書き込みを終了して保存する
	void pcdSave();
	void setDirName(std::string dirname);
};


/*
*
*　URGでデータを取得してマッピングを行うクラス
*
*/
class urg_unko
{
protected:
	/*
	*	メンバ変数
	*/
	int COMport;	//URGのCOMポート

	int COMport3D;

	//float urgpos[3];	
	//NCWCの回転中心から見たURGの位置．センサの地面からの高さ，センサの基準位置からの距離，および水平面からの俯角

	float urgpos[4];	
	//NCWCの回転中心から見たURGの位置．センサの地面からの高さ，センサの基準位置からのx方向の距離，ｙ方向の距離，および水平面からの俯角
	
	float urgpos3D[4];

	urg_t urg;			//URGオブジェクト
	long *data = NULL;	
	long time_stamp;

	float scaninterval = 0.0;//計測を実施する最低間隔[mm]

	enum {
		CAPTURE_TIMES = 1,
	};

	float currentCoord_x = 0.0, currentCoord_y = 0.0;//車椅子の座標
	float distance = 0.0, distance_old = 0.0;//車椅子の移動距離
	float rob_x= 0.0, rob_y= 0.0;//spurの座標
	float radian = 0.0;

	float* pointpos[2];//URG測定点
	float* rawpointpos[2];
	int data_n;

	static writePCD pcd;

	/***********************
	 *	privateなメソッド  *
	 ***********************/

	//URGとの接続を切断
	int disconnectURG();
	//URGと接続
	int connectURG();

	//取得したデータから実際の二次元情報を計算してマップ，pcdファイルへの書き込みを行う
	void calcSurface2D();

	//取得したデータから実際の三次元情報を計算して三次元マップ，pcdファイルへの書き込みを行う
	void calcSurface3D();

public:
	/*
	*	publicなメソッド
	*/
	//コンストラクタ
	urg_unko();
	//デストラクタ
	virtual ~urg_unko();

	//自身の初期化処理を行う
	void init(int COM, float pos[]);
	void init3D(int COM, float pos[]);
	//URGからデータを取得するメソッド
	int getData4URG(float dist, float old, float rad, float spur_x, float spur_y);

	void savePCD();
	void saveRawPCD(float dist,float rad);
	void saveRawPCD3D(float dist, float rad);

	void updateCurrentCoord(float coord_x, float coord_y);
	void updateCurrentCoord(float coordXY[]);

	int getData_n();
	void getData(float data[], int data_n, int offset = 0);
};

class urg_mapping : 
	public urg_unko
{
private:
	static PCImage pcimage;	//マップ画像作成用クラス

public:
	~urg_mapping();

	void setWriteLine(bool isLine);
	std::string	getDirName();
	void setPCDDir(std::string dirname = "");

	static void initPCImage(PCImage& pci);
	static void initPCImage(int width, int height, int resolution);
	static void setPCImageOrigin(int x, int y);
	static void getPCImage(cv::Mat& m, int num = -1);

	void setPCImageColor(PCImage::BGR bgr);

	void writeMap(float dist, float old, float rad, float spur_x, float spur_y);
};

#endif