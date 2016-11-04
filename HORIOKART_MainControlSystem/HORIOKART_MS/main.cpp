// HORIOKART_MainControlSystem.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
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


//ルートのファイル名
const char *routefile = "1029_1.csv";
//const char *routefile = "../../TeachingSystem_HORIOKART/TeachingSystem_HORIOKART/SampleRoute.csv";
FILE *rt;

#define vel 2500		//最高速度(m/h)
#define acc	1500		//最大加速度(m/h/s)

//試しに時間計測してみる
LARGE_INTEGER freq;
LARGE_INTEGER start, now;


//試しにトルク記録してみる
FILE *trq;
const char *torq_record = "TorqRecord.csv";

extern int init_URG();			//urgのinitialize
extern int obstacle_detection();

extern int Euler_state(void);

extern int initCamera(void);

#define detect 1			//1で有効　0無効

extern void Detect_RoadEdge(double *edge);
extern void init_Euler(void);


//目的地の座標（LC）
double tar_x_LC, tar_y_LC, tar_th_LC;

//左右のLCの値(ｙ)の制限 左：正（+）　右：負（-）
double PassibleRange_left = 1.5, PassibleRange_right = -1.5;
double RoadEdge_buf1[2] = { 0.0, 0.0 }, RoadEdge_buf2[2] = { 0.0, 0.0 };

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

	/*Spur_set_vel(vel / 3600);		//速度0.3m/sec
	Spur_set_accel(acc / 3600);	//加速度（m/s/s）
	Spur_set_angvel(90 * PI / 180);	//角速度（rad/s)
	Spur_set_angaccel(180 * PI / 180);		//角加速度（rad/s/s)
	*/

	Spur_set_vel(0.3);		//速度0.3m/sec
	Spur_set_accel(1.0);	//加速度（m/s/s）
	Spur_set_angvel(0.5);	//角速度（rad/s)
	Spur_set_angaccel(1.5);		//角加速度（rad/s/s)
	
	
	return 0;
}



int initialize(){

	//経路データを開く
	fopen_s(&rt, routefile, "r");
	/*if ((rt = fopen(routefile, "r")) == NULL){
		//出来れば1度読みこんで有効な経路が入っているかどうかを確認する（今後の課題）

		printf("Can't open Route file.....\n");
		return 1;
	}*/


	//バックグラウンドでコントローラを起動しておく
	if (system("start ../Debug/MS_Controller.exe")){
		std::cout << "controller open error....\n";
	}
	else{ std::cout << "cotroller Open\n\n"; }

	init_Euler();

	//initCamera();

	//障害物検知用のURGとの通信の開始
	if (init_URG()){
		std::cout << "URG Error...\n";
		return 1;
	}
	else{
		std::cout << "URG for Detect : Open\n\n";
	}




	//YPSpurとの通信を開始する
	if (initSpur())
	{
		return 1;
	}



	//トルク記録用のファイルオープン
	fopen_s(&trq, torq_record, "w");
	fprintf(trq, "TimeStamp,fase,mode,right_torque,left_torque,x,y,th,xLC,y_LC,th_LC\n");




	return 0;



}

/*------ここまでがinitialize------*/


void finalise(void){

	//デバッグ用
	//トルク計測ファイルの保存
	fclose(trq);

}


//非常停止の判断する関数
//(非常停止の指令は別のプログラムから出される)
int EmergencyButtonState(void){
	
	double RightAngVel, LeftAngVel;
	double Avel;
	int emergency_time_count=0;

	YP_get_wheel_vel(&RightAngVel, &LeftAngVel);
	
	Avel = (abs(RightAngVel) + abs(LeftAngVel)) / 2;

	//std::cout << "r:" << RightAngVel << "L:" << LeftAngVel << "\n";

	//回転数が非常に小さい場合にループに入る
	while(Avel < 0.01){
		YP_get_wheel_vel(&RightAngVel, &LeftAngVel);
		Avel = (abs(RightAngVel) + abs(LeftAngVel)) / 2;
		//std::cout << "r:" << RightAngVel << "L:" << LeftAngVel << "\n";
		//std::cout << Avel;
		std::cout << "emergency stop?\n";
		emergency_time_count++;
		if (emergency_time_count>5){
			//1秒間止まったままだと非常停止だと判断する

			std::cout << "I think emergency stop now!!\n";
			std::cout << "If start again, Please hit key!!\n";
			
			if (getchar() == 'q')
			{
				finalise();
				std::cout << "file saved";
				return 1;
			}

			//再開する場合サイド指令を送る
			Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);

			break;
		}
		
		Sleep(1000);

	}
	return 0;

}



//回避不能の障害物に対するループ
int Unvoidable_Obstacle(){


	Spur_stop();		//回避不能の障害物を発見した場合直ちに停止する

	int obstacle_state=8;

	std::cout << "Detect obstacle!!\n\n";

	while (1){
		//障害物の確認
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

//障害物検知
//書き換えby小宮　20161104
//項目
//1:緊急停止の条件変更

int run_Obstacledetection(void){
	
	int obstacle_state;

	obstacle_state = obstacle_detection();	//障害物のステータスの取得
	//std::cout << obstacle_state << "\n";
	//回避不能の障害物を検知したとき停止してループに突入する
	if (obstacle_state == 1 || obstacle_state == 2 ){
		Unvoidable_Obstacle(); 
		Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
	}
	

	switch (obstacle_state){
		case 6:
			Unvoidable_Obstacle();
			Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
			break;
		case 3:
		case 7:
			if (abs(PassibleRange_left) < 0.3){
				Unvoidable_Obstacle();
				Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
			}
			else{
				tar_y_LC = tar_y_LC + 0.3;
				Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
			}
		case 4:
		case 8:
			if (abs(PassibleRange_right) < 0.3){
				Unvoidable_Obstacle();
				Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
			}
			else{
				tar_y_LC = tar_y_LC - 0.3;
				Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
			}

	}

	return 0;

}


//試しにトルクを取得してみる
//とりあえずはcsvにためるだけ
//何かに使うかもしれない(空転検知等)
void RecordTorq(int num,int mode){
	
	double R_torq, L_torq,x,y,th;
	double x_LC, y_LC, th_LC;

	YP_get_wheel_torque(&R_torq, &L_torq);
	Spur_get_pos_GL(&x, &y, &th);
	Spur_get_pos_LC(&x_LC, &y_LC, &th_LC);

	QueryPerformanceCounter(&now);

	fprintf(trq, "%lf,%d,%d,%lf,%lf,%lf,%lf,%lf%lf,%lf,%lf\n", (((double)now.QuadPart - (double)start.QuadPart) / (double)freq.QuadPart),
																							num, mode, R_torq, L_torq, x, y, th, x_LC, y_LC, th_LC);

	std::cout << "POS : " << x << " , " << " , " << y << " , " << th << "\n";
	std::cout << "POSLC : " << x_LC << " , " << " , " << y_LC << " , " << th_LC << "\n";


}



void RoadEdge_syori(void){
	
	double RoadEdge[2];

	Detect_RoadEdge(RoadEdge);
	
	//例外処理：前２回の結果から1次関数を取得（2個前が0その次が0.5）
	//その直線との距離を導出し閾値を用いて例外処理
	//bufは1が手前　2がその0.5ｍ先（こっちが近い）


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


//走行制御用のメインループ
void RunControl_mainloop(void){

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
	double PassibleRange_left = 1.5, PassibleRange_right = -1.5;
	
		
	//エンコーダ値取得用の変数
	double x_GL = 0.0f, y_GL = 0.0f, th_GL = 0.0f;
	double x_LC = 0.0f, y_LC = 0.0f, th_LC = 0.0f;

	int over = 0;
	int online = 0;			//目標点・角度のなす直線上にいるか　　0：false 1:true

	double border = 0.5;

	int border_count = 0;

	int roadmode;


	//各座標系を原点に設定
	Spur_set_pos_GL(0.0, 0.0, 0.0);
	Spur_set_pos_LC(0.0, 0.0, 0.0);
	
	//一応時間計測：開始
	QueryPerformanceCounter(&start);

	std::cout << "runcontrol start\n";


	//ルートをたどるループの開始
	while (fgets(buf, 5412, rt) != NULL){
		
		//目標点の読み込み
		//文法：番号，ｘ，ｙ，画像処理の有無
		sscanf_s(buf, "%d,%lf,%lf,%d", &num, &tar_x_GL, &tar_y_GL, &roadmode);
		
		
		tar_th_GL = atan((tar_y_GL - before_y_GL) / (tar_x_GL - before_x_GL));
		
		std::cout << "num:" << num << "\n";
		
		
		
		//LCでの目標座標も取得しておく
		tar_x_LC = sqrt((tar_x_GL - before_x_GL)*(tar_x_GL - before_x_GL) + (tar_y_GL - before_y_GL)*(tar_y_GL - before_y_GL));
		tar_y_LC = 0.0;
		tar_th_LC = 0.0;

		//
		std::cout << "targetLC:" << tar_x_LC << "," << tar_y_LC << "," << tar_th_LC << "\n";
		
		
		//自己位置のLCのリセット
		Spur_get_pos_GL(&x_GL,&y_GL,&th_GL);
		Spur_set_pos_LC(x_GL - before_x_GL, y_GL - before_y_GL, th_GL - tar_th_GL);

		//これ以下ではすべてLCで座標を扱う

		//駆動指令
		Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
		

					
		while (!Spur_over_line_LC(tar_x_LC - 0.5, tar_y_LC, tar_th_LC)){
			
			//各センサからのステータス

			//緊急停止の状態取得：緊急停止中にqが押されると1が返され処理を終了する
			if (EmergencyButtonState())
				return;

			//障害物の位置検知
			if (detect)
				run_Obstacledetection();

			//直線状かどうか
			if (!online){
				online = Spur_near_ang_LC(tar_th_LC, 0.1);
			}
			
			//傾斜・回転角の補正
			Euler_state();


			//トルクの計測（お試し)
			RecordTorq(num, 2);


			//駆動指令を修正する場合の動作をここに挿入
			//0.5m進む度に画像取得・2ｍごとに駆動指令を入れなおす
			if (Spur_over_line_LC(border, tar_y_LC, tar_th_LC)){
				border_count++;
				if (roadmode)
					RoadEdge_syori();

				//片側に寄ってるとき:目的地のｙ座標を変更して駆動指令を入れなおす
				if ((abs(PassibleRange_left) > 0.01) && (abs(PassibleRange_left)<0.3)){
					tar_y_LC = tar_y_LC-0.5;
					Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
				}
				
				else if ((abs(PassibleRange_right) > 0.01) && (abs(PassibleRange_right)<0.3)){
					tar_y_LC = tar_y_LC+0.5;
					Spur_line_LC(tar_x_LC, tar_y_LC, tar_th_LC);
				}

				//2mごとに指令を入れ直す
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



		//前の目的座標を記録しておく
		before_x_GL = tar_x_GL;
		before_y_GL = tar_y_GL;
		before_th_GL = tar_th_GL;

		//次の経路へ
		over = 0;
		online = 0;
		border = 0.5;
		border_count = 0;

		PassibleRange_left = 0.0;
		PassibleRange_right = 0.0;

	}

	//すべての経路が終了するとここに到達する
	std::cout << "All Route Complete!!\n";

	Spur_stop();


	//終了処理
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

	//キー入力を待つ
	std::cout << "\nHit key to start";
	getchar();

	//走行制御ループに突入
	RunControl_mainloop();


	return 0;
}

