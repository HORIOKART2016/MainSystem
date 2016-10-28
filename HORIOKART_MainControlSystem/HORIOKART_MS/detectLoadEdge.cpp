#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <string>
#include <windows.h>

using namespace std;
using namespace cv;

VideoCapture cap(0);

Mat original_image; // ���摜�o�b�N�A�b�v

int PixcelRate = 95;		//1m������̃s�N�Z�����@�@pix/m

int initCamera(void){
	
	if (!cap.isOpened()){
		cout << "Camera cannot Open!!" << endl;
		return-1;
	}

	cap >> original_image;
	imshow("Image", original_image);

	return 0;
}


/*
*	�J���[�摜�̃q�X�g�O�����ψꉻ
*	�[�x3�Ȃ�Ȃ�ł�OK
*/
void equalizeColorHist(Mat& src, Mat& dst)
{
	vector<Mat> planes(3);
	vector<Mat> output_planes(3);
	vector<Mat> rgb;

	split(src, planes); // bgr

	equalizeHist(planes[0], output_planes[0]);
	equalizeHist(planes[1], output_planes[1]);
	equalizeHist(planes[2], output_planes[2]);

	rgb.push_back(output_planes[0]);
	rgb.push_back(output_planes[1]);
	rgb.push_back(output_planes[2]);

	merge(rgb, dst);
}



/*
*	�w��͈͂̉�f�l�𕽋ς���
*/
int calcAverage(Mat& src, Point center, Size size)
{
	int val = 0;
	for (int y = center.y - size.height / 2; y <= center.y + size.height / 2; y++)
	{
		for (int x = center.x - size.width / 2; x <= center.x + size.width / 2; x++)
		{
			val += src.data[y * src.cols + x];
		}
	}

	return val / (size.width*size.height);
}


/*
*	�摜���ōő�̉�f�l��Ԃ�
*/
int maxBrightness(Mat pic)
{
	int maxBrightness = 0;

	for (int y = 0; y < pic.rows - 1; y++)
	{
		for (int x = 0; x < pic.cols - 1; x++)
		{
			if (pic.data[y * pic.cols + x] > maxBrightness) maxBrightness = pic.data[y * pic.cols + x];
		}
	}
	return maxBrightness;
}


/*
*  �V�O���C�h�֐��Ɋ�Â��ăR���g���X�g����
*/
Mat contrastSigmoid(Mat &pic)
{
	float gain = 50;
	int maxbrightness = maxBrightness(pic);

	uchar lut[256];
	for (int i = 0; i < 256; i++)
	{
		lut[i] = 255.0 / (1 + exp(-gain * (i - maxbrightness / 2) / maxbrightness));
	}

	Mat p = pic.reshape(0, 1).clone();
	for (int i = 0; i < p.cols; i++)
	{
		p.at<uchar>(0, i) = lut[p.at<uchar>(0, i)];
	}
	return p.reshape(0, pic.rows).clone();

}



/*
*	��_�̉�f�l�Ɋ�Â��ē������Z�o
*	�F�������Ⴒ���Ⴕ�Ă���_��
*	(�ʓ|������c���Ŋ֐������܂���.�C����������܂Ƃ߂܂�)
*
*	����
*		Mat src : ���o�Ώۉ摜�iBGR�j
*		int retLR, retTB : �Ԃ�l���i�[����z��.[left, right]or[top, bottom]�ŕԂ�.
*		Point center : ��_.���̉�f�l�Ɋ�Â�,�������獶�E�ɕ���T������.
*		Size size : ��l�𕽋ς���͈͂�,�����T������͈�
*/
void getHeight(Mat src, int retTB[2], Point criteria, Size size = Size(10, 10))
{
	// ��ƂȂ��f�l���Z�o
	int criteria_val = calcAverage(src, criteria, size);

	// ��l�ɑ΂��鍷���v�Z
	int calc_height = src.rows - size.height;
	int calc_width = size.width;
	Mat diff_img(calc_height, calc_width, CV_8U);
	for (int y = size.height / 2; y < src.rows - size.height / 2; y++)
	{
		for (int x = criteria.x - calc_width / 2; x < criteria.x + (int)(calc_width / 2.0 + 0.5); x++)
		{
			//int val = calcAverage(pic, Point(x, y), size) - criteria_val;
			int val = src.data[y * src.cols + x] - criteria_val;
			diff_img.data[(y - size.height / 2) * diff_img.cols + x - criteria.x + calc_width / 2] = abs(val);
		}
	}
	Mat result = contrastSigmoid(diff_img); // 2�l���ł�������

	// �m�C�Y����
	morphologyEx(result, diff_img, MORPH_OPEN, Mat(), Point(-1, -1), 3);

	// ���o�Ώۂ̕����Z�o
	retTB[0] = retTB[1] = 0;
	for (int x = 0; x < diff_img.cols; x++)
	{
		int y;

		for (y = criteria.y; y > 0; y--)
		if (diff_img.data[y * diff_img.cols + x] > 200)
			break;
		retTB[0] += y;

		for (y = criteria.y; y < diff_img.rows; y++)
		if (diff_img.data[y * diff_img.cols + x] > 200)
			break;
		retTB[1] += y;
	}
	retTB[1] = retTB[1] / diff_img.cols + size.height / 2; // ���ƂŊO��l�΍􂷂�(����)
	retTB[0] = retTB[0] / diff_img.cols + size.height / 2;
}



void getWidth(Mat src, int retLR[2], Point criteria, Size size = Size(10, 10))
{
	// ��ƂȂ��f�l���Z�o
	int criteria_val = calcAverage(src, criteria, size);

	// ��l�ɑ΂��鍷���v�Z
	int calc_height = size.height;
	int calc_width = src.cols - size.width;
	Mat diff_img(calc_height, calc_width, CV_8U);
	for (int y = criteria.y - calc_height / 2; y < criteria.y + (int)(calc_height / 2.0 + 0.5); y++)
	{
		for (int x = size.width / 2; x < src.cols - size.width / 2; x++)
		{
			//int val = calcAverage(pic, Point(x, y), size) - criteria_val;
			int val = src.data[y * src.cols + x] - criteria_val;
			diff_img.data[(y - criteria.y + calc_height / 2) * diff_img.cols + x - size.width / 2] = abs(val);
		}
	}
	Mat result = contrastSigmoid(diff_img); // 2�l���ł�������

	// �m�C�Y����
	morphologyEx(result, diff_img, MORPH_OPEN, Mat(), Point(-1, -1), 3);

	// ���o�Ώۂ̕����Z�o
	retLR[0] = retLR[1] = 0;
	for (int y = 0; y < diff_img.rows; y++)
	{
		int x;

		for (x = criteria.x; x > 0; x--)
		if (diff_img.data[y * diff_img.cols + x] > 200)
			break;
		retLR[0] += x;

		for (x = criteria.x; x < diff_img.cols; x++)
		if (diff_img.data[y * diff_img.cols + x] > 200)
			break;
		retLR[1] += x;
	}
	retLR[1] = retLR[1] / diff_img.rows + size.width / 2; // ���ƂŊO��l�΍􂷂�(����)
	retLR[0] = retLR[0] / diff_img.rows + size.width / 2;
}


/*
*	�q�X�g�O�����ψꉻ�����F���摜��Ԃ�
*
*	����
*		Mat& src : ���o�Ώۉ摜�iBGR�j
*		Mat& dst : ���ʊi�[�p
*/
void getHueImage(Mat& src, Mat& dst)
{
	Mat tmp;
	// bgr�̃q�X�g�O�����ψꉻ
	equalizeColorHist(src, tmp);

	// hsv��h�̂ݒ��o
	cvtColor(tmp, dst, CV_BGR2HSV);
	vector<Mat> hsv(3);
	split(dst, hsv);
	// h�̃q�X�g�O�����ψꉻ
	equalizeHist(hsv[0], dst);
}



void Detect_RoadEdge(double *edge){

	Mat src;
	
	Mat pic;
	
	cap >> src;
	original_image = src;
	pic = src;

	// �O�������ĐF���摜�ɕϊ�
	Mat hue_img;
	getHueImage(pic, hue_img);

	// �c�����擾
	Point criteria(pic.cols / 2, pic.rows / 4); // ��_
	Size size(10, 10);
	int widthLR[2], heightTB[2]; // Left Right, Top Bottom
	getWidth(hue_img, widthLR, criteria, size);
	getHeight(hue_img, heightTB, criteria, size);

	cout << widthLR[0] << "," << widthLR[1] << endl;;

	//��O�����F�O�Q��̌��ʂ���1���֐����擾�i2�O��0���̎���0.5�j
	//���̒����Ƃ̋����𓱏o��臒l��p���ė�O����



	edge[0] = -(widthLR[0] - 320) / (double)PixcelRate;
	edge[1] = -(widthLR[1] - 320) / (double)PixcelRate;

	// �����`��
	line(original_image, Point(0, criteria.y - size.height / 2), Point(original_image.cols, criteria.y - size.height / 2), Scalar(0, 0, 255));
	line(original_image, Point(0, criteria.y + size.height / 2), Point(original_image.cols, criteria.y + size.height / 2), Scalar(0, 0, 255));

	line(original_image, Point(widthLR[1], 0), Point(widthLR[1], original_image.rows), Scalar(0, 0, 255));
	line(original_image, Point(widthLR[0], 0), Point(widthLR[0], original_image.rows), Scalar(0, 0, 255));

	// �c���`��
	line(original_image, Point(0, heightTB[0]), Point(original_image.cols, heightTB[0]), Scalar(255, 0, 255));
	line(original_image, Point(0, heightTB[1]), Point(original_image.cols, heightTB[1]), Scalar(255, 0, 255));

	line(original_image, Point(criteria.x - size.width / 2, 0), Point(criteria.x - size.width / 2, original_image.rows), Scalar(255, 0, 255));
	line(original_image, Point(criteria.x + size.width / 2, 0), Point(criteria.x + size.width / 2, original_image.rows), Scalar(255, 0, 255));

	imshow("Image", original_image);


}


void opencv_close(void){
	destroyAllWindows();
}