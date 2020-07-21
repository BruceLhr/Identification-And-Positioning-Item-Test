#pragma once
#include <fstream>
#include <math.h>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2\ml.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "definename.h"
#include"myTcp.h"

#define PI 3.1415926

using namespace std;
using namespace cv;
using namespace ml;
using namespace cv::xfeatures2d;

class CCamParameter
{
public:
	CCamParameter();
	~CCamParameter();

	Size board_count = Size(4, 5);//�궨�����С��еĽǵ���
	Size square_size = Size(20.0, 20.0);//�궨�������̸��С

	Mat camInMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));//�ڲ�
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));//����
	Mat H_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));//��Ӧ
	Mat camCatch;//��������
	vector<Point2f> direction_point;
	float Ratio_Zx;
	float Ratio_Zy;
	void CamShow();
	void CamCalibration();
	void ImgCorrect();
	void CamCatch();
	void CamConnect();
	void DirTest();
	Point2f PointCal(Point2f img_Point);
	void TrainMod();
	void ANNTest();
	void ModeOne();
	void ModeTwo();
	void ModeThree();
	void BgSub();

	void GetShowImg();
private:

};