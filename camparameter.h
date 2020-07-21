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

	Size board_count = Size(4, 5);//标定板上行、列的角点数
	Size square_size = Size(20.0, 20.0);//标定板上棋盘格大小

	Mat camInMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));//内参
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));//畸变
	Mat H_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));//单应
	Mat camCatch;//公共区域
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