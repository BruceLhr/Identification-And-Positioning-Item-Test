//#include "mytcp.h"
#define CVUI_IMPLEMENTATION
#include <iostream>
#include <opencv2/opencv.hpp>
#include "definename.h"
#include "camparameter.h"
#include "cvui.h"


#define WINDOW_NAME "Intelligent Grabbing System"
using namespace std;
using namespace cv;
int main(int argc, const char* argv[])
{
	CCamParameter* cam = new CCamParameter;
	cv::Mat frame = cv::Mat(150, 460, CV_8UC3);
	int count = 0;
	cv::namedWindow(WINDOW_NAME);
	cvui::init(WINDOW_NAME);
	while (cv::waitKey(30) != 27) {
		frame = cv::Scalar(49, 52, 49);
		if (cvui::button(frame, 10, 10, "Show parameters")) {
			system("cls");
			cam->CamShow();
		}
		if (cvui::button(frame, 170, 10, "Bg subtract")) {
			system("cls");
			cam->BgSub();
		}
		if (cvui::button(frame, 320, 10, "Get image")) {
			system("cls");
			cam->CamCatch();
		}
		if (cvui::button(frame, 10, 40, "Camera calibration")) {
			system("cls");
			cam->CamCalibration();
			cam->CamConnect();
		}
		if (cvui::button(frame, 170, 40, "Calibration test")) {
			system("cls");
			cam->DirTest();
		}

		if (cvui::button(frame, 10, 70, "Training Model")) {
			system("cls");
			cam->TrainMod();
		}
		if (cvui::button(frame, 170, 70, "Model test")) {
			system("cls");
			cam->ANNTest();
		}
		if (cvui::button(frame, 10, 100, "Mode one:M")) {
			system("cls");
			cam->ModeOne();
		}
		if (cvui::button(frame, 170, 100, "Mode two:R+M")) {
			system("cls");
			cam->ModeTwo();
		}
		if (cvui::button(frame, 320, 100, "Mode three:R+S")) {
			system("cls");
			cam->ModeThree();
		}
		cvui::printf(frame, 350, 60, 0.4, 0xf0f000, "by LiHaoRun");
		/*
		if (cvui::button(frame, 320, 40, "test")) {
			system("cls");
			cam->GetShowImg();
		}
		*/
		cvui::printf(frame, 185, 132, 0.4, 0xf00000, "Press ESC to exit");
		cvui::update();
		cv::imshow(WINDOW_NAME, frame);
	}
	delete cam;
	return 0;
}