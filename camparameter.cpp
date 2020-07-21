#include "camparameter.h"
#define __SHOW  1
#define __FINISH 1

CCamParameter::CCamParameter()
{
	ifstream ifs;
	ifs.open(CAM_MATRIX, ios::in);
	double tempd;
	for (int i = 0; i < 9; i++)
	{
		ifs >> tempd;
		this->camInMatrix.at<float>(i) = tempd;
	}
	for (int i = 0; i < 5; i++)
	{
		ifs >> tempd;
		this->distCoeffs.at<float>(i) = tempd;
	}
	ifs.close();

	float tempx;
	float tempy;
	ifs.open(CAM_HOMO, ios::in);
	for (int i = 0; i < 2; i++)
	{
		ifs >> tempx;
		ifs >> tempy;
		direction_point.push_back(Point2f(tempx, tempy));
	}
	for (int i = 0; i <9; i++)
	{
		ifs >> tempd;
		this->H_matrix.at<float>(i) = tempd;
	}
	ifs >> tempd;
	this->Ratio_Zx = tempd;
	ifs >> tempd;
	this->Ratio_Zy = tempd;
	ifs.close();
}

CCamParameter::~CCamParameter()
{
}


void CCamParameter::CamShow()
{
	cout << "\n内参矩阵:\n" << this->camInMatrix << endl<<endl;
	cout << "畸变矩阵:\n" << this->distCoeffs << endl<<endl;
	cout <<"方向点:\n"<< this->direction_point << endl<<endl;
	cout << "单应矩阵:\n" << this->H_matrix << endl<<endl;
	cout << endl;
	return;
}

void CCamParameter::CamCalibration()
{
	ifstream InputImg(INPUT_IMG);//输入的图像
	ofstream outResult(CAM_MATRIX);
	int img_count = 0;//图像总数量
	Size img_size;//图像尺寸
	
	vector<Point2f> img_points;//每幅图像检测到的角点坐标
	vector<vector<Point2f>> img_points_all;//保存所有角点图像坐标
	string filename;
	while (getline(InputImg, filename))
	{
		img_count++;
		Mat imgInput = imread(filename);
		if (img_count == 1)
		{
			img_size.width = imgInput.cols;
			img_size.height = imgInput.rows;
			std::cout << "图像宽度 = " << img_size.width << endl;
			std::cout << "图像高度 = " << img_size.height << endl;
		}

		//提取角点
		if (0 == findChessboardCorners(imgInput, this->board_count, img_points))
		{
			std::cout << "找不到角点!\n"; //找不到角点
			exit(1);
		}
		else
		{
			Mat img_gray;
			cv::cvtColor(imgInput, img_gray, CV_RGB2GRAY);
			//找到棋盘角的亚像素精确位置
			find4QuadCornerSubpix(img_gray, img_points, Size(1, 1));
			img_points_all.push_back(img_points);//保存
		}
	}
	cout << "角点检测完成！\n";

	cout << "开始标定\n";
	vector<int> point_counts;//每幅图像中的角点数量
	vector<vector<Point3f>> img_points3D_all;//标定板上角点的三维坐标
	Mat camInMatrixTemp = Mat(3, 3, CV_32FC1, Scalar::all(0));//摄像机内参
	Mat distCoeffsTemp = Mat(1, 5, CV_32FC1, Scalar::all(0));//畸变系数 k1,k2  p1,p2  k3
	vector<Mat> tvecsMat;//平移向量
	vector<Mat> rvecsMat;//旋转向量

	int i, j, t;
	for (t = 0; t < img_count; t++)//初始化角点三维坐标
	{
		vector<Point3f> tempPoint3D;
		for (i = 0; i < this->board_count.height; i++)
		{
			for (j = 0; j < this->board_count.width; j++)
			{
				Point3f realPoint;
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;//标定板在z=0的平面上
				tempPoint3D.push_back(realPoint);
			}
		}
		img_points3D_all.push_back(tempPoint3D);
	}

	for (i = 0; i < img_count; i++)//初始化每幅图像的角点数量
	{
		point_counts.push_back(this->board_count.width * this->board_count.height);
	}

	calibrateCamera(img_points3D_all, img_points_all, img_size, camInMatrixTemp, distCoeffsTemp, rvecsMat, tvecsMat, 0);


	cout << "开始保存标定结果\n";
	
	for (int i = 0; i < 9; i++)
	{
		this->camInMatrix.at<float>(i) = camInMatrixTemp.at<double>(i);
		outResult << this->camInMatrix.at<float>(i)<<" ";

	}
	outResult << endl;
	
	for (int i = 0; i < 5; i++)
	{
		this->distCoeffs.at<float>(i) = distCoeffsTemp.at<double>(i);
		outResult << this->distCoeffs.at<float>(i) << " ";
	}
	cout << "标定结果保存完成\n";
	outResult.close();
	InputImg.close();
}


void CCamParameter::ImgCorrect()
{
	Size img_size;
	img_size.width = this->camCatch.cols;
	img_size.height = this->camCatch.rows;
	Mat mapx = Mat(img_size, CV_32FC1);
	Mat mapy = Mat(img_size, CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
	initUndistortRectifyMap(this->camInMatrix, this->distCoeffs, R, this->camInMatrix, img_size, CV_32FC1, mapx, mapy);
	remap(this->camCatch, this->camCatch, mapx, mapy, INTER_LINEAR);
}


void CCamParameter::CamCatch()
{
	VideoCapture capture;
	Mat tempImg;
	int c;
	int index = 1;
	string imgname = "./BD/BD";
	string suffix = ".jpg";
	string outname;
	//capture.open("http://admin:admin@192.168.43.1:8081");
	capture.open(0);
	//capture.set(CV_CAP_PROP_FRAME_WIDTH, 1024);//宽度 
	//capture.set(CV_CAP_PROP_FRAME_HEIGHT, 768);//高度
	cout << "按数字1截图，共需要截5张，第一张获得原点(图像左下角第一个角点)，第二张获得传送带正方向，其余在图像范围内移动到未覆盖的位置。\n";
	while (index < 21)
	{
		capture >> tempImg;
		if (tempImg.empty())
		{
			cout << "截图失败\n";
			std::system("pause");
			return;
		}
		cv::imshow("img", tempImg);
		c = cv::waitKey(20);
		if ((c % 4) == 1)
		{
			outname = imgname + to_string(index) + suffix;
			imwrite(outname, tempImg);
			c = 0;
			cout << "第 " << index << " 张完成，按任意键继续！\n";
			index++;
			cv::waitKey(0);
		}
	}
	cv::destroyWindow("img");
	capture.release();
	return;
}

void CCamParameter::CamConnect()
{
	ofstream outResult(CAM_HOMO);
	ifstream InputImg(INPUT_IMG);//输入的图像
	string filename;
	vector<Point2f> img_points;//每幅图像检测到的角点坐标
	vector<Point2f> dst_points;
	Mat Homo;
	Mat tempH;
	this->Ratio_Zx = 1;
	this->Ratio_Zy = 1;
	direction_point.clear();
	for (int tt = 0; tt < board_count.height; tt++)
	{
		for (int bb = board_count.width; 0 < bb; bb--)
		{
			dst_points.push_back(Point2f(square_size.height * tt , square_size.width * bb ));
		}
	}
	for (int i = 0; i < 2; i++)
	{
		img_points.clear();
		getline(InputImg, filename);
		this->camCatch = imread(filename);
		ImgCorrect();
		if (0 == findChessboardCorners(camCatch, this->board_count, img_points))
		{
			std::cout << "找不到角点!\n"; //找不到角点
			exit(1);
		}
		else
		{
			if (i == 0)
			{
				Homo = findHomography(dst_points, img_points);
				tempH = camCatch;
				
			}
			direction_point.push_back(img_points[0]);
		}
	}
	for (int t = 0; t < 2; t++)
	{
		outResult << direction_point[t].x<< " " << direction_point[t].y<<" ";
	}
	outResult << endl;
	for (int zz = 0; zz < 9; zz++)
	{
		this->H_matrix.at<float>(zz) = Homo.at<double>(zz);
		outResult << this->H_matrix.at<float>(zz)<<" ";
	}
	outResult << endl;
	img_points.clear();
	dst_points.clear();
	if (0 == findChessboardCorners(tempH, this->board_count, img_points))
	{
		std::cout << "找不到角点!\n"; //找不到角点
		exit(1);
	}
	dst_points.push_back(PointCal(img_points[0]));
	dst_points.push_back(PointCal(img_points[2]));
	dst_points.push_back(PointCal(img_points[board_count.width*2]));
	this->Ratio_Zy = (abs(sqrt(pow((dst_points[1].x - dst_points[0].x), 2) + pow((dst_points[1].y - dst_points[0].y), 2)))/2) / (square_size.height);
	this->Ratio_Zx = (abs(sqrt(pow((dst_points[2].x - dst_points[0].x), 2) + pow((dst_points[2].y - dst_points[0].y), 2)))/2) / (square_size.width);
	outResult << this->Ratio_Zx<<" ";
	outResult << this->Ratio_Zy<<" ";
	outResult.close();
	InputImg.close();
	cout << "关联完成" << endl;
	return;
}


void CCamParameter::DirTest()
{
	ifstream InputImg(BDTEST_IMG);//输入的图像
	string filename;
	vector<Point2f> img_points;
	vector<Point2f> dst_points;
	float v_Errory=0;
	float v_Errorx=0;
	float v_tempdy = 0;
	float v_tempdx = 0;
	int index = 0;
	while (getline(InputImg, filename))
	{
		this->camCatch = imread(filename);
		ImgCorrect();
		if (0 == findChessboardCorners(camCatch, this->board_count, img_points))
		{
			std::cout << "找不到角点!\n"; //找不到角点
			exit(1);
		}
		else
		{
			dst_points.clear();
			//circle(camCatch, img_points[0], 3, Scalar(0, 0, 255), 3);
			//circle(camCatch, img_points[board_count.width - 1], 3, Scalar(0, 255, 0), 3);
			//circle(camCatch, img_points[board_count.width * board_count.height - board_count.width], 3, Scalar(255, 0, 0), 3);
			dst_points.push_back(PointCal(img_points[0]));
			dst_points.push_back(PointCal(img_points[board_count.width - 1]));
			dst_points.push_back(PointCal(img_points[board_count.width * board_count.height - board_count.width]));
			float tempdy = abs(sqrt(pow((dst_points[1].x - dst_points[0].x), 2) + pow((dst_points[1].y - dst_points[0].y), 2)) / (board_count.width - 1));
			float tempdx = abs(sqrt(pow((dst_points[2].x - dst_points[0].x), 2) + pow((dst_points[2].y - dst_points[0].y), 2)) / (board_count.height - 1));
			cout <<index+1<< "，垂直方向计算值：" << tempdy << "  误差率：" << abs(tempdy - square_size.width) / square_size.width \
				<< "  水平方向计算值：" << tempdx << "  误差率：" << abs(tempdx - square_size.height) / square_size.height << endl;
			v_Errory += abs(tempdy - square_size.width) / square_size.width;
			v_Errorx += abs(tempdx - square_size.height) / square_size.height;
			v_tempdy += tempdy;
			v_tempdx += tempdx;
			//dst_points.push_back(PointCal(img_points[1]));//一格
			//dst_points.push_back(PointCal(img_points[board_count.width]));
			//float tempdy = abs(sqrt(pow((dst_points[1].x - dst_points[0].x), 2) + pow((dst_points[1].y - dst_points[0].y), 2)));
			//float tempdx = abs(sqrt(pow((dst_points[2].x - dst_points[0].x), 2) + pow((dst_points[2].y - dst_points[0].y), 2)));
			index++;
		}
	}
	cout << endl;
	cout << "棋盘格垂直方向大小:" << square_size.width << "  棋盘格水平方向大小:" << square_size.height << endl;
	cout << "垂直方向平均计算值" << v_tempdy / index << "  水平方向平均计算值" << v_tempdx / index << endl;
	cout << "垂直方向平均误差" << v_Errory / index << "  水平方向平均误差率" << v_Errorx / index << endl<<endl;
	InputImg.close();
}


Point2f CCamParameter::PointCal(Point2f img_Point)
{
	Point2f real_Point;
	float tempx = ((H_matrix.at<float>(0) * img_Point.x) + (H_matrix.at<float>(1) * img_Point.y) + H_matrix.at<float>(2)) / Ratio_Zx / 10;
	float tempy = ((H_matrix.at<float>(3) * img_Point.x) + (H_matrix.at<float>(4) * img_Point.y) + H_matrix.at<float>(5)) / Ratio_Zy / 10;
	real_Point = Point2f(tempx, tempy);
	return real_Point;
}

void CCamParameter::TrainMod()
{
	const int classSum = 3;//物体的类型数
	const int imagesSum = 12;//每类物体的图片数
	const int histSum = 90;//直方图的区间数

	Mat bsmaskMOG2,inputImg;
	float trainingData[classSum * imagesSum][histSum] = { {0} };
	float labels[classSum * imagesSum][classSum] = { {0} };
	Mat hsvbase;
	int hhistSize[] = { histSum };
	float h_ranges[] = { 0, 180 };
	const float* hranges[] = { h_ranges };
	int hchannels[] = { 0 };
	Mat h_hist;
	Mat kernel1 = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
	Mat kernel2 = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
	Ptr<BackgroundSubtractor> pMOG2 = createBackgroundSubtractorMOG2();
	ifstream InputImg(TRAIN_IMG);//输入的图像
	string filename;
	ifstream InputBG(BG_IMG);
	while (getline(InputBG, filename))
	{
		inputImg = imread(filename);
		pMOG2->apply(inputImg, bsmaskMOG2);
	}
	for (int i = 0; i < classSum; i++)
	{
		for (int k = 0; k < imagesSum; k++)
		{
			getline(InputImg, filename);
			inputImg = imread(filename);
			pMOG2->apply(inputImg, bsmaskMOG2);
			cv::threshold(bsmaskMOG2, bsmaskMOG2, 250, 255, THRESH_BINARY);
			cv::morphologyEx(bsmaskMOG2, bsmaskMOG2, MORPH_OPEN, kernel1, Point(-1, -1));
			cv::dilate(bsmaskMOG2, bsmaskMOG2, kernel2, Point(-1, -1));
			//imshow("MOG2", bsmaskMOG2);
			//cout << filename << endl;
			
			vector<vector<Point>> contours;
			vector<Vec4i> hireachy;
			Rect maxRect;
			contours.clear();
			hireachy.clear();
			cv::findContours(bsmaskMOG2, contours, hireachy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
			for (size_t t = 0; t < contours.size(); t++) {
				Rect rect = boundingRect(contours[t]);
				if (rect.width > (inputImg.cols - 30)) continue;
				if (rect.height > (inputImg.rows - 30)) continue;
				if ((rect.width > maxRect.width) || (rect.height > maxRect.height))
				{
					maxRect = rect;
				}
			}
			//rectangle(inputImg, maxRect, (0, 0, 255), 2, 8, 0);
			Mat newRoi = inputImg(maxRect);
			//imshow("newRoi", newRoi);

			cv::cvtColor(newRoi, hsvbase, CV_BGR2HSV);
			cv::calcHist(&hsvbase, 1, hchannels, Mat(), h_hist, 1, hhistSize, hranges, true, false);
			for (int j = 0; j < histSum; j++)
			{
				trainingData[i * imagesSum + k][j] = h_hist.at<float>(j);
			}
			for (int q = 0; q < classSum; q++)
			{
				if (q == i)
				{
					labels[i * imagesSum + k][q] = 1;
				}
				else
				{
					labels[i * imagesSum + k][q] = 0;
				}
			}
			
		}
	}
	
	Mat trainingDataMat(classSum * imagesSum, histSum, CV_32FC1, trainingData);
	Mat labelsMat(classSum * imagesSum, classSum, CV_32FC1, labels);
	Ptr<ANN_MLP>model = ANN_MLP::create();
	Mat layerSizes = (Mat_<int>(1, 3) << histSum, 15, classSum);
	model->setLayerSizes(layerSizes);
	model->setTrainMethod(ANN_MLP::BACKPROP);
	model->setActivationFunction(ANN_MLP::SIGMOID_SYM, 1.0, 1.0);
	model->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 1000, 0.001));
	Ptr<TrainData> trainData = TrainData::create(trainingDataMat, ROW_SAMPLE, labelsMat);
	model->train(trainData);
	model->save(MODEL);
	cout << "训练完成\n";
	InputImg.close();
	return;
}

void CCamParameter::ANNTest()
{
	const int classSum = 3;//物体的类型数
	const int imagesSum = 12;//每类物体的图片数
	const int histSum = 90;//直方图的区间数

	Ptr<ANN_MLP> model = StatModel::load<ANN_MLP>(MODEL);
	Mat dst;
	Mat bsmaskMOG2, inputImg;
	string filename;
	float trainingData[classSum * imagesSum][histSum] = { {0} };
	float labels[classSum * imagesSum][classSum] = { {0} };
	Mat hsvbase;
	int hhistSize[] = { histSum };
	float h_ranges[] = { 0, 180 };
	const float* hranges[] = { h_ranges };
	int hchannels[] = { 0 };
	Mat test_hist;
	Mat kernel1 = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
	Ptr<BackgroundSubtractor> pMOG2 = createBackgroundSubtractorMOG2();

	ifstream InputTemp(TRAIN_TEMP);
	vector<string> vTempName;
	while (getline(InputTemp, filename))
	{
		vTempName.push_back(filename);
	}
	ifstream InputBG(BG_IMG);
	while (getline(InputBG, filename))
	{
		inputImg = imread(filename);
		pMOG2->apply(inputImg, bsmaskMOG2);
	}
	ifstream InputImg(TRAIN_TEST);//输入的图像
	while (getline(InputImg, filename))
	{
		inputImg = imread(filename);
		pMOG2->apply(inputImg, bsmaskMOG2);
		cv::threshold(bsmaskMOG2, bsmaskMOG2, 250, 255, THRESH_BINARY);
		cv::morphologyEx(bsmaskMOG2, bsmaskMOG2, MORPH_OPEN, kernel1, Point(-1, -1));
		cv::dilate(bsmaskMOG2, bsmaskMOG2, kernel1, Point(-1, -1));

		vector<vector<Point>> contours;
		vector<Vec4i> hireachy;
		Rect maxRect;
		contours.clear();
		hireachy.clear();
		cv::findContours(bsmaskMOG2, contours, hireachy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
		for (size_t t = 0; t < contours.size(); t++) {
			Rect rect = boundingRect(contours[t]);
			if (rect.width > (inputImg.cols - 30)) continue;
			if (rect.height > (inputImg.rows - 30)) continue;
			if ((rect.width > maxRect.width) || (rect.height > maxRect.height))
			{
				maxRect = rect;
			}
		}
		Mat newRoi = inputImg(maxRect);
		cv::cvtColor(newRoi, hsvbase, CV_BGR2HSV);
		cv::calcHist(&hsvbase, 1, hchannels, Mat(), test_hist, 1, hhistSize, hranges, true, false);

		Mat_<float> testMat(1, histSum);
		for (int j = 0; j < histSum; j++)
		{
			testMat.at<float>(0, j) = test_hist.at<float>(j);
		}

		model->predict(testMat, dst);
		Point maxLoc;
		double maxV;
		cv::minMaxLoc(dst, NULL, &maxV, NULL, &maxLoc, Mat());
		if (maxV > 1)
		{
			maxV = 1.00;
		}
		for (int ia = 0; ia < 3; ia++)
		{
			if (abs(dst.at<float>(ia)) > 1)
				dst.at<float>(ia) = 1;
		}

		cout << "输入:" << filename << "   " << "X" <<":"<< abs(dst.at<float>(0)) \
			<< "   " << "P" <<":" << abs(dst.at<float>(1))<<"   "<< "G" <<":" << abs(dst.at<float>(2))<<endl;
		Mat ANNtemp = imread(vTempName[maxLoc.x]);
		cv::putText(ANNtemp, to_string(maxV), Point(1, ANNtemp.rows / 2), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
		cv::imshow("src", inputImg);
		cv::imshow("predict", ANNtemp);
		cv::waitKey(0);
	}
	cv::destroyWindow("src");
	cv::destroyWindow("predict");
	InputTemp.close();
	InputImg.close();
}

void CCamParameter::ModeOne()
{
	string filename;
	Mat bsmaskMOG2, inputImg;
	Ptr<BackgroundSubtractor> pMOG2 = createBackgroundSubtractorMOG2();
	ifstream InputBG(MODE_ONE_T);
	Mat kernel1 = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
	while (getline(InputBG, filename))
	{
		inputImg = imread(filename);
		pMOG2->apply(inputImg, bsmaskMOG2);
	}
	
	this->camCatch = bsmaskMOG2;
	ImgCorrect();
	bsmaskMOG2 = this->camCatch;

	cv::threshold(bsmaskMOG2, bsmaskMOG2, 250, 255, THRESH_BINARY);
	cv::morphologyEx(bsmaskMOG2, bsmaskMOG2, MORPH_OPEN, kernel1, Point(-1, -1));
	//imshow("bsmaskMOG2", bsmaskMOG2);
	vector<vector<Point>> contours;
	vector<Vec4i> hireachy;
	int objIndex = 0;
	int objSize;//1 small  2 big  3 long
	Rect maxRect;
	contours.clear();
	hireachy.clear();
	cv::findContours(bsmaskMOG2, contours, hireachy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
	for (size_t t = 0; t < contours.size(); t++) {
		Rect rect = boundingRect(contours[t]);
		if (rect.width > (inputImg.cols - 15)) continue;
		if (rect.height > (inputImg.rows - 15)) continue;
		if ((rect.width > maxRect.width) || (rect.height > maxRect.height))
		{
			Rect maxRect = boundingRect(contours[t]);
			objIndex = t;
		}
	}
	if (maxRect.width < (inputImg.cols / 5) && (maxRect.height < (inputImg.rows / 5)))
	{
		objSize = 1;
	}
	else if (maxRect.width < (inputImg.cols / 2) && (maxRect.height < (inputImg.rows / 5)))
	{
		objSize = 2;
	}
	else
	{
		objSize =3;
	}
	Moments objMoments;
	Point2f objPoint;
	objMoments = moments(contours[objIndex]);
	objPoint = Point((objMoments.m10 / objMoments.m00), (objMoments.m01 / objMoments.m00));
	float theta = fitEllipse(contours[objIndex]).angle;
	/*
	double a = objMoments.m20 / objMoments.m00 - center.x*center.x;
	double b = objMoments.m11 / objMoments.m00 - center.x*center.y;
	double c = objMoments.m02 / objMoments.m00 - center.y*center.y;
	double theta = fastAtan2(2*b,(a - c))/2;
	*/

	Point2f xy1, xy2;
	Point2f realObjPoint;
	xy1 = PointCal(direction_point[0]);
	xy2 = PointCal(direction_point[1]);
	realObjPoint = PointCal(objPoint);
	float xEdge = xy1.x - xy2.x;
	float yEdge = xy1.y - xy2.y;
	float longEdge = sqrt(pow((xEdge), 2) + pow((yEdge), 2));
	float cosDire = yEdge / longEdge;
	float sinDire = xEdge / longEdge;//x'=x*cos+y*sin	y'=y*cos-x*sin
	float actDire = acos(cosDire);
	float objRealAngle = actDire - theta;
	Point2f objRealPoint(((realObjPoint.x * cosDire) + (realObjPoint.y * sinDire)), ((realObjPoint.y * cosDire) - (realObjPoint.x * sinDire)));
	//cout << objRealPoint << endl << theta << endl;
	InputBG.close();
	
#if __SHOW
	this->camCatch = inputImg;
	ImgCorrect();
	Mat showImg = this->camCatch;
	cv::line(showImg, objPoint, Point(objPoint.x + (cos(theta * 3.1416 / 180) * 100), objPoint.y + (sin(theta * 3.1416 / 180) * 100)), Scalar(0, 255, 0), 2);
	cv::drawContours(showImg, contours, static_cast<int>(objIndex), Scalar(0, 0, 255), 1, 8, hireachy);
	cv::circle(showImg, objPoint, 1, Scalar(0, 0, 255), 2);
	cv::putText(showImg, (string("("+to_string((int)objPoint.x) + "," + to_string((int)objPoint.y) + ")" + to_string(theta))), \
		Point(showImg.cols/3, showImg.rows/3), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255),2);
	cv::imshow("ObjShow", showImg);
	cv::waitKey(0);
	CMyTcp tcp;
	tcp.tcpInit();
	tcp.setPort(8080);
	tcp.connectTcp();
	string send_buf;
	send_buf = to_string(objSize) + "," + to_string(objRealPoint.x)+"," + to_string(objRealPoint.y)+","+ to_string((double)theta);
	tcp.sendMsg(send_buf);
	tcp.tcpEnd();
	cv::destroyWindow("ObjShow");
#endif 
	return;
}

void CCamParameter::ModeTwo()
{
	const int classSum = 3;//物体的类型数
	const int imagesSum = 12;//每类物体的图片数
	const int histSum = 90;//直方图的区间数

	Ptr<ANN_MLP> model = StatModel::load<ANN_MLP>(MODEL);
	Mat test, dst;
	Mat bsmaskMOG2, inputImg;
	string filename;
	float trainingData[classSum * imagesSum][histSum] = { {0} };
	float labels[classSum * imagesSum][classSum] = { {0} };
	Mat hsvbase;
	int hhistSize[] = { histSum };
	float h_ranges[] = { 0, 180 };
	const float* hranges[] = { h_ranges };
	int hchannels[] = { 0 };
	Mat test_hist;
	Mat kernel1 = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
	Ptr<BackgroundSubtractor> pMOG2 = createBackgroundSubtractorMOG2();
	ifstream InputBG(MODE_TWO_T);
	ifstream InputTemp(TRAIN_TEMP);
	vector<string> vTempName;
	while (getline(InputTemp, filename))
	{
		vTempName.push_back(filename);
	}

	while (getline(InputBG, filename))
	{
		inputImg = imread(filename);
		pMOG2->apply(inputImg, bsmaskMOG2);
	}

	Mat srcImg = inputImg;

	cv::threshold(bsmaskMOG2, bsmaskMOG2, 250, 255, THRESH_BINARY);
	cv::morphologyEx(bsmaskMOG2, bsmaskMOG2, MORPH_OPEN, kernel1, Point(-1, -1));
	cv::dilate(bsmaskMOG2, bsmaskMOG2, kernel1, Point(-1, -1));

	vector<vector<Point>> contours;
	vector<Vec4i> hireachy;
	Rect maxRect;
	contours.clear();
	hireachy.clear();
	cv::findContours(bsmaskMOG2, contours, hireachy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
	for (size_t t = 0; t < contours.size(); t++) {
		Rect rect = boundingRect(contours[t]);
		if (rect.width > (inputImg.cols - 30)) continue;
		if (rect.height > (inputImg.rows - 30)) continue;
		if ((rect.width > maxRect.width) || (rect.height > maxRect.height))
		{
			maxRect = rect;
		}
	}
	Mat newRoi = inputImg(maxRect);
	cv::cvtColor(newRoi, hsvbase, CV_BGR2HSV);
	cv::calcHist(&hsvbase, 1, hchannels, Mat(), test_hist, 1, hhistSize, hranges, true, false);

	Mat_<float> testMat(1, histSum);
	for (int j = 0; j < histSum; j++)
	{
		testMat.at<float>(0, j) = test_hist.at<float>(j);
	}

	model->predict(testMat, dst);
	Point maxLoc;
	double maxV;
	cv::minMaxLoc(dst, NULL, &maxV, NULL, &maxLoc, Mat());
	if (maxV > 1)
	{
		maxV = 1;
	}

	int objIndex=0;
	this->camCatch = bsmaskMOG2;
	ImgCorrect();
	bsmaskMOG2 = this->camCatch;
	contours.clear();
	hireachy.clear();
	maxRect.height = 0;
	cv::findContours(bsmaskMOG2, contours, hireachy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
	for (size_t t = 0; t < contours.size(); t++) {
		Rect rect = boundingRect(contours[t]);
		if (rect.width > (inputImg.cols - 30)) continue;
		if (rect.height > (inputImg.rows - 30)) continue;
		if ((rect.width > maxRect.width) || (rect.height > maxRect.height))
		{
			maxRect = rect;
			objIndex = t;
		}
	}
	Moments objMoments;
	Point2f objPoint;
	objMoments = moments(contours[objIndex]);
	objPoint = Point((objMoments.m10 / objMoments.m00), (objMoments.m01 / objMoments.m00));

	float theta = fitEllipse(contours[objIndex]).angle;
	Point2f xy1, xy2;
	Point2f realObjPoint;
	xy1 = PointCal(direction_point[0]);
	xy2 = PointCal(direction_point[1]);
	realObjPoint = PointCal(objPoint);
	float xEdge = xy1.x - xy2.x;
	float yEdge = xy1.y - xy2.y;
	float longEdge = sqrt(pow((xEdge), 2) + pow((yEdge), 2));
	float cosDire = yEdge / longEdge;
	float sinDire = xEdge / longEdge;//x'=x*cos+y*sin	y'=y*cos-x*sin
	float actDire = acos(cosDire);
	float objRealAngle = actDire - theta;
	Point2f objRealPoint(((realObjPoint.x * cosDire) + (realObjPoint.y * sinDire)), ((realObjPoint.y * cosDire) - (realObjPoint.x * sinDire)));
	//cout << objRealPoint << endl << theta;
	InputBG.close();
#if __SHOW
	Mat ANNtemp = imread(vTempName[maxLoc.x]);
	this->camCatch = srcImg;
	ImgCorrect();
	srcImg = this->camCatch;
	cv::line(srcImg, objPoint, Point(objPoint.x + (cos(theta * 3.1416 / 180) * 100), objPoint.y + (sin(theta * 3.1416 / 180) * 100)), Scalar(0, 255, 0), 2);
	cv::drawContours(srcImg, contours, static_cast<int>(objIndex), Scalar(0, 0, 255), 1, 8, hireachy);
	cv::putText(srcImg, (string("(" + to_string((int)objPoint.x) + "," + to_string((int)objPoint.y) + ")" + to_string(theta))), \
		Point(srcImg.cols / 3, srcImg.rows / 3), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 2);
	cv::circle(srcImg, objPoint, 1, Scalar(0, 0, 255), 2);
	cv::imshow("ObjShow", srcImg);
	cv::imshow("ANNtemp", ANNtemp);
	cv::waitKey(0);
	cv::destroyWindow("ObjShow");
	cv::destroyWindow("ANNtemp");
#endif
	return;
}

void CCamParameter::ModeThree()
{
#if !__FINISH
	const int classSum = 3;//物体的类型数
	const int imagesSum = 12;//每类物体的图片数
	const int histSum = 90;//直方图的区间数

	Ptr<ANN_MLP> model = StatModel::load<ANN_MLP>(MODEL);
	Mat test, dst;
	Mat bsmaskMOG2, inputImg;
	string filename;
	float trainingData[classSum * imagesSum][histSum] = { {0} };
	float labels[classSum * imagesSum][classSum] = { {0} };
	Mat hsvbase;
	int hhistSize[] = { histSum };
	float h_ranges[] = { 0, 180 };
	const float* hranges[] = { h_ranges };
	int hchannels[] = { 0 };
	Mat test_hist;
	Mat kernel1 = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
	Ptr<BackgroundSubtractor> pMOG2 = createBackgroundSubtractorMOG2();
	ifstream InputBG(MODE_THR_T);

	ifstream InputTemp(TRAIN_TEMP);
	vector<string> vTempName;
	while (getline(InputTemp, filename))
	{
		vTempName.push_back(filename);
	}

	while (getline(InputBG, filename))
	{
		inputImg = imread(filename);
		pMOG2->apply(inputImg, bsmaskMOG2);
	}

	Mat srcImg = inputImg;

	threshold(bsmaskMOG2, bsmaskMOG2, 250, 255, THRESH_BINARY);
	morphologyEx(bsmaskMOG2, bsmaskMOG2, MORPH_OPEN, kernel1, Point(-1, -1));
	dilate(bsmaskMOG2, bsmaskMOG2, kernel1, Point(-1, -1));

	vector<vector<Point>> contours;
	vector<Vec4i> hireachy;
	Rect maxRect;
	contours.clear();
	hireachy.clear();
	findContours(bsmaskMOG2, contours, hireachy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
	for (size_t t = 0; t < contours.size(); t++) {
		Rect rect = boundingRect(contours[t]);
		if (rect.width > (inputImg.cols - 30)) continue;
		if (rect.height > (inputImg.rows - 30)) continue;
		if ((rect.width > maxRect.width) || (rect.height > maxRect.height))
		{
			maxRect = rect;
		}
	}
	Mat newRoi = inputImg(maxRect);
	cvtColor(newRoi, hsvbase, CV_BGR2HSV);
	calcHist(&hsvbase, 1, hchannels, Mat(), test_hist, 1, hhistSize, hranges, true, false);

	Mat_<float> testMat(1, histSum);
	for (int j = 0; j < histSum; j++)
	{
		testMat.at<float>(0, j) = test_hist.at<float>(j);
	}

	model->predict(testMat, dst);
	Point maxLoc;
	double maxV;
	minMaxLoc(dst, NULL, &maxV, NULL, &maxLoc, Mat());
	if (maxV > 1)
	{
		maxV = 1;
	}
	Mat ANNtemp = imread(vTempName[maxLoc.x]);
#else
	ifstream InputTemp(MODE_THR_T);
	vector<string> vTempName;
	string filename;
	while (getline(InputTemp, filename))
	{
		vTempName.push_back(filename);
	}
	Mat srcImg = imread(vTempName[1]);
	Mat ANNtemp = imread(vTempName[0]);
	Mat dstImg = srcImg;
	Mat dstTemp = ANNtemp;
	cv::cvtColor(srcImg, srcImg, COLOR_BGR2GRAY);
	cv::cvtColor(ANNtemp, ANNtemp, COLOR_BGR2GRAY);
#endif
	
	int minHessian = 2200;
	Ptr<SURF> detector = SURF::create(minHessian);
	vector<KeyPoint> keypoints_obj;
	vector<KeyPoint> keypoints_scene;
	Mat descriptor_obj, descriptor_scene;

	vector<KeyPoint> keypoints_scene2;

	detector->detectAndCompute(ANNtemp, Mat(), keypoints_obj, descriptor_obj);
	detector->detectAndCompute(srcImg, Mat(), keypoints_scene, descriptor_scene);

	FlannBasedMatcher matcher;
	vector<DMatch> matches;
	matcher.match(descriptor_obj, descriptor_scene, matches);
	double minDist = 3000;
	for (int i = 0; i < descriptor_obj.rows; i++) {
		double dist = matches[i].distance;
		if (dist < minDist) {
			minDist = dist;
		}
	}
	
	
	vector<DMatch> goodMatches;
	for (int i = 0; i < descriptor_obj.rows; i++) {
		double dist = matches[i].distance;
		//if (dist < max(3 * minDist, 0.02)) {
		if (dist < 3 * minDist) {
			goodMatches.push_back(matches[i]);//距离较近即匹配较好的点
		}
	}
	vector<Point2f> obj;
	vector<Point2f> objInScene;
	for (size_t t = 0; t < goodMatches.size(); t++) {
		obj.push_back(keypoints_obj[goodMatches[t].queryIdx].pt);
		objInScene.push_back(keypoints_scene[goodMatches[t].trainIdx].pt);
	}
	Mat H = findHomography(obj, objInScene, RANSAC);
	vector<Point2f> obj_corners(4);
	vector<Point2f> scene_corners(4);
	obj_corners[0] = Point(0, 0);
	obj_corners[1] = Point(ANNtemp.cols, 0);
	obj_corners[2] = Point(ANNtemp.cols, ANNtemp.rows);
	obj_corners[3] = Point(0, ANNtemp.rows);

	vector<Point2f> obj_key(4);
	vector<Point2f> scene_key(4);
	obj_key[0] = Point(40, 50);
	obj_key[1] = Point(ANNtemp.cols-60, 40);
	obj_key[2] = Point(ANNtemp.cols-50, ANNtemp.rows-40);
	obj_key[3] = Point(50, ANNtemp.rows-40);
	perspectiveTransform(obj_corners, scene_corners, H);
	perspectiveTransform(obj_key, scene_key, H);
#if __SHOW
	/*
	Mat matchesImg;
	drawMatches(ANNtemp, keypoints_obj, srcImg, keypoints_scene, goodMatches, matchesImg, Scalar::all(-1),
		Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
	);
	line(matchesImg, scene_corners[0] + Point2f(ANNtemp.cols, 0), scene_corners[1] + Point2f(ANNtemp.cols, 0), Scalar(0, 0, 255), 2, 8, 0);
	line(matchesImg, scene_corners[1] + Point2f(ANNtemp.cols, 0), scene_corners[2] + Point2f(ANNtemp.cols, 0), Scalar(0, 0, 255), 2, 8, 0);
	line(matchesImg, scene_corners[2] + Point2f(ANNtemp.cols, 0), scene_corners[3] + Point2f(ANNtemp.cols, 0), Scalar(0, 0, 255), 2, 8, 0);
	line(matchesImg, scene_corners[3] + Point2f(ANNtemp.cols, 0), scene_corners[0] + Point2f(ANNtemp.cols, 0), Scalar(0, 0, 255), 2, 8, 0);
	*/
	for (int u = 0; u < obj_corners.size(); u++)
	{
		cv::circle(dstTemp, obj_key[u], 3, Scalar(0, 255, 0), 6);
		cv::circle(dstImg, scene_key[u], 3, Scalar(0, 255, 0), 6);
	}
	//cv::line(dstImg, scene_corners[0], scene_corners[1], Scalar(0, 0, 255), 2, 8, 0);
	//cv::line(dstImg, scene_corners[1], scene_corners[2], Scalar(0, 0, 255), 2, 8, 0);
	//cv::line(dstImg, scene_corners[2], scene_corners[3], Scalar(0, 0, 255), 2, 8, 0);
	//cv::line(dstImg, scene_corners[3], scene_corners[0], Scalar(0, 0, 255), 2, 8, 0);
	cv::putText(dstImg, (string("(" + to_string((int)scene_corners[0].x) + "," + to_string((int)scene_corners[0].y)+")"\
		+"(" + to_string((int)scene_corners[1].x) + ", " + to_string((int)scene_corners[1].y) + ")"\
		+"(" + to_string((int)scene_corners[2].x) + ", " + to_string((int)scene_corners[2].y) + ")"\
		+"(" + to_string((int)scene_corners[3].x) + ", " + to_string((int)scene_corners[3].y) + ")")),\
		Point(dstImg.cols / 3, dstImg.rows / 3), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 2);
	imshow("temp in scene", dstImg);
	imshow("temp", dstTemp);
	cv::waitKey(0);
#endif
	cv::destroyWindow("temp in scene");
	cv::destroyWindow("temp");

	return;

}

void CCamParameter::BgSub()
{
	VideoCapture capture;
	capture.open("./input/MODETEST/BGtest.mp4");
	if (!capture.isOpened()) {
		printf("could not find the video file...\n");
		return ;
	}
	Mat frame;
	Mat bsmaskMOG2;
	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
	Ptr<BackgroundSubtractor> pMOG2 = createBackgroundSubtractorMOG2();
	while (capture.read(frame)) {
		imshow("input video", frame);
		pMOG2->apply(frame, bsmaskMOG2, 0.002);
		//imshow("dst", bsmaskMOG2);
		morphologyEx(bsmaskMOG2, bsmaskMOG2, MORPH_OPEN, kernel, Point(-1, -1));
		cv::threshold(bsmaskMOG2, bsmaskMOG2, 250, 255, THRESH_BINARY);
		imshow("MOG2", bsmaskMOG2);
		char cc = cv::waitKey(20);
		if (cc == 27) {
			break;
		}
	}
	capture.release();
	cv::destroyWindow("input video");
	cv::destroyWindow("MOG2");
	return;
}

void CCamParameter::GetShowImg()
{
	Mat getImg1 = imread("./input/BD/BD1.jpg");
	Mat getImg2 = getImg1.clone();
	
	vector<Point2f> img_points;//每幅图像检测到的角点坐标
	//findChessboardCorners(getImg1, this->board_count, img_points);
	//drawChessboardCorners(getImg1, this->board_count, img_points, false);
	
	this->camCatch = getImg2;
	ImgCorrect();
	getImg2 = this->camCatch;
	findChessboardCorners(getImg2, this->board_count, img_points);
	drawChessboardCorners(getImg2, this->board_count, img_points, false);
	//circle(getImg2, img_points[0], 3, Scalar(0, 0, 255), 3);
	//circle(getImg2, img_points[board_count.width - 1], 3, Scalar(0, 255, 0), 3);
	//circle(getImg2, img_points[board_count.width * board_count.height - board_count.width], 3, Scalar(255, 0, 0), 3);
	imshow("角点1", getImg1);
	imshow("角点2", getImg2);

	cv::waitKey(0);
	return;
}


