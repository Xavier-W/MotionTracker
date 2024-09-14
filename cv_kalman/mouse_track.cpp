#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
using namespace cv;
using namespace std;

const int winHeight = 1200;
const int winWidth = 1600;

#pragma comment(lib, "opencv_world3412")
//400,300
Point mousePosition = Point(winWidth >> 1, winHeight >> 1);

//mouse event callback
void mouseEvent(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_MOUSEMOVE) {   ///CV_EVENT_MOUSEMOVE ：鼠标移动
		mousePosition = Point(x, y);
	}
}

int main(void)
{
	RNG rng;
	//1.kalman filter setup
	const int stateNum = 4;                                      //状态值4×1向量(x,y,△x,△y)
	const int measureNum = 2;                                    //测量值2×1向量(x,y)	
	KalmanFilter KF(stateNum, measureNum, 0);

	/*初始化状态转移矩阵A 和 测量矩阵H*/
	KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);  //转移矩阵A
	setIdentity(KF.measurementMatrix);                                             //测量矩阵H

	/*两个噪声的协方差矩阵*/
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R

	/*0时刻的后验状态和协方差*/
	rng.fill(KF.statePost, RNG::UNIFORM, 0, winHeight > winWidth ? winWidth : winHeight);   //初始状态值x(0)
	setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验估计协方差矩阵P

	Mat measurement = Mat::zeros(measureNum, 1, CV_32F);           //测量向量z(k)

	namedWindow("kalman");
	setMouseCallback("kalman", mouseEvent);

	Mat image(winHeight, winWidth, CV_8UC3, Scalar(0));

	while (1)
	{
		//2.kalman prediction
		Mat prediction = KF.predict();
		Point predict_pt = Point(prediction.at<float>(0), prediction.at<float>(1));   //预测值(x',y')

		//3.update measurement
		measurement.at<float>(0) = (float)mousePosition.x;
		measurement.at<float>(1) = (float)mousePosition.y;

		//4.update
		KF.correct(measurement);

		//draw 
		image.setTo(Scalar(255, 255, 255, 0));   //这里将图像的像素值设置为白色
		circle(image, predict_pt, 5, Scalar(0, 255, 0), 3);    //绿色点是预测位置
		circle(image, mousePosition, 5, Scalar(255, 0, 0), 3); //蓝色点是正确位置	

		/*
		首先定义了一个char类型的数组buf，大小为256。然后使用snprintf函数将字符串格式化后存储到buf中。
		第一个字符串是"predicted position:(%3d,%3d)"，其中%3d表示输出一个3位的整数，predictpt.x和predictpt.y是两个整数变量。
		第二个字符串是"current position :(%3d,%3d)"，其中%3d表示输出一个3位的整数，mousePosition.x和mousePosition.y是两个整数变量。
		最后，使用putText函数将buf中的字符串绘制到图像上，位置分别为(10,30)和(10,60)。绘制的字体为CVFONTHERSHEYSCRIPTCOMPLEX，
		大小为1，颜色为黑色，线宽为1，线型为8。
		*/
		char buf[256];
		snprintf(buf, 256, "predicted position:(%3d,%3d)", predict_pt.x, predict_pt.y);
		/*putText函数，用于在图像上绘制文本*/
		putText(image, buf, Point(10, 30), CV_FONT_HERSHEY_SCRIPT_COMPLEX, 1, Scalar(0, 0, 0), 1, 8);   //它将字符串变量buf中的内容绘制在图像image上
		snprintf(buf, 256, "current position :(%3d,%3d)", mousePosition.x, mousePosition.y);
		putText(image, buf, cvPoint(10, 60), CV_FONT_HERSHEY_SCRIPT_COMPLEX, 1, Scalar(0, 0, 0), 1, 8);

		imshow("kalman", image);
		int key = waitKey(3);
		if (key == 27) {//esc   
			break;
		}
	}
}