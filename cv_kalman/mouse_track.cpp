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
	if (event == CV_EVENT_MOUSEMOVE) {   ///CV_EVENT_MOUSEMOVE ������ƶ�
		mousePosition = Point(x, y);
	}
}

int main(void)
{
	RNG rng;
	//1.kalman filter setup
	const int stateNum = 4;                                      //״ֵ̬4��1����(x,y,��x,��y)
	const int measureNum = 2;                                    //����ֵ2��1����(x,y)	
	KalmanFilter KF(stateNum, measureNum, 0);

	/*��ʼ��״̬ת�ƾ���A �� ��������H*/
	KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);  //ת�ƾ���A
	setIdentity(KF.measurementMatrix);                                             //��������H

	/*����������Э�������*/
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //ϵͳ�����������Q
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //���������������R

	/*0ʱ�̵ĺ���״̬��Э����*/
	rng.fill(KF.statePost, RNG::UNIFORM, 0, winHeight > winWidth ? winWidth : winHeight);   //��ʼ״ֵ̬x(0)
	setIdentity(KF.errorCovPost, Scalar::all(1));                                  //�������Э�������P

	Mat measurement = Mat::zeros(measureNum, 1, CV_32F);           //��������z(k)

	namedWindow("kalman");
	setMouseCallback("kalman", mouseEvent);

	Mat image(winHeight, winWidth, CV_8UC3, Scalar(0));

	while (1)
	{
		//2.kalman prediction
		Mat prediction = KF.predict();
		Point predict_pt = Point(prediction.at<float>(0), prediction.at<float>(1));   //Ԥ��ֵ(x',y')

		//3.update measurement
		measurement.at<float>(0) = (float)mousePosition.x;
		measurement.at<float>(1) = (float)mousePosition.y;

		//4.update
		KF.correct(measurement);

		//draw 
		image.setTo(Scalar(255, 255, 255, 0));   //���ｫͼ�������ֵ����Ϊ��ɫ
		circle(image, predict_pt, 5, Scalar(0, 255, 0), 3);    //��ɫ����Ԥ��λ��
		circle(image, mousePosition, 5, Scalar(255, 0, 0), 3); //��ɫ������ȷλ��	

		/*
		���ȶ�����һ��char���͵�����buf����СΪ256��Ȼ��ʹ��snprintf�������ַ�����ʽ����洢��buf�С�
		��һ���ַ�����"predicted position:(%3d,%3d)"������%3d��ʾ���һ��3λ��������predictpt.x��predictpt.y����������������
		�ڶ����ַ�����"current position :(%3d,%3d)"������%3d��ʾ���һ��3λ��������mousePosition.x��mousePosition.y����������������
		���ʹ��putText������buf�е��ַ������Ƶ�ͼ���ϣ�λ�÷ֱ�Ϊ(10,30)��(10,60)�����Ƶ�����ΪCVFONTHERSHEYSCRIPTCOMPLEX��
		��СΪ1����ɫΪ��ɫ���߿�Ϊ1������Ϊ8��
		*/
		char buf[256];
		snprintf(buf, 256, "predicted position:(%3d,%3d)", predict_pt.x, predict_pt.y);
		/*putText������������ͼ���ϻ����ı�*/
		putText(image, buf, Point(10, 30), CV_FONT_HERSHEY_SCRIPT_COMPLEX, 1, Scalar(0, 0, 0), 1, 8);   //�����ַ�������buf�е����ݻ�����ͼ��image��
		snprintf(buf, 256, "current position :(%3d,%3d)", mousePosition.x, mousePosition.y);
		putText(image, buf, cvPoint(10, 60), CV_FONT_HERSHEY_SCRIPT_COMPLEX, 1, Scalar(0, 0, 0), 1, 8);

		imshow("kalman", image);
		int key = waitKey(3);
		if (key == 27) {//esc   
			break;
		}
	}
}