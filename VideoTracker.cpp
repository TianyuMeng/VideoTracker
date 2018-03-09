#include <opencv2/highgui/highgui.hpp>    
#include <opencv2/imgproc/imgproc.hpp>    
#include <opencv2/core/core.hpp>    
#include <opencv2/tracking.hpp>
#include<iostream>
#include <sys/time.h>//计时
#include <wiringPi.h>   

using namespace cv;
using namespace std;

const int GEARpin = 1;

void OnMouseAction(int event, int x, int y, int flags, void *ustc);  //鼠标回调事件函数

int rowsBegin = 0, rowsEnd = 100; //Y轴
int colsBegin = 0, colsEnd = 100; //X轴
//用来记录鼠标点击的起始位置

int cent_X = 320;
int cent_Y = 0;
//全局变量，跟踪目标的质心

bool isFound = false;
int nextFrame = 0;//全局变量，处理好一帧后的标志

Mat frame;
bool stop = false;

struct timeval t1, t2;//用于计时


/**************副线程处理目标追踪***********************/
void *thread(void *ptr)
{
	if (-1 == wiringPiSetup())
	{
		cout << "wiringPi setup error" << endl;
		//return -1;
	}

	pinMode(GEARpin, OUTPUT);

	int highTime = 1500;

	for (int i = 0; i < 10; i++)
	{
		digitalWrite(GEARpin, HIGH);
		delayMicroseconds(highTime);
		digitalWrite(GEARpin, LOW);
		delayMicroseconds(3333 - highTime);
	}


	cout << "The Gear is online" << endl;

	while (1)
	{
		if (nextFrame)
		{
			if (cent_X < 240)
			{
				highTime = highTime + 2;
				for (int i = 0; i < 2; i++)
				{
					digitalWrite(GEARpin, HIGH);
					delayMicroseconds(highTime);
					digitalWrite(GEARpin, LOW);
					delayMicroseconds(3333 - highTime);
				}
				cout << "The Gear is moving to LEFT==== HighTime : " << highTime << endl;
			}
			else if (cent_X >400)
			{
				highTime = highTime - 2;
				for (int i = 0; i < 2; i++)
				{
					digitalWrite(GEARpin, HIGH);
					delayMicroseconds(highTime);
					digitalWrite(GEARpin, LOW);
					delayMicroseconds(3333 - highTime);
				}
				cout << "The Gear is moving to RIGHT==== HighTime : " << highTime << endl;

			}
			nextFrame = 0;
		}//每获取一帧转一次

		
		if (highTime<600 || highTime>2300)
		{
			cout << "The Gear is offline" << endl;
			break;
		}

	}
	return 0;
}



int main(int argc, char *argv[])
{

	Ptr<Tracker> tracker;
	tracker = TrackerKCF::create();


	VideoCapture cap(0);//打开USB摄像头  
	if (!cap.isOpened())
	{
		cout << "No found Camra" << endl;
		return -1;
	}

	int getKey=0;
	Mat frame_showRect;

	pthread_t id;
	int ret = pthread_create(&id, NULL, thread, NULL);//打开舵机
	if (ret) {
		cout << "Create pthread error!" << endl;
		return 1;
	}

	while (!stop)
	{
		cap.read(frame); //  或cap>>frame;           
		//cout << "x=" << frame.cols << "  y=" << frame.rows << endl;
		imshow("Video", frame);

		getKey = waitKey(30);
		if (getKey == 10) //回车键笔记本为13，树莓派为10
		{
			cout << "The Enter has been put down" << endl
				<< "Please set Selection area." << endl;
			frame.copyTo(frame_showRect);
			rectangle(frame_showRect, Rect(colsBegin, rowsBegin, colsEnd - colsBegin, rowsEnd - rowsBegin), Scalar(0, 255, 0), 2, 8, 0);
			imshow("Video", frame_showRect);

			//鼠标设置选区
			//setMouseCallback("Video", OnMouseAction);
			
			//键盘设置选区
			while (!stop)
			{
				getKey = waitKey(1);
				if (getKey != -1)
				{
					switch (getKey)
					{
					case 81://左方向键
						if (colsBegin > 0)
						{
							colsBegin-=5;
							colsEnd-=5;
						}
						break;
					case 82://上方向键
						if (rowsBegin > 0)
						{
							rowsBegin-=5;
							rowsEnd-=5;
						}
						break;
					case 83://右方向键
						if (colsEnd + 5 < 640)
						{
							colsBegin+=5;
							colsEnd+=5;
						}
						break;
					case 84:
						if (rowsEnd + 5 < 480)
						{
							rowsBegin+=5;
							rowsEnd+=5;
						}
						break;
					default:
						stop = true;
						break;
					}
					frame.copyTo(frame_showRect);
					rectangle(frame_showRect, Rect(colsBegin, rowsBegin, colsEnd - colsBegin, rowsEnd - rowsBegin), Scalar(0, 255, 0), 2, 8, 0);
					imshow("Video", frame_showRect);
				}
			}
			//waitKey(0);//鼠标选区模式下，用于拦截结束选择的按键
			stop = true;
		}

	}

	stop = false;

	//划定ROI之后进行以下的视频输出
	Rect2d box(colsBegin, rowsBegin, colsEnd - colsBegin, rowsEnd - rowsBegin);//

	Point p(colsBegin + (colsEnd - colsBegin) / 2, rowsBegin + (rowsEnd - rowsBegin) / 2);
	cent_X = p.x;
	cent_Y = p.y;


	tracker->init(frame, box);

	while (!stop)
	{
		gettimeofday(&t1, NULL);//计时开始

		cap.read(frame); //  或cap>>frame;   

		isFound = tracker->update(frame, box);

		p.x = box.x + box.width / 2;
		p.y = box.y + box.height / 2;//更新中心点位置

		cent_X = p.x;
		cent_Y = p.y;

		nextFrame = 1;//通知副线程转动
		
		if (isFound)
		{
			rectangle(frame, box, Scalar(0, 0, 255), 2, 1);//画红色矩形
		}
		else
		{
			rectangle(frame, box, Scalar(0, 255, 0), 2, 1);//画绿色矩形
		}
		circle(frame, p, 5, Scalar(0, 0, 255), -1);//画点

		line(frame, Point(240, 0), Point(240, 480), Scalar(255, 0, 0), 2);
		line(frame, Point(400, 0), Point(400, 480), Scalar(255, 0, 0), 2);

		//cout << "X = " << p.x -320 << "  Y = " << p.y - 240 << endl;
		cout << "X = " << p.x << "  Y = " << p.y << endl;

		//rectangle(frame, Rect(colsBegin, rowsBegin, colsEnd - colsBegin, rowsEnd - rowsBegin), Scalar(0, 255, 0), 2, 8, 0);

		imshow("Video", frame);

		getKey = waitKey(1);//因为追踪时图像处理速度慢，此处不设置帧间间隔




		if (getKey == 27)//Esc键退出  
		{
			stop = true;
		}

		gettimeofday(&t2, NULL);//计时结束
		cout << "Time spend : " << ((t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec) / 1000 << " ms" << endl;

	}
	return 0;
}

//*******************************************************************//
//鼠标回调函数
void OnMouseAction(int event, int x, int y, int flags, void *ustc)
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		rowsBegin = y;
		colsBegin = x;
		cout << "触发左键按下事件" << endl;
	}
	if (event == CV_EVENT_LBUTTONUP)
	{
		rowsEnd = y;
		colsEnd = x;
		//line(image, Point(colsBegin, rowsBegin), Point(colsEnd, rowsEnd), Scalar(255, 0, 0), 2, 8, 0);
		rectangle(frame, Rect(colsBegin, rowsBegin, colsEnd - colsBegin, rowsEnd - rowsBegin), Scalar(0, 255, 0), 2, 8, 0);

		cout << "x = " << colsBegin << " -- " << colsEnd << endl;

		cout << "y = " << rowsBegin << " -- " << rowsEnd << endl;


		//circle(image, Point((colsEnd + colsBegin) / 2, (rowsEnd + rowsBegin) / 2), rowsEnd - rowsBegin, Scalar(0, 0, 255), 2, 8, 0);
		imshow("Video", frame);
		stop = true;

		cout << "触发左键抬起事件" << endl;
	}

	/*
	if (event == CV_EVENT_MOUSEMOVE)
	{
	//cout << "触发鼠标移动事件" << endl;
	}
	if (event == CV_EVENT_RBUTTONDOWN)
	{
		cout << "触发右键按下事件" << endl;
	}
	if (event == CV_EVENT_RBUTTONUP)
	{
		cout << "触发右键抬起事件" << endl;
	}
	if (event == CV_EVENT_LBUTTONDBLCLK)
	{
		cout << "触发左键双击事件" << endl;
	}
	if (event == CV_EVENT_RBUTTONDBLCLK)
	{
		cout << "触发右键双击事件" << endl;
	}*/
}
