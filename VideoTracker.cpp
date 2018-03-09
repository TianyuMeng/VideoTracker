#include <opencv2/highgui/highgui.hpp>    
#include <opencv2/imgproc/imgproc.hpp>    
#include <opencv2/core/core.hpp>    
#include <opencv2/tracking.hpp>
#include<iostream>
#include <sys/time.h>//��ʱ
#include <wiringPi.h>   

using namespace cv;
using namespace std;

const int GEARpin = 1;

void OnMouseAction(int event, int x, int y, int flags, void *ustc);  //���ص��¼�����

int rowsBegin = 0, rowsEnd = 100; //Y��
int colsBegin = 0, colsEnd = 100; //X��
//������¼���������ʼλ��

int cent_X = 320;
int cent_Y = 0;
//ȫ�ֱ���������Ŀ�������

bool isFound = false;
int nextFrame = 0;//ȫ�ֱ����������һ֡��ı�־

Mat frame;
bool stop = false;

struct timeval t1, t2;//���ڼ�ʱ


/**************���̴߳���Ŀ��׷��***********************/
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
		}//ÿ��ȡһ֡תһ��

		
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


	VideoCapture cap(0);//��USB����ͷ  
	if (!cap.isOpened())
	{
		cout << "No found Camra" << endl;
		return -1;
	}

	int getKey=0;
	Mat frame_showRect;

	pthread_t id;
	int ret = pthread_create(&id, NULL, thread, NULL);//�򿪶��
	if (ret) {
		cout << "Create pthread error!" << endl;
		return 1;
	}

	while (!stop)
	{
		cap.read(frame); //  ��cap>>frame;           
		//cout << "x=" << frame.cols << "  y=" << frame.rows << endl;
		imshow("Video", frame);

		getKey = waitKey(30);
		if (getKey == 10) //�س����ʼǱ�Ϊ13����ݮ��Ϊ10
		{
			cout << "The Enter has been put down" << endl
				<< "Please set Selection area." << endl;
			frame.copyTo(frame_showRect);
			rectangle(frame_showRect, Rect(colsBegin, rowsBegin, colsEnd - colsBegin, rowsEnd - rowsBegin), Scalar(0, 255, 0), 2, 8, 0);
			imshow("Video", frame_showRect);

			//�������ѡ��
			//setMouseCallback("Video", OnMouseAction);
			
			//��������ѡ��
			while (!stop)
			{
				getKey = waitKey(1);
				if (getKey != -1)
				{
					switch (getKey)
					{
					case 81://�����
						if (colsBegin > 0)
						{
							colsBegin-=5;
							colsEnd-=5;
						}
						break;
					case 82://�Ϸ����
						if (rowsBegin > 0)
						{
							rowsBegin-=5;
							rowsEnd-=5;
						}
						break;
					case 83://�ҷ����
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
			//waitKey(0);//���ѡ��ģʽ�£��������ؽ���ѡ��İ���
			stop = true;
		}

	}

	stop = false;

	//����ROI֮��������µ���Ƶ���
	Rect2d box(colsBegin, rowsBegin, colsEnd - colsBegin, rowsEnd - rowsBegin);//

	Point p(colsBegin + (colsEnd - colsBegin) / 2, rowsBegin + (rowsEnd - rowsBegin) / 2);
	cent_X = p.x;
	cent_Y = p.y;


	tracker->init(frame, box);

	while (!stop)
	{
		gettimeofday(&t1, NULL);//��ʱ��ʼ

		cap.read(frame); //  ��cap>>frame;   

		isFound = tracker->update(frame, box);

		p.x = box.x + box.width / 2;
		p.y = box.y + box.height / 2;//�������ĵ�λ��

		cent_X = p.x;
		cent_Y = p.y;

		nextFrame = 1;//֪ͨ���߳�ת��
		
		if (isFound)
		{
			rectangle(frame, box, Scalar(0, 0, 255), 2, 1);//����ɫ����
		}
		else
		{
			rectangle(frame, box, Scalar(0, 255, 0), 2, 1);//����ɫ����
		}
		circle(frame, p, 5, Scalar(0, 0, 255), -1);//����

		line(frame, Point(240, 0), Point(240, 480), Scalar(255, 0, 0), 2);
		line(frame, Point(400, 0), Point(400, 480), Scalar(255, 0, 0), 2);

		//cout << "X = " << p.x -320 << "  Y = " << p.y - 240 << endl;
		cout << "X = " << p.x << "  Y = " << p.y << endl;

		//rectangle(frame, Rect(colsBegin, rowsBegin, colsEnd - colsBegin, rowsEnd - rowsBegin), Scalar(0, 255, 0), 2, 8, 0);

		imshow("Video", frame);

		getKey = waitKey(1);//��Ϊ׷��ʱͼ�����ٶ������˴�������֡����




		if (getKey == 27)//Esc���˳�  
		{
			stop = true;
		}

		gettimeofday(&t2, NULL);//��ʱ����
		cout << "Time spend : " << ((t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec) / 1000 << " ms" << endl;

	}
	return 0;
}

//*******************************************************************//
//���ص�����
void OnMouseAction(int event, int x, int y, int flags, void *ustc)
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		rowsBegin = y;
		colsBegin = x;
		cout << "������������¼�" << endl;
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

		cout << "�������̧���¼�" << endl;
	}

	/*
	if (event == CV_EVENT_MOUSEMOVE)
	{
	//cout << "��������ƶ��¼�" << endl;
	}
	if (event == CV_EVENT_RBUTTONDOWN)
	{
		cout << "�����Ҽ������¼�" << endl;
	}
	if (event == CV_EVENT_RBUTTONUP)
	{
		cout << "�����Ҽ�̧���¼�" << endl;
	}
	if (event == CV_EVENT_LBUTTONDBLCLK)
	{
		cout << "�������˫���¼�" << endl;
	}
	if (event == CV_EVENT_RBUTTONDBLCLK)
	{
		cout << "�����Ҽ�˫���¼�" << endl;
	}*/
}
