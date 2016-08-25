#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/video/tracking.hpp"
#include <opencv2/ml/ml.hpp>
#include<opencv2/video/video.hpp>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <iostream>
#include <io.h>
#include <vector>
#include <string>
#include <fstream>
#include <math.h>

using namespace std;
using namespace cv;
//---------------------将要调用的函数
vector<Rect>  CarSift(vector<Rect> boundrect, Mat sequence,int requestType);
int CarDetection(Mat roi);
Mat sobelExectue(Mat &sequence1);
int getD(int rectny,bool selectObject,Rect selection,Point origin);//根据论文获得距离
void savePoint(Point &pnew,Rect rect,int d,vector<Point> &rectps,int Threshold,int 
	&countNum,int typeXL);
//ZhenMain为传入的Rect数组，sequence2为原gray图，sequence21为原RGB图，type为车辆种类
vector<Rect> meanshiftTYC(vector<Rect> ZhenMain,Mat &sequence2,Mat &sequence21,int typeXL);
void compressiveOne(Rect selectionFromXL,Mat &sequence2);//compressivetracker代码
//------------------------将要使用的共有的常量

int type1=0,type2=0,type3=0;//检测出的三种车型
bool selectObject = false;//代表是否在选要跟踪的初始目标，true表示正在用鼠标选择
int trackObjectB = 0; //代表跟踪目标数目,trackObject用在了MeanShift中，所以这里用trackObjectB
//sequence1,sequence2,sequence3分别相差一帧,sequence21、3为RGB图,sequence1、2为灰度图
Mat sequence1, sequence2,sequence21,sequence3,dst;


Point origin;//用于保存鼠标选择第一次单击时点的位置
Point result;//用于保存鼠标选择送开时点的位置
Rect selection;//用于保存鼠标选择的矩形框
VideoCapture cap;
//------------------------compressive track----

/************************************************************************/
/*计数论文相关代码《工业控制计算机》27卷7期 基于车辆分类器的视频车流量检测》                                                                   */
/************************************************************************/
int Threshold=50;//车辆间距！可以手动设置!
vector<Point> rectps1,rectps2,rectps3;
//map<int,Point> rectps2;
int countNum1=0,countNum2=0,countNum3=0;//////计数！
void on_mouse(int event, int x, int y, int flags, void* ustc)//event代表发生的事件，x，y代表事件发生时鼠标的位置
{
	static Point pre_pt = (-1,-1);//初始坐标  
	static Point cur_pt = (-1,-1);//实时坐标  
	if(selectObject)
	{
		selection.x = MIN(x, origin.x);//矩形左上角顶点坐标
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);//矩形宽
		selection.height = std::abs(y - origin.y);//矩形高
		selection &= Rect(0, 0, sequence2.cols, sequence2.rows);//用于确保所选的矩形区域在图片范围内
	}
	switch( event )
	{
	case CV_EVENT_LBUTTONDOWN:
		origin = Point(x,y);
		selection = Rect(x,y,0,0);//鼠标刚按下去时初始化了一个矩形区域
		selectObject = true;
		break;
	case CV_EVENT_LBUTTONUP:
		selectObject = false;
		result=Point(x,y);
		if( selection.width > 0 && selection.height > 0 )
			trackObjectB = -1;
		break;
	}
	//if( selectObject && selection.width > 0 && selection.height > 0 )
	//{
	//	Mat roi(sequence2, selection);
	//	bitwise_not(roi, roi);//bitwise_not为将每一个bit位取反
	//}
	
}  



const char* keys = 
{
	"{1|  | 0 | camera number}"
};

class BackModel
{
private:
	int serial;  //计数，用于表示当前处理到视频序列中的第几帧
	char *video;  //读入的视频位置
	//Mat sequence3;  //视频序列
	Mat foreGround;  //前景图
public:
	BackModel() { serial = 0; }
	void setVideoPath(char *path) { video = path; }
	void BackMode();  //高斯背景建模

	
};


void BackModel::BackMode()
{
	Mat dstimg1, dstimg2, dstimg3;
	BackgroundSubtractorMOG mog;
	cap.open(video);
	while (waitKey(1) != 27)
	{
		cap >> sequence1;
		if (!sequence1.data)
			break;
		int H=500;		
		int W = H*sequence1.cols/sequence1.rows;		
		resize(sequence1, sequence1, Size(W, H));//重新确定窗口大小
		cvtColor(sequence1, sequence1, CV_BGR2GRAY);
		cap >> sequence2;
		sequence2.copyTo(sequence21);
		resize(sequence21,sequence21,Size(W,H));
		if (!sequence2.data)
			break;
		resize(sequence2, sequence2, Size(W, H));
		cap>>sequence3;
		resize(sequence3, sequence3, Size(W, H));
		cvtColor(sequence2, sequence2, CV_BGR2GRAY);
		subtract(sequence1, sequence2, dstimg1);
		threshold(dstimg1, dstimg1, 20, 255, THRESH_BINARY);
		//高斯
		mog(sequence2, dstimg2, 0.01);
		threshold(dstimg2, dstimg2, 50, 255, THRESH_BINARY);
		foreGround = Mat::zeros(dstimg2.size(), CV_8UC1);
		//合并（与）
		H=dstimg1.rows;
		W=dstimg1.cols;
		for (int i = 0;i < H;i++)
		{
			uchar *data = foreGround.ptr<uchar>(i);
			uchar *data1 = dstimg1.ptr<uchar>(i);
			uchar *data2 = dstimg2.ptr<uchar>(i);
			for (int j = 0;j < W;j++)
			{
				if (data1[j] ==255&&data2[j] == 255)//当二帧差法得到的图像像素大于5且背景建模得到的像素为255时为前景图
					data[j] = 255;
				else
					data[j] = 0;
			}
		}
		//膨胀5次
		Mat element = getStructuringElement(MORPH_RECT, Size(9, 9));
		dilate(foreGround, foreGround, element, Point(-1, -1), 5);
		erode(foreGround, foreGround, element, Point(-1, -1), 2);
		dilate(foreGround, foreGround, element, Point(-1, -1), 3);
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		findContours(foreGround, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
		vector<vector<Point>> contours_poly(contours.size());


		vector<Rect> boundrect(contours.size());
		int len=contours.size();
		for (size_t i = 0; i < len; i++)
		{
			approxPolyDP(Mat(contours[i]), contours_poly[i], 10, 1);  //倒数第二个参数越大，最后的矩形框越少。差别不是很大。。
			boundrect[i] = boundingRect(Mat(contours_poly[i]));
		}	
		///////////////熊林的检测（已经加入圆点）
		vector<Rect> boundrect1 = CarSift(boundrect, sequence2,1);//筛选有车的矩形,类型为1
		vector<Rect> boundrect2 = CarSift(boundrect, sequence2,2);
		vector<Rect> boundrect3 = CarSift(boundrect, sequence2,3);
		vector<Rect> resultRectTYC1=meanshiftTYC(boundrect1,sequence2,sequence21,1);//boundrect1为识别后每一帧的框（会闪烁） sequence2为grey图 sequence3为原图
		vector<Rect> resultRectTYC2=meanshiftTYC(boundrect2,sequence2,sequence21,2);
		vector<Rect> resultRectTYC3=meanshiftTYC(boundrect3,sequence2,sequence21,3);
		//在画线后才进行计数
		if(!selectObject&&selection.width>0&&selection.height>0)
		{
		
			for (size_t i = 0; i < resultRectTYC1.size(); i++)
			{
				int rectny=resultRectTYC1[i].tl().y;
				int d=getD(rectny,selectObject,selection,origin);
				savePoint(resultRectTYC1[i].tl(),resultRectTYC1[i],d,rectps1,Threshold,countNum1,1);
			}

			for (size_t i = 0; i < resultRectTYC2.size(); i++)
			{
				int rectny=resultRectTYC2[i].tl().y;
				int d=getD(rectny,selectObject,selection,origin);
				savePoint(resultRectTYC2[i].tl(),resultRectTYC2[i],d,rectps2,Threshold,countNum2,2);
			}

			for (size_t i = 0; i < resultRectTYC3.size(); i++)
			{
				int rectny=resultRectTYC3[i].tl().y;
				int d=getD(rectny,selectObject,selection,origin);
				savePoint(resultRectTYC3[i].tl(),resultRectTYC3[i],d,rectps3,Threshold,countNum3,3);
			}
		}
		
		namedWindow("sequence2");
		setMouseCallback("sequence2",on_mouse,0);

		if(!selectObject&&selection.width>0&&selection.height>0)
		{
			int width = abs(origin.x - result.x);  
			int height = abs(origin.y - result.y);  
			if (width == 0 || height == 0)  
			{  

				printf("width == 0 || height == 0");  
				return;  
			}  
			dst = sequence2(Rect(min(result.x,origin.x),min(result.y,origin.y),width,height));  
			line(sequence2,origin,Point(result.x,origin.y),Scalar(30,70,150),2,8,0);
		}
		imshow("sequence2",sequence2);
	}
}



//判断有无车或者什么车.
/************************************************************************/
/* sequence为原图像，boundrect为根据车辆检测轮廓画的矩形框                                                                     */
/************************************************************************/
vector<Rect>  CarSift(vector<Rect> boundrect, Mat sequence,int requestType)
{
	vector<Rect> boundrect1,boundrect2,boundrect3;
	/************************************************************************/
	/* 刚开始，不分类                                                                     */
	/************************************************************************/
	for(int i = 0; i < boundrect.size(); i++)
	{
		if(boundrect[i].area() < 15000 )//|| boundrect[i].area() > 40000)//初筛选，去掉极大或极小的运动物体
		{
			boundrect.erase(boundrect.begin() + i);
			i--;
		}
	}
		for(int i = 0; i < boundrect.size(); i++)				//区分不同车型
		{
			Mat roi = sequence(Rect(boundrect[i].x, boundrect[i].y, boundrect[i].width, boundrect[i].height));
			int result = CarDetection(roi); 
			if(result == 1)
				boundrect1.push_back(boundrect[i]);
			else if(result == 2)
				boundrect2.push_back(boundrect[i]);
			else if(result == 3)
				boundrect3.push_back(boundrect[i]);
			else
			{
				boundrect.erase(boundrect.begin() + i);
				i--;
			}
		}
		/************************************************************************/
		/* 分类画框                                                                     */
		/************************************************************************/
		if (requestType==1)
		{
			return boundrect1;

		}
		else if(requestType==2)
		{
			return boundrect2;

		}
		else if(requestType==3)
		{
			return boundrect3;
		}
}

