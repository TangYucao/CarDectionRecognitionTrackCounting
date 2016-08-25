#include "OPENCV_TYC_FORM.h"
/************************************************************************/
/*Author:唐雨操 2016.8
/*来源：改编自
/*《工业控制计算机》27卷7期 《基于车辆分类器的视频车流量检测》                                                                   */
/************************************************************************/

int getD(int rectny,bool selectObject,Rect selection,Point origin)
{
	int dd;
	if(!selectObject&&selection.width>0&&selection.height>0)
	{
		int local_y=origin.y;
		dd=local_y-rectny;

	}else 
	{
		dd=0;
	}

	return dd;

}
//返回值，如果找到了以前的就返回位置i，如果没有，就返回-1
int isSameCar(Point pnew,vector<Point> rectps,int Threshold)
{
	size_t len = rectps.size();

	for (size_t i =0; i < len; i ++) {

		Point tmp = rectps[i];
		if (abs(pnew.x-tmp.x)<Threshold&&pnew.y-tmp.y>0)//这一步还有问题！有些框会突然增大。
		{
			return i;
		}
	}
	return -1;
}

void savePoint(Point &pnew,Rect rect,int d,vector<Point> &rectps,int Threshold,int &countNum,int typeXL)
{
	if(0<d&&d<rect.height)
	{
		if (rectps.size()==0)
		{
			rectps.push_back(pnew);
		}
		else
		{
			int issamecarInt=isSameCar(pnew,rectps,Threshold);

			//
			
			//
			if(issamecarInt!=-1)//与之前同一辆车
			{
				vector<Point>::const_iterator itr=rectps.begin()+issamecarInt;
				
				rectps[issamecarInt]=pnew;
				//替代rectps[countNum]
			}
			else if(issamecarInt==-1)
			{
				//新的！
				rectps.push_back(pnew);
			}
		}


	}
	countNum=rectps.size();
	if (typeXL==1)
	{
		cout<<"countNum1:"<<countNum<<endl;
	}
	else if (typeXL==2)
	{
		cout<<"countNum2:"<<countNum<<endl;
	}
	else if (typeXL==3)
	{
		cout<<"countNum3:"<<countNum<<endl;
	}
	else cout<<"countNum全部:"<<countNum<<endl;
}