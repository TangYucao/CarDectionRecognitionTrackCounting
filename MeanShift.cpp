#include "OPENCV_TYC_FORM.h"
#include "Merge.h"
/************************************************************************/
/*Author:唐雨操 2016.8
/*来源：改编自
/*林耀的meanShift多目标跟踪                                               */
/************************************************************************/
//这是多个同时跟踪！！此时，熊林的识别已经识别出来了，准备画框，执行了这个函数.
vector<Rect> selectionLY1,selectionLY2,selectionLY3;//接受发来的每帧的ZhenMain,如果是同一辆车，那么替换原来的元素！
//vector<Rect> trackWindow;//绘图用的！最终的结果
int i=0;//此时画面中的框的个数！与trackWindow元素个数相对应！
///------------------------------常量----------
const int size=10;
int trackObject1 = 0,trackObject2 = 0,trackObject3 = 0;//很重要！hist_change[]与他挂钩。
int vmin = 10, vmax = 256, smin = 30;//对hsv通道范围限定
bool paused = false;
int hsize = 16;
float hranges[] = {0,180};//hranges在后面的计算直方图函数中要用到
const float* phranges = hranges;
double ThresholdGENZONG=0.9;
Mat frame1, hsv1, hue1, mask1[size], hist1[size],hist_change1[size], histimg1 = Mat::zeros(200, 320, CV_8UC3), backproj1[size],back1;
Mat frame2, hsv2, hue2, mask2[size], hist2[size],hist_change2[size], histimg2 = Mat::zeros(200, 320, CV_8UC3), backproj2[size],back2;
Mat frame3, hsv3, hue3, mask3[size], hist3[size],hist_change3[size], histimg3 = Mat::zeros(200, 320, CV_8UC3), backproj3[size],back3;

vector<Rect> meanshiftTYC(vector<Rect> ZhenMain,Mat &sequence2,Mat &sequence21,int typeXL)

{
	if (typeXL==1)
	{
		//每一帧，都去接受所有方框，因为后面有判断，是不是同一辆车

		size_t len = ZhenMain.size();
		for (size_t k =0; k < len; k ++) {

			Rect tmp = ZhenMain[k];
			if (!selectionLY1.empty())
			{
				selectionLY1.push_back(tmp);
				i++;
				trackObject1=-1;
			}
			else
			{
				selectionLY1.push_back(tmp);
				i++;
				trackObject1=-1;
			}


		}

		len=selectionLY1.size();
		for (size_t k =0; k < len; k ++)
		{
			if (selectionLY1[k].area()>0)
			{
				Point br=selectionLY1[k].br();
				if (br.y>=sequence2.rows)
				{
					selectionLY1[k]=selectionLY1[k+1];
				}
			}

		}


		i=selectionLY1.size();
		len=selectionLY1.size();
		if(i>0)
		{





			cvtColor(sequence21, hsv1, CV_BGR2HSV);//将rgb摄像头帧转化成hsv空间的


			int _vmin = vmin, _vmax = vmax;

			//inRange函数的功能是检查输入数组每个元素大小是否在2个给定数值之间，可以有多通道,mask保存0通道的最小值，也就是h分量
			//这里利用了hsv的3个通道，比较h,0~180,s,smin~256,v,min(vmin,vmax),max(vmin,vmax)。如果3个通道都在对应的范围内，则
			//mask对应的那个点的值全为1(0xff)，否则为0(0x00).
			for(int j=0;j<i;j++)
			{
				inRange(hsv1, Scalar(0, smin, MIN(_vmin,_vmax)),Scalar(180, 256, MAX(_vmin, _vmax)), mask1[j]);
			}	
			int ch[] = {0, 0};
			hue1.create(hsv1.size(), hsv1.depth());//hue初始化为与hsv大小深度一样的矩阵，色调的度量是用角度表示的，红绿蓝之间相差120度，反色相差180度
			mixChannels(&hsv1, 1, &hue1, 1, ch, 1);//将hsv第一个通道(也就是色调)的数复制到hue中，0索引数组
			//按到第8下报错
			//此处的构造函数roi用的是Mat hue的矩阵头，且roi的数据指针指向hue，即共用相同的数据，select为其感兴趣的区域
			for(int j=0;j<selectionLY1.size();j++)
			{
				//trackWindow.push_back(selectionLY[j]);
			}
			//处理mask
			for(int j=0;j<selectionLY1.size();j++)
			{
				Mat roi(hue1,selectionLY1[j]);
				Mat maskroi(mask1[j],selectionLY1[j]);//mask保存的hsv的最小值


				//calcHist()函数第一个参数为输入矩阵序列，第2个参数表示输入的矩阵数目，第3个参数表示将被计算直方图维数通道的列表，第4个参数表示可选的掩码函数
				//第5个参数表示输出直方图，第6个参数表示直方图的维数，第7个参数为每一维直方图数组的大小，第8个参数为每一维直方图bin的边界
				calcHist(&roi, 1, 0, maskroi, hist1[j], 1, &hsize, &phranges);//将roi的0通道计算直方图并通过mask放入hist中，hsize为每一维直方图的大小
				if(trackObject1<0)
				{
					hist_change1[j]=hist1[j];
				}
				normalize(hist1[j], hist1[j], 0, 255, CV_MINMAX);//将hist矩阵进行数组范围归一化，都归一化到0~255
			}
			if (selectionLY1.size()>=2)
			{
				WindowsMerge windowsMerge;
				selectionLY1=windowsMerge.merge(selectionLY1);

			}
			len=selectionLY1.size();
			for(int j=0;j<len;j++){
				calcBackProject(&hue1, 1, 0, hist1[j], backproj1[j], &phranges);//计算直方图的反向投影，计算hue图像0通道直方图hist的反向投影，并让入backproj中
				backproj1[j] &= mask1[j];//问题出在mask的尺寸上。

				meanShift(backproj1[j], selectionLY1[j],               //trackWindow为鼠标选择的区域，TermCriteria为确定迭代终止的准则
					TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));//CV_TERMCRIT_EPS是通过forest_accuracy,CV_TERMCRIT_ITER

				if (selectionLY1[j].area()>0)
				{
					Point br=selectionLY1[j].br();
					if (br.y>=sequence2.rows)
					{
						vector<Rect>::const_iterator itr=selectionLY1.begin()+j;
						selectionLY1.erase(itr);
					}
				}
				len=selectionLY1.size();
			}
		}
		len=selectionLY1.size();
		if (len>0)
		{
			for (int j=0;j<len;j++)
			{
				rectangle(sequence2,selectionLY1[j],Scalar(0),2);
			}
		}
		return selectionLY1;
	}




	else if (typeXL==2)
	{
		//每一帧，都去接受所有方框，因为后面有判断，是不是同一辆车

		size_t len = ZhenMain.size();
		for (size_t k =0; k < len; k ++) {

			Rect tmp = ZhenMain[k];
			selectionLY2.push_back(tmp);
			trackObject2=-1;
		


		}

		


		i=selectionLY2.size();
		len=selectionLY2.size();
		if(i>0)
		{





			cvtColor(sequence21, hsv2, CV_BGR2HSV);//将rgb摄像头帧转化成hsv空间的


			int _vmin = vmin, _vmax = vmax;

			//inRange函数的功能是检查输入数组每个元素大小是否在2个给定数值之间，可以有多通道,mask保存0通道的最小值，也就是h分量
			//这里利用了hsv的3个通道，比较h,0~180,s,smin~256,v,min(vmin,vmax),max(vmin,vmax)。如果3个通道都在对应的范围内，则
			//mask对应的那个点的值全为1(0xff)，否则为0(0x00).
			for(int j=0;j<i;j++)
			{
				inRange(hsv2, Scalar(0, smin, MIN(_vmin,_vmax)),Scalar(180, 256, MAX(_vmin, _vmax)), mask2[j]);
			}	
			int ch[] = {0, 0};
			hue2.create(hsv2.size(), hsv2.depth());//hue初始化为与hsv大小深度一样的矩阵，色调的度量是用角度表示的，红绿蓝之间相差120度，反色相差180度
			mixChannels(&hsv2, 1, &hue2, 1, ch, 1);//将hsv第一个通道(也就是色调)的数复制到hue中，0索引数组
			//按到第8下报错
			//此处的构造函数roi用的是Mat hue的矩阵头，且roi的数据指针指向hue，即共用相同的数据，select为其感兴趣的区域
			for(int j=0;j<selectionLY2.size();j++)
			{
				//trackWindow.push_back(selectionLY[j]);
			}
			//处理mask
			for(int j=0;j<selectionLY2.size();j++)
			{
				Mat roi(hue2,selectionLY2[j]);
				Mat maskroi(mask2[j],selectionLY2[j]);//mask保存的hsv的最小值


				//calcHist()函数第一个参数为输入矩阵序列，第2个参数表示输入的矩阵数目，第3个参数表示将被计算直方图维数通道的列表，第4个参数表示可选的掩码函数
				//第5个参数表示输出直方图，第6个参数表示直方图的维数，第7个参数为每一维直方图数组的大小，第8个参数为每一维直方图bin的边界
				calcHist(&roi, 1, 0, maskroi, hist2[j], 1, &hsize, &phranges);//将roi的0通道计算直方图并通过mask放入hist中，hsize为每一维直方图的大小
				if(trackObject2<0)
				{
					hist_change2[j]=hist2[j];
				}
				normalize(hist2[j], hist2[j], 0, 255, CV_MINMAX);//将hist矩阵进行数组范围归一化，都归一化到0~255
			}
			if (selectionLY2.size()>=2)
			{
				WindowsMerge windowsMerge;
				selectionLY2=windowsMerge.merge(selectionLY2);

			}
			len=selectionLY2.size();
			for(int j=0;j<len;j++){
				calcBackProject(&hue2, 1, 0, hist2[j], backproj2[j], &phranges);//计算直方图的反向投影，计算hue图像0通道直方图hist的反向投影，并让入backproj中
				backproj2[j] &= mask2[j];//问题出在mask的尺寸上。

				meanShift(backproj2[j], selectionLY2[j],               //trackWindow为鼠标选择的区域，TermCriteria为确定迭代终止的准则
					TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));//CV_TERMCRIT_EPS是通过forest_accuracy,CV_TERMCRIT_ITER

				//if( trackWindow[j].area() <= 1 ){
				//	int cols = backproj[j].cols, rows = backproj[j].rows, r = (MIN(cols, rows) + 5)/6;
				//	trackWindow[j] = Rect(trackWindow[j].x - r, trackWindow[j].y - r,
				//		trackWindow[j].x + r, trackWindow[j].y + r) &
				//		Rect(0, 0, cols, rows);//Rect函数为矩阵的偏移和大小，即第一二个参数为矩阵的左上角点坐标，第三四个参数为矩阵的宽和高
				//}
				//清除已经经过检测线框的车辆的Rect!
				if (selectionLY2[j].area()>0)
				{
					Point br=selectionLY2[j].br();
					if (br.y>=sequence2.rows)
					{
						vector<Rect>::const_iterator itr=selectionLY2.begin()+j;
						selectionLY2.erase(itr);
					}
				}
				len=selectionLY2.size();
			}
		}
		len=selectionLY2.size();
		if (len>0)
		{
			for (int j=0;j<len;j++)
			{
				rectangle(sequence2,selectionLY2[j],Scalar(0),2);
			}
		}
		return selectionLY2;

	}

	else if(typeXL==3)
	{
		//每一帧，都去接受所有方框，因为后面有判断，是不是同一辆车
		size_t len = ZhenMain.size();
		for (size_t k =0; k < len; k ++) {

			Rect tmp = ZhenMain[k];
			if (!selectionLY3.empty())
			{
				selectionLY3.push_back(tmp);
				i++;
				trackObject3=-1;
			}
			else//初始化
			{
				selectionLY3.push_back(tmp);
				i++;
				trackObject3=-1;
			}


		}

		len=selectionLY3.size();
		for (size_t k =0; k < len; k ++)
		{
			if (selectionLY3[k].area()>0)
			{
				Point br=selectionLY3[k].br();
				if (br.y>=sequence2.rows)
				{
					selectionLY3[k]=selectionLY3[k+1];
				}
			}

		}


		i=selectionLY3.size();
		len=selectionLY3.size();
		if(i>0)
		{





			cvtColor(sequence21, hsv3, CV_BGR2HSV);//将rgb摄像头帧转化成hsv空间的


			int _vmin = vmin, _vmax = vmax;

			//inRange函数的功能是检查输入数组每个元素大小是否在2个给定数值之间，可以有多通道,mask保存0通道的最小值，也就是h分量
			//这里利用了hsv的3个通道，比较h,0~180,s,smin~256,v,min(vmin,vmax),max(vmin,vmax)。如果3个通道都在对应的范围内，则
			//mask对应的那个点的值全为1(0xff)，否则为0(0x00).
			for(int j=0;j<i;j++)
			{
				inRange(hsv3, Scalar(0, smin, MIN(_vmin,_vmax)),Scalar(180, 256, MAX(_vmin, _vmax)), mask3[j]);
			}	
			int ch[] = {0, 0};
			hue3.create(hsv3.size(), hsv3.depth());//hue初始化为与hsv大小深度一样的矩阵，色调的度量是用角度表示的，红绿蓝之间相差120度，反色相差180度
			mixChannels(&hsv3, 1, &hue3, 1, ch, 1);//将hsv第一个通道(也就是色调)的数复制到hue中，0索引数组
			//按到第8下报错
			//此处的构造函数roi用的是Mat hue的矩阵头，且roi的数据指针指向hue，即共用相同的数据，select为其感兴趣的区域
			for(int j=0;j<selectionLY3.size();j++)
			{
				//trackWindow.push_back(selectionLY[j]);
			}
			//处理mask
			for(int j=0;j<selectionLY3.size();j++)
			{
				Mat roi(hue3,selectionLY3[j]);
				Mat maskroi(mask3[j],selectionLY3[j]);//mask保存的hsv的最小值


				//calcHist()函数第一个参数为输入矩阵序列，第2个参数表示输入的矩阵数目，第3个参数表示将被计算直方图维数通道的列表，第4个参数表示可选的掩码函数
				//第5个参数表示输出直方图，第6个参数表示直方图的维数，第7个参数为每一维直方图数组的大小，第8个参数为每一维直方图bin的边界
				calcHist(&roi, 1, 0, maskroi, hist3[j], 1, &hsize, &phranges);//将roi的0通道计算直方图并通过mask放入hist中，hsize为每一维直方图的大小
				if(trackObject3<0)
				{
					hist_change3[j]=hist3[j];
				}
				normalize(hist3[j], hist3[j], 0, 255, CV_MINMAX);//将hist矩阵进行数组范围归一化，都归一化到0~255
			}
			if (selectionLY3.size()>=2)
			{
				WindowsMerge windowsMerge;
				selectionLY3=windowsMerge.merge(selectionLY3);

			}
			len=selectionLY3.size();
			for(int j=0;j<len;j++){
				calcBackProject(&hue3, 1, 0, hist3[j], backproj3[j], &phranges);//计算直方图的反向投影，计算hue图像0通道直方图hist的反向投影，并让入backproj中
				backproj3[j] &= mask3[j];//问题出在mask的尺寸上。

				meanShift(backproj3[j], selectionLY3[j],               //trackWindow为鼠标选择的区域，TermCriteria为确定迭代终止的准则
					TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));//CV_TERMCRIT_EPS是通过forest_accuracy,CV_TERMCRIT_ITER

				//if( trackWindow[j].area() <= 1 ){
				//	int cols = backproj[j].cols, rows = backproj[j].rows, r = (MIN(cols, rows) + 5)/6;
				//	trackWindow[j] = Rect(trackWindow[j].x - r, trackWindow[j].y - r,
				//		trackWindow[j].x + r, trackWindow[j].y + r) &
				//		Rect(0, 0, cols, rows);//Rect函数为矩阵的偏移和大小，即第一二个参数为矩阵的左上角点坐标，第三四个参数为矩阵的宽和高
				//}
				//清除已经经过检测线框的车辆的Rect!
				if (selectionLY3[j].area()>0)
				{
					Point br=selectionLY3[j].br();
					if (br.y>=sequence2.rows)
					{
						vector<Rect>::const_iterator itr=selectionLY3.begin()+j;
						selectionLY3.erase(itr);
					}
				}
				len=selectionLY3.size();
			}
		}
		len=selectionLY3.size();
		if (len>0)
		{
			for (int j=0;j<len;j++)
			{
				rectangle(sequence2,selectionLY3[j],Scalar(0),2);
			}
		}
		return selectionLY3;
	}
	

}





