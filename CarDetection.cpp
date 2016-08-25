#include <iostream>
#include <io.h>
#include <vector>
#include <string>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/ml/ml.hpp>
using namespace std;
using namespace cv;
class MySVM : public CvSVM
{
public:
	//获得SVM的决策函数中的alpha数组
	double * get_alpha_vector()
	{
		return this->decision_func->alpha;
	}

	//获得SVM的决策函数中的rho参数,即偏移量
	float get_rho()
	{
		return this->decision_func->rho;
	}
};
extern MySVM svm1, svm2, svm3;

int CarDetection(Mat roi)
{
	//下面可以判断有无相应的车，而不标出车在哪
	/******************读入单个64*128的测试图并对其HOG描述子进行分类*********************/
	////读取测试图片(64*128大小)，并计算其HOG描述子
	vector<float> descriptor;
	HOGDescriptor hog;
	resize(roi, roi, Size(64, 128));
	hog.compute(roi,descriptor,Size(8,8));//计算HOG描述子，检测窗口移动步长(8,8)
	Mat testFeatureMat = Mat::zeros(1,3780,CV_32FC1);//测试样本的特征向量矩阵
	//将计算好的HOG描述子复制到testFeatureMat矩阵中
	for(int i=0; i<descriptor.size(); i++)
		testFeatureMat.at<float>(0,i) = descriptor[i];

	//用训练好的SVM分类器对测试图片的特征向量进行分类
	//int result1 = svm1.predict(testFeatureMat);//返回类标
	//int result2 = svm2.predict(testFeatureMat);
	//int result3 = svm3.predict(testFeatureMat);
	if(svm1.predict(testFeatureMat) == 1)
		return 1;
	else if(svm2.predict(testFeatureMat) == 1)
		return 2;
	else if(svm3.predict(testFeatureMat) == 1)
		return 3;
	else
		return 0;
}