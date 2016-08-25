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
#include "BackMod.h"

using namespace std;
using namespace cv;
//继承自CvSVM的类，因为生成setSVMDetector()中用到的检测子参数时，需要用到训练好的SVM的decision_func参数，
//但通过查看CvSVM源码可知decision_func参数是protected类型变量，无法直接访问到，只能继承之后通过函数访问
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
MySVM svm1=MySVM();
MySVM svm2=MySVM();
MySVM svm3=MySVM();
int main()
{	
	cout<<"加载xml文件"<<endl;
	char *xmlPath1 = "E:/TestVideo/svm1.xml";
	char *xmlPath2 = "E:/TestVideo/svm2.xml";
	char *xmlPath3 = "E:/TestVideo/svm3.xml";
	///////先不执行
	svm1.load(xmlPath1);
	svm2.load(xmlPath2);
	svm3.load(xmlPath3);
	cout<<"加载成功";

	char *filename = "E://TestVideo//1.nsf";
	BackModel m;
	m.setVideoPath(filename);
	m.BackMode();//高斯背景建模
}