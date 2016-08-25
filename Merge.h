#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/ml/ml.hpp>
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
class WindowsMerge
{
public:
	WindowsMerge() {}
	vector<Rect> merge(vector<Rect> input);
};

vector<Rect> WindowsMerge::merge(vector<Rect> input)
{
		Rect overrect;
		int s, s1, s2;
		double r = 0.3;
		if (input.size() > 0)		//¡¤??????????????¨¬???????????¡À?¨¢????
			for (size_t i = 0; i < input.size()-1; i++)
			{
				for (size_t j = i + 1; j < input.size(); j++)
				{

					s1 = input[i].area();
					s2 = input[j].area();
					if ((input[i] & input[j]).area() > r * s1 || (input[i] & input[j]).area() > r * s2)
					{
						overrect = input[i] & input[j];
						s = overrect.area();
						if (s1*r < s && s < s2*r)
						{
							input.erase(input.begin() + j);
							j--;
						}
						else if (s2*r < s && s < s1*r)
						{
							input.erase(input.begin() + i);
							i--;
							break;
						}
						else if (s1*r < s && s2*r < s)
						{
							input[i] = Rect((input[i].x+input[j].x)/2, (input[i].y+input[j].y)/2, (input[i].width+input[j].width)/2, (input[i].height+input[j].height)/2);
							input.erase(input.begin() + j);
							i--;
							break;

						}
					}
				}
			}
			return input;
}