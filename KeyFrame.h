#pragma once

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <iostream>
#include <list>
#include <vector>

using namespace std;
using namespace cv;

struct KeyFrameHeader
{
	char sig[3];
	int keyPointsSize;
	int IndexInMapSize;
	MatHeader descriptorHeader;
	//MatHeader pheader;
};

class KeyFrame
{
public:
	Mat image;
	vector<KeyPoint> keyPoints;
	vector<KeyPoint> keyPointsGood;
	vector<int> indexInMap;
	Mat descriptors;
	cv::Matx34d Pmats;


	//KeyFrame makeKeyFrame(Mat image);
	KeyFrame(void);
	~KeyFrame(void);

	int countIndex();
};

