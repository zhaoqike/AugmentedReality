#pragma once

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <iostream>
#include <list>
#include <vector>

#include "stdafx.h"

using namespace std;
using namespace cv;



class PreProcessor
{
public:
	int nFrames;
	int numOfUse;
	CloudMap map;
	 
	bool getFirst;
	bool getSecond;
	bool startTracking;

	PreProcessor(void);
	PreProcessor(bool flag);
	~PreProcessor(void);
	int Run(void);
	int Test(std::vector<cv::Mat>& images);
	int ProcessFrame(Mat &frame);
	int setFirstFlag();
	int setSecondFlag();
	int setTrackingFlag();


	PC getAll(vector<Mat>& images,int i,int j);







	void drawMatchesRelative(const vector<KeyPoint>& train, const vector<KeyPoint>& query,
		std::vector<cv::DMatch>& matches, Mat& img);
	void keypoints2points(const vector<KeyPoint>& in, vector<Point2f>& out);
	void points2keypoints(const vector<Point2f>& in, vector<KeyPoint>& out);
	//Uses computed homography H to warp original input points to new planar position
	void warpKeypoints(const Mat& H, const vector<KeyPoint>& in, vector<KeyPoint>& out);
	//Converts matching indices to xy points
	void matches2points(const vector<KeyPoint>& train, const vector<KeyPoint>& query,
    const std::vector<cv::DMatch>& matches, std::vector<cv::Point2f>& pts_train,
		std::vector<Point2f>& pts_query);
	void resetH(Mat&H);
};

