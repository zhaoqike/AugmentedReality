#pragma once
#include "stdafx.h"
#include "MapPoint.h"
#include "KeyFrame.h"


struct MapHeader
{
	char sig[3];
	int mappointSize;
	int keyframeSize;
};


class CloudMap
{
public:

	int threshold;
	int numLostFrames;
	int numAfterLastKeyFrame;
	double minKeyFrameDistance;
	
	cv::Mat K;
	cv::Mat_<double> Kinv;
	
	cv::Mat cam_matrix,distortion_coeff;
	cv::Mat distcoeff_32f; 
	cv::Mat K_32f;
	std::vector<cv::KeyPoint> correspImg1Pt; //TODO: remove
	std::vector<CloudPoint> pcloud;
	std::vector<cv::Vec3b> pointCloudRGB;
	std::vector<CloudPoint> pointcloud_beforeBA;
	std::vector<cv::Vec3b> pointCloudRGB_beforeBA;
	//std::vector<cv::Vec3b> pointCloudRGB;

	//std::vector<std::vector<cv::KeyPoint> > imgpts;
	//std::vector<std::vector<cv::KeyPoint> > fullpts;
	//std::vector<std::vector<cv::KeyPoint> > imgpts_good;




	vector<MapPoint*> mapPoints;
	vector<KeyFrame*> keyFrames;
	//MultiCameraPnP distance;

	CloudMap(void);
	~CloudMap(void);
	bool NeedFirst();
	bool NeedSecond();
	int InitMap();
	KeyFrame* MakeKeyFrame(Mat& image);
	KeyFrame* MakeKeyFrameLite(Mat& image);
	int TrackFrame(Mat& image);
	bool NeedNewKeyFrame(KeyFrame& keyframe);
	double ClosestKeyFrameDistance(KeyFrame& keyframe);

	double ComputeDistance(KeyFrame& keyframe1,KeyFrame& keyframe2);

	int StoreMap(string filename);
	int StoreKeyFrame(fstream &file,KeyFrame &keyframe);
	int StoreKeyFrames(fstream &file);
	int StoreMapPoint(fstream &file,CloudPoint &cp);
	int StoreMapPoints(fstream &file);
	int StoreMapHeaderInfo(fstream &file);
	int StoreKeyFrameHeaderInfo(fstream &file,KeyFrame &keyframe);
	int StoreMapPointHeaderInfo(fstream &file,CloudPoint &cp);

	bool LoadMap(string filename);
	bool LoadKeyFrame(fstream &file,KeyFrame &keyframe);
	bool LoadKeyFrames(fstream &file,int keyframeSize);
	bool LoadMapPoint(fstream &file,CloudPoint &cp);
	bool LoadMapPoints(fstream &file,int mappointSize);
	bool LoadMapHeaderInfo(fstream &file,MapHeader &header);
	bool LoadKeyFrameHeaderInfo(fstream &file,KeyFrameHeader &keyframeHeader);
	bool LoadMapPointHeaderInfo(fstream &file,CloudPointHeader &cpHeader);

	int writeCloud();


	int setCamera(cv::Size size);
	



	void GetBaseLineTriangulation(vector<DMatch> &matches);
	bool TriangulatePointsBetweenViews(
		int working_view, 
		int second_view,
		vector<DMatch> &matches,
		std::vector<struct CloudPoint>& new_triangulated,
		std::vector<int>& add_to_cloud);
	void AdjustCurrentBundle(int queryIdx,int trainIdx,vector<DMatch> &matches);
	void Find2D3DCorrespondences(int working_view, 
		KeyFrame * nowframe,
		vector<DMatch>& matches,
		std::vector<cv::Point3f>& ppcloud, 
		std::vector<cv::Point2f>& imgPoints);
	bool FindPoseEstimation(
	int working_view,
		cv::Mat_<double>& rvec,
		cv::Mat_<double>& t,
		cv::Mat_<double>& R,
		std::vector<cv::Point3f> &ppcloud,
		std::vector<cv::Point2f> &imgPoints,
		std::vector<int> &inliers); 

	void GetRGBForPointCloud(
		const std::vector<struct CloudPoint>& _pcloud,
		std::vector<cv::Vec3b>& RGBforCloud);
	std::vector<cv::Vec3b>& getPointCloudRGB();
};

