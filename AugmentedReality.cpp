
#include "stdafx.h"
#include <iostream>
#include <string.h>

#include "Distance.h"
#include "MultiCameraPnP.h"
//#include "Visualization.h"
#include "PreProcessor.h"
#include "Tracker.h"

using namespace std;
using namespace cv;

//#include <opencv2/gpu/gpu.hpp>

//class VisualizerListener : public SfMUpdateListener {
//public:
//	void update(std::vector<cv::Point3d> pcld,
//				std::vector<cv::Vec3b> pcldrgb, 
//				std::vector<cv::Point3d> pcld_alternate,
//				std::vector<cv::Vec3b> pcldrgb_alternate, 
//				std::vector<cv::Matx34d> cameras) {
//		ShowClouds(pcld, pcldrgb, pcld_alternate, pcldrgb_alternate);
//		
//		vector<cv::Matx34d> v = cameras;
//		for(unsigned int i=0;i<v.size();i++) {
//			stringstream ss; ss << "camera" << i;
//			cv::Matx33f R; 
//			R(0,0)=v[i](0,0); R(0,1)=v[i](0,1); R(0,2)=v[i](0,2);
//			R(1,0)=v[i](1,0); R(1,1)=v[i](1,1); R(1,2)=v[i](1,2);
//			R(2,0)=v[i](2,0); R(2,1)=v[i](2,1); R(2,2)=v[i](2,2);
//			visualizerShowCamera(R,cv::Vec3f(v[i](0,3),v[i](1,3),v[i](2,3)),255,0,0,0.2,ss.str());
//		}
//	}
//};

std::vector<cv::Mat> images;
std::vector<std::string> images_names;

bool Compare(CloudMap m1,CloudMap m2)
{
	if(m1.pcloud.size()!=m2.pcloud.size())
	{
		cout<<"pcloud size error"<<endl;
		return false;
	}
	if(m1.keyFrames.size()!=m2.keyFrames.size())
	{
		cout<<"keyframe size error"<<endl;
		return false;
	}
	for(int i=0;i<m1.pcloud.size();i++)
	{
		if(m1.pcloud[i].reprojection_error!=m2.pcloud[i].reprojection_error)
		{
			cout<<"pcloud "<<i<<" reproject error"<<endl;
			return false;
		}
		if(m1.pcloud[i].pt!=m2.pcloud[i].pt)
		{
			cout<<"pcloud "<<i<<" point error"<<endl;
			return false;
		}
		for(int j=0;j<m1.pcloud[i].imgpt_for_img.size();j++)
		{
			if(m1.pcloud[i].imgpt_for_img[j]!=m2.pcloud[i].imgpt_for_img[j])
			{
				cout<<"pcloud "<<i<<" index " <<j<<" error"<<endl;
				return false;
			}
		}
	}
	for(int i=0;i<m1.keyFrames.size();i++)
	{
		if(m1.keyFrames[i]->keyPoints.size()!=m2.keyFrames[i]->keyPoints.size())
		{
			cout<<"keyframe "<<i<<"keypoints size error"<<endl;
			return false;
		}
		if(m1.keyFrames[i]->indexInMap.size()!=m2.keyFrames[i]->indexInMap.size())
		{
			cout<<"keyframe "<<i<<"index in map size error"<<endl;
			return false;
		}
		for(int j=0;j<m1.keyFrames[i]->indexInMap.size();j++)
		{
			if(m1.keyFrames[i]->indexInMap[j]!=m2.keyFrames[i]->indexInMap[j])
			{
				cout<<"keyframe "<<i<<" index in map "<<j<<error<<endl;
				return false;
			}
		}
	}
	cout<<"compare success"<<endl;
	return true;
}
		

int main() {
	//get time
	time_t t;
	t=time(NULL);
	std::stringstream ss;
	string str;
	ss<<t;
	ss>>str;

	string filename="out"+str+".txt";

	freopen(filename.c_str(),"w",stdout);
	//freopen("out.txt","w",stderr);
	cout<<"hello world"<<endl;
	PreProcessor preprocessor(false);
	Tracker tracker;
	open_imgs_dir("a",images,images_names,1.0);
	for(int i=0;i<images_names.size();i++)
	{
		cout<<images_names[i]<<endl;
	}
	//preprocessor.Run();
	preprocessor.Test(images);
	//tracker.Load3DModel();
	//Compare(preprocessor.map,tracker.cloudmap);
	return 0;
//	if (freopen("D:\\OUTPUT.txt", "w", stdout)==NULL)
//        fprintf(stderr, "error redirecting\stdout\n");
//	if (argc < 2) {
//		cerr << "USAGE: " << argv[0] << " <path_to_images> [use rich features (RICH/OF) = RICH] [use GPU (GPU/CPU) = GPU] [downscale factor = 1.0]" << endl;
//		return 0;
//	}
//	
//	double downscale_factor = 1.0;
//	if(argc >= 5)
//		downscale_factor = atof(argv[4]);
//
	//open_imgs_dir(argv[1],images,images_names,downscale_factor);
//	if(images.size() == 0) { 
//		cerr << "can't get image files" << endl;
//		return 1;
//	}
//
//	
//	cv::Ptr<MultiCameraPnP> distance = new MultiCameraPnP(images,images_names,string(argv[1]));
//	if(argc < 3)
//		distance->use_rich_features = true;
//	else
//		distance->use_rich_features = (strcmp(argv[2], "RICH") == 0);
//	distance->use_gpu=0;
//	//if(argc < 4)
//	//	distance->use_gpu = (cv::gpu::getCudaEnabledDeviceCount() > 0);
//	//else
//	//	distance->use_gpu = (strcmp(argv[3], "GPU") == 0);
//	
//	cv::Ptr<VisualizerListener> visualizerListener = new VisualizerListener; //with ref-count
//	distance->attach(visualizerListener);
//	RunVisualizationThread();
//
//	distance->RecoverDepthFromImages();
//
//	//get the scale of the result cloud using PCA
//	double scale_cameras_down = 1.0;
//	{
//		vector<cv::Point3d> cld = distance->getPointCloud();
//		if (cld.size()==0) cld = distance->getPointCloudBeforeBA();
//		cv::Mat_<double> cldm(cld.size(),3);
//		for(unsigned int i=0;i<cld.size();i++) {
//			cldm.row(i)(0) = cld[i].x;
//			cldm.row(i)(1) = cld[i].y;
//			cldm.row(i)(2) = cld[i].z;
//		}
//		cv::Mat_<double> mean;
//		cv::PCA pca(cldm,mean,CV_PCA_DATA_AS_ROW);
//		scale_cameras_down = pca.eigenvalues.at<double>(0) / 5.0;
//		//if (scale_cameras_down > 1.0) {
//		//	scale_cameras_down = 1.0/scale_cameras_down;
//		//}
//	}
//	
//	visualizerListener->update(distance->getPointCloud(),
//							   distance->getPointCloudRGB(),
//							   distance->getPointCloudBeforeBA(),
//							   distance->getPointCloudRGBBeforeBA(),
//							   distance->getCameras());
//							   
//
//	//ShowCloud(distance->getPointCloud(), 
//	//		   distance->getPointCloudRGB(),
//	//		   "baseline_only");
//	//WaitForVisualizationThread();
//	//return 1;
//	
////	ShowClouds(distance->getPointCloud(), 
////			   distance->getPointCloudRGB(),
////			   distance->getPointCloudBeforeBA(),
////			   distance->getPointCloudRGBBeforeBA()
////			   );
//	WaitForVisualizationThread();
}
