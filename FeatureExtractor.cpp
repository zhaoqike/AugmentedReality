#include "StdAfx.h"
#include "FeatureExtractor.h"


FeatureExtractor::FeatureExtractor(void)
{
}

FeatureExtractor::FeatureExtractor(int threshold)
{
	this->threshold=threshold;
	
}

FeatureExtractor::~FeatureExtractor(void)
{
}


int FeatureExtractor::ExtractFeatures(KeyFrame &keyframe, Mat &image)
{
	//detector = FeatureDetector::create("PyramidFAST");
	//extractor = DescriptorExtractor::create("ORB");

	detector = new FastFeatureDetector();// FeatureDetector::create("PyramidFAST");
	extractor = new BriefDescriptorExtractor(); //DescriptorExtractor::create("ORB");
	
	//std::cout << " -------------------- extract feature points for all images -------------------\n";
	detector->detect(image, keyframe.keyPoints);
	extractor->compute(image, keyframe.keyPoints, keyframe.descriptors);
	//std::cout << " ------------------------------------- done -----------------------------------\n";

	//ORB orb(50000);
	//orb(image, Mat(), keyframe.keyPoints, keyframe.descriptors);

	return 0;
}