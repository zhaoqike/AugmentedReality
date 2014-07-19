#pragma once
class FeatureExtractor
{
public:
	int threshold;
	
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorExtractor> extractor;

	FeatureExtractor(void);
	FeatureExtractor(int threshold);
	~FeatureExtractor(void);
	int ExtractFeatures(KeyFrame &keyframe, Mat &image);
	
};

