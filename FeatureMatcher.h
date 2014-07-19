#pragma once
class FeatureMatcher
{
public:
	float ratio; // max ratio between 1st and 2nd NN
	bool refineF; // if true will refine the F matrix
	double distance; // min distance to epipolar
	double confidence; // confidence level (probability)

	FeatureMatcher(void);
	~FeatureMatcher(void);
	int MaskMatch(KeyFrame * queryframe,KeyFrame *trainframe,vector<DMatch> &matches);
	int SymmetryMatch(KeyFrame * queryframe,KeyFrame *trainframe,vector<DMatch> &matches);
	int Match(KeyFrame * queryframe,KeyFrame *trainframe,vector<DMatch> &matches);

	int ratioTest(std::vector<std::vector<cv::DMatch>>&matches); 
	void symmetryTest(const vector<vector<DMatch>>& matches1,const vector<vector<DMatch>>& matches2,vector<DMatch>& symMatches);
	Mat ransacTest(const vector<DMatch>& matches,const vector<KeyPoint>& keypoints1, const vector<KeyPoint>& keypoints2,vector<DMatch>& outMatches);
};

