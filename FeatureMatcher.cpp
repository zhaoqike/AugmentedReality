#include "StdAfx.h"
#include "FeatureMatcher.h"


FeatureMatcher::FeatureMatcher(void)
{
	ratio=0.65f;
}


FeatureMatcher::~FeatureMatcher(void)
{
}

int FeatureMatcher::MaskMatch(KeyFrame * queryframe,KeyFrame *trainframe,vector<DMatch> &matches)
{
	//matching descriptor vectors using Brute Force matcher
    BFMatcher matcher(NORM_HAMMING,true); //allow cross-check. use Hamming distance for binary descriptor (ORB)
	std::vector< DMatch > good_matches_,very_good_matches_;
    /*std::vector< DMatch > matches_;
    if (matches == NULL) {
        matches = &matches_;
    }*/
	cerr<<"begin match"<<endl;
	cerr<<queryframe->descriptors.size()<<"  "<<trainframe->descriptors.size()<<endl;
    matcher.match( queryframe->descriptors, trainframe->descriptors, matches );
	cerr<<queryframe->descriptors.size()<<"  "<<trainframe->descriptors.size()<<endl;
	cerr<<"end match"<<endl;

    
	assert(matches.size() > 0);
    
    vector<KeyPoint> imgpts1_good,imgpts2_good;
    
    //Eliminate any re-matching of training points (multiple queries to one training)
	//double cutoff = 4.0*min_dist;
    /*std::set<int> existing_trainIdx;
    for(unsigned int i = 0; i < matches.size(); i++ )
    { 
        //"normalize" matching: somtimes imgIdx is the one holding the trainIdx
        if (matches[i].trainIdx <= 0) {
            matches[i].trainIdx = matches[i].imgIdx;
        }
        
        if( existing_trainIdx.find(matches[i].trainIdx) == existing_trainIdx.end() && 
			matches[i].trainIdx >= 0 && matches[i].trainIdx < (int)(trainframe->keyPoints.size())  ) 
        {
            good_matches_.push_back( matches[i]);
			imgpts1_good.push_back(queryframe->keyPoints[matches[i].queryIdx]);
			imgpts2_good.push_back(trainframe->keyPoints[matches[i].trainIdx]);
            existing_trainIdx.insert(matches[i].trainIdx);
        }
    }*/
	cerr<<"begin elli"<<endl;
	vector<bool> exist_trainIdx(trainframe->keyPoints.size(),false);
	for(unsigned int i = 0; i < matches.size(); i++ )
    { 
        //"normalize" matching: somtimes imgIdx is the one holding the trainIdx
        if (matches[i].trainIdx <= 0) {
            matches[i].trainIdx = matches[i].imgIdx;
        }
        
        if( exist_trainIdx[matches[i].trainIdx] == false && 
			matches[i].trainIdx >= 0 && matches[i].trainIdx < (int)(trainframe->keyPoints.size())  ) 
        {
            good_matches_.push_back( matches[i]);
			imgpts1_good.push_back(queryframe->keyPoints[matches[i].queryIdx]);
			imgpts2_good.push_back(trainframe->keyPoints[matches[i].trainIdx]);
            exist_trainIdx[matches[i].trainIdx]=true;
        }
    }
	cerr<<"end elli"<<good_matches_.size()<<endl;
	cerr << "keypoints_1.size() " << queryframe->keyPoints.size() << " imgpts1_good.size() " << imgpts1_good.size() << endl;
	cerr << "keypoints_2.size() " << trainframe->keyPoints.size() << " imgpts2_good.size() " << imgpts2_good.size() << endl;

	vector<uchar> status;
    vector<KeyPoint> imgpts2_very_good,imgpts1_very_good;
    
	//use good_matches instead of init matches
	GetFundamentalMat(queryframe->keyPoints,trainframe->keyPoints,imgpts1_very_good,imgpts2_very_good,good_matches_);

	//BFMatcher desc_matcher(NORM_HAMMING);
	//Mat mask = windowedMatchingMask(queryframe->keyPoints, trainframe->keyPoints, 25, 25);
	//desc_matcher.match(queryframe->descriptors, trainframe->descriptors, matches);

	//GetFundamentalMat(queryframe->keyPoints,trainframe->keyPoints,queryframe->keyPointsGood,trainframe->keyPointsGood,matches);
	return 0;
}


int FeatureMatcher::SymmetryMatch(KeyFrame * queryframe,KeyFrame *trainframe,vector<DMatch> &matches)
{

	// Construction of the matcher 
	cv::BruteForceMatcher<cv::L2<float>> matcher;
	// from image 1 to image 2
	// based on k nearest neighbours (with k=2)
	std::vector<std::vector<cv::DMatch>> matches1;
	cerr<<"begin knn"<<endl;
	matcher.knnMatch(queryframe->descriptors,trainframe->descriptors, 
		matches1, // vector of matches (up to 2 per entry) 
		2); // return 2 nearest neighbours
	// from image 2 to image 1
	// based on k nearest neighbours (with k=2)
	std::vector<std::vector<cv::DMatch>> matches2;
	matcher.knnMatch(trainframe->descriptors,queryframe->descriptors, 
		matches2, // vector of matches (up to 2 per entry) 
		2); // return 2 nearest neighbours
	// 3. Remove matches for which NN ratio is 
	// > than threshold
	// clean image 1 -> image 2 matches
	cout<<"matches1: "<<matches1.size()<<endl;
	int removed= ratioTest(matches1);
	cout<<"matches1: "<<matches1.size()<<endl;
	// clean image 2 -> image 1 matches
	cout<<"matches2: "<<matches2.size()<<endl;
	removed= ratioTest(matches2);
	cout<<"matches2: "<<matches2.size()<<endl;
	// 4. Remove non-symmetrical matches
	std::vector<cv::DMatch> symMatches;
	cout<<"symmatches: "<<symMatches.size()<<endl;
	symmetryTest(matches1,matches2,symMatches);
	cout<<"symmatches: "<<symMatches.size()<<endl;
	// 5. Validate matches using RANSAC
	cout<<"symmatches: "<<symMatches.size()<<endl;
	cout<<"matches: "<<matches.size()<<endl;
	cerr<<"begin ransac"<<endl;
	matches=symMatches;
	cv::Mat fundemental= ransacTest(symMatches, queryframe->keyPoints, trainframe->keyPoints, matches);
	cout<<"symmatches: "<<symMatches.size()<<endl;
	cout<<"matches: "<<matches.size()<<endl;
	// return the found fundemental matrix
	return 0;
}

// Clear matches for which NN ratio is > than threshold
// return the number of removed points 
// (corresponding entries being cleared, 
// i.e. size will be 0)
int FeatureMatcher::ratioTest(vector<vector<DMatch>>&matches) 
{
	int removed=0;
	// for all matches
	for (vector<vector<DMatch>>::iterator 
		matchIterator= matches.begin();
		matchIterator!= matches.end(); ++matchIterator) {
			// if 2 NN has been identified
			if (matchIterator->size() > 1) {
				// check distance ratio
				if ((*matchIterator)[0].distance/
					(*matchIterator)[1].distance > ratio) {
						matchIterator->clear(); // remove match
						removed++;
				}
			} else { // does not have 2 neighbours
				matchIterator->clear(); // remove match
				removed++;
			}
	}
	return removed;
}


// Insert symmetrical matches in symMatches vector
void FeatureMatcher::symmetryTest(const vector<vector<DMatch>>& matches1,const vector<vector<DMatch>>& matches2,vector<DMatch>& symMatches) 
{
	// for all matches image 1 -> image 2
	for (vector<vector<DMatch>>::
		const_iterator matchIterator1= matches1.begin();
		matchIterator1!= matches1.end(); ++matchIterator1) {
			// ignore deleted matches
			if (matchIterator1->size() < 2) 
				continue;
			// for all matches image 2 -> image 1
			for (vector<vector<DMatch>>::
				const_iterator matchIterator2= matches2.begin();
				matchIterator2!= matches2.end(); 
			++matchIterator2) {
				// ignore deleted matches
				if (matchIterator2->size() < 2) 
					continue;
				// Match symmetry test
				if ((*matchIterator1)[0].queryIdx == 
					(*matchIterator2)[0].trainIdx && 
					(*matchIterator2)[0].queryIdx == 
					(*matchIterator1)[0].trainIdx) {
						// add symmetrical match
						symMatches.push_back(
							DMatch((*matchIterator1)[0].queryIdx, 
							(*matchIterator1)[0].trainIdx,
							(*matchIterator1)[0].distance));
						break; // next match in image 1 -> image 2
				}
			}
	}
}

// Identify good matches using RANSAC
// Return fundemental matrix
Mat FeatureMatcher::ransacTest(const vector<DMatch>& matches,const vector<KeyPoint>& keypoints1, const vector<KeyPoint>& keypoints2,vector<DMatch>& outMatches) {
	// Convert keypoints into Point2f 
	vector<Point2f> points1, points2; 
	for (vector<DMatch>::
		const_iterator it= matches.begin();
		it!= matches.end(); ++it) {
			// Get the position of left keypoints
			float x= keypoints1[it->queryIdx].pt.x;
			float y= keypoints1[it->queryIdx].pt.y;
			points1.push_back(Point2f(x,y));
			// Get the position of right keypoints
			x= keypoints2[it->trainIdx].pt.x;
			y= keypoints2[it->trainIdx].pt.y;
			points2.push_back(Point2f(x,y));
	}
	// Compute F matrix using RANSAC
	vector<uchar> inliers(points1.size(),0);
	Mat fundemental= findFundamentalMat(
		Mat(points1),Mat(points2), // matching points
		inliers, // match status (inlier or outlier) 
		CV_FM_RANSAC, // RANSAC method
		distance, // distance to epipolar line
		confidence); // confidence probability
	// extract the surviving (inliers) matches
	vector<uchar>::const_iterator 
		itIn= inliers.begin();
	vector<DMatch>::const_iterator 
		itM= matches.begin();
	// for all matches
	for ( ;itIn!= inliers.end(); ++itIn, ++itM) {
		if (*itIn) { // it is a valid match
			outMatches.push_back(*itM);
		}
	}
	if (refineF) {
		// The F matrix will be recomputed with 
		// all accepted matches
		// Convert keypoints into Point2f 
		// for final F computation 
		points1.clear();
		points2.clear();
		for (vector<DMatch>::
			const_iterator it= outMatches.begin();
			it!= outMatches.end(); ++it) {
				// Get the position of left keypoints 
				float x= keypoints1[it->queryIdx].pt.x;
				float y= keypoints1[it->queryIdx].pt.y;
				points1.push_back(Point2f(x,y));
				// Get the position of right keypoints
				x= keypoints2[it->trainIdx].pt.x;
				y= keypoints2[it->trainIdx].pt.y;
				points2.push_back(Point2f(x,y));
		}
		// Compute 8-point F from all accepted matches
		fundemental= findFundamentalMat(
			Mat(points1),Mat(points2), // matches
			CV_FM_8POINT); // 8-point method
	}
	return fundemental;
}

int FeatureMatcher::Match(KeyFrame * queryframe,KeyFrame *trainframe,vector<DMatch> &matches)
{
	//SymmetryMatch(queryframe,trainframe,matches);
	//MaskMatch(queryframe,trainframe,matches);
	cerr<<"begin mathc"<<endl;
	SymmetryMatch(queryframe,trainframe,matches);
	cerr<<"end match"<<endl;
	return 0;
}