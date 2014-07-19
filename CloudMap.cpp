#include "StdAfx.h"
//#include "Map.h"


CloudMap::CloudMap(void)
{
	threshold=100;
	numLostFrames=0;
	numAfterLastKeyFrame=0;
	minKeyFrameDistance=-1;
}


CloudMap::~CloudMap(void)
{
}

bool CloudMap::NeedFirst()
{
	return keyFrames.size()==0;
}
bool CloudMap::NeedSecond()
{
	return keyFrames.size()==1;
}

int CloudMap::InitMap()
{
	vector<DMatch> matches;
	FeatureMatcher matcher;
	matcher.Match(keyFrames[1],keyFrames[0],matches);
	cerr<<matches.size()<<endl;
	cerr<<"keyframe 0: "<<keyFrames[0]->keyPoints.size()<<"  "<<keyFrames[0]->keyPointsGood.size()<<endl;
	cerr<<"keyframe 1: "<<keyFrames[1]->keyPoints.size()<<"  "<<keyFrames[1]->keyPointsGood.size()<<endl;
	/*for(int i=0;i<matches.size();i++)
	{
	if(matches[i].queryIdx==22852 || matches[i].trainIdx==22852)
	{
	cerr<<"found "<<i<<endl;
	}
	}
	cerr<<"not found"<<endl;
	cerr<<matches[295].queryIdx<<"  "<<matches[295].trainIdx<<endl;*/
	GetBaseLineTriangulation(matches);
	cerr<<"after base line: "<<matches.size()<<endl;
	/*Mat out;
	drawMatches(keyFrames[1]->image, keyFrames[1]->keyPoints, keyFrames[0]->image, keyFrames[0]->keyPoints,
		matches, out, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	imshow("MatchORB", out);
	cerr<<"before wait"<<endl;
	waitKey(0);*/
	cerr<<"end wait"<<endl;
	return 0;
}
KeyFrame* CloudMap::MakeKeyFrame(Mat& image)
{
	KeyFrame *keyframe=MakeKeyFrameLite(image);
	/*image=image.clone();
	ORB orb;
	orb(image, Mat(), keyframe->keyPoints, keyframe->descriptors);
	keyframe->indexInMap.resize(keyframe->keyPoints.size());
	for(int i=0;i<keyframe->indexInMap.size();i++)
	{
	keyframe->indexInMap[i]=-1;
	}*/
	//keyframe->makeKeyFrame(image);
	keyFrames.push_back(keyframe);
	return keyframe;
}
KeyFrame* CloudMap::MakeKeyFrameLite(Mat& image)
{
	KeyFrame *keyframe=new KeyFrame();
	keyframe->image=image.clone();
	//ORB orb;
	//orb(image, Mat(), keyframe->keyPoints, keyframe->descriptors);
	FeatureExtractor extractor(threshold);
	extractor.ExtractFeatures(*keyframe, image);
	keyframe->indexInMap.resize(keyframe->keyPoints.size());
	cerr<<"keypoints size"<<endl;
	cerr<<keyframe->keyPoints.size()<<endl;
	cerr<<"descriptores size"<<endl;
	cerr<<keyframe->descriptors.rows<<endl;
	if(keyframe->keyPoints.size()>200)
	{
		threshold-=5;
	}
	else
	{
		threshold+=10;
	}
	for(int i=0;i<keyframe->indexInMap.size();i++)
	{
		keyframe->indexInMap[i]=-1;
	}
	return keyframe;
}

int CloudMap::TrackFrame(Mat& image)
{
	KeyFrame * nowframe=MakeKeyFrameLite(image);
	int trainIdx=1;//=keyFrames.size()-1;


	vector<DMatch> matches;
	//FeatureMatcher matcher;
	//matcher.MaskMatch(nowframe,keyFrames[trainIdx],matches);


	cv::Matx34d P1 = keyFrames[trainIdx]->Pmats;
	cv::Mat_<double> t = (cv::Mat_<double>(1,3) << P1(0,3), P1(1,3), P1(2,3));
	cv::Mat_<double> R = (cv::Mat_<double>(3,3) << P1(0,0), P1(0,1), P1(0,2), 
		P1(1,0), P1(1,1), P1(1,2), 
		P1(2,0), P1(2,1), P1(2,2));
	cv::Mat_<double> rvec(1,3); Rodrigues(R, rvec);

	int _i=0;

	//while (done_views.size() != imgs.size())
	//{
	//	//find image with highest 2d-3d correspondance [Snavely07 4.2]
	//	unsigned int max_2d3d_view = -1, max_2d3d_count = 0;
	vector<cv::Point3f> max_3d; vector<cv::Point2f> max_2d; vector<int> inliers;
	//	for (unsigned int _i=0; _i < imgs.size(); _i++) {
	//		if(done_views.find(_i) != done_views.end()) continue; //already done with this view

	//vector<cv::Point3f> tmp3d; vector<cv::Point2f> tmp2d;
	//		cout << imgs_names[_i] << ": ";
	Find2D3DCorrespondences(trainIdx,nowframe,matches,max_3d,max_2d);
	//	if(tmp3d.size() > max_2d3d_count) {
	//		max_2d3d_count = tmp3d.size();
	//		max_2d3d_view = _i;
	//		max_3d = tmp3d; max_2d = tmp2d;
	//	}
	//}
	int i = 0;// max_2d3d_view; //highest 2d3d matching view

	//std::cout << "-------------------------- " << imgs_names[i] << " --------------------------\n";
	//done_views.insert(i); // don't repeat it for now

	bool pose_estimated = FindPoseEstimation(i,rvec,t,R,max_3d,max_2d,inliers);
	if(!pose_estimated)
	{
		numLostFrames++;
		cout<<"lost a frame"<<endl;
		return -1;
	}

	numLostFrames=0;
	numAfterLastKeyFrame++;

	//store estimated pose	
	nowframe->Pmats = cv::Matx34d	(R(0,0),R(0,1),R(0,2),t(0),
		R(1,0),R(1,1),R(1,2),t(1),
		R(2,0),R(2,1),R(2,2),t(2));
	cout<<"frame pose " <<Mat(nowframe->Pmats)<<endl;
	// start triangulating with previous GOOD views
	//for (set<int>::iterator done_view = good_views.begin(); done_view != good_views.end(); ++done_view) 
	//{
	//int view = *done_view;
	//if( view == i ) continue; //skip current...

	//cout << " -> " << imgs_names[view] << endl;
	if(true)//(0&&NeedNewKeyFrame(*nowframe))
	{
		vector<DMatch> inliersMatch;
		inliersMatch.clear();
		cout<<"next to triangulate ,inliers size: "<<inliers.size()<<endl;
		for(int i=0;i<inliers.size();i++)
		{
			inliersMatch.push_back(matches[inliers[i]]);
		}
		cout<<"match size: "<<matches.size()<<endl;
		matches=inliersMatch;
		cout<<"match size: "<<matches.size()<<endl;


		vector<CloudPoint> new_triangulated;
		vector<int> add_to_cloud;

		//add keyframe
		keyFrames.push_back(nowframe);
		numAfterLastKeyFrame=0;


		bool good_triangulation = TriangulatePointsBetweenViews(keyFrames.size()-1,trainIdx,matches,new_triangulated,add_to_cloud);
		if(!good_triangulation) 
		{
			return -1;
		}

		std::cout << "before triangulation: " << pcloud.size();
		/*for (int j=0; j<add_to_cloud.size(); j++) {
		if(add_to_cloud[j] == 1)
		pcloud.push_back(new_triangulated[j]);
		}*/
		std::cout << " after " << pcloud.size() << std::endl;
		//break;
		//}
		//good_views.insert(i);

		AdjustCurrentBundle(keyFrames.size()-1,trainIdx,matches);
		//update();
	}
}

bool CloudMap::NeedNewKeyFrame(KeyFrame& keyframe)
{
	if(numAfterLastKeyFrame>=20&&ClosestKeyFrameDistance(keyframe)>minKeyFrameDistance)
	{
		return true;
	}
	return false;
}

double CloudMap::ClosestKeyFrameDistance(KeyFrame& keyframe)
{
	return 0.0;
}

double CloudMap::ComputeDistance(KeyFrame& keyframe1,KeyFrame& keyframe2)
{
	Matx34d p1=keyframe1.Pmats;
	Matx34d p2=keyframe2.Pmats;
	Mat_<double> t1(3,1);
	Mat_<double> t2(3,1);
	//t1=p1.col(3);
	//t1=Matx31d(p1(0,3),p1(1,3),p1(2,3));
	return 0.0;
}


int CloudMap::StoreMap(string filename)
{
	fstream file;
	file.open(filename.c_str(),ios::out|ios::binary);
	StoreMapHeaderInfo(file);

	StoreMapPoints(file);
	StoreKeyFrames(file);
	return 0;
}

int CloudMap::StoreKeyFrame(fstream &file,KeyFrame &keyframe)
{
	StoreKeyFrameHeaderInfo(file,keyframe);
	//keypoints
	for(int i=0;i<keyframe.keyPoints.size();i++)
	{
		file.write((char*)(&keyframe.keyPoints[i]),sizeof(keyframe.keyPoints[i]));
	}
	//index in map
	for(int i=0;i<keyframe.indexInMap.size();i++)
	{
		file.write((char*)(&keyframe.indexInMap[i]),sizeof(keyframe.indexInMap[i])); 
	}
	//descriptors
	file.write((char*)(keyframe.descriptors.data),sizeof(keyframe.descriptors.total()*keyframe.descriptors.elemSize()));
	//pmats
	file.write((char*)(keyframe.Pmats.val),sizeof(keyframe.Pmats.val));
	return 0;
}
int CloudMap::StoreKeyFrames(fstream &file)
{
	for(int i=0;i<keyFrames.size();i++)
	{
		KeyFrame keyframe=*(keyFrames[i]);
		StoreKeyFrame(file,keyframe);
	}
	return 0;
}
int CloudMap::StoreMapPoint(fstream &file,CloudPoint &cp)
{
	StoreMapPointHeaderInfo(file,cp);

	file.write((char*)(&cp.pt),sizeof(cp.pt));
	file.write((char*)(&cp.reprojection_error),sizeof(cp.reprojection_error));
	for(int i=0;i<cp.imgpt_for_img.size();i++)
	{
		file.write((char*)(&cp.imgpt_for_img[i]),sizeof(cp.imgpt_for_img[i]));
	}
	return 0;
}
int CloudMap::StoreMapPoints(fstream &file)
{
	for(int i=0;i<pcloud.size();i++)
	{
		StoreMapPoint(file,pcloud[i]);
	}
	return 0;
}

int CloudMap::StoreMapHeaderInfo(fstream &file)
{
	MapHeader header;
	header.sig[0]='m';
	header.sig[1]='h';
	header.sig[2]='d';
	header.mappointSize=pcloud.size();
	header.keyframeSize=keyFrames.size();
	file.write((char*)(&header),sizeof(header));
	return 0;
}

int CloudMap::StoreKeyFrameHeaderInfo(fstream &file,KeyFrame &keyframe)
{
	KeyFrameHeader kfh;
	kfh.sig[0]='k';
	kfh.sig[1]='f';
	kfh.sig[2]='m';
	kfh.keyPointsSize=keyframe.keyPoints.size();
	kfh.IndexInMapSize=keyframe.indexInMap.size();

	kfh.descriptorHeader.rows=keyframe.descriptors.rows;
	kfh.descriptorHeader.cols=keyframe.descriptors.cols;
	kfh.descriptorHeader.type=keyframe.descriptors.type();

	//kfh.pheader.rows=keyframe.Pmats.rows;
	//kfh.pheader.cols=keyframe.Pmats.cols;
	//kfh.pheader.type=keyframe.Pmats.type();

	file.write((char*)(&kfh),sizeof(kfh));
	return 0;
}
int CloudMap::StoreMapPointHeaderInfo(fstream &file,CloudPoint &cp)
{
	CloudPointHeader cph;
	cph.sig[0]='m';
	cph.sig[1]='p';
	cph.sig[2]='t';
	cph.indexInImageSize=cp.imgpt_for_img.size();
	file.write((char*)(&cph),sizeof(cph));
	return 0;
}



bool CloudMap::LoadMap(string filename)
{
	fstream file;
	file.open(filename.c_str(),ios::in|ios::binary);
	MapHeader header;
	bool flag=true;
	flag=LoadMapHeaderInfo(file,header);
	cout<<"&&"<<header.sig[0]<<"&&"<<header.sig[1]<<"&&"<<header.sig[2]<<endl;
	if(false==flag)
	{
		cout<<"load header fail"<<endl;
		return false;
	}
	cout<<"load header success"<<endl;
	cout<<header.keyframeSize<<endl;
	cout<<header.mappointSize<<endl;
	flag=LoadMapPoints(file,header.mappointSize);
	if(false==flag)
	{
		cout<<"load map points fail"<<endl;
		return false;
	}
	cout<<"load map points success"<<endl;
	flag=LoadKeyFrames(file,header.keyframeSize);
	if(false==flag)
	{
		cout<<"load keyframes fail"<<endl;
		return false;
	}
	cout<<"load keyframes success"<<endl;
	return 0;
}

bool CloudMap::LoadKeyFrame(fstream &file,KeyFrame &keyframe)
{
	KeyFrameHeader header;
	bool flag=true;
	flag=LoadKeyFrameHeaderInfo(file,header);
	if(false==flag)
	{
		return false;
	}
	keyframe.keyPoints.resize(header.keyPointsSize);
	keyframe.indexInMap.resize(header.IndexInMapSize);
	keyframe.descriptors.create(header.descriptorHeader.rows,header.descriptorHeader.cols,header.descriptorHeader.type);

	//keypoints
	for(int i=0;i<keyframe.keyPoints.size();i++)
	{
		file.read((char*)(&keyframe.keyPoints[i]),sizeof(keyframe.keyPoints[i]));
	}
	//index in map
	for(int i=0;i<keyframe.indexInMap.size();i++)
	{
		file.read((char*)(&keyframe.indexInMap[i]),sizeof(keyframe.indexInMap[i])); 
	}
	//descriptors
	file.read((char*)(keyframe.descriptors.data),sizeof(keyframe.descriptors.total()*keyframe.descriptors.elemSize()));
	//pmats
	file.read((char*)(keyframe.Pmats.val),sizeof(keyframe.Pmats.val));
	return true;
}
bool CloudMap::LoadKeyFrames(fstream &file,int keyframeSize)
{
	keyFrames.resize(keyframeSize);
	bool flag=true;
	for(int i=0;i<keyframeSize;i++)
	{
		keyFrames[i]=new KeyFrame();
		flag=LoadKeyFrame(file,*(keyFrames[i]));
		if(false==flag)
		{
			return false;
		}
	}
	return true;
}
bool CloudMap::LoadMapPoint(fstream &file,CloudPoint &cp)
{
	CloudPointHeader header;
	bool flag=true;
	flag=LoadMapPointHeaderInfo(file,header);
	if(false==flag)
	{
		return false;
	}
	cp.imgpt_for_img.resize(header.indexInImageSize);
	file.read((char*)(&cp.pt),sizeof(cp.pt));
	file.read((char*)(&cp.reprojection_error),sizeof(cp.reprojection_error));
	for(int i=0;i<cp.imgpt_for_img.size();i++)
	{
		file.read((char*)(&cp.imgpt_for_img[i]),sizeof(cp.imgpt_for_img[i]));
	}
	return true;
}

bool CloudMap::LoadMapPoints(fstream &file,int mappointSize)
{
	pcloud.resize(mappointSize);
	bool flag=true;
	for(int i=0;i<mappointSize;i++)
	{
		flag=LoadMapPoint(file,pcloud[i]);
		if(false==flag)
		{
			cout<<"mappoint "<<i<<" failed"<<endl;
			return false;
		}
	}
	return true;
}

bool CloudMap::LoadMapHeaderInfo(fstream &file,MapHeader &header)
{
	file.read((char*)(&header),sizeof(header));
	return true;
}

bool CloudMap::LoadKeyFrameHeaderInfo(fstream &file,KeyFrameHeader &keyframeHeader)
{
	file.read((char*)(&keyframeHeader),sizeof(keyframeHeader));
	if('k'==keyframeHeader.sig[0] &&	'f'==keyframeHeader.sig[1] &&	'm'==keyframeHeader.sig[2])
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CloudMap::LoadMapPointHeaderInfo(fstream &file,CloudPointHeader &cpHeader)
{
	file.read((char*)(&cpHeader),sizeof(cpHeader));
	//cout<<"&&"<<cpHeader.sig[0]<<"&&"<<cpHeader.sig[1]<<"&&"<<cpHeader.sig[2]<<"&&"<<endl;
	if('m'==cpHeader.sig[0]&&	'p'==cpHeader.sig[1]&&	't'==cpHeader.sig[2])
	{
		return true;
	}
	else
	{
		return false;
	}
}

int CloudMap::writeCloud()
{
	fstream file;
	file.open("cloud.txt",ios::out);
	file<<pcloud.size()<<endl;
	for(int i=0;i<pcloud.size();i++)
	{
		file<<pcloud[i].pt.x<<"  "<<pcloud[i].pt.y<<"  "<<pcloud[i].pt.z<<"  "<<(int)pointCloudRGB[i].val[0]<<"  "<<(int)pointCloudRGB[i].val[1]<<"  "<<(int)pointCloudRGB[i].val[2]<<"  "<<endl;
	}
	return 0;
}








int CloudMap::setCamera(cv::Size size)
{
	cv::FileStorage fs;
	if(fs.open("out_camera_data.yml",cv::FileStorage::READ)) {
		fs["camera_matrix"]>>cam_matrix;
		fs["distortion_coefficients"]>>distortion_coeff;
	} else {
		//no calibration matrix file - mockup calibration
		double max_w_h = MAX(size.height,size.width);
		cam_matrix = (cv::Mat_<double>(3,3) <<	max_w_h ,	0	,		size.width/2.0,
			0,			max_w_h,	size.height/2.0,
			0,			0,			1);
		distortion_coeff = cv::Mat_<double>::zeros(1,4);
	}


	K = cam_matrix;
	invert(K, Kinv); //get inverse of camera matrix

	distortion_coeff.convertTo(distcoeff_32f,CV_32FC1);
	K.convertTo(K_32f,CV_32FC1);
	return 0;
}








void CloudMap::GetBaseLineTriangulation(vector<DMatch> &matches) {
	std::cout << "=========================== Baseline triangulation ===========================\n";
	cerr<<"triangulate begin"<<endl;
	cv::Matx34d P_train(1,0,0,0,
		0,1,0,0,
		0,0,1,0),
		P_query(1,0,0,0,
		0,1,0,0,
		0,0,1,0);

	std::vector<CloudPoint> tmp_pcloud;

	//sort pairwise matches to find the lowest Homography inliers [Snavely07 4.2]
	//cout << "Find highest match...";
	//list<pair<int,pair<int,int> > > matches_sizes;
	////TODO: parallelize!
	//for(std::map<std::pair<int,int> ,std::vector<cv::DMatch> >::iterator i = matches_matrix.begin(); i != matches_matrix.end(); ++i) {
	//	if((*i).second.size() < 100)
	//		matches_sizes.push_back(make_pair(100,(*i).first));
	//	else {
	//		int Hinliers = FindHomographyInliers2Views((*i).first.first,(*i).first.second);
	//		int percent = (int)(((double)Hinliers) / ((double)(*i).second.size()) * 100.0);
	//		cout << "[" << (*i).first.first << "," << (*i).first.second << " = "<<percent<<"] ";
	//		matches_sizes.push_back(make_pair((int)percent,(*i).first));
	//	}
	//}
	//cout << endl;
	//matches_sizes.sort(sort_by_first);

	//Reconstruct from two views
	bool goodF = false;
	int highest_pair = 0;
	int trainIdx =0;
	int queryIdx = 1;




	//reverse iterate by number of matches
	//for(list<pair<int,pair<int,int> > >::iterator highest_pair = matches_sizes.begin(); 
	//	highest_pair != matches_sizes.end() && !goodF; 
	//	++highest_pair) 
	//{
	//	m_second_view = (*highest_pair).second.second;
	//	m_first_view  = (*highest_pair).second.first;

	//std::cout << " -------- " << imgs_names[m_first_view] << " and " << imgs_names[m_second_view] << " -------- " <<std::endl;
	//what if reconstrcution of first two views is bad? fallback to another pair
	//See if the Fundamental Matrix between these two views is good
	goodF = FindCameraMatrices(K, Kinv, distortion_coeff,
		keyFrames[queryIdx]->keyPoints, 
		keyFrames[trainIdx]->keyPoints, 
		keyFrames[queryIdx]->keyPointsGood,
		keyFrames[trainIdx]->keyPointsGood, 
		P_query, 
		P_train,
		matches,
		tmp_pcloud
#ifdef __SFM__DEBUG__
		,imgs[m_first_view],imgs[m_second_view]
#endif
	);
	if (goodF) {
		vector<CloudPoint> new_triangulated;
		vector<int> add_to_cloud;

		keyFrames[trainIdx]->Pmats = P_train;
		keyFrames[queryIdx]->Pmats = P_query;
		cerr<<"triangule begins"<<endl;
		bool good_triangulation = TriangulatePointsBetweenViews(queryIdx,trainIdx,matches,new_triangulated,add_to_cloud);
		cerr<<"triangulate,end"<<endl;
		if(!good_triangulation || cv::countNonZero(add_to_cloud) < 10) {
			std::cout << "triangulation failed" << std::endl;
			goodF = false;
			//Pmats[m_first_view] = 0;
			//Pmats[m_second_view] = 0;
			//m_second_view++;
		} else {
			//std::cout << "before triangulation: " << pcloud.size();
			//for (unsigned int j=0; j<add_to_cloud.size(); j++) {
			//	if(add_to_cloud[j] == 1)
			//		pcloud.push_back(new_triangulated[j]);
			//}
			std::cout << " after " << pcloud.size() << std::endl;
		}				
	}
	//}

	/*if (!goodF) {
	cerr << "Cannot find a good pair of images to obtain a baseline triangulation" << endl;
	exit(0);
	}*/

	//cout << "Taking baseline from " << imgs_names[m_first_view] << " and " << imgs_names[m_second_view] << endl;

	//	double reproj_error;
	//	{
	//		std::vector<cv::KeyPoint> pt_set1,pt_set2;
	//		
	//		std::vector<cv::DMatch> matches = matches_matrix[std::make_pair(m_first_view,m_second_view)];
	//
	//		GetAlignedPointsFromMatch(imgpts[m_first_view],imgpts[m_second_view],matches,pt_set1,pt_set2);
	//		
	//		pcloud.clear();
	//		reproj_error = TriangulatePoints(pt_set1, 
	//										 pt_set2, 
	//										 Kinv, 
	//										 distortion_coeff,
	//										 Pmats[m_first_view], 
	//										 Pmats[m_second_view], 
	//										 pcloud, 
	//										 correspImg1Pt);
	//		
	//		for (unsigned int i=0; i<pcloud.size(); i++) {
	//			pcloud[i].imgpt_for_img = std::vector<int>(imgs.size(),-1);
	//			//matches[i] corresponds to pointcloud[i]
	//			pcloud[i].imgpt_for_img[m_first_view] = matches[i].queryIdx;
	//			pcloud[i].imgpt_for_img[m_second_view] = matches[i].trainIdx;
	//		}
	//	}
	//	std::cout << "triangulation reproj error " << reproj_error << std::endl;
}


bool CloudMap::TriangulatePointsBetweenViews(
	int query_view, 
	int train_view,
	vector<DMatch> &matches,
	vector<struct CloudPoint>& new_triangulated,
	vector<int>& add_to_cloud
	) 
{
	//cout << " Triangulate " << imgs_names[working_view] << " and " << imgs_names[older_view] << endl;
	//get the left camera matrix
	//TODO: potential bug - the P mat for <view> may not exist? or does it...
	cv::Matx34d P_query = keyFrames[query_view]->Pmats;
	cv::Matx34d P_train = keyFrames[train_view]->Pmats;

	cout<<"begin triangulation"<<endl;
	cout<<"p query index "<<query_view<<endl;
	cout<<Mat(P_query)<<endl;
	cout<<"p train index "<<train_view<<endl;
	cout<<Mat(P_train)<<endl;

	std::vector<cv::KeyPoint> pt_queryset,pt_trainset;
	cout<<"getalign"<<endl;
	GetAlignedPointsFromMatch(keyFrames[query_view]->keyPoints,keyFrames[train_view]->keyPoints,matches,pt_queryset,pt_trainset);


	//adding more triangulated points to general cloud
	cout<<"triangulate points"<<endl;
	double reproj_error = TriangulatePoints(pt_queryset, pt_trainset, K, Kinv, distortion_coeff, P_query, P_train, new_triangulated, correspImg1Pt);
	std::cout << "triangulation reproj error " << reproj_error << std::endl;

	vector<uchar> trig_status;
	cout<<"test triangulate"<<endl;
	if(!TestTriangulation(new_triangulated, P_query, trig_status) || !TestTriangulation(new_triangulated, P_train, trig_status)) {
		cout << "Triangulation did not succeed" << endl;
		return false;
	}
	//	if(reproj_error > 20.0) {
	//		// somethign went awry, delete those triangulated points
	//		//				pcloud.resize(start_i);
	//		cerr << "reprojection error too high, don't include these points."<<endl;
	//		return false;
	//	}

	//filter out outlier points with high reprojection
	vector<double> reprj_errors;
	for(int i=0;i<new_triangulated.size();i++) { reprj_errors.push_back(new_triangulated[i].reprojection_error); }
	std::sort(reprj_errors.begin(),reprj_errors.end());
	//get the 80% precentile
	double reprj_err_cutoff = reprj_errors[4 * reprj_errors.size() / 5] * 2.4; //threshold from Snavely07 4.2

	vector<CloudPoint> new_triangulated_filtered;
	std::vector<cv::DMatch> new_matches;
	for(int i=0;i<new_triangulated.size();i++) {
		if(trig_status[i] == 0)
			continue; //point was not in front of camera
		if(new_triangulated[i].reprojection_error > 16.0) {
			continue; //reject point
		} 
		if(new_triangulated[i].reprojection_error < 4.0 ||
			new_triangulated[i].reprojection_error < reprj_err_cutoff) 
		{
			new_triangulated_filtered.push_back(new_triangulated[i]);
			new_matches.push_back(matches[i]);
		} 
		else 
		{
			continue;
		}
	}

	cout << "filtered out " << (new_triangulated.size() - new_triangulated_filtered.size()) << " high-error points" << endl;

	//all points filtered?
	if(new_triangulated_filtered.size() <= 0) return false;

	new_triangulated = new_triangulated_filtered;

	matches = new_matches;
	//matches_matrix[std::make_pair(older_view,working_view)] = new_matches; //just to make sure, remove if unneccesary
	//matches_matrix[std::make_pair(working_view,older_view)] = FlipMatches(new_matches);
	add_to_cloud.clear();
	add_to_cloud.resize(new_triangulated.size(),1);
	int found_other_views_count = 0;
	int num_views = keyFrames.size();

	//scan new triangulated points, if they were already triangulated before - strengthen cloud
	//#pragma omp parallel for num_threads(1)
	cerr<<"my work"<<endl;
	for (int j = 0; j<new_triangulated.size(); j++) {
		//cerr<<j<<endl;
		new_triangulated[j].imgpt_for_img = std::vector<int>(maxKeyframeNum,-1);

		//matches[j] corresponds to new_triangulated[j]
		//matches[j].queryIdx = point in <older_view>
		//matches[j].trainIdx = point in <working_view>
		new_triangulated[j].imgpt_for_img[query_view] = matches[j].queryIdx;	//2D reference to <older_view>
		new_triangulated[j].imgpt_for_img[train_view] = matches[j].trainIdx;		//2D reference to <working_view>
		bool found_in_other_view = false;
		int trainIdx=matches[j].trainIdx;
		int queryIdx=matches[j].queryIdx;
		/*if(j==295)
		{
		cerr<<"older"<<endl;
		cerr<<query_view<<"  "<<train_view<<endl;
		cerr<<matches.size()<<endl;
		cerr<<matches[j].queryIdx<<"  "<<matches[j].trainIdx<<endl;
		}
		if(j==295)
		{
		cerr<<keyFrames[keyFrames.size()-1]->indexInMap.size()<<endl;
		}*/
		int pt3d=keyFrames[train_view]->indexInMap[trainIdx];
		/*if(j==295)
		{
		cerr<<keyFrames[keyFrames.size()-1]->indexInMap.size()<<endl;
		}*/
		if(pt3d!=-1)
		{
			found_in_other_view=true;
			pcloud[pt3d].imgpt_for_img[train_view] = matches[j].trainIdx;
			pcloud[pt3d].imgpt_for_img[query_view] = matches[j].queryIdx;
			keyFrames[train_view]->indexInMap[trainIdx]=pt3d;
			keyFrames[query_view]->indexInMap[queryIdx]=pt3d;
			found_in_other_view = true;
			add_to_cloud[j] = 0;
		}
		else
		{

			pcloud.push_back(new_triangulated[j]);
			pt3d=pcloud.size()-1;
			pcloud.back().imgpt_for_img[train_view] = matches[j].trainIdx;
			pcloud.back().imgpt_for_img[query_view] = matches[j].queryIdx;
			keyFrames[train_view]->indexInMap[trainIdx]=pt3d;
			keyFrames[query_view]->indexInMap[queryIdx]=pt3d;
		}
		if(j==295)
		{
			cerr<<add_to_cloud.size()<<endl;
		}
		{
			if (found_in_other_view) {
				found_other_views_count++;
			} else {
				add_to_cloud[j] = 1;
			}
		}
	}
	std::cout << found_other_views_count << "/" << new_triangulated.size() << " points were found in other views, adding " << cv::countNonZero(add_to_cloud) << " new\n";
	return true;
}

void CloudMap::AdjustCurrentBundle(int queryIdx,int trainIdx,vector<DMatch> &matches) {
	cout << "======================== Bundle Adjustment ==========================\n";

	pointcloud_beforeBA = pcloud;
	//GetRGBForPointCloud(pointcloud_beforeBA,pointCloudRGB_beforeBA);

	cv::Mat _cam_matrix = K;
	BundleAdjuster BA;
	std::map<int,cv::Matx34d> Pmats;
	BA.adjustRecentBundle(*this,queryIdx,trainIdx,matches,_cam_matrix);
	K = cam_matrix;
	Kinv = K.inv();

	cout << "use new K " << endl << K << endl;

	//GetRGBForPointCloud(pcloud,pointCloudRGB);
}	


void CloudMap::Find2D3DCorrespondences(int kftrainIdx, 
	KeyFrame * nowframe,
	vector<DMatch>& matches,
	std::vector<cv::Point3f>& ppcloud, 
	std::vector<cv::Point2f>& imgPoints) 
{
	ppcloud.clear(); 
	imgPoints.clear();


	KeyFrame *keyframetrain=keyFrames[kftrainIdx];
	//vector<DMatch> matches;
	FeatureMatcher matcher;
	matcher.Match(nowframe,keyframetrain,matches);
	cout<<"-----------------------------------------------------------------"<<endl;
	cout<<"find 2d 3d correspondences"<<endl;
	cout<<"query frame keypoints: "<<nowframe->keyPoints.size()<<endl;
	cout<<"train frame keypoints: "<<keyframetrain->keyPoints.size()<<endl;
	cout<<"match size: "<<matches.size()<<endl;
	cout<<"-----------------------------------------------------------------"<<endl;

	for(int i=0;i<matches.size();i++)
	{
		int trainIdx=matches[i].trainIdx;
		int idxInMap=keyframetrain->indexInMap[trainIdx];
		if(idxInMap!=-1)
		{
			ppcloud.push_back(pcloud[idxInMap].pt);
			int queryIdx=matches[i].queryIdx;
			imgPoints.push_back(nowframe->keyPoints[queryIdx].pt);
		}
	}
	//vector<int> pcloud_status(pcloud.size(),0);
	//for (set<int>::iterator done_view = good_views.begin(); done_view != good_views.end(); ++done_view) 
	//{
	//	int old_view = *done_view;
	//	//check for matches_from_old_to_working between i'th frame and <old_view>'th frame (and thus the current cloud)
	//	std::vector<cv::DMatch> matches_from_old_to_working = matches_matrix[std::make_pair(old_view,working_view)];

	//	for (unsigned int match_from_old_view=0; match_from_old_view < matches_from_old_to_working.size(); match_from_old_view++) {
	//		// the index of the matching point in <old_view>
	//		int idx_in_old_view = matches_from_old_to_working[match_from_old_view].queryIdx;

	//		//scan the existing cloud (pcloud) to see if this point from <old_view> exists
	//		for (unsigned int pcldp=0; pcldp<pcloud.size(); pcldp++) {
	//			// see if corresponding point was found in this point
	//			if (idx_in_old_view == pcloud[pcldp].imgpt_for_img[old_view] && pcloud_status[pcldp] == 0) //prevent duplicates
	//			{
	//				//3d point in cloud
	//				ppcloud.push_back(pcloud[pcldp].pt);
	//				//2d point in image i
	//				imgPoints.push_back(imgpts[working_view][matches_from_old_to_working[match_from_old_view].trainIdx].pt);

	//				pcloud_status[pcldp] = 1;
	//				break;
	//			}
	//		}
	//	}
	//}
	cout << "found " << ppcloud.size() << " 3d-2d point correspondences"<<endl;
}


bool CloudMap::FindPoseEstimation(
	int working_view,
	cv::Mat_<double>& rvec,
	cv::Mat_<double>& t,
	cv::Mat_<double>& R,
	std::vector<cv::Point3f> &ppcloud,
	std::vector<cv::Point2f> &imgPoints,
	std::vector<int> &inliers
	) 
{
	if(ppcloud.size() <= 7 || imgPoints.size() <= 7 || ppcloud.size() != imgPoints.size()) { 
		//something went wrong aligning 3D to 2D points..
		cout << "couldn't find [enough] corresponding cloud points... (only " << ppcloud.size() << ")" <<endl;
		return false;
	}

	//vector<int> inliers;

	double minVal,maxVal; cv::minMaxIdx(imgPoints,&minVal,&maxVal);
	//cv::solvePnPRansac(ppcloud, imgPoints, K, distortion_coeff, rvec, t, true, 100, 20.0F, 0.1 * (double)(imgPoints.size()), inliers, CV_EPNP);
	cv::solvePnPRansac(ppcloud, imgPoints, K, distortion_coeff, rvec, t, true, 1000, 0.006 * maxVal, 0.25 * (double)(imgPoints.size()), inliers, CV_EPNP);
	//CV_PROFILE("solvePnP",cv::solvePnP(ppcloud, imgPoints, K, distortion_coeff, rvec, t, true, CV_EPNP);)

	//cout<<0.006 * maxVal<<endl;
	vector<cv::Point2f> projected3D;
	cv::projectPoints(ppcloud, rvec, t, K, distortion_coeff, projected3D);

	cout<<"-------------------------------------------------------------------------"<<endl;
	cout<<"find pose estimation"<<endl;
	cout<<"3d point: "<<ppcloud.size()<<endl;
	cout<<"2d point: "<<imgPoints.size()<<endl;
	cout<<"inliers size: "<<inliers.size()<<endl;
	/*for(int i=0;i<inliers.size();i++)
	{
		cout<<inliers[i]<<"  ";
	}
	cout<<endl;*/
	cout<<"projected point: "<<projected3D.size()<<endl;
	cout<<"--------------------------------------------------------------------------"<<endl;

	if(inliers.size()==0) { //get inliers
		for(int i=0;i<projected3D.size();i++) {
			if(norm(projected3D[i]-imgPoints[i]) < 10.0)
				inliers.push_back(i);
		}
	}


#if 0
	//display reprojected points and matches
	cv::Mat reprojected; imgs_orig[working_view].copyTo(reprojected);
	for(int ppt=0;ppt<imgPoints.size();ppt++) {
		cv::line(reprojected,imgPoints[ppt],projected3D[ppt],cv::Scalar(0,0,255),1);
	}
	for (int ppt=0; ppt<inliers.size(); ppt++) {
		cv::line(reprojected,imgPoints[inliers[ppt]],projected3D[inliers[ppt]],cv::Scalar(0,0,255),1);
	}
	for(int ppt=0;ppt<imgPoints.size();ppt++) {
		cv::circle(reprojected, imgPoints[ppt], 2, cv::Scalar(255,0,0), CV_FILLED);
		cv::circle(reprojected, projected3D[ppt], 2, cv::Scalar(0,255,0), CV_FILLED);			
	}
	for (int ppt=0; ppt<inliers.size(); ppt++) {
		cv::circle(reprojected, imgPoints[inliers[ppt]], 2, cv::Scalar(255,255,0), CV_FILLED);
	}
	stringstream ss; ss << "inliers " << inliers.size() << " / " << projected3D.size();
	putText(reprojected, ss.str(), cv::Point(5,20), CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,255), 2);

	cv::imshow("__tmp", reprojected);
	cv::waitKey(0);
	cv::destroyWindow("__tmp");
#endif
	//cv::Rodrigues(rvec, R);
	//visualizerShowCamera(R,t,0,255,0,0.1);
	cout<<"("<<inliers.size()<<"/"<<imgPoints.size()<<")"<< endl;
	if(inliers.size() < (double)(imgPoints.size())/5.0) {
		cout << "not enough inliers to consider a good pose ("<<inliers.size()<<"/"<<imgPoints.size()<<")"<< endl;
		return false;
	}

	if(cv::norm(t) > 200.0) {
		// this is bad...
		cout << "estimated camera movement is too big, skip this camera\r\n";
		return false;
	}

	cv::Rodrigues(rvec, R);
	if(!CheckCoherentRotation(R)) {
		cout << "rotation is incoherent. we should try a different base view..." << endl;
		return false;
	}

	std::cout << "found t = " << t << "\nR = \n"<<R<<std::endl;
	return true;
}




void CloudMap::GetRGBForPointCloud(
	const std::vector<struct CloudPoint>& _pcloud,
	std::vector<cv::Vec3b>& RGBforCloud
	) 
{
	RGBforCloud.resize(_pcloud.size());
	for (unsigned int i=0; i<_pcloud.size(); i++) {
		unsigned int good_view = 0;
		std::vector<cv::Vec3b> point_colors;
		for(; good_view < keyFrames.size(); good_view++) {
			if(_pcloud[i].imgpt_for_img[good_view] != -1) {
				int pt_idx = _pcloud[i].imgpt_for_img[good_view];
				if(pt_idx >= keyFrames[good_view]->keyPoints.size()) {
					std::cerr << "BUG: point id:" << pt_idx << " should not exist for img #" << good_view << " which has only " << keyFrames[good_view]->keyPoints.size() << std::endl;
					continue;
				}
				cv::Point _pt = keyFrames[good_view]->keyPoints[pt_idx].pt;
				assert(good_view < keyFrames.size() && _pt.x < keyFrames[good_view]->image.cols && _pt.y < keyFrames[good_view]->image.rows);
				
				point_colors.push_back(keyFrames[good_view]->image.at<cv::Vec3b>(_pt));
				
//				std::stringstream ss; ss << "patch " << good_view;
//				imshow_250x250(ss.str(), imgs_orig[good_view](cv::Range(_pt.y-10,_pt.y+10),cv::Range(_pt.x-10,_pt.x+10)));
			}
		}
//		cv::waitKey(0);
		cv::Scalar res_color = cv::mean(point_colors);
		RGBforCloud[i] = (cv::Vec3b(res_color[0],res_color[1],res_color[2])); //bgr2rgb
		if(good_view == keyFrames.size()) //nothing found.. put red dot
			RGBforCloud.push_back(cv::Vec3b(255,0,0));
	}
}


std::vector<cv::Vec3b>& CloudMap::getPointCloudRGB() 
{ 
	if(pointCloudRGB.size()==0) 
	{ 
		GetRGBForPointCloud(pcloud,pointCloudRGB); 
	} 
	return pointCloudRGB; 
}