#include "StdAfx.h"
#include "PreProcessor.h"
#include "KeyFrame.h"
#include <fstream>

using namespace std;


PreProcessor::PreProcessor(void)
{
	getFirst=getSecond=startTracking=false;
}

PreProcessor::PreProcessor(bool flag)
{
	getFirst=getSecond=startTracking=false;
}

PreProcessor::~PreProcessor(void)
{
}


int PreProcessor::Run(void)
{
	cout<<"preprocessor begin"<<endl;
	VideoCapture capture;
    capture.open(1);

    if (!capture.isOpened())
    {

        cout << "capture device 1 failed to open!" << endl;
        return 1;
    }

    cout << "camera open successfully"<<endl;
	cout<<"getFirst  "<<getFirst<<endl;
	cout<<"getSecond  "<<getSecond<<endl;

    Mat frame;


	//set camera
	capture >> frame;
	if (frame.empty())
	{
		cout<<"frame is empty"<<endl;
        return 0;
	}
	map.setCamera(frame.size());

	cout<<"Frame size: "<<frame.size().width<<"  "<<frame.size().height<<endl;

    
    for (;;)
    {
        capture >> frame;
		if (frame.empty())
		{
			cout<<"frame is empty"<<endl;
            break;
		}
		//cout<<"capture a frame"<<endl;
        ProcessFrame(frame);

		imshow("frame",frame);
		char key = (char)waitKey(2);
		switch (key)
        {
        case 'f':
            setFirstFlag();
            break;
        case 's':
            setSecondFlag();
            break;
		case 't':
			setTrackingFlag();
			break;
        case 27:
        case 'q':
			map.getPointCloudRGB();
			cout<<map.pcloud.size()<<"  "<<map.pointCloudRGB.size()<<endl;
			if(map.pointCloudRGB.size()>=map.pcloud.size())
			{
				map.writeCloud();
			}
            return 0;
            break;
        }
		
	}
	
    return 0;
}


PC PreProcessor::getAll(vector<Mat>& images,int i,int j)
{
	map.keyFrames.clear();
	map.pcloud.clear();
	map.MakeKeyFrame(images[i]);
	map.MakeKeyFrame(images[j]);
	map.InitMap();

	PC pc;
	pc.i=i;
	pc.j=j;
	pc.count=map.pcloud.size();
	pc.P=map.keyFrames[0]->Pmats;
	pc.P1=map.keyFrames[1]->Pmats;
	pc.R=Matx33d(pc.P(0,0),pc.P(0,1),pc.P(0,2),
				pc.P(1,0),pc.P(1,1),pc.P(1,2),
				pc.P(2,0),pc.P(2,1),pc.P(2,2));
	pc.R1=Matx33d(pc.P1(0,0),pc.P1(0,1),pc.P1(0,2),
				pc.P1(1,0),pc.P1(1,1),pc.P1(1,2),
				pc.P1(2,0),pc.P1(2,1),pc.P1(2,2));
	cv::invert(pc.R,pc.Rinv);
	cv::invert(pc.R1,pc.R1inv);
	return pc;
}

int PreProcessor::Test(std::vector<cv::Mat>& images)
{
	/*Mat images[7];
	images[0]=imread("a/P1000965.jpg");
	images[1]=imread("a/P1000966.jpg");
	images[2]=imread("a/P1000967.jpg");
	images[3]=imread("a/P1000968.jpg");
	images[4]=imread("a/P1000969.jpg");
	images[5]=imread("a/P1000970.jpg");
	images[6]=imread("a/P1000971.jpg");

	/*Mat images[7];
	images[0]=imread("a/rawoutput0000.jpg");
	images[1]=imread("a/rawoutput0001.jpg");*/
	map.setCamera(images[0].size());
	vector<PC> vec;
	for(int i=0;i<images.size()-1;i++)
	{
		for(int j=i+1;j<images.size();j++)
		{
			PC pc;
			cout<<"================================================================="<<endl;
			cout<<"-----------------------------------------------------------------"<<endl;
			cout<<"compute "<<i<<"  "<<j<<endl;
			pc=getAll(images,i,j);
			vec.push_back(pc);
			cout<<"-----------------------------------------------------------------"<<endl;
			cout<<"================================================================="<<endl;

			cout<<"================================================================="<<endl;
			cout<<"-----------------------------------------------------------------"<<endl;
			cout<<"compute "<<j<<"  "<<i<<endl;
			pc=getAll(images,j,i);
			vec.push_back(pc);
			cout<<"-----------------------------------------------------------------"<<endl;
			cout<<"================================================================="<<endl;
		}
	}
	cout<<"vec size"<<vec.size()<<endl;
	for(int i=0;i<vec.size();i++)
	{
		cout<<"index " <<vec[i].i<< " "<<vec[i].j<<endl;
		cout<<"point count "<<vec[i].count<<endl;
		cout<<"P"<<endl;
		cout<<vec[i].P<<endl;
		cout<<"P1"<<endl;
		/*cout<<vec[i].P1<<endl;
		cout<<"R"<<endl;
		cout<<vec[i].R<<endl;
		cout<<"R1"<<endl;
		cout<<vec[i].R1<<endl;
		cout<<"Pinv"<<endl;
		cout<<vec[i].Rinv<<endl;
		cout<<"P1inv"<<endl;
		cout<<vec[i].R1inv<<endl;*/
	}
	/*map.MakeKeyFrame(images[0]);
	map.MakeKeyFrame(images[1]);
	cerr<<"begin init map"<<endl;
	cerr<<"keyframe 0: "<<map.keyFrames[0]->keyPoints.size()<<"  "<<map.keyFrames[0]->keyPointsGood.size()<<endl;
	cerr<<"keyframe 1: "<<map.keyFrames[1]->keyPoints.size()<<"  "<<map.keyFrames[1]->keyPointsGood.size()<<endl;
	map.InitMap();
	cerr<<"end init map"<<endl;
	cout<<"after init map size: "<<map.pcloud.size()<<endl;
	for(int i=2;i<7;i++)
	{
		map.TrackFrame(images[i]);
		cout<<"after " + intToString(i)+ " map size: "<<map.pcloud.size()<<endl;
	}
	//imshow("image",images[0]);
	//char key = (char)waitKey(0);
	//for(int i=0;i<map.keyFrames.size();i++)
	//{
	//	KeyFrame *k=map.keyFrames[i];
	//	cout<<k->image.cols<<"  "<<k->image.rows<<endl;
	//	imwrite("keyframe"+intToString(i)+".jpg",k->image);
	//}
	map.StoreMap(mapPath);

	cout<<"get color: "<<endl;
	map.getPointCloudRGB();
	cout<<map.pcloud.size()<<"  "<<map.pointCloudRGB.size()<<endl;
	if(map.pointCloudRGB.size()>=map.pcloud.size())
	{
		map.writeCloud();
	}*/
	return 0;
}


int PreProcessor::ProcessFrame(Mat &frame)
{
	if (frame.empty())
	{
		cout<<"frame empty"<<endl;
		return -1;
	}
	if(getFirst==false)
	{
		//cerr<<"fenzhi 1"<<endl;
		KeyFrame *keyframe = map.MakeKeyFrameLite(frame);
		//cout<<"keypoints num: "<<keyframe->keyPoints.size()<<endl;
		drawKeypoints(frame, keyframe->keyPoints, frame, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_OVER_OUTIMG);
	}
	else if(getFirst==true && map.NeedFirst())
	{
		cerr<<"fenzhi 2"<<endl;
		KeyFrame *keyframe = map.MakeKeyFrame(frame);
		drawKeypoints(frame, keyframe->keyPoints, frame, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_OVER_OUTIMG);

		cout<<"create first keyframe"<<endl;
		cout<<"keyframe number: "<<map.keyFrames.size()<<endl;
		cout<<"first keyframe keypoints number: "<<map.keyFrames[0]->keyPoints.size();
	}
	else if(getSecond==false)
	{
		//cerr<<"fenzhi 3"<<endl;
		KeyFrame *keyframe = map.MakeKeyFrameLite(frame);


		KeyFrame *trainframe=map.keyFrames.front();
		vector<KeyPoint> query_kpts=keyframe->keyPoints;
		vector<KeyPoint> train_kpts=trainframe->keyPoints;
		vector<DMatch> matches;
		FeatureMatcher matcher;
		matcher.Match(keyframe,trainframe,matches);
		drawMatchesRelative(query_kpts, train_kpts, matches, frame);
	}
	else if(getSecond==true && map.NeedSecond())
	{
		cerr<<"fenzhi 4"<<endl;
		KeyFrame *keyframe = map.MakeKeyFrame(frame);


		KeyFrame *trainframe=map.keyFrames.front();
		vector<KeyPoint> query_kpts=keyframe->keyPoints;
		vector<KeyPoint> train_kpts=trainframe->keyPoints;
		vector<DMatch> matches;
		FeatureMatcher matcher;
		matcher.Match(keyframe,trainframe,matches);
		drawMatchesRelative(query_kpts, train_kpts, matches, frame);


		map.InitMap();

		cout<<"create second keyframe"<<endl;
		cout<<"create init map"<<endl;
		cout<<"keyframe number: "<<map.keyFrames.size()<<endl;
		cout<<"second keyframe keypoints number: "<<map.keyFrames[1]->keyPoints.size()<<endl;
		cout<<"point number: "<<map.pcloud.size()<<endl;
		map.keyFrames[0]->countIndex();
		map.keyFrames[1]->countIndex();
		imwrite("keyframe1.jpg", map.keyFrames[0]->image);
		imwrite("keyframe2.jpg", map.keyFrames[1]->image);
	}
		
	else if(startTracking==false)
	{
		//cout<<"fenzhi 5"<<endl;
		
	}
	else
	{
		cerr<<"fenzhi 6"<<endl;
		map.TrackFrame(frame);

	}
}

int PreProcessor::setFirstFlag()
{
	getFirst=true;
	cout<<"get first is true now"<<endl;
	return 0;
}

int PreProcessor::setSecondFlag()
{
	getSecond=true;
	cout<<"get second is true now"<<endl;
	return 0;
}

int PreProcessor::setTrackingFlag()
{
	startTracking=true;
	cout<<"start tracking is true now"<<endl;
	return 0;
}










































void PreProcessor::drawMatchesRelative(const vector<KeyPoint>& query, const vector<KeyPoint>& train,
    std::vector<cv::DMatch>& matches, Mat& img)
{
    for (int i = 0; i < (int)matches.size(); i++)
    {
        
        Point2f pt_new = query[matches[i].queryIdx].pt;
        Point2f pt_old = train[matches[i].trainIdx].pt;

        cv::line(img, pt_new, pt_old, Scalar(125, 255, 125), 1);
        cv::circle(img, pt_new, 2, Scalar(255, 0, 125), 1);

        
    }
}

//Takes a descriptor and turns it into an xy point
void PreProcessor::keypoints2points(const vector<KeyPoint>& in, vector<Point2f>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        out.push_back(in[i].pt);
    }
}

//Takes an xy point and appends that to a keypoint structure
void PreProcessor::points2keypoints(const vector<Point2f>& in, vector<KeyPoint>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        out.push_back(KeyPoint(in[i], 1));
    }
}

//Uses computed homography H to warp original input points to new planar position
void PreProcessor::warpKeypoints(const Mat& H, const vector<KeyPoint>& in, vector<KeyPoint>& out)
{
    vector<Point2f> pts;
    keypoints2points(in, pts);
    vector<Point2f> pts_w(pts.size());
    Mat m_pts_w(pts_w);
    perspectiveTransform(Mat(pts), m_pts_w, H);
    points2keypoints(pts_w, out);
}

//Converts matching indices to xy points
void PreProcessor::matches2points(const vector<KeyPoint>& train, const vector<KeyPoint>& query,
    const std::vector<cv::DMatch>& matches, std::vector<cv::Point2f>& pts_train,
    std::vector<Point2f>& pts_query)
{

    pts_train.clear();
    pts_query.clear();
    pts_train.reserve(matches.size());
    pts_query.reserve(matches.size());

    size_t i = 0;

    for (; i < matches.size(); i++)
    {

        const DMatch & dmatch = matches[i];

        pts_query.push_back(query[dmatch.queryIdx].pt);
        pts_train.push_back(train[dmatch.trainIdx].pt);

    }

}

void PreProcessor::resetH(Mat&H)
{
    H = Mat::eye(3, 3, CV_32FC1);
}
