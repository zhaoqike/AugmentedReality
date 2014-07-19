#include "StdAfx.h"
#include "KeyFrame.h"


KeyFrame::KeyFrame(void)
{
}


KeyFrame::~KeyFrame(void)
{
}

int KeyFrame::countIndex()
{
	int size=indexInMap.size();
	int count=0;
	for(int i=0;i<indexInMap.size();i++)
	{
		if(indexInMap[i]!=-1)
		{
			count++;
		}
	}
	cout<<count<<"/"<<size<<endl;
	return count;
}

//KeyFrame KeyFrame::makeKeyFrame(Mat img)
//{
//	image=img.clone();
//	ORB orb;
//	orb(image, Mat(), keyPoints, descriptors);
//	return this;
//}
