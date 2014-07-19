#include "StdAfx.h"
#include "Tracker.h"


Tracker::Tracker(void)
{
	isFirst=true;
}


Tracker::~Tracker(void)
{
}


void Tracker::TrackForInitialMap(void)
{
}


void Tracker::Run(void)
{
	Load3DModel();
	if(isFirst)
	{
		isFirst=false;
	}

}


void Tracker::Load3DModel()
{
	cloudmap.LoadMap(mapPath);
	cout<<"tracker"<<endl;
	cout<<cloudmap.keyFrames.size()<<endl;
	cout<<cloudmap.pcloud.size()<<endl;
}
