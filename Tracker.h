#pragma once
class Tracker
{
public:
	CloudMap cloudmap;
	bool isFirst;
	Tracker(void);
	~Tracker(void);
	void TrackForInitialMap(void);
	void Run(void);
	void Load3DModel();

};

