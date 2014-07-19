#pragma once
#include "PreProcessor.h"
#include "Tracker.h"
class ARSystem
{
public:
	PreProcessor preprocessor;
	Tracker tracker;
	
	ARSystem(void);
	~ARSystem(void);
	void Run();
};

