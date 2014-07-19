#include "StdAfx.h"
#include "ARSystem.h"


ARSystem::ARSystem(void)
{
}


ARSystem::~ARSystem(void)
{
}

void ARSystem::Run()
{
	preprocessor.Run();
	tracker.Run();
}