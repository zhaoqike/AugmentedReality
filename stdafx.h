// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>
#include <ctime>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include "Timer.h"

#include "Common.h"
#include "CloudMap.h"
#include "MultiCameraPnP.h"
#include "BundleAdjuster.h"
#include "FeatureExtractor.h"
#include "FeatureMatcher.h"

using namespace cv;
using namespace std;



// TODO: 在此处引用程序需要的其他头文件
