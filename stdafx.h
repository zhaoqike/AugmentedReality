// stdafx.h : ��׼ϵͳ�����ļ��İ����ļ���
// ���Ǿ���ʹ�õ��������ĵ�
// �ض�����Ŀ�İ����ļ�
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>
#include <ctime>
#include <algorithm>

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



// TODO: �ڴ˴����ó�����Ҫ������ͷ�ļ�
