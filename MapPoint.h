#pragma once
class MapPoint
{
public:

	cv::Point3d pt;
	std::vector<int> imgpt_for_img;
	double reprojection_error;


	MapPoint(void);
	~MapPoint(void);
};

