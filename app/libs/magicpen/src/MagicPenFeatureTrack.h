#pragma once

#include <opencv2/imgproc.hpp>

class MagicPenFeatureTrack
{
public:
	bool Init(cv::Mat image_gray, cv::Rect roi);

	void Track(cv::Mat image_gray);

	std::vector<cv::Point2f> ROICorners();

private:
	void ShowTrackFeature();

	cv::Mat _pre_image;
	std::vector<cv::Point2f> _pre_feature_corners;
	std::vector<cv::Point2f> _pre_image_ROI_corners;
	bool _init = false;
};

