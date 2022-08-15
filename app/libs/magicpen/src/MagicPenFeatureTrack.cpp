#include "MagicPenFeatureTrack.h"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>

bool MagicPenFeatureTrack::Init(cv::Mat image_gray, cv::Rect roi) {

	_pre_image_ROI_corners.resize(4);

	_init = false;

	cv::goodFeaturesToTrack(image_gray(roi),  _pre_feature_corners, 100, 0.01, 3.0, cv::Mat(), 3, false, 0.04);
	
	if (_pre_feature_corners.size() < 20) {
        return false;
    }

	for (size_t i = 0; i < _pre_feature_corners.size(); i++) {
		_pre_feature_corners[i].x += roi.x;
		_pre_feature_corners[i].y += roi.y;
	}

	_pre_image = image_gray;
	_init = true;

	_pre_image_ROI_corners[0].x = roi.x;
	_pre_image_ROI_corners[0].y = roi.y;

	_pre_image_ROI_corners[1].x = roi.x;
	_pre_image_ROI_corners[1].y = roi.y + roi.height;

	_pre_image_ROI_corners[2].x = roi.x + roi.width;
	_pre_image_ROI_corners[2].y = roi.y + roi.height;

	_pre_image_ROI_corners[3].x = roi.x + roi.width;
	_pre_image_ROI_corners[3].y = roi.y;

    std::cout << "init pre_feature_corners.size():" << _pre_feature_corners.size()  << std::endl;

	ShowTrackFeature();

	return true;
}

static void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

std::vector<cv::Point2f> MagicPenFeatureTrack::ROICorners() {
    return _pre_image_ROI_corners;
}

void MagicPenFeatureTrack::Track(cv::Mat image_gray) {
	if(!_init) {
		return;
	}

	std::vector<uchar> status;
	std::vector<float> err;
	std::vector<cv::Point2f> next_feature_corners;

	cv::calcOpticalFlowPyrLK(_pre_image, image_gray, _pre_feature_corners, next_feature_corners, status, err, cv::Size(21,21), 3);

	reduceVector(_pre_feature_corners, status);
	reduceVector(next_feature_corners, status);

	assert(_pre_feature_corners.size() == next_feature_corners.size());

	cv::Mat homography = cv::findHomography(_pre_feature_corners, next_feature_corners, cv::RANSAC, 5);
	cv::perspectiveTransform( _pre_image_ROI_corners, _pre_image_ROI_corners, homography);

	_pre_image = image_gray;
	_pre_feature_corners = next_feature_corners;

	std::cout << "track pre_feature_corners.size():" << _pre_feature_corners.size()  << std::endl;
	ShowTrackFeature();
}


void MagicPenFeatureTrack::ShowTrackFeature() {
#ifdef _WIN32
	cv::Mat show_track_image;
	_pre_image.copyTo(show_track_image);

	for (size_t i = 0; i < _pre_feature_corners.size(); i++) {
		cv::circle(show_track_image, _pre_feature_corners[i], 2, cv::Scalar(255.f));
	}

	cv::line(show_track_image, _pre_image_ROI_corners[0], _pre_image_ROI_corners[1], cv::Scalar(255.f));
	cv::line(show_track_image, _pre_image_ROI_corners[1], _pre_image_ROI_corners[2], cv::Scalar(255.f));
	cv::line(show_track_image, _pre_image_ROI_corners[2], _pre_image_ROI_corners[3], cv::Scalar(255.f));
	cv::line(show_track_image, _pre_image_ROI_corners[3], _pre_image_ROI_corners[0], cv::Scalar(255.f));

	cv::imshow("show_track_image", show_track_image);
#endif
}