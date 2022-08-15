
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>

#include "MagicPenFeatureTrack.h"
#include "MagicPenUtil.h"

static void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

static bool IsHomographyValid(cv::Mat homography, std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints) {

	std::vector<cv::Point2f> dstCmpPoints;

	cv::perspectiveTransform(srcPoints, dstCmpPoints, homography);

	int outline_count = 0;
	for (size_t i = 0; i < srcPoints.size(); i++) {
		if(MagicPenUtil::GetDistance(dstCmpPoints[i], dstPoints[i]) > 2.f) {
			outline_count ++;
		}
	}

	float out_scale = outline_count / (float)srcPoints.size();

	std::cout << "out_scale:" << out_scale << std::endl;

	if(out_scale > 0.6) {
		return false;
	}
	return true;
}


bool MagicPenFeatureTrack::Init(cv::Mat image_gray, cv::Rect roi) {

	_pre_image_ROI_corners.resize(4);
	_origin_image_ROI_corners.resize(4);

	_init = false;

	cv::Mat roi_image = image_gray(roi);
	cv::goodFeaturesToTrack(roi_image,  _pre_feature_corners, 200, 0.01, 3.0, cv::Mat(), 3, false, 0.04);
	
	if (_pre_feature_corners.size() < 50) {
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

	_origin_image_ROI_corners = _pre_image_ROI_corners;
	_origin_feature_corners = _pre_feature_corners;
	_origin_image = image_gray;

	//ShowTrackFeature("show_track_image", _pre_feature_corners, _pre_image_ROI_corners);

	return true;
}

std::vector<cv::Point2f> MagicPenFeatureTrack::ROICorners() {
	if(!_init) {
		return _origin_image_ROI_corners;
	}
    return _pre_image_ROI_corners;
}


bool MagicPenFeatureTrack::CompareWithFirstImage(cv::Mat cur_image_gray) {

	std::vector<uchar> status;
	std::vector<float> err;
	std::vector<cv::Point2f> cur_feature_corners;

	cv::calcOpticalFlowPyrLK(_origin_image, cur_image_gray, _origin_feature_corners, cur_feature_corners, status, err, cv::Size(21,21), 3);

	std::vector<cv::Point2f> origin_feature_corners = _origin_feature_corners;

	reduceVector(origin_feature_corners, status);
	reduceVector(cur_feature_corners, status);

	if(_pre_feature_corners.size() < 10) {
		return false;
	}

	std::vector<cv::Point2f> cur_image_ROI_corners;
	cv::Mat homography = cv::findHomography(origin_feature_corners, cur_feature_corners, cv::RANSAC, 5);
	cv::perspectiveTransform( _origin_image_ROI_corners, cur_image_ROI_corners, homography);

	if(IsHomographyValid(homography, origin_feature_corners, cur_feature_corners)) {
		return false;
	}

	std::cout << "track origin_feature_corners.size():" << origin_feature_corners.size()  << std::endl;

	ShowTrackFeature("show_track_origin_image", cur_feature_corners, cur_image_ROI_corners);

	return true;
}

void MagicPenFeatureTrack::TrackWithCalcOpticalFlowPyrLK(cv::Mat cur_image_gray) {
	std::vector<uchar> status;
	std::vector<float> err;
	std::vector<cv::Point2f> cur_feature_corners;

	cv::calcOpticalFlowPyrLK(_pre_image, cur_image_gray, _pre_feature_corners, cur_feature_corners, status, err, cv::Size(21,21), 3);

	reduceVector(_pre_feature_corners, status);
	reduceVector(cur_feature_corners, status);

	if(_pre_feature_corners.size() < 10) {
		return;
	}

	cv::Mat homography = cv::findHomography(_pre_feature_corners, cur_feature_corners, cv::RANSAC, 5);

	if(!IsHomographyValid(homography, _pre_feature_corners, cur_feature_corners)) {
		_init = false;
		return;
	}

	cv::perspectiveTransform(_pre_image_ROI_corners, _pre_image_ROI_corners, homography);
	
	_pre_image = cur_image_gray;
#if 1
	_pre_feature_corners = cur_feature_corners;
#else
	cv::Rect roi = cv::boundingRect(_pre_image_ROI_corners);
	cv::goodFeaturesToTrack(cur_image_gray(roi),  _pre_feature_corners, 200, 0.01, 3.0, cv::Mat(), 3, false, 0.04);
	
	if (_pre_feature_corners.size() < 30) {
        return;
    }
	for (size_t i = 0; i < _pre_feature_corners.size(); i++) {
		_pre_feature_corners[i].x += roi.x;
		_pre_feature_corners[i].y += roi.y;
	}
#endif
	std::cout << "track pre_feature_corners.size():" << _pre_feature_corners.size()  << std::endl;
	ShowTrackFeature("show_track_image", _pre_feature_corners, _pre_image_ROI_corners);
}

void MagicPenFeatureTrack::Track(cv::Mat image_gray) {

	if(!_init) {
		//CompareWithFirstImage(image_gray);
		return;
	}

	TrackWithCalcOpticalFlowPyrLK(image_gray);
	//CompareWithFirstImage(image_gray);
}


void MagicPenFeatureTrack::ShowTrackFeature(char * name, std::vector<cv::Point2f> feature_corners, std::vector<cv::Point2f> image_ROI_corners) {
#ifdef _WIN32
	cv::Mat show_track_image;
	_pre_image.copyTo(show_track_image);

	for (size_t i = 0; i < feature_corners.size(); i++) {
		cv::circle(show_track_image, feature_corners[i], 2, cv::Scalar(255.f));
	}

	cv::line(show_track_image, image_ROI_corners[0], image_ROI_corners[1], cv::Scalar(255.f));
	cv::line(show_track_image, image_ROI_corners[1], image_ROI_corners[2], cv::Scalar(255.f));
	cv::line(show_track_image, image_ROI_corners[2], image_ROI_corners[3], cv::Scalar(255.f));
	cv::line(show_track_image, image_ROI_corners[3], image_ROI_corners[0], cv::Scalar(255.f));

	cv::imshow(name, show_track_image);
#endif
}