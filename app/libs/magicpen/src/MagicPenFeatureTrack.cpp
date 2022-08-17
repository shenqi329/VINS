
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>

#include "MagicPenFeatureTrack.h"
#include "MagicPenUtil.h"

#ifdef ANDROID
#include <android/log.h>
#define LOG_TAG "MagicPenFeatureTrack"
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#define LOGI(...)
#define LOGE(...)
#endif

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

MagicPenFeatureTrack::MagicPenFeatureTrack() {
	static double intrinsic[9] = {  284.0,       0,             160.0,
							            0,       284.0,         180.0,
							            0,       0,             1};

	static double distCoeff[5] = {0.00, 0.00, 0.000, 0.00, 0.00};

	_camera_matrix = cv::Mat(3, 3, CV_64FC1, intrinsic);
	_distortion_coefficients = cv::Mat(5,1,CV_64FC1, distCoeff);
}

//float projMatrix[16];
float* MagicPenFeatureTrack::buildProjectionMatrix(float nearp, float farp) {
    _projMatrix.create(4, 4);
    _projMatrix.setTo(0.f);

    float f_x = _camera_matrix.at<double>(0, 0);
    float f_y = _camera_matrix.at<double>(1, 1);

    float c_x = _camera_matrix.at<double>(0, 2);
    float c_y = _camera_matrix.at<double>(1, 2);

    _projMatrix.at<float>(0, 0) = 2 * f_x / (float)320;
    _projMatrix.at<float>(1, 1) = 2 * f_y / (float)360;

    _projMatrix.at<float>(2, 0) = 1.0f - 2 * c_x / (float)320;
    _projMatrix.at<float>(2, 1) = 2 * c_y / (float)360 - 1.0f;
    _projMatrix.at<float>(2, 2) = -(farp + nearp) / (farp - nearp);
    _projMatrix.at<float>(2, 3) = -1.0f;

    _projMatrix.at<float>(3, 2) = -2.0f*farp*nearp / (farp - nearp);

    return (float*)_projMatrix.data;
}

bool MagicPenFeatureTrack::Init(cv::Mat image_gray, cv::Rect roi) {

	_pre_image_ROI_corners.resize(4);
	_origin_image_ROI_corners.resize(4);
    _origin_image_ROI_corners_3d.resize(4);

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

	_pre_image_ROI_corners[1].x = roi.x + roi.width;
	_pre_image_ROI_corners[1].y = roi.y;

	_pre_image_ROI_corners[2].x = roi.x + roi.width;
	_pre_image_ROI_corners[2].y = roi.y + roi.height;

	_pre_image_ROI_corners[3].x = roi.x;
	_pre_image_ROI_corners[3].y = roi.y + roi.height;

    std::cout << "init pre_feature_corners.size():" << _pre_feature_corners.size()  << std::endl;

	_origin_image_ROI_corners = _pre_image_ROI_corners;
	_origin_feature_corners = _pre_feature_corners;
	_origin_image = image_gray;

    for (size_t i = 0; i < 4; i++)
        _origin_image_ROI_corners_3d[i] = cv::Point3f(_origin_image_ROI_corners[i].x/(float)image_gray.cols  - 0.5,
                                                      -_origin_image_ROI_corners[i].y/(float)image_gray.rows + 0.5, 0);

	//ShowTrackFeature("show_track_image", _pre_feature_corners, _pre_image_ROI_corners);

	return true;
}

std::vector<cv::Point2f> MagicPenFeatureTrack::GetROICorners() {
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

bool MagicPenFeatureTrack::TrackWithCalcOpticalFlowPyrLK(cv::Mat cur_image_gray) {
	std::vector<uchar> status;
	std::vector<float> err;
	std::vector<cv::Point2f> cur_feature_corners;

    if(_pre_feature_corners.size() < 10) {
        return false;
    }
	cv::calcOpticalFlowPyrLK(_pre_image, cur_image_gray, _pre_feature_corners, cur_feature_corners, status, err, cv::Size(21,21), 3);

	reduceVector(_pre_feature_corners, status);
	reduceVector(cur_feature_corners, status);

	if(_pre_feature_corners.size() < 10) {
		return false;
	}

	cv::Mat homography = cv::findHomography(_pre_feature_corners, cur_feature_corners, cv::RANSAC, 5);

	if(!IsHomographyValid(homography, _pre_feature_corners, cur_feature_corners)) {
		return false;
	}

	cv::perspectiveTransform(_pre_image_ROI_corners, _pre_image_ROI_corners, homography);


    // camera pose
	cv::Vec3d rvec, tvec;
	cv::solvePnP(_origin_image_ROI_corners_3d, _pre_image_ROI_corners, _camera_matrix, _distortion_coefficients, rvec, tvec);
	cv::Mat viewMatrixf = cv::Mat::zeros(4, 4, CV_32F);
	cv::Mat rot;

	Rodrigues(rvec, rot);
	for (unsigned int row = 0; row < 3; ++row) {
		for (unsigned int col = 0; col < 3; ++col) {
			viewMatrixf.at<float>(row, col) = (float)rot.at<double>(row, col);
		}
		viewMatrixf.at<float>(row, 3) = (float)tvec[row];
	}
	viewMatrixf.at<float>(3, 3) = 1.0f;

	cv::Mat cvToGl = cv::Mat::zeros(4, 4, CV_32F);
	cvToGl.at<float>(0, 0) = 1.0f;
	cvToGl.at<float>(1, 1) = -1.0f; // Invert the y axis
	cvToGl.at<float>(2, 2) = -1.0f; // invert the z axis
	cvToGl.at<float>(3, 3) = 1.0f;
	viewMatrixf = cvToGl * viewMatrixf;
	cv::transpose(viewMatrixf, viewMatrixf);
	_viewMatrix = viewMatrixf;
    std::cout << "viewMatrixf:" << viewMatrixf << std::endl;

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

    return true;
}

void MagicPenFeatureTrack::Track(cv::Mat image_gray) {

	if(!_init) {
		//CompareWithFirstImage(image_gray);
		return;
	}

	if(!TrackWithCalcOpticalFlowPyrLK(image_gray)) {
		_init = false;
	}
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