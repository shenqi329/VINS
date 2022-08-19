#pragma once

#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

class MagicPenFeatureTrack
{
public:
	MagicPenFeatureTrack();

	bool Init(cv::Mat image_gray, cv::Rect roi);

	void Track(cv::Mat image_gray);

    float* buildProjectionMatrix(float nearp, float farp);
	float* GetViewMatrix();

	std::vector<cv::Point2f> GetROICorners();
private:
	void run();
	bool TrackWithCalcOpticalFlowPyrLK(cv::Mat image_gray);
    void MatchOriginFrame(cv::Mat image_gray);
	void GetOriginFrameROIKeyPoint(cv::Mat image_gray, cv::Rect roi);
	void ShowTrackFeature(char * name, std::vector<cv::Point2f> feature_corners, std::vector<cv::Point2f> _image_ROI_corners);

	cv::Mat _pre_image;
	cv::Mat _origin_image;

	std::vector<cv::Point2f> _origin_feature_corners;
	std::vector<cv::Point2f> _origin_image_ROI_corners;
    std::vector<cv::Point3f> _origin_image_ROI_corners_3d;

	std::vector<cv::Point2f> _pre_feature_corners;
	std::vector<cv::Point2f> _pre_image_ROI_corners;

	cv::Mat _camera_matrix;
	cv::Mat _distortion_coefficients;
	cv::Mat_<float> _projMatrix;
    cv::Mat _viewMatrix;
	bool _init = false;

	cv::Ptr<cv::Feature2D> _feature_detector;
	cv::Ptr<cv::DescriptorExtractor> _brief_extractor;
	std::vector<cv::KeyPoint> _origin_key_point;
	cv::Mat _origin_key_point_descs;
};

