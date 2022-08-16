#include "MagicPenMaLiang.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include "polypartition.h"
#include "MagicPenUtil.h"

#define PI 3.14159265

#ifdef ANDROID
#include <android/log.h>
#define LOG_TAG "MagicPenMaLiang"
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#define LOGI(...)
#define LOGE(...)
#endif


void OffsetCache::Reset() {

}

void OffsetCache::AddInfo(OffsetInfo info) {

	float offset_x = info.offset_x - _validInfo.offset_x;
	float offset_y= info.offset_y - _validInfo.offset_y;

	if (abs(offset_x) > 20 || abs(offset_y) > 20) {
		if(ignore_count < 2) {
			ignore_count++;
			return;
		}else {
			ignore_count = 0;
		}
	}
	if(offset_x > 5.0f) {
		info.offset_x = _validInfo.offset_x + 5;
	} else if(offset_x < -5.0f) {
		info.offset_x = _validInfo.offset_x - 5;
	}

	if(offset_y > 5.0f) {
		info.offset_y = _validInfo.offset_y + 5;
	} else if(offset_y < -5.0f) {
		info.offset_y = _validInfo.offset_y - 5;
	}

	_validInfo = info;
}

OffsetInfo OffsetCache::GetValidOffsetInfo() {
	return _validInfo;
}


// MagicPenMaLiang begin
static void ApproxPoly(std::vector<std::vector<cv::Point> > &contours) {
	
	for (size_t i = 0; i < contours.size(); i++) {
		std::vector<cv::Point> outContour;
		cv::approxPolyDP(contours[i], outContour, 0.5, false);
		contours[i] = outContour;
	}
}

static bool IsLineBreak(cv::Mat &detected_edges, int row, int col) {

    int count = 0;

    int first_i;
    int first_j;
    bool near_first_second = false;

    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            
            if (255 == detected_edges.at<uchar>(row + i, col +j)) {
                if (i == 0 && j == 0) {
                    continue;
                }
                if (0 == count) {
                    first_i = i;
                    first_j = j;
                } else if (1 == count) {
                    near_first_second = (abs(first_i - i) + abs(first_j - j)) <= 1;
                }
                count++;    
            }
        }
    }

    if(count <= 1) {
        return true;
    }

    if (count == 2) {
        return near_first_second;
    }

    return false;
}

bool GetMaskMaxWhiteRect(cv::Mat mask , cv::Rect &rect) {

	bool setBegin = false;
	
	struct WhiteSpace {
		int beginRow = 0;
		int endRow = 0;
	};

	WhiteSpace maxWhiteSpace;
	WhiteSpace whiteSpace;

	std::vector<int> whiteTag(mask.rows);
	for (size_t row = 0; row < mask.rows; row++) {
		uchar* p1 = mask.ptr<uchar>(row);
		int whiteCount = 0;
		for (size_t col = 0; col < mask.cols; col++) {
			uchar c = p1[col];
			if (0 != c) {
				whiteCount++;
			}
			if (whiteCount > mask.rows / 2) {
				if (!setBegin) {
					whiteSpace.beginRow = row;
					setBegin = true;
				}
				break;
			}
		}
		if(setBegin) {
			
			whiteSpace.endRow = row;

			if (whiteCount < mask.rows / 2) {
				setBegin = false;
			}
			if (whiteSpace.endRow - whiteSpace.beginRow > maxWhiteSpace.endRow - maxWhiteSpace.beginRow) {
				maxWhiteSpace = whiteSpace;
			}
		}
	}

	rect.x = 0;
	rect.y = maxWhiteSpace.beginRow;
	rect.width = mask.cols;
	rect.height = maxWhiteSpace.endRow - maxWhiteSpace.beginRow + 1;


    if (maxWhiteSpace.endRow - maxWhiteSpace.beginRow < mask.rows / 4) {
        return false;
    }

    return true;
}


void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

float minDistance(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2) {
	
	float minDistance = std::numeric_limits<float>::max();
	for (size_t i = 0; i < contour1.size(); i++) {
		float distance = cv::pointPolygonTest(contour2, contour1[i], true);
		distance = - distance;
		if (minDistance > distance) {
			minDistance = distance;
		}
	}
	return minDistance;
}

void findNearbyContours(std::vector< std::vector<cv::Point>> &contours, cv::Point center) {
	
	std::vector<cv::Point> nearCenterContour;
	std::vector< std::vector<cv::Point>> nearbyContours;

	int mostNearCenterIndex = 0;
	float mostNearCenterDist = std::numeric_limits<float>::max();

	for (size_t i = 0; i < contours.size(); i++) {
		float distance = cv::pointPolygonTest(contours[i], center, true);
		distance = -distance;
		if (mostNearCenterDist > distance) {
			mostNearCenterDist = distance;
			mostNearCenterIndex = i;
		}
	}

	std::set<size_t> findSet;
	findSet.insert(mostNearCenterIndex);
	
	for (size_t i = 0; i < contours.size(); i++) {
		if (findSet.find(i) != findSet.end()){
			continue;
		}
		for (auto it = findSet.begin(); it != findSet.end(); ++it) {
			size_t index = *it;
			if (i == index) {
				continue;
			}
			float distance = minDistance(contours[i], contours[index]);
			if (distance < 20) {
				findSet.insert(i);
				i = 0;
				break;
			}
		}
	}


	for (size_t i = 0; i < contours.size(); i++) {
		if (findSet.find(i) != findSet.end()){
			nearbyContours.push_back(contours[i]);
		}
	}

	contours = nearbyContours;
}

bool isContoursVaild(std::vector< std::vector<cv::Point>> &contours, int cols, int rows) {
	std::vector<cv::Point> all_points;
	for (size_t i = 0; i < contours.size(); i++) {
		all_points.insert(all_points.end(), contours[i].begin(), contours[i].end());
	}

	cv::Rect boundingRect = cv::boundingRect(all_points);
	if (boundingRect.width > cols / 1.5) {
        LOGI("boundingRect.width big");
		return false;
	}

	if (boundingRect.width < cols / 8) {
        LOGI("boundingRect.width small");
		return false;
	}

	if (boundingRect.height > rows / 1.5) {
        LOGI("boundingRect.height width");
		return false;
	}

	if (boundingRect.height < rows / 8) {
        LOGI("boundingRect.height small");
		return false;
	}

	float centerXDistance = abs(boundingRect.x + boundingRect.width/2 - cols/2);
	if (centerXDistance > cols / 6) {
        LOGI("XDistance big:%f", centerXDistance);
		return false;
	}
	float centerYDistance = abs(boundingRect.y + boundingRect.height/2 - rows/2);
	if (centerYDistance > rows / 4) {
        LOGI("YDistance big:%f", centerYDistance);
		return false;
	}
	
	return true;
}

// MagicPenMaLiang begin
void MagicPenMaLiang::Init() {
    _render.Init();
}

void MagicPenMaLiang::setEdgeImageByte(std::vector<uchar> data) {

    cv::Mat edge_texture_image = cv::imdecode(data, cv::IMREAD_COLOR);
	cv::Mat edge_texture_image_rgba;
	cv::cvtColor(edge_texture_image , edge_texture_image_rgba, cv::COLOR_BGR2RGBA);

	_render.SetTextureEdgeImage(edge_texture_image_rgba);
}

void MagicPenMaLiang::setRotate(float x, float y) {
	_rotate_x = x;
	_rotate_y = y;
}

std::vector<std::vector<cv::Point> > MagicPenMaLiang::findContours(cv::Mat image, cv::Mat image_gray, cv::Rect &validRect) {
    if(!_init_image) {
        cv::imwrite("/storage/emulated/0/SBML/origin.jpg",image);
    }

	cv::Mat	detected_edges;
	cv::Mat	mask;

	// 查找白色区域，作为有效的检测区域
	std::vector<int> lower_bound = {100, 100, 100};
    std::vector<int> upper_bound = {255, 255, 255};
	cv::inRange(image, lower_bound, upper_bound, mask);

    if(!GetMaskMaxWhiteRect(mask , validRect)) {
        LOGI("GetMaskMaxWhiteRect Fail");
        return std::vector<std::vector<cv::Point> >(0);
    }

	// 获取白色区域灰度图，用于后续检测
    image_gray(validRect).copyTo(detected_edges);

    //![reduce_noise]
    /// Reduce noise with a kernel 3x3
    // cv::blur(detected_edges, detected_edges, cv::Size(3,3));
    //![reduce_noise]
    if(!_init_image) {
        cv::imwrite("/storage/emulated/0/SBML/detected_edges.jpg", detected_edges);
    }
    //![canny]
    /// Canny detector
    cv::Canny(detected_edges, detected_edges, 10, 200);
    //![canny]
    if(!_init_image) {
#ifdef _WIN32
		cv::imshow("detected_edges", detected_edges);
#else
        cv::imwrite("/storage/emulated/0/SBML/Canny_detected_edges.jpg", detected_edges);
#endif
    }
    ConnectAdjacentEdge(detected_edges);

	// 查找轮廓
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(detected_edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	if (contours.size() <= 0) {
        LOGI("findContours zero");
		return std::vector<std::vector<cv::Point> >(0);
	}

	ApproxPoly(contours);

	return contours;
}

cv::Rect MagicPenMaLiang::expandRect(cv::Rect src, float expand_value, cv::Mat image) {
	if (src.x > 10) {
		src.x -= 10;
	} else {
		src.x = 0;
	}

	int width = src.width + 20;
	src.width = src.x + width <= image.cols ? width : image.cols - src.x;

	if (src.y > 10) {
		src.y -= 10;
	} else {
		src.y = 0;
	}
	int height = src.height + 20;
	src.height = src.y + height <= image.cols ? height : image.rows - src.y;

	return src;
}

void ShowROI(cv::Mat image, cv::Rect roi) {
#if 0
	cv::Mat roi_image;
	image.copyTo(roi_image);

	cv::rectangle(roi_image, roi, cv::Scalar(127.f));
	cv::imshow("roi_image", roi_image);
#endif
}

bool MagicPenMaLiang::Magic(cv::Mat image, int texture_side_width, int texture_side_height) {

	cv::Mat image_gray;
	cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

	if(!_init_image) {

		cv::Rect validRect;
		// 获取轮廓
		std::vector<std::vector<cv::Point> > contours = findContours(image, image_gray, validRect);
		if(contours.size() <= 0) {
			return false;
		}
		std::cout << "contours.size()"<< contours.size() << std::endl;
#ifdef _WIN32
		cv::Mat contours_dst = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);
		for (size_t i = 0; i < contours.size(); i++) {
			cv::drawContours(contours_dst, contours, i , cv::Scalar(255.f));
			char buffer[128];
			sprintf(buffer, "contour_%d", i);
		}
		// cv::imshow("contour", contours_dst);
#endif

        // 找到位于中心区域的轮廓
        findNearbyContours(contours, cv::Point(image.cols / 2, image.rows / 2));

        if(!isContoursVaild(contours, image.cols, image.rows)) {
            LOGI("isContoursVaild Fail");
            return false;
        }

        if(contours.size() <= 0){
            return false;
        }

		std::vector<cv::Point> all_contour;
		for (size_t i = 0; i < contours.size(); i++) {
			all_contour.insert(all_contour.end(), contours[i].begin(), contours[i].end());
		}

        cv::Rect ROI = cv::minAreaRect(all_contour).boundingRect();
        ROI.x += validRect.x;
        ROI.y += validRect.y;

		if(!_feature_track.Init(image_gray, ROI)) {
			return false;
		}
		std::vector<cv::Point2f> roi_corners = _feature_track.GetROICorners();
		_pre_x = (roi_corners[1].x + roi_corners[2].x) / 2;
		_pre_y = (roi_corners[1].y + roi_corners[2].y) / 2;
		_pre_distance = MagicPenUtil::GetDistance(roi_corners[1], roi_corners[2]);

		// 根据轮廓生成3D模型
		cv::Mat image_rgba;
		cv::cvtColor(image, image_rgba, cv::COLOR_BGR2RGBA);
		_3dModels.InitFromContours(contours, validRect.x, validRect.y, image.cols, image.rows, texture_side_width, texture_side_height, image_rgba);

		_init_image = true;

		ShowROI(image, ROI);
	} else {

		_feature_track.Track(image_gray);
        std::vector<cv::Point2f> roi_corners = _feature_track.GetROICorners();
		float cur_x = (roi_corners[1].x + roi_corners[2].x) / 2;
		float cur_y = (roi_corners[1].y + roi_corners[2].y) / 2;
		float distance = MagicPenUtil::GetDistance(roi_corners[1], roi_corners[2]);

		//_3dModels._offset_x =   (cur_x - _pre_x) / (image.cols / 2);
		//_3dModels._offset_y =   - (cur_y - _pre_y) / (image.rows / 2);
		//_3dModels._scale    =   distance / _pre_distance;
		return false;
	}
	return true;
}

std::vector<cv::Point2f> MagicPenMaLiang::GetTrackRectPoints() {
	return _feature_track.GetROICorners();
}


void MagicPenMaLiang::Draw(double timeStampSec) {
#ifdef _WIN32
	glClear(GL_COLOR_BUFFER_BIT);
#endif

	_render.Draw(&_3dModels, timeStampSec, _rotate_x, _rotate_y);
}

static TPPLOrientation GetOrientation(std::vector<cv::Point> &contour, long startIndex, long size) {
	long i1, i2;
	tppl_float area = 0;

	int count = 0;
	for (i1 = startIndex; i1 < startIndex + size; i1++) {
		i1 = i1 % contour.size();
		i2 = (i1 + 1) % contour.size();

		count++;
		if(size == count) {
			i2 = startIndex;
		}

		area += contour[i1].x * contour[i2].y - contour[i1].y * contour[i2].x;

		if(size == count) {
			break;
		}
	}
	if (area > 0) {
		return TPPL_ORIENTATION_CCW;
	}
	if (area < 0) {
		return TPPL_ORIENTATION_CW;
	}
	return TPPL_ORIENTATION_NONE;
}
#if 0
void MagicPenMaLiang::CalculationLimbInfoType(MagicPenContourHander &hander, cv::Rect boundRect) {

	size_t max_point_index = (limbInfo.start_point_index + limbInfo.max_point_offset) % _origin_contour.contour_points.size();

	cv::Point2f start_point = _origin_contour.contour_points[limbInfo.start_point_index].point;
	cv::Point2f max_point	= _origin_contour.contour_points[max_point_index].point;
	
	int x_offset = max_point.x - start_point.x;
	int y_offset = max_point.y - start_point.y;

	float x_pos = (float)(max_point.x - boundRect.x) / boundRect.width;
	float y_pos = (float)(max_point.y - boundRect.y) / boundRect.height;

	
	if (y_pos < 0.7) {
		if (x_pos < 0.3) {
			limbInfo.type = MagicPenContourArm_Left;
		}
		if (x_pos > 0.7) {
			limbInfo.type = MagicPenContourArm_Right;
		}	
	} else {
		if (x_pos < 0.5) {
			limbInfo.type = MagicPenContourLeg_Left;
		} else {
			limbInfo.type = MagicPenContourLeg_Right;
		}
	}
}

void MagicPenMaLiang::FindLimbs(MagicPenContourHander &hander) {

	std::vector<MagicPenLimbInfo> limbInfos;

	cv::RotatedRect minAreaRect = cv::minAreaRect(contour);
	cv::Rect rect = minAreaRect.boundingRect();

	int minLimbThickThreshold = MIN(rect.width, rect.height) / 10;
	int maxLimbLengthThreshold = MAX(rect.width, rect.height) / 8;

	for (size_t i = 0; i < contour.size(); i++) {
		
		float  minDistance = FLT_MAX;
		float  maxDistance = 0;
		float  maxDistanceTmp = 0;
		size_t end_point_offset = 0;
		size_t max_point_offset = 0;
		size_t max_point_offset_tmp = 0;

		for (size_t j = 1; j < contour.size() / 4; j++) {
			
			size_t next = (i + j) % contour.size();
			
			int distance_x = contour[i].x - contour[next].x;
			int distance_y = contour[i].y - contour[next].y;

			float distance = sqrtf(distance_x*distance_x + distance_y*distance_y);
			
			if (maxDistanceTmp < distance) {
				maxDistanceTmp = distance;
				max_point_offset_tmp = j;
			}

			if (minDistance > distance) {
				minDistance = distance;
				end_point_offset = j;
				if (maxDistance < maxDistanceTmp) {
					maxDistance = maxDistanceTmp;
					max_point_offset = max_point_offset_tmp;
				}
			}
		}

		if (minDistance > minLimbThickThreshold) {
			continue;
		}
		if (maxDistance < maxLimbLengthThreshold) {
			continue;
		}

		TPPLOrientation orientation = GetOrientation(contour, i, end_point_offset);
		if (TPPL_ORIENTATION_CCW == orientation) {
			continue;
		}

		MagicPenLimbInfo limbInfo;
		limbInfo.minDistance = minDistance;
		limbInfo.start_point_index = i;
		limbInfo.end_point_offset = end_point_offset;
		limbInfo.max_point_offset = max_point_offset;

		limbInfos.push_back(limbInfo);
	}

	_limbInfo.clear();
	
	for (size_t i = 0; i < limbInfos.size(); ) {

		MagicPenLimbInfo bestInfo = limbInfos[i];
		size_t j = i + 1;
		for (; j < limbInfos.size(); j++) {
			
			if(bestInfo.start_point_index + bestInfo.end_point_offset < limbInfos[j].start_point_index) {
				i = j;
				break;
			}
		}
		i = j;
		
		CalculationLimbInfoType(bestInfo, rect);
		_limbInfo.push_back(bestInfo);
	}
}
#endif

void MagicPenMaLiang::ConnectAdjacentEdge(cv::Mat &detected_edges) {
	for (int row = 1; row < detected_edges.rows - 1; row++) {

		for (int col = 1; col < detected_edges.cols - 1; col++) {
      
			if(0 == detected_edges.at<uchar>(row, col)) {
				continue;
			}

			if(!IsLineBreak(detected_edges, row, col)) {
				continue; 
			}

			for (int i = -1; i <= 1; i++) {
				for (int j = -1; j <= 1; j++) {
					detected_edges.at<uchar>(row + i, col +j) = 255;
				}
			}
		}
	}
}
// MagicPenMaLiang end