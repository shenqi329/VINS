#include "MagicPenMaLiang.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include "polypartition.h"

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
		return false;
	}

	if (boundingRect.width < cols / 8) {
		return false;
	}

	if (boundingRect.height > rows / 1.5) {
		return false;
	}

	if (boundingRect.height < rows / 8) {
		return false;
	}

	float centerXDistance = abs(boundingRect.x + boundingRect.width/2 - cols/2);
	if (centerXDistance > cols / 6) {
		return false;
	}
	float centerYDistance = abs(boundingRect.y + boundingRect.height/2 - rows/2);
	if (centerYDistance > rows / 6) {
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

bool MagicPenMaLiang::Magic(cv::Mat image, int texture_side_width, int texture_side_height) {

	if(_init_image) {
        return false;
	}

	_image = image;

	_texture_side_width = texture_side_width;
	_texture_side_height = texture_side_height;

    cv::imwrite("/storage/emulated/0/SBML/origin.jpg",image);

	cv::Mat image_gray;
	cv::Mat	detected_edges;
	cv::Mat	mask;

	// 查找白色区域
	std::vector<int> lower_bound = {100, 100, 100};
    std::vector<int> upper_bound = {255, 255, 255};
	cv::inRange(image, lower_bound, upper_bound, mask);

	cv::Rect maxWhiteRect;
	if(!GetMaskMaxWhiteRect(mask , maxWhiteRect)) {
        return false;
    }

	
	cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

	// 获取白色区域灰度图，用于后续检测
	image_gray(maxWhiteRect).copyTo(detected_edges);

    //![reduce_noise]
    /// Reduce noise with a kernel 3x3
    // cv::blur(detected_edges, detected_edges, cv::Size(3,3));
    //![reduce_noise]

    cv::imwrite("/storage/emulated/0/SBML/detected_edges.jpg",detected_edges);

    //![canny]
    /// Canny detector
    cv::Canny(detected_edges, detected_edges, 10, 200);
    //![canny]

	cv::imwrite("/storage/emulated/0/SBML/Canny_detected_edges.jpg",detected_edges);

    ConnectAdjacentEdge(detected_edges);

	// 查找轮廓
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(detected_edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	if (contours.size() <= 0) {
		return false;
	}

	ApproxPoly(contours);

	findNearbyContours(contours, cv::Point(_image.cols / 2, _image.rows / 2));
	
	if(!isContoursVaild(contours, _image.cols, _image.rows)) {
		return false;
	}
	
	cv::Mat image_rgba;
	cv::cvtColor(image, image_rgba, cv::COLOR_BGR2RGBA);
	_3dModels.InitFromContours(contours, maxWhiteRect.x, maxWhiteRect.y, image.cols, image.rows, texture_side_width,  texture_side_height, image_rgba);

    _init_image = true;
	return true;
}

void MagicPenMaLiang::Draw(double timeStampSec) {
#ifdef _WIN32
	glClear(GL_COLOR_BUFFER_BIT);
#endif

	_render.Draw(&_3dModels, timeStampSec, _rotate_x, _rotate_y);
}

#ifdef MagicPenMaLiang_DEBUG
void MagicPenMaLiang::ShowDebugWindows_Points() {
#if 0
	cv::Mat points = cv::Mat::zeros(_image.size(), CV_8U);
    for (size_t i = 0; i < _origin_contour.contour_points.size(); i++) {

		cv::circle(points, _origin_contour.contour_points[i].point, 1, cv::Scalar(255));
    }
	cv::imshow("Points", points);
#endif
}


void MagicPenMaLiang::ShowDebugWindows_Triangulate() {
#if 0
	// Create the marker image for the watershed algorithm
    cv::Mat triangulate = cv::Mat::zeros(_image.size(), CV_8U);

    std::list<TPPLPoly>::iterator iter;

    for(iter = _triangulate_result.begin(); iter != _triangulate_result.end() ;iter++) {
        if(3 == iter->GetNumPoints()) {
            cv::Point point_a(iter->GetPoints()[0].x, iter->GetPoints()[0].y);
            cv::Point point_b(iter->GetPoints()[1].x, iter->GetPoints()[1].y);
            cv::Point point_c(iter->GetPoints()[2].x, iter->GetPoints()[2].y);
            cv::line(triangulate, point_a, point_b, cv::Scalar(255));
            cv::line(triangulate, point_b, point_c, cv::Scalar(255));
            cv::line(triangulate, point_c, point_a, cv::Scalar(255));
        }
    }
    cv::imshow("Triangulate", triangulate);
#endif
}

void MagicPenMaLiang::ShowDebugWindows(cv::Mat detected_edges, cv::Mat &markers) {
	
#if 0
	ShowDebugWindows_Points();

    ShowDebugWindows_Triangulate();

	for (size_t i = 0; i < _limbInfo.size(); i++) {
		cv::line(markers, _origin_contour.contour_points[_limbInfo[i].start_point_index].point, _origin_contour.contour_points[(_limbInfo[i].start_point_index + _limbInfo[i].end_point_offset) % _origin_contour.contour_points.size()].point, cv::Scalar(125), 2);
		cv::circle(markers, _origin_contour.contour_points[_limbInfo[i].start_point_index].point, 1, 125);
		//cv::circle(markers, contour[(i + minSteps)%contour.size()], 1, cv::Scalar(i % 125));
	}
	// Draw the background marker
    cv::imshow("Markers", markers);

    //![display]
    cv::imshow("Edge Map", detected_edges);
    //![display]
#endif
}
#endif

#if 0
MagicPen3DModel *MagicPenMaLiang::Get3DModel() {
	return &_3dModel;
}
#endif

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