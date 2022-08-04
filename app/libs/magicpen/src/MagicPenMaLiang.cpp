#include "MagicPenMaLiang.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
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

void MagicPenMaLiang::Init() {
    _render.Init();
}

void MagicPenMaLiang::setEdgeImageByte(std::vector<uchar> data) {

    cv::Mat edge_texture_image = cv::imdecode(data, cv::IMREAD_COLOR);
	cv::Mat edge_texture_image_rgba;
	cv::cvtColor(edge_texture_image , edge_texture_image_rgba, cv::COLOR_BGR2RGBA);

	_render.SetTextureEdgeImage(edge_texture_image_rgba);
}

bool MagicPenMaLiang::Magic(cv::Mat image, int texture_side_width, int texture_side_height) {

    if (_init_image) {
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
	std::vector<int> lower_bound = {155, 155, 155};
    std::vector<int> upper_bound = {255, 255, 255};
	cv::inRange(image, lower_bound, upper_bound, mask);

	cv::Rect maxWhiteRect;
	if(!GetMaskMaxWhiteRect(mask , maxWhiteRect)) {
        return false;
    }

	// 获取白色区域灰度图，用于后续检测
	cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
	image_gray(maxWhiteRect).copyTo(detected_edges);

    //![reduce_noise]
    /// Reduce noise with a kernel 3x3
    // cv::blur(detected_edges, detected_edges, cv::Size(3,3));
    //![reduce_noise]

    cv::imwrite("/storage/emulated/0/SBML/detected_edges.jpg",detected_edges);

    //![canny]
    /// Canny detector
    cv::Canny(detected_edges, detected_edges, threshold, threshold*ratio, kernel_size);
    //![canny]

	cv::imwrite("/storage/emulated/0/SBML/Canny_detected_edges.jpg",detected_edges);

    ConnectAdjacentEdge(detected_edges);

	// 查找轮廓
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(detected_edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	if (contours.size() <= 0) {
		return  false;
	}

	ApproxPoly(contours);

	// 筛选最大且居中的轮廓
	float maxArea = 0;
	int maxAreaIndex = -1;
    for (size_t i = 0; i < contours.size(); i++) {

        if (contours[i].size() < 50) {
            continue;
        }

		cv::RotatedRect minAreaRect = cv::minAreaRect(contours[i]);

        if (minAreaRect.size.area() < image.cols * image.rows / 64) {
            continue;
        }

		if (abs(minAreaRect.center.x - image.cols / 2) > image.cols / 6) {
			continue;
		}

		if (abs(minAreaRect.center.y + maxWhiteRect.y  - image.rows / 2) > image.rows / 6) {
			continue;
		}

		if (minAreaRect.size.area() > maxArea) {
			maxArea = minAreaRect.size.area();
			maxAreaIndex = i;
		}
    }

	if (maxAreaIndex < 0) {
		return false;
	}

	cv::Mat contours_img(detected_edges.size(), CV_8U, cv::Scalar(0));
	drawContours(contours_img, contours, maxAreaIndex, cv::Scalar(255), 1);
    cv::imwrite("/storage/emulated/0/SBML/contours_img.jpg",contours_img);
	//imshow("contours_img", contours_img);

	_origin_contour.contour_points.resize(contours[maxAreaIndex].size());
	for (size_t i = 0; i < contours[maxAreaIndex].size(); i++){
		_origin_contour.contour_points[i].point.x = contours[maxAreaIndex][i].x + maxWhiteRect.x;
		_origin_contour.contour_points[i].point.y = contours[maxAreaIndex][i].y + maxWhiteRect.y;
	}

	// 查找肢体(arms and legs)
	//FindLimbs(contours[maxAreaIndex]);

	MagicPenContour body_contour;
	_division_contour.clear();
	for (size_t i = 0; i < _origin_contour.contour_points.size(); i++) {
		
		bool isLimb = false;
		for (size_t j = 0; j < _limbInfo.size(); j++) {

			size_t end_index = _limbInfo[j].start_point_index + _limbInfo[j].end_point_offset;

			if(i > _limbInfo[j].start_point_index && i < end_index) {
				isLimb = true;
			}
		}

		if (isLimb) {
			continue;
		}

		MagicPenPoint point;
		point.point = _origin_contour.contour_points[i].point;
		point.index = i;

		_origin_contour.contour_points[i].indexs[0] = _division_contour.size();
		_origin_contour.contour_points[i].indexs[1] = body_contour.contour_points.size();

		body_contour.contour_points.push_back(point);
		body_contour.limb_info.type = MagicPenContourBody;
	}
	_division_contour.push_back(body_contour);

	for (size_t i = 0; i < _limbInfo.size(); i++) {
		MagicPenContour limbContour;
		MagicPenPoint point;

		for (size_t j = 0; j <= _limbInfo[i].end_point_offset; j++) {
			size_t index = (_limbInfo[i].start_point_index + j) % _origin_contour.contour_points.size();
			point.point = _origin_contour.contour_points[index].point;
			point.index = index;

			_origin_contour.contour_points[index].indexs[0] = _division_contour.size();
			_origin_contour.contour_points[index].indexs[1] = limbContour.contour_points.size();

			limbContour.contour_points.push_back(point);
		}
		limbContour.limb_info = _limbInfo[i];
		_division_contour.push_back(limbContour);
	}

	//_origin_contour = body_contour;
	// 三角形填充多边形
	PolyTriangulate(_division_contour);

    cv::Mat image_rgba;
    cv::cvtColor(image, image_rgba, cv::COLOR_BGR2RGBA);
	_3dModel.Init(_triangulate_result, _origin_contour, _division_contour,image.cols, image.rows, texture_side_width, texture_side_height, image_rgba);

#ifdef MagicPenMaLiang_DEBUG
	// Create the marker image for the watershed algorithm
    cv::Mat markers = cv::Mat::zeros(detected_edges.size(), CV_8U);
	drawContours(markers, contours, static_cast<int>(maxAreaIndex), cv::Scalar(255), -1);

	ShowDebugWindows(detected_edges, markers);
#endif
    _init_image = true;
	return true;
}

void MagicPenMaLiang::Draw(float timeStampSec) {
    _render.Draw(&_3dModel, timeStampSec);
}

void MagicPenMaLiang::Tick(float tick, MagicPenContour &contour) {

    cv::Point2f origin_start_point			= _origin_contour.contour_points[contour.contour_points[0									].index].point;
    cv::Point2f origin_end_point			= _origin_contour.contour_points[contour.contour_points[contour.contour_points.size() - 1	].index].point;
    cv::Point2f origin_max_distance_point	= _origin_contour.contour_points[contour.contour_points[contour.limb_info.max_point_offset	].index].point;

    cv::Point2f origin_mid_point = (origin_start_point + origin_end_point) / 2.0f;

    float angle = _tickSum * PI / 180;

    for (size_t j = 0; j < contour.contour_points.size(); j++) {

        cv::Point2f point = _origin_contour.contour_points[contour.contour_points[j].index].point;
        point -= origin_mid_point;
        cv::Point2f new_point;

        new_point.x = point.x * cos(angle) - point.y * sin(angle);
        new_point.y = point.x * sin(angle) + point.y * cos(angle);

        new_point += origin_mid_point;

        contour.contour_points[j].point = new_point;
    }
}


#ifdef MagicPenMaLiang_DEBUG
void MagicPenMaLiang::ShowDebugWindows_Points() {

	cv::Mat points = cv::Mat::zeros(_image.size(), CV_8U);
    for (size_t i = 0; i < _origin_contour.contour_points.size(); i++) {

		cv::circle(points, _origin_contour.contour_points[i].point, 1, cv::Scalar(255));
    }
	cv::imshow("Points", points);
}

void MagicPenMaLiang::ShowDebugWindows_Triangulate() {
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
}

void MagicPenMaLiang::ShowDebugWindows(cv::Mat detected_edges, cv::Mat &markers) {
	
	
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
}
#endif

MagicPen3DModel *MagicPenMaLiang::Get3DModel() {
	return &_3dModel;
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

void MagicPenMaLiang::CalculationLimbInfoType(MagicPenLimbInfo &limbInfo, cv::Rect boundRect) {

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

void MagicPenMaLiang::FindLimbs(std::vector<cv::Point> &contour) {

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


void MagicPenMaLiang::PolyTriangulate(std::vector<MagicPenContour> &contours) {

    TPPLPartition pp;

	TPPLPolyList inpolys;

	_triangulate_result.clear();

	for (size_t i = 0; i < contours.size(); i++) {
		MagicPenContour contour = contours[i];
		
		TPPLPoly poly;
		poly.Init(contour.contour_points.size());
		poly.SetHole(false);

		int reverseIndex = contour.contour_points.size() - 1;
		for (size_t j = 0; j < contour.contour_points.size(); j++) {
			
			int origin_index = contour.contour_points[reverseIndex - j].index;
			poly[j].x = contour.contour_points[reverseIndex - j].point.x;
			poly[j].y = contour.contour_points[reverseIndex - j].point.y;
			poly[j].id = origin_index;
		}

		inpolys.push_back(poly);
	}
    pp.Triangulate_EC(&inpolys, &_triangulate_result);
}

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