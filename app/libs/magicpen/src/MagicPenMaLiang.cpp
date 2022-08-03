#include "MagicPenMaLiang.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "polypartition.h"

#define PI 3.14159265

// MagicPenModel3D begin

void MagicPen3DModel::Init(std::list<TPPLPoly> triangles, MagicPenContour origin_contour, std::vector<MagicPenContour> division_contour, int cols, int rows,  int texture_side_width, int texture_side_height) {
	

	FreeVertice();

	InitVerticesFront(triangles, origin_contour, cols, rows);
	
	InitVerticesEdge(origin_contour, division_contour, cols, rows, texture_side_width, texture_side_height);
}

void MagicPen3DModel::InitVerticesFront(std::list<TPPLPoly> triangles, MagicPenContour origin_contour, int cols, int rows) {

	_vertices_front_size = sizeof(MagicPenVertice) * 3 * triangles.size();
	_vertices_front = (MagicPenVertice*)malloc(_vertices_front_size);

	_indices_front_size = sizeof(int) * 3 * triangles.size();
	_indices_front = (int*)malloc(_indices_front_size);

	int index = 0;
	for(std::list<TPPLPoly>::iterator iter = triangles.begin() ; iter != triangles.end() ;iter++, index++) {
		
		if(3 != iter->GetNumPoints()) {
			continue;
		}

		for (size_t i = 0; i < 3; i++) {

			int offset = (index*3 + i);

			// position
			_vertices_front[offset].positions[0] =  (iter->GetPoints()[i].x - cols/2) * 2 / cols;
			_vertices_front[offset].positions[1] = -(iter->GetPoints()[i].y - rows/2) * 2 / rows;
			_vertices_front[offset].positions[2] = 0.0f;

			// color
			_vertices_front[offset].colors[0] = 0.5f;
			_vertices_front[offset].colors[1] = 0.5f;
			_vertices_front[offset].colors[2] = 0.5f;

			// texture coords
			size_t origin_contour_index = iter->GetPoints()[i].id;
			_vertices_front[offset].textures[0] = (float)origin_contour.contour_points[origin_contour_index].point.x / cols;
			_vertices_front[offset].textures[1] = (float)origin_contour.contour_points[origin_contour_index].point.y / rows;

			_indices_front[index*3 + i] = index*3 + i;
		}
	}

}

void MagicPen3DModel::InitVerticesEdge(MagicPenContour origin_contour, std::vector<MagicPenContour> division_contour, int cols, int rows, int texture_side_width, int texture_side_height) {

	_vertices_side_size = sizeof(MagicPenVertice) * 2 * origin_contour.contour_points.size();
	_vertices_side = (MagicPenVertice*)malloc(_vertices_side_size);

	_indices_side_size = sizeof(int) * 3 * 2 * origin_contour.contour_points.size();
	_indices_side = (int*)malloc(_indices_side_size);

	float distance_total = 0;
	for (int index = 0; index < origin_contour.contour_points.size(); index++) {

		int point_next_index = (index + 1) % origin_contour.contour_points.size();

		float point_x =  (float(origin_contour.contour_points[index].point.x) - cols/2) * 2 / cols;
		float point_y = -(float(origin_contour.contour_points[index].point.y) - rows/2) * 2 / rows;

		size_t id0 = origin_contour.contour_points[index].indexs[0];
		size_t id1 = origin_contour.contour_points[index].indexs[1];

		float point_x_tick = (float(division_contour[id0].contour_points[id1].point.x) - cols/2) * 2 / cols;
		float point_y_tick = -(float(division_contour[id0].contour_points[id1].point.y) - rows/2) * 2 / rows;

		float point_next_x =  (float(origin_contour.contour_points[point_next_index].point.x) - cols/2) * 2 / cols;
		float point_next_y = -(float(origin_contour.contour_points[point_next_index].point.y) - rows/2) * 2 / rows;

		float x_distance = point_x - point_next_x;
		float y_distance = point_y - point_next_y;
		float distance = sqrtf(x_distance*x_distance + y_distance * y_distance);
		distance = distance * texture_side_width / 0.1f / texture_side_height;

		distance_total += distance;
		for (int i = 0; i < 2; i++) {
			int offset = (index*2 + i);

			_vertices_side[offset].positions[0] =  point_x_tick;
			_vertices_side[offset].positions[1] =  point_y_tick;

			if(0 == i) {
				_vertices_side[offset].positions[2] = 0.0f;
				_vertices_side[offset].textures[0] = 0;  //  texture coords x
			} else {
				_vertices_side[offset].positions[2] = -0.1f;
				_vertices_side[offset].textures[0] = 1.0f; // texture coords x
			}
			_vertices_side[offset].textures[1] = distance_total;  //texture coords y

			// color
			_vertices_side[offset].colors[0] = 0.5f;
			_vertices_side[offset].colors[1] = 0.5f;
			_vertices_side[offset].colors[2] = 0.5f;
		}
		
		if (distance_total >= 0.5f) {
			distance_total = 0.0f;
		}

		int offset = index * 3 * 2;
		// first triangle 
		_indices_side[offset + 0] = index * 2;
		_indices_side[offset + 1] = index * 2 + 1;
		_indices_side[offset + 2] = point_next_index * 2;

		// second triangle
		_indices_side[offset + 3] = index * 2 + 1;
		_indices_side[offset + 4] = point_next_index * 2;
		_indices_side[offset + 5] = point_next_index * 2 + 1;
	}
}

MagicPen3DModel::~MagicPen3DModel() {

	FreeVertice();

}

void MagicPen3DModel::FreeVertice() {

	// vertices front
	if(_vertices_front) {
		free(_vertices_front);
		_vertices_front = nullptr;
		_vertices_front_size = 0;
	}

	if(_indices_front) {
		free(_indices_front);
		_indices_front = nullptr;
		_indices_front_size = 0;
	}

	// vertices edge
	if (_vertices_side) {
		free(_vertices_side);
		_vertices_side = nullptr;
		_vertices_side_size = 0;
	}

	if (_indices_side) {
		free(_indices_side);
		_indices_side = nullptr;
		_indices_side_size = 0;
	}
}

// MagicPenModel3D end


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


bool MagicPenMaLiang::Magic(cv::Mat image, int texture_side_width, int texture_side_height) {

	_image = image;
	_texture_side_width = texture_side_width;
	_texture_side_height = texture_side_height;

	cv::Mat image_gray;
	cv::Mat	detected_edges;

	cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

    //![reduce_noise]
    /// Reduce noise with a kernel 3x3
    cv::blur(image_gray, detected_edges, cv::Size(3,3));
    //![reduce_noise]

    //![canny]
    /// Canny detector
    cv::Canny(detected_edges, detected_edges, threshold, threshold*ratio, kernel_size);
    //![canny]

    ConnectAdjacentEdge(detected_edges);

	// 查找轮廓
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(detected_edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	if (contours.size() <= 0) {
		return  false;
	}

	ApproxPoly(contours);
	float maxArea = 0;
	int maxAreaIndex = 0;
    for (size_t i = 0; i < contours.size(); i++) {
		cv::RotatedRect minAreaRect = cv::minAreaRect(contours[i]);
		float area = minAreaRect.size.width * minAreaRect.size.height;
		if (area > maxArea) {
			maxAreaIndex = i;
		}
    }

	_origin_contour.contour_points.resize(contours[maxAreaIndex].size());
	for (size_t i = 0; i < contours[maxAreaIndex].size(); i++){
		_origin_contour.contour_points[i].point = contours[maxAreaIndex][i];
	}
	
	// 查找肢体(arms and legs)
	FindLimbs(contours[maxAreaIndex]);

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

	_3dModel.Init(_triangulate_result, _origin_contour, _division_contour,image.cols, image.rows, texture_side_width, texture_side_height);

#ifdef MagicPenMaLiang_DEBUG
	// Create the marker image for the watershed algorithm
    cv::Mat markers = cv::Mat::zeros(detected_edges.size(), CV_8U);
	drawContours(markers, contours, static_cast<int>(maxAreaIndex), cv::Scalar(255), -1);

	ShowDebugWindows(detected_edges, markers);
#endif

	return true;
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


void MagicPenMaLiang::Tick(float tick) {
	_tickSum += 0.2;
	if (_tickSum > 45) {
		_tickSum = 0;
	}

	printf("_tickSum = %f\n", _tickSum);

	for (size_t i = 1; i < _division_contour.size(); i++) {
		if (_division_contour[i].limb_info.type < MagicPenContourLeg_Left || _division_contour[i].limb_info.type > MagicPenContourLeg_Right) {
			continue;
		}

		Tick(tick, _division_contour[i]);
	}
	
	PolyTriangulate(_division_contour);
	_3dModel.Init(_triangulate_result, _origin_contour, _division_contour,_image.cols, _image.rows, _texture_side_width, _texture_side_height);
	
#ifdef MagicPenMaLiang_DEBUG
	ShowDebugWindows_Triangulate();
#endif
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
		i2 = i1 + 1;

		count++;
		if(size == count) {
			i2 = startIndex;	
		}

		area += contour[i1].x * contour[i2].y - contour[i1].y * contour[i2].x;
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