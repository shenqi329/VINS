//
// Created by DELL on 2022/8/4.
//

#include "MagicPen3DModel.h"
// MagicPenModel3D begin

void MagicPen3DLimbModel::PolyTriangulate() {

    TPPLPartition pp;

	TPPLPolyList inpolys;

	_triangulate_result.clear();

	for (size_t i = 0; i < _division_contour.size(); i++) {
		MagicPenContour contour = _division_contour[i];
		
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


void MagicPen3DLimbModel::InitFromContours(std::vector<cv::Point> contour, float offset_x,float offset_y, int cols, int rows, int texture_side_width, int texture_side_height,
              cv::Mat image_rgba)
{
	_origin_contour.contour_points.resize(contour.size());
	for (size_t i = 0; i < contour.size(); i++){
		_origin_contour.contour_points[i].point.x = contour[i].x + offset_x;
		_origin_contour.contour_points[i].point.y = contour[i].y + offset_y;
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

	// 三角形填充多边形
	PolyTriangulate();

	Init(_triangulate_result, _origin_contour, _division_contour, cols, rows, texture_side_width, texture_side_height, image_rgba);
}

void MagicPen3DLimbModel::Init(std::list<TPPLPoly> triangles, MagicPenContour origin_contour, std::vector<MagicPenContour> division_contour,
                           int cols, int rows,  int texture_side_width, int texture_side_height,
                           cv::Mat image_rgba
                           ) {

    _image_rgba = image_rgba;

    FreeVertice();

    InitVerticesFront(triangles, origin_contour, cols, rows);

    InitVerticesEdge(origin_contour, division_contour, cols, rows, texture_side_width, texture_side_height);
}

void MagicPen3DLimbModel::InitVerticesFront(std::list<TPPLPoly> triangles, MagicPenContour origin_contour, int cols, int rows) {

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
            _vertices_front[offset].positions[0] = (iter->GetPoints()[i].x - cols/2) * 2 / cols;
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

void MagicPen3DLimbModel::InitVerticesEdge(MagicPenContour origin_contour, std::vector<MagicPenContour> division_contour, int cols, int rows, int texture_side_width, int texture_side_height) {

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

MagicPen3DLimbModel::~MagicPen3DLimbModel() {

    FreeVertice();

}

void MagicPen3DLimbModel::FreeVertice() {

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

void MagicPen3DModel::ClearLimbModel() {
    for (int i = 0; i < _3dModels.size(); ++i) {
        delete _3dModels[i];
    }
    _3dModels.clear();
}

void MagicPen3DModel::InitFromContours(std::vector< std::vector<cv::Point> > contours, 
	float offset_x,float offset_y, int cols, int rows, int texture_side_width, int texture_side_height, cv::Mat image_rgba) {

    ClearLimbModel();

	for (size_t contour_index = 0; contour_index < contours.size(); contour_index++) {
		MagicPen3DLimbModel *p3DModel = new MagicPen3DLimbModel();
		p3DModel->InitFromContours(contours[contour_index], offset_x, offset_y, cols, rows, texture_side_width,  texture_side_height, image_rgba);
		_3dModels.push_back(p3DModel);
	}
}