//
// Created by DELL on 2022/8/4.
//

#ifndef VINS_MAGICPEN3DMODEL_H
#define VINS_MAGICPEN3DMODEL_H

#include <opencv2/imgproc.hpp>
#include "polypartition.h"

// 顶点
struct MagicPenVertice {
    float positions[3]; // x,y,z 坐标
    float colors[3];	// 颜色
    float textures[2];  // 纹理坐标
};

struct MagicPenPoint {
    cv::Point2f point;
    union
    {
        size_t		index;
        short       indexs[2];
    };
};

enum MagicPenContourType {
    MagicPenContourBody = 0,	// 躯干
    MagicPenContourArm_Left,	// 左手
    MagicPenContourArm_Right,	// 右手
    MagicPenContourLeg_Left,	// 左脚
    MagicPenContourLeg_Right,	// 右脚
};


struct MagicPenLimbInfo {
    float  minDistance;			// 距离
    size_t start_point_index;	// 肢体开始点的 index (相对于 contour 数组)
    size_t end_point_offset;	// 肢体结束点相对于 start_point_index 的偏移
    size_t max_point_offset;	// 距离起始点最远的点相对于 start_point_index 的偏移

    MagicPenContourType type = MagicPenContourBody;
};

struct MagicPenContour {
    std::vector<MagicPenPoint> contour_points;

    MagicPenLimbInfo limb_info;
};

// 3D模型
class MagicPen3DLimbModel {

public:
	void InitFromContours(std::vector<cv::Point> contour, float offset_x,float offset_y, int cols, int rows, int texture_side_width, int texture_side_height,
              cv::Mat image_rgba);
	 
	~MagicPen3DLimbModel();

private:
    void Init(std::list<TPPLPoly> triangles, MagicPenContour origin_contour, std::vector<MagicPenContour> division_contour,
              int cols, int rows, int texture_side_width, int texture_side_height,
              cv::Mat image_rgba
              );
   
	void PolyTriangulate();

    void InitVerticesFront(std::list<TPPLPoly> triangles, MagicPenContour origin_contour, int cols, int rows);

    void InitVerticesEdge(MagicPenContour origin_contour, std::vector<MagicPenContour> division_contour, int cols, int rows, int texture_side_width, int texture_height);

    void FreeVertice();

public:
	// 最原始的轮廓
	MagicPenContour _origin_contour;
	std::vector<MagicPenLimbInfo> _limbInfo;
	
	// 分割轮廓（躯干、四肢）
	std::vector<MagicPenContour> _division_contour;
	std::list<TPPLPoly> _triangulate_result;

public:
    cv::Mat _image_rgba;

    // 正面信息
    MagicPenVertice *_vertices_front = nullptr;
    int _vertices_front_size = 0;

    int _indices_front_size = 0;
    int* _indices_front = nullptr;

    // 侧面信息
    MagicPenVertice *_vertices_side = nullptr;
    int _vertices_side_size = 0;

    int _indices_side_size = 0;
    int* _indices_side = nullptr;
};

class MagicPen3DModel {
public:
	std::vector<MagicPen3DLimbModel*> _3dModels;

	std::vector<std::vector<cv::Point> > _contours;

    float _offset_x = 0;
    float _offset_y = 0;
    float _scale = 0;
public:
	void InitFromContours(std::vector< std::vector<cv::Point> > contours, 
		float offset_x,float offset_y, int cols, int rows, int texture_side_width, int texture_side_height, cv::Mat image_rgba);
private:
    void ClearLimbModel();
};


#endif //VINS_MAGICPEN3DMODEL_H
