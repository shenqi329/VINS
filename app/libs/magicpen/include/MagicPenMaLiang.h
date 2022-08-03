#ifndef MAGIC_PEN_MALIANG_H
#define MAGIC_PEN_MALIANG_H

#include <opencv2/imgproc.hpp>
#include "polypartition.h"

#ifdef _WIN32
#define MagicPenMaLiang_DEBUG
#endif

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
class MagicPen3DModel {
public:
	void Init(std::list<TPPLPoly> triangles, MagicPenContour origin_contour, std::vector<MagicPenContour> division_contour, int cols, int rows, int texture_side_width, int texture_side_height);

	~MagicPen3DModel();
private:
	void InitVerticesFront(std::list<TPPLPoly> triangles, MagicPenContour origin_contour, int cols, int rows);

	void InitVerticesEdge(MagicPenContour origin_contour, std::vector<MagicPenContour> division_contour, int cols, int rows, int texture_side_width, int texture_height);

	void FreeVertice();

public:
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


class MagicPenMaLiang
{
public:
	
	bool Magic(cv::Mat image, int texture_side_width, int texture_side_height);

	void Tick(float tick);

	void Tick(float tick, MagicPenContour &contour);

	MagicPen3DModel *Get3DModel();
private:
	
	// 连接临近的edge
	void ConnectAdjacentEdge(cv::Mat &detected_edges);

	// 计算填充满多边形的三角形
	void PolyTriangulate(std::vector<MagicPenContour> &contours);

	// 寻找肢体(arms and legs)
	void FindLimbs(std::vector<cv::Point> &contour);

	void CalculationLimbInfoType(MagicPenLimbInfo &limbInfo, cv::Rect boundRect);

#ifdef MagicPenMaLiang_DEBUG
	void ShowDebugWindows(cv::Mat detected_edges, cv::Mat &markers);
	void ShowDebugWindows_Points();
	void ShowDebugWindows_Triangulate();
#endif
private:
	cv::Mat _image;
	
	int _texture_side_width;
	
	int _texture_side_height;

	// 最原始的轮廓
	MagicPenContour _origin_contour;
	std::vector<MagicPenLimbInfo> _limbInfo;
	
	// 分割轮廓（躯干、四肢）
	std::vector<MagicPenContour> _division_contour;
	std::list<TPPLPoly> _triangulate_result;
	
	MagicPen3DModel _3dModel;

	float _tickSum = 0.0f;
	
	static const int threshold = 100;
	
	static const int ratio = 3;
	
	static const int kernel_size = 3;
};

#endif

