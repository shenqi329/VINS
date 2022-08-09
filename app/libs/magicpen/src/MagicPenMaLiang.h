#ifndef MAGIC_PEN_MALIANG_H
#define MAGIC_PEN_MALIANG_H

#include <opencv2/imgproc.hpp>
#include "MagicPen3DModel.h"

#ifdef _WIN32
#define MagicPenMaLiang_DEBUG
#endif

#include "MagicPenRender.h"

class MagicPenMaLiang
{
public:

    void Init();
	
	bool Magic(cv::Mat image, int texture_side_width, int texture_side_height);

    void setRotate(float x, float y);

    void setEdgeImageByte(std::vector<uchar> data);

    void Draw(double timeStampSec);

	MagicPen3DModel *Get3DModel();
private:

	void Tick(float tick, MagicPenContour &contour);
	
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
	cv::Mat _pre_image_gray;
	cv::Rect _ROI;
	std::vector<cv::Point2f>  _pre_image_corners;

	glm::mat4 _transformM = glm::mat4(1.0f);
	
	int _texture_side_width;
	
	int _texture_side_height;

	// 最原始的轮廓
	MagicPenContour _origin_contour;
	std::vector<MagicPenLimbInfo> _limbInfo;
	
	// 分割轮廓（躯干、四肢）
	std::vector<MagicPenContour> _division_contour;
	std::list<TPPLPoly> _triangulate_result;
	
	MagicPen3DModel _3dModel;
    MagicPenRender _render;

    bool _init_image = false;

	float _tickSum = 0.0f;

    float _rotate_x = 0;
    float _rotate_y = 0;

    cv::RotatedRect _beginMinAreaRect;
    cv::RotatedRect _curMinAreaRect;
	
	static const int _threshold = 100;
	static const int _ratio = 3;
	static const int _kernel_size = 3;
};

#endif

