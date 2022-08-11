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

private:

	std::vector<std::vector<cv::Point> > findContours(cv::Mat image, cv::Rect &validRect);
	
	// 连接临近的edge
	void ConnectAdjacentEdge(cv::Mat &detected_edges);

	cv::Rect expandRect(cv::Rect src, float expand_value, cv::Mat image);
#if 0
	// 寻找肢体(arms and legs)
	void FindLimbs(MagicPenContourHander &hander);

	void CalculationLimbInfoType(MagicPenContourHander &hander, cv::Rect boundRect);
#endif
private:
	cv::RotatedRect _rotatedRectROI;
	cv::Rect _ROI;
	MagicPen3DModel _3dModels;

    MagicPenRender _render;

    bool _init_image = false;

	float _tickSum = 0.0f;

    float _rotate_x = 0;
    float _rotate_y = 0;
};

#endif

