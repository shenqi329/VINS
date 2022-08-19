#ifndef MAGIC_PEN_MALIANG_H
#define MAGIC_PEN_MALIANG_H

#include <queue>
#include <opencv2/imgproc.hpp>
#include "MagicPen3DModel.h"
#include "MagicPenFeatureTrack.h"

#ifdef _WIN32
#define MagicPenMaLiang_DEBUG
#endif

#include "MagicPenRender.h"

struct OffsetInfo {
	float offset_x = 0;
	float offset_y = 0;
    float scale;
};

class OffsetCache {
public:
	void Reset();
	void AddInfo(OffsetInfo info);
	OffsetInfo GetValidOffsetInfo();
private:
	OffsetInfo _cache[3];
	OffsetInfo _validInfo;
    size_t  ignore_count = 0;
};

class MagicPenMaLiang
{
public:

    void Init();
	
	bool Magic(cv::Mat image, int texture_side_width, int texture_side_height);

    void setRotate(float x, float y);

    void setEdgeImageByte(std::vector<uchar> data);

    void Draw(double timeStampSec);

	std::vector<cv::Point2f> GetTrackRectPoints();

private:

	std::vector<std::vector<cv::Point> > findContours(cv::Mat image, cv::Mat image_gray, cv::Rect &validRect);
	
	// 连接临近的edge
	void ConnectAdjacentEdge(cv::Mat &detected_edges);

	cv::Rect expandRect(cv::Rect src, float expand_value, cv::Mat image);
#if 0
	// 寻找肢体(arms and legs)
	void FindLimbs(MagicPenContourHander &hander);

	void CalculationLimbInfoType(MagicPenContourHander &hander, cv::Rect boundRect);
#endif
private:
	MagicPen3DModel _3dModels;
    MagicPenRender _render;

    bool _init_image = false;

    float _rotate_x = 0;
    float _rotate_y = 0;

	MagicPenFeatureTrack _feature_track;
};

#endif

