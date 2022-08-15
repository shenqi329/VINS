//
// Created by DELL on 2022/8/15.
//

#ifndef VINS_MAGICPENUTIL_H
#define VINS_MAGICPENUTIL_H

#include <opencv2/core.hpp>

class MagicPenUtil {
public:
    static float GetDistance(cv::Point pointO, cv::Point pointA);
};


#endif //VINS_MAGICPENUTIL_H
