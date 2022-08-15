//
// Created by DELL on 2022/8/15.
//

#include "MagicPenUtil.h"


float MagicPenUtil::GetDistance(cv::Point pointO, cv::Point pointA) {

    float distance;

    distance = powf((pointO.x - pointA.x),2) + powf((pointO.y - pointA.y),2);
    distance = sqrtf(distance);

    return distance;
}