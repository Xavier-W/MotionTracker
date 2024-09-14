#pragma once
#ifndef TRACKER_H
#define TRACKER_H

struct Point2D {
    float x;
    float y;
    Point2D(float _x = 0, float _y = 0) : x(_x), y(_y) {}
};

class MotionTracker
{
public:
    MotionTracker(int stateNum = 4, int measureNum = 2);

    // ��ʼ�����������������һ������ֵ
    void init(const Point2D& firstMeasuredPoint);

    // ִ��һ���˲�����
    Point2D track(const Point2D& measuredPoint);

private:
    class Impl;
    Impl* pImpl;
};

#endif  // TRACKER_H
