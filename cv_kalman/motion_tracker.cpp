#include "motion_tracker.h"
#include "opencv2/video/tracking.hpp"

class MotionTracker::Impl {
public:
    cv::KalmanFilter KF;
    cv::Mat measurement;
    bool isInitialized;

    Impl(int stateNum, int measureNum) : isInitialized(false) {
        KF.init(stateNum, measureNum, 0);
        KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
        cv::setIdentity(KF.measurementMatrix);
        cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
        cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
        measurement = cv::Mat::zeros(measureNum, 1, CV_32F);
        cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));
    }
};

MotionTracker::MotionTracker(int stateNum, int measureNum)
    : pImpl(new Impl(stateNum, measureNum)) {}

void MotionTracker::init(const Point2D& firstMeasuredPoint) {
    pImpl->KF.statePost.at<float>(0) = firstMeasuredPoint.x;
    pImpl->KF.statePost.at<float>(1) = firstMeasuredPoint.y;
    pImpl->KF.statePost.at<float>(2) = 0;
    pImpl->KF.statePost.at<float>(3) = 0;
    pImpl->isInitialized = true;
}

Point2D MotionTracker::track(const Point2D& measuredPoint) {
    if (!pImpl->isInitialized) {
        throw std::runtime_error("MotionTracker is not initialized.");
    }

    cv::Mat prediction = pImpl->KF.predict();
    Point2D predict_pt(prediction.at<float>(0), prediction.at<float>(1));

    pImpl->measurement.at<float>(0) = measuredPoint.x;
    pImpl->measurement.at<float>(1) = measuredPoint.y;

    pImpl->KF.correct(pImpl->measurement);

    return predict_pt;
}
