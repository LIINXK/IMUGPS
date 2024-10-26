#ifndef GPS_IMUGPS_H
#define GPS_IMUGPS_H

#include<opencv2/opencv.hpp>
#include <utility>
#include "Quaternion.cpp"

class IMUGPS {
public:
    class Settings{
        public:
            Settings(double contrastThreshold, int blurRadius, int searchRadius, cv::Mat innerMatrix): contrastThreshold(contrastThreshold), blurRadius(blurRadius), searchRadius(searchRadius), innerMatrix(innerMatrix){}
            const double contrastThreshold;
            const int blurRadius;
            const int searchRadius;
            const cv::Mat innerMatrix;
    };

    IMUGPS(Settings st): settings(st){}

    cv::Mat run(cv::Mat frame, Quaternion  pose);

    void clear();

private:

    cv::Scalar singeWB();
    cv::Scalar GrayPointShift();
    cv::Scalar correct();
    cv::Mat adjustWhiteBalace();

    std::vector<cv::Point2f> sortGrayPoint();

    void init(cv::Mat frame, Quaternion pose);
    void finish();

    /* 算法 */
    Settings settings;
    int grayPointNumber;
    double low = 0.0315, high = 0.95;
    bool firstFrame = true;

    cv::Mat blured[3] = {cv::Mat(), cv::Mat(), cv::Mat()};
    cv::Mat lightSum, logSum, log_R, log_B, degammaSum;
    cv::Mat C_R, C_B;
    cv::Mat GrayIndex;
    cv::Mat contrast[3];

    std::vector<cv::Point2f> curGrayPoints;
    std::vector<cv::Point2f> preGrayPoints;
    std::vector<cv::Point2f> shiftedGrayPoints;

    cv::Scalar L_0;
    cv::Scalar L_0_pre;
    cv::Scalar L_s;
    cv::Scalar L_ref;
    cv::Scalar L_f;
    cv::Scalar L_f_pre;
    double theta_s, theta_0, w;

    /* 图像部分 */
    cv::Mat curRGBFrame;
    std::vector<cv::Mat> curRGBChannels;
    cv::Mat curGrayFrame;
    cv::Mat preRGBFrame;
    cv::Mat preGrayFrame;

    std::vector<cv::Mat> adjustChannels = {cv::Mat(), cv::Mat(), cv::Mat()};
    cv::Mat result;

    /* IMU部分 */
    Quaternion curPose;
    Quaternion prePose;

    /* degamma */
    cv::Mat floatFrame;
    cv::Mat floatFrameChannels[3];
    std::vector<cv::Mat> degammaFrame = {cv::Mat(), cv::Mat(), cv::Mat()};
};


#endif //GPS_IMUGPS_H
