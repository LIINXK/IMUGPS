#include <fstream>
#include <iostream>
#include <android/log.h>

#include "IMUGPS.h"
#include "Cluster.h"

double calAngel(const cv::Scalar a, const cv::Scalar b) {
    double cosin = a.dot(b)
                   / sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
                   / sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    return cosin > 1 ?0 : acos(cosin);
}

double calAngel(const cv::Scalar a, const cv::Vec3b b) {
    double cosin = (a[0]*b[0] + a[1]*b[1] + a[2]*b[2])
                   / sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
                   / sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    return cosin > 1 ?0 : acos(cosin);
}

void DOG(cv::Mat src, cv::Mat& dst, double sigma=0.5, double k=1.2){
    cv::Mat dog1, dog2;
    cv::GaussianBlur(src, dog1, cv::Size(5,5), k * sigma, k * sigma);
    cv::GaussianBlur(src, dog2, cv::Size(5,5), sigma, sigma);

    dst = dog1 - dog2;
}

class WrapPoint: public cv::Point2d {
public:
    double value;
    WrapPoint(double i, double j, double value): cv::Point2d(i, j), value(value){}
};
class cmp {
public:
    bool operator() (WrapPoint& a, WrapPoint& b){
        return a.value > b.value;
    }
};

#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, "MyCamera", __VA_ARGS__)

cv::Mat IMUGPS::run(cv::Mat frame, Quaternion pose) {
    init(frame, pose);

    L_0 = singeWB();
    if (firstFrame) {
        L_f = L_0;
        firstFrame = false;
    }
    else{
        L_s = GrayPointShift();

        theta_0 = calAngel(L_0, L_0_pre);
        theta_s = calAngel(L_s, L_f_pre);
        w = exp(- 40 * theta_0 * theta_s);
        L_ref = L_f_pre * w + L_0 * (1-w);

        L_f = correct();
    }
    adjustWhiteBalace();

    finish();
    return result;
}

cv::Scalar IMUGPS::singeWB() {
    cv::Mat temp1, temp2, temp3;
    for (int i=0; i<3; i++) {
        cv::blur(degammaFrame[i], blured[i], cv::Size(settings.blurRadius, settings.blurRadius));
    }

    lightSum = (blured[0] + blured[1] + blured[2]);

    cv::log(blured[0], log_B);
    cv::log(blured[2], log_R);
    cv::log(lightSum, logSum);

    C_R = log_R - logSum;
    C_B = log_B - logSum;

    DOG(C_R, temp1);
    DOG(C_B, temp2);

    cv::pow(temp1, 2, temp1);
    cv::pow(temp2, 2, temp2);
    temp3 = temp1 + temp2;
    cv::sqrt(temp3, GrayIndex);
    cv::blur(GrayIndex, GrayIndex, cv::Size(settings.blurRadius, settings.blurRadius));

    curGrayPoints = sortGrayPoint();

    double b=0, g=0, r=0;
    cv::String temp;
    for (cv::Point2f p : curGrayPoints){
        b += degammaFrame[0].at<double>(p);
        g += degammaFrame[1].at<double>(p);
        r += degammaFrame[2].at<double>(p);
    }

    double sum = b + g + r;
    L_0[0] = b / sum;
    L_0[1] = g / sum;
    L_0[2] = r / sum;
    return L_0;
}

cv::Scalar IMUGPS::GrayPointShift() {
    std::vector<uchar> status;
    std::vector<float> err;
    std::vector<cv::Point2f> preFeaturePoints, curFeaturePoints;
    std::vector<cv::Point2d> shiftValue;

    goodFeaturesToTrack(preGrayFrame, preFeaturePoints, 50, 0.3, 10);
    calcOpticalFlowPyrLK(preGrayFrame, curGrayFrame, preFeaturePoints, curFeaturePoints, status, err);

    // 计算旋转矩阵
    cv::Mat R_cur = curPose.quaternionToR();
    cv::Mat R_pre = prePose.quaternionToR();
    cv::Mat R = R_cur * R_pre.inv();
    cv::Mat R_camera = settings.innerMatrix * R.inv() * settings.innerMatrix.inv();

    // 计算特征点平移
    std::vector<int> counter;
    for (int i=0; i<preFeaturePoints.size(); i++){
        if (status[i] == 1){
            // 当前坐标
            cv::Point2d p = preFeaturePoints[i];
            cv::Mat location = (cv::Mat_<double>(3,1) << p.x, p.y, 1.0);

            // 乘上旋转矩阵
            cv::Mat rotateLocation = R_camera * location;

            // 归一化旋转后坐标，旋转后的成像平面坐标
            double x_rotate = rotateLocation.at<double>(0,0) / rotateLocation.at<double>(2,0);
            double y_rotate = rotateLocation.at<double>(1,0) / rotateLocation.at<double>(2,0);

            // 计算旋转、平移分量
            double u_trans = curFeaturePoints[i].x - x_rotate;
            double v_trans = curFeaturePoints[i].y - y_rotate;

            // 记录平移分量
            shiftValue.push_back(cv::Point2d(u_trans, v_trans));
            counter.push_back(i);
        }
    }

    if (shiftValue.size() != 0) {
        // 估计平移分量
        int belong[shiftValue.size()];
        int maxIndex = SpectralClustering(shiftValue, 5, belong);
        Point2d meanShift(0, 0);
        int n = 0;
        for (int i = 0; i < shiftValue.size(); i++) {
            if (belong[i] == maxIndex) {
                meanShift += shiftValue[i];
                n++;
            }
        }
        meanShift /= n;


        // 计算灰点漂移
        for (auto prePoint: preGrayPoints) {
            Mat location = (Mat_<double>(3, 1) << prePoint.x, prePoint.y, 1.0);

            Mat rotateLocation = R_camera * location;

            double x_rotate = rotateLocation.at<double>(0, 0) / rotateLocation.at<double>(2, 0);
            double y_rotate = rotateLocation.at<double>(1, 0) / rotateLocation.at<double>(2, 0);

            Point2d shiftPoint(x_rotate + meanShift.x, y_rotate + meanShift.y);
            if (0 <= shiftPoint.x && shiftPoint.x < curGrayFrame.cols && 0 <= shiftPoint.y && shiftPoint.y < curGrayFrame.rows) {
                shiftedGrayPoints.push_back(shiftPoint);
            }
        }
    }

    // 范围搜索灰点
    if(shiftedGrayPoints.size() <= preGrayPoints.size()/2){
        L_s = L_0;
    }else{
        std::priority_queue<WrapPoint, std::vector<WrapPoint>, cmp> minPoints;
        for (auto p : shiftedGrayPoints)
            for (int i=-settings.searchRadius; i<=settings.searchRadius; i++){
                for (int j=-settings.searchRadius; j<=settings.searchRadius; j++){
                    double newX = p.x+i, newY = p.y+j;
                    if (0<=newX && newX<GrayIndex.cols && 0 <=newY && newY<GrayIndex.rows)
                        minPoints.emplace(newX, newY, GrayIndex.at<double>(newY, newX));
                }
            }

        curGrayPoints.clear();
        int n = grayPointNumber < minPoints.size() ?grayPointNumber :minPoints.size();
        for (int i=0; i<n; i++){
            curGrayPoints.push_back(minPoints.top());
            minPoints.pop();
        }
        //计算当前帧的L_s
        double b=0, g=0, r=0;
        for (Point2d p : curGrayPoints){
            b += degammaFrame[0].at<double>((int)p.y, (int)p.x);
            g += degammaFrame[1].at<double>((int)p.y, (int)p.x);
            r += degammaFrame[2].at<double>((int)p.y, (int)p.x);
        }
        L_s[0] = b / curGrayPoints.size();
        L_s[1] = g / curGrayPoints.size();
        L_s[2] = r / curGrayPoints.size();
    }
    return L_s;
}

cv::Scalar IMUGPS::correct() {
    return L_ref;
}

cv::Mat IMUGPS::adjustWhiteBalace() {
    cv::Mat temp1, temp2;
    for (int i=0; i<3; i++){
        temp1 = degammaFrame[i] * (L_f[1] / L_f[i]);

        // gamma
        cv::pow(temp1, 1./2.2, temp2);
        temp1 = temp2 * 255;
        cv::threshold(temp1, temp2, 255, 0, cv::THRESH_TRUNC);
        temp2.convertTo(adjustChannels[i], CV_8UC1);
    }

    merge(adjustChannels, result);
    return result;
}

void IMUGPS::init(cv::Mat frame, Quaternion pose) {
    cv::Mat temp;
    cv::cvtColor(frame, curGrayFrame, cv::COLOR_BGR2GRAY);
    frame.convertTo(floatFrame, CV_64FC3);
    cv::split(floatFrame, floatFrameChannels);
    for (int i=0; i<3; i++) {
        // RGB图像归一化并degamma
        temp = floatFrameChannels[i] / 255;
        cv::pow(temp, 2.2, degammaFrame[i]);
    }
    curPose = pose;
    grayPointNumber = frame.cols * frame.rows * 1e-3;
}

void IMUGPS::finish() {
    preGrayPoints = curGrayPoints;

    preGrayFrame = curGrayFrame;
    prePose = curPose;

    L_0_pre = L_0;
    L_f_pre = L_f;
}

std::vector<cv::Point2f> IMUGPS::sortGrayPoint() {
    cv::Mat temp1, temp2;
    for (int i=0; i<3; i++) {
        DOG(blured[i], temp1);
        contrast[i] = cv::abs(temp1);
    }
    degammaSum = degammaFrame[0] + degammaFrame[1] + degammaFrame[2];

    std::priority_queue<WrapPoint, std::vector<WrapPoint>, cmp> minPoints;
    for (int i=0; i<GrayIndex.rows; i++) {
        for (int j = 0; j < GrayIndex.cols; j++) {
            if (
                    // 排除低对比度
                    contrast[0].at<double>(i, j) > settings.contrastThreshold && contrast[1].at<double>(i, j) > settings.contrastThreshold && contrast[2].at<double>(i, j) > settings.contrastThreshold
                    // 排除某一通道为0
                    && blured[0].at<double>(i, j) != 0.0 && blured[1].at<double>(i, j) != 0.0 && blured[2].at<double>(i, j) != 0.0
                    // 排除黑色像素
                    && degammaSum.at<double>(i, j) > low
                    // 排除饱和像素
                    && degammaFrame[0].at<double>(i, j) <= high && degammaFrame[1].at<double>(i, j) <= high && degammaFrame[2].at<double>(i, j) <= high
                    ) {
                minPoints.emplace(j, i, GrayIndex.at<double>(i, j));
            }
        }
    }

    curGrayPoints.clear();
    if (grayPointNumber > minPoints.size()){
        grayPointNumber = minPoints.size();
    }
    for (int i=0; i<grayPointNumber; i++) {
        curGrayPoints.push_back(minPoints.top());
        minPoints.pop();
    }

    return curGrayPoints;
}

void IMUGPS::clear() {
    firstFrame = true;
}
