#ifndef CAMERAOPENCV_CLUSTER_H
#define CAMERAOPENCV_CLUSTER_H

#include <opencv2/opencv.hpp>
using namespace cv;

float similarCal(Point2d a, Point2d b);
float calDis(Mat a, Mat b);
int MyKmeans(std::vector<Mat> atoms, int nCluster, int belonging[], int terminalTime);
int SpectralClustering(std::vector<Point2d> atoms, int number, int belong[]);


#endif //CAMERAOPENCV_CLUSTER_H
