#include "Cluster.h"

#include <vector>
#include <opencv2/opencv.hpp>
#include <random>

using namespace std;
using namespace cv;

float similarCal(Point2d a, Point2d b){
    double x = a.x - b.x, y = a.y - b.y;
    return exp(-sqrt(x*x + y*y) / 2);
}

float calDis(Mat a, Mat b){
    float  res = 0;
    for (int i=0; i<a.rows; i++){
        float temp = a.at<float>(i, 0)-b.at<float>(i, 0);
        res += temp*temp;
    }
    return sqrt(res);
}

int MyKmeans(vector<Mat> atoms, int nCluster, int belonging[], int terminalTime=20){
    int size = atoms.size();
    uniform_int_distribution<int> u(0, size-1);
    default_random_engine e;

    vector<Mat> center;
    for (int i=0; i<nCluster; i++)
        center.push_back(atoms.at(u(e)));

    float dis[size][nCluster];
    for (int i=0; i<size; i++) {
        belonging[i] = -1;
        for (int j =0; j<nCluster; j++)
            dis[i][j] = -1;
    }

    int maxIndex = 0;
    while(terminalTime--){
        vector<vector<Mat>> clasters;
        for (int i=0; i<nCluster; i++){
            vector<Mat> w;
            clasters.push_back(w);
        }

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < nCluster; j++)
                dis[i][j] = calDis(atoms.at(i), center.at(j));
        }

        for (int i = 0; i < size; i++) {
            int index = 0;
            for (int j = 1; j < nCluster; j++) {
                if (dis[i][index] > dis[i][j])
                    index = j;
            }
            clasters.at(index).push_back(atoms.at(i));
            belonging[i] = index;
        }

        for (int i=0; i<nCluster; i++){
            Mat temp = Mat::zeros(nCluster, 1, CV_32F);
            for (Mat x : clasters.at(i)){
                temp = temp + x;
            }
            center.at(i) = temp / clasters.at(i).size();

            if (clasters.at(i).size() > clasters.at(maxIndex).size()){
                maxIndex = i;
            }
        }
    }
    return maxIndex;
}

int SpectralClustering(vector<Point2d> atoms, int number, int belong[]){
    int size = atoms.size();
    number = size>number ?number :size;

    Mat W = Mat::zeros(size, size, CV_32F);
    for (int i=0; i<size; i++)
        for (int j=0; j<i; j++){
            float v = similarCal(atoms.at(i), atoms.at(j));
            W.at<float>(i, j) = v;
            W.at<float>(j, i) = v;
        }

    Mat D = Mat::zeros(size, size, CV_32F);
    Mat D_half = Mat::zeros(size, size, CV_32F);
    for (int i = 0; i < size; i++) {
        float val = 0;
        for (int j = 0; j < size; j++) {
            val += W.at<float>(i, j);
        }
        D.at<float>(i, i) = val;
        val = 1 / sqrt(val);
        D_half.at<float>(i, i) = val;
    }

    Mat L = D - W;

    Mat L_std = D_half * L * D_half;
    Mat eigenValues, eigenVectors;
    eigen(L_std, eigenValues, eigenVectors);

    Mat F = eigenVectors(Range(size-number, size), Range::all());

    for (int i=0; i<F.rows; i++){
        Mat f = F.row(i);
        F.row(i) = f / norm(f);
    }
    vector<Mat> as;
    for (int i=0; i<size; i++)
        as.push_back(F.col(i));
    return MyKmeans(as, number, belong);
}
