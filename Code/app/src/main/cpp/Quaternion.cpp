#ifndef GPS_QUATERNION_H
#define GPS_QUATERNION_H

#include <opencv2/opencv.hpp>

class Quaternion {
public:
    double w, x, y, z;

    Quaternion(){};
    Quaternion(double w, double x, double y, double z):w(w), x(x), y(y), z(z) {
    }

    Quaternion add(Quaternion q){return Quaternion(w+q.w, x+q.x, y+q.y, z+q.z);}
    Quaternion sub(Quaternion q){return Quaternion(w-q.w, x-q.x, y-q.y, z-q.z);}
    Quaternion mul(Quaternion q){
        double new_w = w*q.w - x*q.x - y*q.y - z*q.z;
        double new_x = x*q.w + w*q.x - z*q.y + y*q.z;
        double new_y = y*q.w + z*q.x + w*q.y - x*q.z;
        double new_z = z*q.w - y*q.x + x*q.y + w*q.z;
        return Quaternion(new_w, new_x, new_y, new_z);
    }

    Quaternion conjugate(){
        return Quaternion(w, -x, -y, -z);
    }
    double norm(){
        return sqrt(w*w + x*x + y*y + z*z);
    }
    Quaternion inv(){
        Quaternion conjugate = this->conjugate();
        double norm = this->norm();
        double temp = norm * norm;
        return Quaternion(conjugate.w / temp, conjugate.x / temp, conjugate.y / temp, conjugate.z / temp);
    }
    static Quaternion eulerToQuaternion(double roll, double pitch, double hdg) {
        Quaternion q_ret(0,0,0,0);
        double cosRoll = cos(roll  * 0.5);
        double sinRoll = sin(roll  * 0.5);
        double cosPitch = cos(pitch * 0.5);
        double sinPitch = sin(pitch * 0.5);
        double cosHeading = cos(hdg   * 0.5);
        double sinHeading = sin(hdg   * 0.5);
        q_ret.w = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
        q_ret.x = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
        q_ret.y = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
        q_ret.z = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
        return q_ret;
    }
    static Quaternion slerp(Quaternion q1, Quaternion q2, double t) {
        Quaternion result = Quaternion(0,0,0,0);
        Quaternion start = q1;
        Quaternion end = q2;
        double cosa = start.w * end.w + start.x * end.x + start.y * end.y + start.z * end.z;
        double k0, k1;
        if(cosa < 0.0){
            end.w = -end.w;
            end.x = -end.x;
            end.y = -end.y;
            end.z = -end.z;
            cosa = -cosa;
        }
        if(cosa > 0.9995){
            k0 = 1.0 - t;
            k1 = t;
        } else {
            double sina = sqrt(1.0 - cosa * cosa);
            double a = atan2(sina, cosa);
            k0 = sin((1.0 - t) * a) / sina;
            k1 = sin(t*a) / sina;
        }
        result.w = start.w * k0 + end.w * k1;
        result.x = start.x * k0 + end.x * k1;
        result.y = start.y * k0 + end.y * k1;
        result.z = start.z * k0 + end.z * k1;
        return result;
    }
    cv::Mat quaternionToR() {
        cv::Mat mat = cv::Mat(3, 3, CV_64F);
        double w = this->w;
        double x = this->x;
        double y = this->y;
        double z = this->z;
        mat.at<double>(0, 0) = 1 - 2 * y * y - 2 * z * z;
        mat.at<double>(0, 1) = 2 * x * y - 2 * z * w;
        mat.at<double>(0, 2) = 2 * x * z + 2 * y * w;
        mat.at<double>(1, 0) = 2 * x * y + 2 * z * w;
        mat.at<double>(1, 1) = 1 - 2 * x * x - 2 * z * z;
        mat.at<double>(1, 2) = 2 * y * z - 2 * x * w;
        mat.at<double>(2, 0) = 2 * x * z - 2 * y * w;
        mat.at<double>(2, 1) = 2 * y * z + 2 * x * w;
        mat.at<double>(2, 2) = 1 - 2 * x * x - 2 * y * y;
        return mat;
    }
};

#endif //GPS_QUATERNION_H