#include <jni.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <android/native_window_jni.h>

#include "MadgwickAHRS.cpp"
#include "IMUGPS.h"

/* IMU姿态估计 */
MadgwickAHRS m;
extern "C"
JNIEXPORT void JNICALL
Java_com_liinx_imugps_ImuListener_updateIMU(JNIEnv *env, jobject thiz, jfloat g0, jfloat g1,
                                            jfloat g2, jfloat a0, jfloat a1, jfloat a2) {
    m.updateIMU(g0, g1, g2, a0, a1, a2);
}

ANativeWindow *window = 0;
ANativeWindow_Buffer buffer;
uint8_t *dstData;
extern "C"
JNIEXPORT void JNICALL
Java_com_liinx_imugps_MyCameraManager_setSurface(JNIEnv *env, jobject thiz, jobject surface) {
    if (window) {
        ANativeWindow_release(window);
        window = 0;
    }
    window = ANativeWindow_fromSurface(env, surface);
}

void updateSurface(cv::Mat rgba){
    if (window) {
        ANativeWindow_setBuffersGeometry(window, rgba.cols,rgba.rows, WINDOW_FORMAT_RGBA_8888);

        if (ANativeWindow_lock(window, &buffer, 0)) {
            ANativeWindow_release(window);
            window = 0;
        }
        dstData  =   static_cast<uint8_t *>(buffer.bits);

        int srclineSize = rgba.cols*4 ;
        int dstlineSize = buffer.stride *4;
        for (int i = 0; i < buffer.height; ++i) {
            memcpy(dstData+i*dstlineSize, rgba.data+i*srclineSize, srclineSize);
        }
        ANativeWindow_unlockAndPost(window);
    }
}

IMUGPS imugps(IMUGPS::Settings(
        1e-4,
        7,
        7,
        (cv::Mat_<double>(3,3)<<
        668.386577725047,   0,                  481.035022498242,
        0,                  667.642510754716,   362.672901347417,
        0,                  0,                  1)
));

extern "C"
JNIEXPORT void JNICALL
Java_com_liinx_imugps_MyCameraManager_reWBandUpdate(JNIEnv *env, jobject thiz, jint height,
                                                    jint width, jbyteArray data, jboolean re_wb) {
    jbyte* imageData = env->GetByteArrayElements(data, NULL);
    cv::Mat src(height+height/2, width, CV_8U, imageData), res, rgba;
    cv::cvtColor(src, src, cv::COLOR_YUV2BGR_NV21);

    cv::rotate(src, src, cv::ROTATE_90_CLOCKWISE);

    /* 核心 */
    if (re_wb)
        res = imugps.run(src, m.getPose());
    else {
        imugps.clear();
        res = src;
    }

    cv::cvtColor(res, rgba, cv::COLOR_BGR2RGBA);
    updateSurface(rgba);
}