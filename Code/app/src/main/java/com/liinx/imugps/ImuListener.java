package com.liinx.imugps;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;



public class ImuListener implements SensorEventListener {

    private static final ImuListener listener = new ImuListener();
    private ImuListener(){
        System.loadLibrary("imugps");
    }
    public static ImuListener getInstance(){return listener;}

    private long preAcceTimestamp = -1;
    private long preGyroTimestamp = -1;
    private float[] acceValues;
    private float[] gyroValues;


    public float[] interpolate(long leftTS, float[] v1, long rightTS, float[] v2, long targetTS){
        double ratio = (targetTS - leftTS) * 1.0 / (rightTS - leftTS);
        double ratio_ = 1 - ratio;
        float[] result = new float[3];
        for (int i = 0; i < 3; i++)
            result[i] = (float) (v1[i] * ratio_ + v2[i] * ratio);
        return result;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION){
            if (preAcceTimestamp != -1 && preGyroTimestamp != -1 && event.timestamp > preGyroTimestamp){
                float[] interAcce = interpolate(preAcceTimestamp, acceValues, event.timestamp, event.values, preGyroTimestamp);
                // IMU坐标旋转至相机坐标，详见 https://source.android.google.cn/docs/core/interaction/sensors/sensor-types?hl=zh-cn#phone_axes
                updateIMU(-gyroValues[1], -gyroValues[0], -gyroValues[2], -interAcce[1], -interAcce[0], -interAcce[2]);
            }
            preAcceTimestamp = event.timestamp;
            acceValues = event.values;
        }
        else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE_UNCALIBRATED){
            preGyroTimestamp = event.timestamp;
            gyroValues = event.values;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

    private native void updateIMU(float g0, float g1, float g2, float a0, float a1, float a2);
}
