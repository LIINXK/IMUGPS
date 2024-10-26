#include <cmath>
#include "Quaternion.cpp"

class MadgwickAHRS{
public:
    // sample frequency in Hz
    const float sampleFreqDef = 200.0f;
    // 2 * proportional gain
    const float betaDef = 0.01f;

    // algorithm gain
    float beta;
    float q0;
    float q1;
    float q2;
    // quaternion of sensor frame relative to auxiliary frame
    float q3;
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    bool anglesComputed;

    void computeAngles(){
        roll = (float) atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
        pitch = (float) asinf(-2.0f * (q1*q3 - q0*q2));
        yaw = (float) atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);

        anglesComputed = true;
    }

    MadgwickAHRS(){
        //0.70710678f
        beta = betaDef;
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        invSampleFreq = 1.0f / sampleFreqDef;
        anglesComputed = false;
    }

    void begin(float sampleFrequency) {
        invSampleFreq = 1.0f / sampleFrequency;
    }

    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az){
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // Convert gyroscope degrees/sec to radians/sec
        //gx *= 0.0174533f;
        //gy *= 0.0174533f;
        //gz *= 0.0174533f;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            // Normalise accelerometer measurement
            recipNorm = (float) (1.0/sqrt(ax * ax + ay * ay + az * az));
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _4q0 = 4.0f * q0;
            _4q1 = 4.0f * q1;
            _4q2 = 4.0f * q2;
            _8q1 = 8.0f * q1;
            _8q2 = 8.0f * q2;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;

            // Gradient decent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
            recipNorm = (float) (1.0/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * invSampleFreq;
        q1 += qDot2 * invSampleFreq;
        q2 += qDot3 * invSampleFreq;
        q3 += qDot4 * invSampleFreq;

        // Normalise quaternion
        recipNorm = (float) (1.0/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3));
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        anglesComputed = false;
    }

    float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }

    float getq0() {
        return q0;
    }
    float getq1() {
        return q1;
    }
    float getq2() {
        return q2;
    }
    float getq3() {
        return q3;
    }

    Quaternion getPose(){
        return Quaternion(q0, -q1, q2, -q3);
    }
};