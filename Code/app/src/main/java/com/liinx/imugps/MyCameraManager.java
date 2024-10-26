package com.liinx.imugps;

import android.annotation.SuppressLint;
import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.TotalCaptureResult;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.util.Size;
import android.view.Surface;
import android.view.View;

import androidx.annotation.NonNull;

import com.liinx.imugps.utils.YUVUtils;

import java.util.Arrays;

public class MyCameraManager implements View.OnClickListener {

    // 相机相关
    private CameraManager cameraManager;
    private String cameraID;
    private int imageFormat = ImageFormat.YUV_420_888;
    private int height, width;
    private CameraDevice openCameraDevice;
    private CameraCaptureSession openSession;
    private SurfaceTexture openSurfaceTexture;
    private ImageReader imageReader;

    private String TAG = "MyCamera";
    private Handler handler;
    private boolean reWB = false;

    @Override
    public void onClick(View v) {
        reWB = !reWB;
    }

    MyCameraManager(CameraManager cameraManager) {
        this.cameraManager = cameraManager;
    }

    public void init() {
        try {
            // 创建子线程
            HandlerThread handlerThread = new HandlerThread("Camera2Manager");
            handlerThread.start();
            handler = new Handler(handlerThread.getLooper());
            
            // 相机ID默认使用第一个，一般为后摄
            cameraID = cameraManager.getCameraIdList()[0];

            // 选取一个输出规格
            Size[] sizes = cameraManager.getCameraCharacteristics(cameraID).get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP).getOutputSizes(imageFormat);
//            for (int i = 0; i < sizes.length; i++) {
//                Log.d(TAG, i + ":" + sizes[i].getWidth() + 'x' + sizes[i].getHeight());
//            }
            int n = 26;
            height = sizes[n].getHeight();
            width = sizes[n].getWidth();
            Log.d(TAG, "Image: " + width + " x " + height);

            createImageReader();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void createImageReader(){
        imageReader = ImageReader.newInstance(width, height, imageFormat, 2);
        imageReader.setOnImageAvailableListener(reader -> {
            Image image = reader.acquireLatestImage();
            if (image != null) {
//                Log.d(TAG, "new image");
                byte[] nv21Image = YUVUtils.yuv2nv21(image);
                reWBandUpdate(height, width, nv21Image, reWB);

                image.close();
            }
        }, handler);
    }

    @SuppressLint("MissingPermission")
    public void openCameraAndConnect(SurfaceTexture surfaceTexture) {
        openSurfaceTexture = surfaceTexture;
        openSurfaceTexture.setDefaultBufferSize(width, height);
        Surface surface = new Surface(surfaceTexture);
        setSurface(surface);
        Log.d(TAG, "SurfaceTexture setDefault.");

        try {
            // 连接相机
            cameraManager.openCamera(cameraID, new CameraDevice.StateCallback() {
                @Override
                public void onOpened(@NonNull CameraDevice cameraDevice) {
                    Log.d(TAG, "Camera opened.");
                    openCameraDevice = cameraDevice;
                    createSession(imageReader.getSurface());
                }

                @Override
                public void onDisconnected(@NonNull CameraDevice cameraDevice) {

                }

                @Override
                public void onError(@NonNull CameraDevice cameraDevice, int i) {

                }
            }, handler);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * 创建与相机的session，并指定图像输出的surface
     *
     * @param surface 图像输出的surface
     */
    private void createSession(Surface surface){
        try {
            openCameraDevice.createCaptureSession(Arrays.asList(
                    // 输出的surface
                    surface
//                    imageReader.getSurface()
            ), new CameraCaptureSession.StateCallback() {
                @Override
                public void onConfigured(@NonNull CameraCaptureSession cameraCaptureSession) {
                    Log.d(TAG, "Session opened.");
                    openSession = cameraCaptureSession;

                    try {
                        // 创建Request并初始化
                        CaptureRequest.Builder builder = openCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                        builder.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);
                        builder.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON_AUTO_FLASH);

                        // 手动调整部分
                        builder.addTarget(surface);
//                        builder.addTarget(imageReader.getSurface());
                        builder.set(CaptureRequest.CONTROL_AWB_MODE, CaptureRequest.CONTROL_AWB_MODE_OFF);
                        builder.set(CaptureRequest.COLOR_CORRECTION_MODE, CaptureRequest.COLOR_CORRECTION_MODE_FAST);

                        Log.d(TAG, "Request built.");
                        openSession.setRepeatingRequest(builder.build(), new CameraCaptureSession.CaptureCallback(){
                            @Override
                            public void onCaptureCompleted(@NonNull CameraCaptureSession session, @NonNull CaptureRequest request, @NonNull TotalCaptureResult result) {
                                super.onCaptureCompleted(session, request, result);
//                                RggbChannelVector rggb = result.get(CaptureResult.COLOR_CORRECTION_GAINS);
//                                assert rggb != null;
//                                Log.d(TAG, "AWB:"+ rggb.getRed() + ',' + (rggb.getGreenEven()+rggb.getGreenOdd())/2 + ',' + rggb.getBlue());

                            }
                        }, handler);

                    } catch (CameraAccessException e) {
                        e.printStackTrace();
                    }
                }

                @Override
                public void onConfigureFailed(@NonNull CameraCaptureSession cameraCaptureSession) {}
            }, handler);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    private native void reWBandUpdate(int height, int width, byte[] image, boolean reWB);
    private native void setSurface(Surface surface);
}
