package com.liinx.imugps;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.SurfaceTexture;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraManager;
import android.os.Bundle;
import android.util.Log;
import android.view.TextureView;
import android.widget.Button;

import com.liinx.imugps.databinding.ActivityMainBinding;

public class MainActivity extends AppCompatActivity {

    // Used to load the 'imugps' library on application startup.
    static {
        System.loadLibrary("imugps");
    }

    private ActivityMainBinding binding;
    private String TAG = "MyCamera";
    private MyCameraManager cm;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        // 初始化
        cm = new MyCameraManager((CameraManager) getSystemService(Context.CAMERA_SERVICE));
        cm.init();
        checkPermission();

        findViewById(R.id.button).setOnClickListener(cm);

        // 绑定TextureView和Camera
        TextureView tv = binding.cameraTextureView;
        tv.setSurfaceTextureListener(new TextureView.SurfaceTextureListener() {
            @Override
            public void onSurfaceTextureAvailable(@NonNull SurfaceTexture surfaceTexture, int i, int i1) {
                Log.d(TAG, "SurfaceTexture ready.");
                cm.openCameraAndConnect(surfaceTexture);

            }

            @Override
            public void onSurfaceTextureSizeChanged(@NonNull SurfaceTexture surfaceTexture, int i, int i1) {}
            @Override
            public boolean onSurfaceTextureDestroyed(@NonNull SurfaceTexture surfaceTexture) {return false;}
            @Override
            public void onSurfaceTextureUpdated(@NonNull SurfaceTexture surfaceTexture) {}
        });

        // 监听传感器
        SensorManager sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        Sensor gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);
        sensorManager.registerListener(ImuListener.getInstance(), accelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(ImuListener.getInstance(), gyroscope, SensorManager.SENSOR_DELAY_FASTEST);
    }

    private void checkPermission() {
        // 检查是否申请了权限
        if(ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)!= PackageManager.PERMISSION_GRANTED){
            if(ActivityCompat.shouldShowRequestPermissionRationale(this,Manifest.permission.CAMERA)){

            }else{
                ActivityCompat.requestPermissions(this,new String[]{Manifest.permission.CAMERA},1);
            }
        }
    }
}