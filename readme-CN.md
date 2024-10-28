# IMUGPS

本代码是论文《IMU-Assisted Gray Pixel Shift for Video White Balance Stabilization》的安卓实现

![pipeline](Assets/pipeline.png)

## 环境和初始化

大部分环境配置包含在gradle文件内，需要额外导入[Opencv 4.8.0 for android](https://github.com/opencv/opencv/releases/download/4.8.0/opencv-4.8.0-android-sdk.zip)作为module

运行代码前需要修改[native-lib.cpp](Code/app/src/main/cpp/native-lib.cpp#L55-L57)中的相机标定矩阵

## 项目结构

```
project/
└── app/src/main/         		# 代码文件
    ├── java/             		# 安卓数据统计
    └── cpp/          			# 算法核心
        ├── native-lib.cpp		# Native文件
        ├── IMUGPS.h			# 核心算法实现
        ├── MadgwickAHRS.cpp	# IMU姿态估计
        └── Cluster.h			# 谱聚类实现
```