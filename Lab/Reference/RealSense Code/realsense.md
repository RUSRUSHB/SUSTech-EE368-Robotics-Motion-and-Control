

# realsense 使用
## 安装 realsense SDK
```
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade

sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

sudo apt-get install librealsense2-dkms librealsense2-utils

sudo apt-get install librealsense2-dev librealsense2-dbg
```

## 安装 realsense-ros
```
sudo apt-get install ros-$ROS_DISTRO-realsense2-*
```

测试
```
roslaunch realsense2_camera rs_camera.launch 
rviz

roslaunch realsense2_camera demo_pointcloud.launch 
```

## 相机内参，相机外参，相机标定
相机标定：眼在手上，眼在手外

[教程](https://blog.csdn.net/yaked/article/details/77161160)
[工具](https://github.com/ros-planning/moveit_calibration)