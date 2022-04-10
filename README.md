# lio_slam

# 1. Installation

## 1.1 Compile Livox SDK

```
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
sudo make install
```

## 1.2 Dependencies

```
# for ROS melodic
sudo apt-get install libglm-dev libglfw3-dev
sudo apt-get install ros-melodic-geodesy ros-melodic-pcl-ros ros-melodic-nmea-msgs ros-melodic-libg2o
```

## 1.3 Compile

```
cd ~/dev_lio_slam
catkin_make -DCMAKE_BUILD_TYPE=Release
```



```
catkin_make -DCATKIN-WHITE-PACKAGES="livox_ros_driver"

catkin_make -DCATKIN_WHITELIST_PACKAGES=""

```

token:

```
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
```

