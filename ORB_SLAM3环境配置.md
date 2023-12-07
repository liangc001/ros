#ORB_SLAM3环境配置

------

[TOC]

```
ORB-SLAM3是一个支持视觉、视觉加惯导、混合地图的SLAM系统,可以在单目,双目和RGB-D相机上利用针孔或者鱼眼模型运行
```

## 1.下载软件安装包

ORB-SLAM3源码

Pangolin

Eigen3

Opencv3.4.3

boost 库

##2.依次安装软件包

###1.补齐前置库

```bash
#顺序可能有误，根据需求安装即可
sudo apt install libgl1-mesa-dev
sudo apt install vim
sudo apt install git
sudo apt install libglew-dev
sudo apt install cmake
sudo apt install libpython2.7-dev
sudo apt install pkg-config
sudo apt install libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols
sudo apt-get install build-essential libgtk2.0-dev libavcodec-dev libavformat-dev libjpeg-dev libtiff5-dev libswscale-dev libjasper-dev
sudo apt-get install libssl-dev
```

####注意事项

#####出现问题A

```bash
E：Sub-process returned an error code
#或
未找到软件包……
#原因：未更换下载源，执行update失败
```

![3](C:\Users\123\Desktop\3.jpg)

##### 解决方式1

![3s](C:\Users\123\Desktop\3s.jpg)

##### 解决方式2

![3s4](C:\Users\123\Desktop\3s4.jpg)

##### 解决方式3

![3s5](C:\Users\123\Desktop\3s5.jpg)

##### 解决方式4

![3s6](C:\Users\123\Desktop\3s6.jpg)

```bash
#再执行
sudo apt-get update
```

### 2.安装Eigen3

```bash
cd Eigen3
mkdir build
cd build
cmake ..
sudo make install
```

### 3.安装Pangolin

```bash
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
```

### 4.安装Opencv3.4.3

```bash
cd opencv-3.4.3
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j4
sudo make install
#配置环境……
```

#### 注意事项

#####出现问题A

```bash
E: 软件包 libjasper-dev 没有可安装候选
```

![安装opencv](C:\Users\123\Desktop\安装opencv.jpg)

##### 解决方式

```bash
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt install libjasper1 libjasper-dev
```

##### 出现问题B

```bash
#安装opencv gcc版本过低 make报错
```

![安装opencv_gcc版本过低_make报错](C:\Users\123\Desktop\安装opencv_gcc版本过低_make报错.jpg)

##### 解决方式

![解决2](C:\Users\123\Desktop\解决2.jpg)

##### 出现问题C

死机

##### 解决方式

*重装虚拟机*

### 5.安装boost 库

```bash
sudo ./bootstrap.sh
sudo ./b2 install
```

### 6.安装ORB-SLAM3

####1.源码编译

```bash
chmod +x build.sh
```

#####注意事项

######出现问题A

```bash
#cmakelist中opencv版本不同
```

######解决方式

![更改cmakelist-opencv版本](C:\Users\123\Desktop\更改cmakelist-opencv版本.jpg)

####2.安装

依次安装DBoW2，g2o，Sophus后安装ORB_SLAM3即可。

####验证案例代码：

```bash
./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml ./dataset/V102 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/V102.txt dataset-V102_monoi
```

#####注意事项

###### 出现问题A

*卡死*

![卡死2](C:\Users\123\Desktop\卡死2.jpg)

###### 解决方式

```bash
#切换 
make  ->  make -j4
```

###### 出现问题B

```bash
make: *** [all] Error 2
```

![42(切换make_-j4----make)](C:\Users\123\Desktop\42(切换make_-j4----make).jpg)

###### 解决方式

```bash
#切换 
make -j4  ->  make
```

####3.编译ORB_SLAM3 ROS模块

```bash
cp -r Examples_old/ROS Examples
vim   ~/.bashrcexportROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/raypc/codes/orb/Examples/ROS/
chmod +x build_ros.sh
./build_ros.sh
```

##### 注意事项

###### 出现问题A

```bash
Error: the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update'
#ros一键安装时漏掉了
```

![ros](C:\Users\123\Desktop\ros.jpg)

###### 解决方式

![ros解决](C:\Users\123\Desktop\ros解决.jpg)

###### 出现问题B

Sophus安装出现问题

```bash
fatal error: sophus/se3.hpp:没有那个文件或目录
#include <sophus/se3.hpp>
```

![sophus重装so](C:\Users\123\Desktop\sophus重装so.jpg)

###### 解决方式

重装Sophus

![ss](C:\Users\123\Desktop\ss.jpg)

###### 出现问题C

opencv默认版本与使用版本不符，引起冲突

```bash
warning: libopencv_imgporc.so.3.2, needed by /……/libcv_bridge.so, may conflict with libopencv_imgporc.so.3.4
```

![failed](C:\Users\123\Desktop\failed.jpg)

###### 解决方式

修改cv_bridgeConfig.cmake

![看看](C:\Users\123\Desktop\看看.png)

![kk2](C:\Users\123\Desktop\kk2.png)

(其实我修改完之后还是不行就又改回来了，但就能编译了)

##3.开启摄像头连接

```bash
#将camera.py放入 /Examples/ROS/ORB_SLAM3/文件夹下
chmod +x camera.py
rosrun ORB_SLAM3 camera.py
rosrun ORB_SLAM3 Mono /home/lcc/SLAM/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/lcc/SLAM/ORB_SLAM3/Examples/Monocular/TUM1.yaml
```

![ok](C:\Users\123\Desktop\ok.jpg)

####注意事项

##### 出现问题A

```bash
#运行后系统提示如下
[ WARN:0] global /tmp/pip-req-build-6amqbhlx/opencv/modules/videoio/src/cap_v4l.cpp (893) open VIDEOIO(V4L2:/dev/video0): can't open camera by index
```

##### 解决方式

启动 虚拟机->可移动设备->关于摄像头设备的选项

##### 出现问题B

初始摄像头黑屏没有画面

##### 解决方式

调整摄像头的位置（物理）
