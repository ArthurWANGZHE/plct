# 从源码编译 slam_gmapping
[github仓库](https://github.com/Project-MANAS/slam_gmapping)


## 克隆仓库并编译
```angular2html
mkdir slam_gmapping
cd slam_gmapping
mkdir src
mkdir build
cd src
git clone https://github.com/Project-MANAS/slam_gmapping.git
cd ../
colcon build
source install/setup.bash
ros2 pkg list | grep slam
```