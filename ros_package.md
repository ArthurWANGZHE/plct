##  ros2 创建功能包
ros2 主要有两种方式编译功能包,分别是 `CMake` 和 `Python` 两种方式在具体命令以及功能包结构上有些不同,本篇文档具体介绍两种方式差别


## 创建cmake功能包
### 准备工作
在创建功能包之前,需要有一个Ros2的工作空间,通过以下命令创建工作空间,以及一个 `src` 文件夹用来存放功能包
```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
```
### 创建cmake功能包
进入工作空间`src`文件夹后,可以使用一下命令创建功能包
```bash
ros2 pkg create --build-type ament_cmake <package_name>
```
创建过程中可以添加`--node-name`选项
```bash
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```
加入 **--dependencies** 可以设置相关依赖
在创建功能包的时候设置相关依赖是最为简单方便的,不容易出现找不到相关包的问题

本文创建一个`hello_world`的工能包,可以输出"hello world" 作为简单示例
包名为`hello_world`,节点名为`say_hi`

```bash
ros2 pkg create --build-type ament_cmake --dependencies rclcpp --node-name say_hi hello_world
```

通过`tree`可以查看当前文件树
```bash
$ tree
.
└── src
    └── hello_world
        ├── CMakeLists.txt
        ├── include
        │   └── hello_world
        ├── package.xml
        └── src
            └── say_hi.cpp

5 directories, 3 files

```

其中

**CMakeLists.txt** 描述如何在包中构建代码的文件

**include/<package_name>** 包含包的公共标头的目录

**package.xml** 包含有关包的元信息的文件

**src** 包含包的源代码的目录

### 编写节点代码
在 `my_package/src` 下编写节点,也就是示例中的 `hello_world/src.say_hi.cpp`

写入以下代码
```cpp
#include "rclcpp/rclcpp.hpp"
int main(int argc, char **argv){
    // 初始化rclcpp
    rclcpp::init(argc, argv);
    // 产生一个节点，并命名为say_hi
    auto node = std::make_shared<rclcpp::Node>("say_hi");
    // 打印消息
    RCLCPP_INFO(node->get_logger(), "hello world!");
    rclcpp::spin(node);
    // 停止运行
    rclcpp::shutdown();

    return 0;
}
```

### 修改 `CMakeLists.txt`
将其添加为可执行文件，在`CMakeLists.txt`最后一行（`ament_package()`之前）加入
```bash
add_executable(hello_world/src.say_hi.cpp)
ament_target_dependencies(say_hi rclcpp)
```
并且使用`install`指令将其安装到`install`目录中，在上面两行代码的后面加入
在 `find_package`部分添加依赖包
```bash
find_package(rclcpp REQUIRED)

ament_target_dependencies(
  say_hi
  "rclcpp"
)
```
修改完毕的CMakeLists.txt文件看起来像

```bash
cmake_minimum_required(VERSION 3.8)
project(hello_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(say_hi src/say_hi.cpp)
target_include_directories(say_hi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(say_hi PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  say_hi
  "rclcpp"
)

install(TARGETS say_hi
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```
### 编译运行
编译使用`colcon`进行编译
使用
```bash
colcon build

$ colcon build
Starting >>> hello_world
Finished <<< hello_world [4.67s]                     

Summary: 1 package finished [4.96s]

```
编译完成后使用`ros2 run`命令进行运行
```bash
ros2 run hello_world say_hi

$ ros2 run hello_world say_hi 
[INFO] [1737363380.601166737] [say_hi]: hello world!

```

## 创建Python功能包

### 创建功能包
相关选项同cmake一样
```bash
ros2 pkg create  --build-type ament_python --dependencies rclpy --node-name  say_hi hello_world
```

此时查看项目树会发现和cmake的结构大相径庭
```bash
$ tree
.
└── src
    └── hello_world
        ├── hello_world
        │   ├── __init__.py
        │   └── say_hi.py
        ├── package.xml
        ├── resource
        │   └── hello_world
        ├── setup.cfg
        ├── setup.py
        └── test
            ├── test_copyright.py
            ├── test_flake8.py
            └── test_pep257.py

5 directories, 9 files
```
### 编辑节点
在`src/hello_world/hello_world/say_hi`中编辑节点
 写入以下代码
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SayHiNode(Node):
    def __init__(self):
        super().__init__('sayhi')
        self.publisher = self.create_publisher(String, 'hello_world_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
        self.get_logger().info('Published: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SayHiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 编辑配置文件
编辑`setup.py`文件，确保`entry_points`部分包含以下内容：
```python
entry_points={
    'console_scripts': [
        'sayhi = helloworld.sayhi:main',
    ],
},
```
### 编译运行
使用 `colcon`指令编译,使用`ros2 run`运行节点

```bash
ros2 run hello_world say_hi 
[INFO] [1737372135.184247876] [sayhi]: Published: "Hello World"
```

这将允许通过ros2 run命令启动say_hi节点。
https://blog.csdn.net/ncnynl/article/details/125729398
https://blog.csdn.net/weixin_42990464/article/details/129274840



























