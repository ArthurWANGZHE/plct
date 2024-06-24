## 虚拟机安装
### 软件下载
#### 安装QEMU
[https://qemu.weilnetz.de/w64/](https://qemu.weilnetz.de/w64/)
#### 下载QEMU UEFI固件
[https://releases.linaro.org/components/kernel/uefi-linaro/latest/release/qemu64/QEMU_EFI.fd](https://releases.linaro.org/components/kernel/uefi-linaro/latest/release/qemu64/QEMU_EFI.fd)
#### 下载镜像文件
[https://www.openeuler.org/zh/download/](https://www.openeuler.org/zh/download/)
#### 安装tap-windows
[https://build.openvpn.net/downloads/releases/tap-windows-9.24.7-I601-Win10.exe](https://build.openvpn.net/downloads/releases/tap-windows-9.24.7-I601-Win10.exe)
### 配置虚拟机
#### X86
##### 设置img
```
qemu-img create -f qcow2 openEuler-24.03-LTS-SP3-x86.img 50G
```

##### 安装虚拟机
```
qemu-system-aarch64 -m 30000 -cpu cortex-a72 -smp 8,cores=2,threads=2,sockets=2 -M virt -bios D:\Programs\Qemu\QEMU_EFI.fd -net nic -net tap,ifname=tap1212 -device nec-usb-xhci -device usb-kbd -device usb-mouse -device VGA -drive if=none,file=D:\VirtuakMachine\openEuler-24.03-LTS-x86-dvd.iso,id=cdrom,media=cdrom -device virtio-scsi-device -device scsi-cd,drive=cdrom -drive if=none,file=D:\VirtuakMachine\qemu\openEuler-24.03-LTS-SP3-x86.img,id=hd0 -device virtio-blk-device,drive=hd0
```


##### 启动虚拟机
```
qemu-system-aarch64 -m 16000 -cpu cortex-a72 -smp 8,cores=2,threads=2,sockets=2 -M virt -bios D:\Programs\Qemu\QEMU_EFI.fd -net nic -net tap,ifname=tap1212 -device nec-usb-hci -device usb-kbd -device usb-mouse -device VGA -drive if=none,file=D:\VirtuakMachine\qemu\openEuler-24.03-LTS-SP3-aarch64.img,id=hd0 -device virtio-blk-device,drive=hd0
```
## 虚拟机配置

安装dde桌面
```
yum install dde
```

设置以图形化界面启动
```
systemctl set-default graphical.target
```

重启生效
```
reboot
```

## ros-humble安装
修改软件源
```
bash -c 'cat << EOF > /etc/yum.repos.d/ROS.repo
[openEulerROS-humble]
name=openEulerROS-humble
baseurl=http://121.36.84.172/dailybuild/EBS-openEuler-24.03-LTS/EBS-openEuler-24.03-LTS/EPOL/multi_version/ROS/humble/x86_64/
enabled=1
gpgcheck=0
EOF'
```

安装
```
dnf install "ros-humble-*" --skip-broken --exclude=ros-humble-generate-parameter-library-example
```

激活
```
source /opt/ros/humble/setup.sh
source ~/.bashrc
```

## 测试
| 测试用例名                     | 状态  |
| ------------------------- | --- |
| 测试ros2 pkg create         | 成功  |
| 测试ros2 pkg executables    | 成功  |
| 测试ros2 pkg list           | 成功  |
| 测试ros2 pkg prefix         | 成功  |
| 测试ros2 pkg xml            | 成功  |
| 测试ros2 run                | 成功  |
| 测试ros2 topic list         | 成功  |
| 测试ros2 topic info         | 成功  |
| 测试ros2 topic type         | 成功  |
| 测试ros2 topic find         | 成功  |
| 测试ros2 topic hz           | 成功  |
| 测试ros2 topic bw           | 成功  |
| 测试ros2 topic echo         | 成功  |
| 测试ros2 param 工具           | 成功  |
| 测试ros2 service 工具         | 成功  |
| 测试ros2 node list          | 成功  |
| 测试ros2 node info          | 成功  |
| 测试ros2 bag 工具             | 成功  |
| 测试ros2 launch 工具          | 成功  |
| 测试ros2 interface list     | 成功  |
| 测试ros2 interface package  | 成功  |
| 测试ros2 interface packages | 成功  |
| 测试ros2 interface show     | 成功  |
| 测试ros2 interface proto    | 成功  |
| 测试 ros 通信组件相关功能           | 成功  |
| 测试 turtlesim功能            | 成功  |
### 1. ROS2 包工具
#### 1.1 创建包
使用命令 ros2 pkg create --build-type ament_cmake riscv-ros-pkg 创建一个新的ROS2包。

![Pasted image 20240624004815.png](attachment%2FPasted%20image%2020240624004815.png)

#### 1.2 包中的可执行文件
通过 ros2 pkg executables turtlesim 查看 turtlesim 包中的可执行文件。

![Pasted image 20240624005117.png](attachment%2FPasted%20image%2020240624005117.png)

#### 1.3 列出所有包
运行 ros2 pkg list 来列出当前系统中所有的ROS2包。

![Pasted image 20240624005220.png](attachment%2FPasted%20image%2020240624005220.png)

#### 1.4 包的安装路径
使用 ros2 pkg prefix turtlesim 来查找 turtlesim 包的安装路径。

![Pasted image 20240624005312.png](attachment%2FPasted%20image%2020240624005312.png)

#### 1.5 包的XML文件
通过 ros2 pkg xml turtlesim 查看 turtlesim 包的XML描述文件。

![Pasted image 20240624005401.png](attachment%2FPasted%20image%2020240624005401.png)

### 2. 运行工具
使用 ros2 run demo_nodes_cpp talker 来运行一个名为 talker 的节点。

![Pasted image 20240624005518.png](attachment%2FPasted%20image%2020240624005518.png)

### 3. 话题工具
#### 3.1 列出所有话题
执行 ros2 topic list 来查看当前系统中所有的话题。

![Pasted image 20240624005756.png](attachment%2FPasted%20image%2020240624005756.png)

使用 ros2 topic info /rosout 来获取 /rosout 话题的详细信息。

![Pasted image 20240624020648.png](attachment%2FPasted%20image%2020240624020648.png)

#### 3.3 话题类型
通过 ros2 topic type /rosout 来查看 /rosout 话题的消息类型。

![Pasted image 20240624020709.png](attachment%2FPasted%20image%2020240624020709.png)

#### 3.4 查找话题类型
使用 ros2 topic find rcl_interfaces/msg/Log 来查找指定消息类型的话题。

![Pasted image 20240624020747.png](attachment%2FPasted%20image%2020240624020747.png)

#### 3.5 话题发布频率
先运行 ros2 run demo_nodes_cpp talker，然后使用 ros2 topic hz /chatter 来测量 /chatter 话题的发布频率。

![Pasted image 20240624021443.png](attachment%2FPasted%20image%2020240624021443.png)

#### 3.6 话题带宽
同样在运行 talker 后，使用 ros2 topic bw /chatter 来测量 /chatter 话题的带宽。

![Pasted image 20240624021516.png](attachment%2FPasted%20image%2020240624021516.png)

#### 3.7 监听话题
在运行 talker 后，使用 ros2 topic echo /chatter 来监听 /chatter 话题的内容。

![Pasted image 20240624021557.png](attachment%2FPasted%20image%2020240624021557.png)

### 4. 参数工具
在运行 ros2 run demo_nodes_cpp talker 后，使用 ros2 param list 来列出当前节点的所有参数。

![Pasted image 20240624021624.png](attachment%2FPasted%20image%2020240624021624.png)

### 5. 服务工具
在运行 talker 后，使用 ros2 service list 来列出当前节点提供的所有服务。

![Pasted image 20240624021733.png](attachment%2FPasted%20image%2020240624021733.png)

### 6. 节点工具
#### 6.1 列出所有节点
在运行 talker 后，使用 ros2 node list 来列出当前系统中所有的节点。

![Pasted image 20240624021802.png](attachment%2FPasted%20image%2020240624021802.png)

#### 6.2 节点信息
在运行 talker 后，使用 ros2 node info /talker 来获取 talker 节点的详细信息。

![Pasted image 20240624021833.png](attachment%2FPasted%20image%2020240624021833.png)

### 7. 包记录工具
使用 ros2 bag record -a 来记录所有话题的数据。

![Pasted image 20240624021916.png](attachment%2FPasted%20image%2020240624021916.png)

#### 7.1 包信息
使用 ros2 bag info rosbag2_2024_06_24-02_18_57/rosbag2_2024_06_24-02_18_57_0.db3 来查看包的详细信息。

![Pasted image 20240624022153.png](attachment%2FPasted%20image%2020240624022153.png)

#### 7.2 播放包
使用 ros2 bag play rosbag2_2024_06_24-02_18_57/rosbag2_2024_06_24-02_18_57_0.db3 来播放记录的数据。

![Pasted image 20240624022346.png](attachment%2FPasted%20image%2020240624022346.png)

### 8. 启动工具
使用 ros2 launch demo_nodes_cpp talker_listener.launch.py 来启动一个包含多个节点的启动文件。

![Pasted image 20240624022425.png](attachment%2FPasted%20image%2020240624022425.png)

### 9. 接口工具
#### 9.1 列出所有接口
使用 ros2 interface list 来列出系统中的所有接口，包括消息、服务和动作。

![Pasted image 20240624022524.png](attachment%2FPasted%20image%2020240624022524.png)

#### 9.2 接口包
使用 ros2 interface package action_msgs 来查看 action_msgs 包中的接口。

![Pasted image 20240624022549.png](attachment%2FPasted%20image%2020240624022549.png)

#### 9.3 接口包列表
使用 ros2 interface packages 来列出所有接口包。

![Pasted image 20240624022644.png](attachment%2FPasted%20image%2020240624022644.png)

#### 9.4 显示接口详细内容
消息
使用 ros2 interface show geometry_msgs/msg/TwistStamped 来查看消息的详细内容。

![Pasted image 20240624022711.png](attachment%2FPasted%20image%2020240624022711.png)

服务
使用 ros2 interface show action_msgs/srv/CancelGoal 来查看服务的详细内容。

![Pasted image 20240624022854.png](attachment%2FPasted%20image%2020240624022854.png)

动作
使用 ros2 interface show action_tutorials_interfaces/action/Fibonacci 来查看动作的详细内容。

![Pasted image 20240624022923.png](attachment%2FPasted%20image%2020240624022923.png)

#### 9.5 消息模板
使用 ros2 interface proto geometry_msgs/msg/TwistStamped 来查看消息的模板。

![Pasted image 20240624022950.png](attachment%2FPasted%20image%2020240624022950.png)

### 10. 测试ROS通信组件
#### 10.1 话题通信
C++实现
使用 ros2 run demo_nodes_cpp talker 和 ros2 run demo_nodes_cpp listener 来测试话题通信。

![Pasted image 20240624023014.png](attachment%2FPasted%20image%2020240624023014.png)

![Pasted image 20240624023039.png](attachment%2FPasted%20image%2020240624023039.png)

Python实现
使用 ros2 run demo_nodes_py talker 和 ros2 run demo_nodes_py listener 来测试话题通信。

![Pasted image 20240624023113.png](attachment%2FPasted%20image%2020240624023113.png)

![Pasted image 20240624023146.png](attachment%2FPasted%20image%2020240624023146.png)

#### 10.2 服务通信
C++实现
使用 ros2 run demo_nodes_cpp add_two_ints_server 和 ros2 run demo_nodes_cpp add_two_ints_client 来测试服务通信。

![Pasted image 20240624023250.png](attachment%2FPasted%20image%2020240624023250.png)

![Pasted image 20240624023306.png](attachment%2FPasted%20image%2020240624023306.png)

Python实现
使用 ros2 run demo_nodes_py add_two_ints_server 和 ros2 run demo_nodes_py add_two_ints_client 来测试服务通信。

![Pasted image 20240624023422.png](attachment%2FPasted%20image%2020240624023422.png)

![Pasted image 20240624023408.png](attachment%2FPasted%20image%2020240624023408.png)

#### 10.3 测试ROS坐标转换
坐标转换的发布和订阅
使用 ros2 run tf2_ros static_transform_publisher 1 1 1 0 0 0 /base_link /odom 和 ros2 run tf2_ros tf2_echo base_link odom 来测试坐标转换。

![Pasted image 20240624095155.png](attachment%2FPasted%20image%2020240624095155.png)

![Pasted image 20240624095242.png](attachment%2FPasted%20image%2020240624095242.png)

tf_monitor监控
使用 ros2 run tf2_ros static_transform_publisher 1 1 1 0 0 0 /base_link /odom 和 ros2 run tf2_ros tf2_monitor 来监控坐标转换。

![Pasted image 20240624095155.png](attachment%2FPasted%20image%2020240624095155.png)

![Pasted image 20240624095330.png](attachment%2FPasted%20image%2020240624095330.png)

#### 10.4 view_frames保存pdf
使用 ros2 run tf2_ros static_transform_publisher 1 1 1 0 0 0 /base_link /odom 和 ros2 run tf2_tools view_frames 来生成坐标转换的PDF视图。

![Pasted image 20240624095414.png](attachment%2FPasted%20image%2020240624095414.png)

### 11. 测试turtlesim
使用 ros2 run turtlesim turtlesim_node 和 ros2 run turtlesim turtle_teleop_key 来测试turtlesim的功能。

![Pasted image 20240624095554.png](attachment%2FPasted%20image%2020240624095554.png)

参考
[OpenEuler安装保姆级教程 | 附可视化界面_openeuler安装图形界面-CSDN博客](https://blog.csdn.net/qq_50824019/article/details/124526889)
[user_doc/ROS-humble-oerv24.03-x86/README.md · openEuler/ros - Gitee.com](https://gitee.com/openeuler/ros/blob/master/user_doc/ROS-humble-oerv24.03-x86/README.md)
