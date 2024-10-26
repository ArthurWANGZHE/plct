## 多机通信
### 网络配置
1. **虚拟机网络设置：** 将虚拟机的网络配置为桥接模式
![1.png](attachments%2F1.png)
2. **连接至小车热点：** 确保设备已连接到小车发出的Wi-Fi热点
![2.png](attachments%2F2.png)

3. **SSH登录：** 通过SSH登录小车终端
```
ssh openeuler@10.42.0.1
```
密码为123456

4. **IP地址确认：** 使用ifconfig命令查看IP地址


小车(主机)
![4.png](attachments%2F4.png)


虚拟机(从机)
![3.png](attachments%2F3.png)


确保两者IP地址的前三位相同，仅最后一位不同（例如：10.42.0.46和10.42.0.1）

5. **网络连通性测试：** 使用ping命令测试网络连通性，应有响应以证明网络配置成功
```
ping 10.42.0.46
ping 10.42.0.1
```

![5.png](attachments%2F5.png)

### 设置环境变量
ROS主机设置：虚拟机仅作为查看接受地图信息因此将小车配置为ROS主机，虚拟机配置为ROS从机。

1. **bashrc配置：** 
在小车的`.bashrc`文件中添加以下环境变量：
```
export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_IP=10.42.0.1
export ROS_DOMAIN_ID=20
```
在虚拟机的`.bashrc`文件中添加一下环境变量:
```
export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_IP=10.42.0.46
export ROS_DOMAIN_ID=20
```

ROS_MASTER_URI应始终指向小车的IP地址
ROS_IP在小车中应为小车的IP地址，在虚拟机中应为虚拟机的IP地址
ROS_DOMAIN_ID保持一致即可

如下所示
![6.png](attachments%2F6.png)

![7.png](attachments%2F7.png)

### 测试
通信测试
测试通信：使用demo_nodes进行通信测试。
在小车终端运行：
```
ros2 run demo_nodes_cpp talker
```
在虚拟机终端运行：
```
ros2 run demo_nodes_cpp listener
```

如果两者能够互相发现并通信，证明多机通信设置成功
![8.png](attachments%2F8.png)