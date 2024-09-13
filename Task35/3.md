# ROS 2 安全飞地

*本文规定了安全飞地的集成*

## 作者：Ruffin White, Mikael Arguedas

## 撰写日期：2020-05

## 最后修改：2020-07

本设计文档正式规定了ROS 2与安全飞地的集成。简而言之，所有安全进程必须使用一个包含该飞地唯一运行时安全工件的飞地，但每个进程不一定有唯一的飞地。多个飞地可以封装在单一的安全策略中，以准确模拟信息流控制。用户可以通过控制部署时应用飞地的作用域来调整这些模型的保真度。例如，每个操作系统进程、每个操作系统用户、每个设备/机器人、每个群体等都有一个唯一的飞地。本文档的其余部分详细说明了如何组织和使用飞地。

## 概念

在详细说明SROS 2集成飞地之前，引入以下概念:

### 参与者
参与者是代表网络上单个实体的对象。在DDS的情况下，参与者是DDS域参与者，具有访问控制权限和安全身份。

### 命名空间
命名空间是ROS中的基本设计模式，广泛用于组织和区分各种类型的资源，使其可唯一识别；例如主题、服务、动作和节点名称。因此，命名空间的概念为当前用户所熟知并理解，得到了现有工具的强烈支持。命名空间通常可以在运行时通过命令行参数配置，或通过启动文件声明静态/程序性地配置。

以前，节点的完全限定名（FQN）直接用于选定的安全目录查找策略，以加载所需的密钥材料。然而，现在参与者映射到上下文而不是节点，这种直接将节点FQN映射到安全工件的方法不再合适。

### 上下文
随着ROS 2的出现，现在可以将多个节点组合到一个进程中以提高性能。然而，以前每个节点都会保留它与单独的中间件参与者一对一的映射。鉴于每个进程的多个参与者会产生不可忽视的开销，引入了将单个参与者映射到上下文的变更，并允许多个节点共享该上下文。

根据DDS安全规范v1.1，参与者只能使用单一安全身份；因此，映射到给定上下文的每个节点适用的访问控制权限必须合并并整合成单一的安全工件集，或飞地。因此，每个进程中的所有上下文及其各自的参与者将使用该单一飞地。因此，需要额外的工具和对SROS 2的扩展来支持这种新范式。

### 密钥库
随着上下文的增加，最好借此机会重新组织密钥库布局。而不是一个平面的、按命名空间组织的节点安全目录，我们可以将所有这些安全目录推入一个指定的飞地子文件夹中。同样，私钥和公钥材料也可以推入根密钥库目录中的各自子文件夹中。这让人想起Keymint [1]中早期使用的模式。

```markdown
$ tree keystore/
keystore
├── enclaves
│   └── ...
│       └── ...
├── private
│   ├── ca.csr.pem
│   └── ca.key.pem
└── public
    ├── ca.cert.pem
    ├── identity_ca.cert.pem -> ca.cert.pem
    └── permissions_ca.cert.pem -> ca.cert.pem
```

- **public** 目录包含任何可公开的内容，例如身份或权限证书颁发机构的公钥证书。因此，可以给所有可执行文件读取访问权限。注意，默认情况下，identity_ca和permissions_ca指向相同的CA证书。
- **private** 目录包含任何可私有的内容，例如上述证书颁发机构的私钥材料。在将密钥库部署到目标设备/机器人之前，应将此目录删除。
- **enclaves** 目录包含与各个飞地相关的安全工件，因此不再相关节点目录。然而，类似于节点目录，飞地文件夹可能仍然可以递归嵌套子路径以组织单独的飞地。按照惯例，`ROS_SECURITY_KEYSTORE`环境变量应指向此目录。

## 集成
随着rcl中上下文的引入，不再依赖节点命名空间从密钥库中查找安全工件，而是将飞地路径与节点命名空间完全解耦，而是作为其自己的唯一资源标识符。尽管同时需要管理这两个标识符空间可能会引入更多的自由度，但仍然应该可以组织密钥库中的飞地以模仿节点命名空间层次结构，以透明地追踪分配的权限。

## 未来工作
自省工具和启动文件界面应更新，以帮助减轻迁移到上下文和飞地引入的复杂性。

## 运行时
鉴于策略中飞地可能特定于单个节点/容器进程的规范情况，节点被重映射到的命名空间将不可避免地影响飞地内所需的安全权限。为了突出这种相互依赖性，并帮助避免飞地路径冲突，适当的层次结构借用命名空间是合适的。按照惯例，ros2launch可以用来为单个进程节点或容器使用范围内的命名空间前缀相对飞地路径，以实现可组合启动文件的惯例，具有可调整和参数化的飞地路径。鉴于运行时命令参数用于指定完全合格的飞地路径，ros2launch将相应解析可执行文件的相对飞地路径，如启动属性所定义。

### 未限定的飞地路径
对于具有未限定飞地路径的单个进程节点，飞地目录将默认为根级飞地。

```xml
<launch>
  <node pkg="demo_nodes_cpp" exec="talker"/>
  <node pkg="demo_nodes_cpp" exec="listener"/>
</launch>
```

```
$ tree enclaves/
enclaves/
├── cert.pem
├── governance.p7s
├── identity_ca.cert.pem -> ../public/identity_ca.cert.pem
├── key.pem
├── permissions_ca.cert.pem -> ../public/permissions_ca.cert.pem
└── permissions.p7s
```

### 推送未限定的飞地路径
对于由命名空间推送的具有未限定飞地路径的单个进程节点，飞地目录将相应地被推送到相对子文件夹中。

```xml
<launch>
  <node pkg="demo_nodes_cpp" exec="talker"/>
  <group>
    <push_ros_namespace namespace="foo"/>
    <node pkg="demo_nodes_cpp" exec="listener"/>
  </group>
</launch>
```

```
$ tree --dirsfirst enclaves/
enclaves/
├── foo
│   ├── cert.pem
│   ├── governance.p7s
│   ├── identity_ca.cert.pem
│   ├── key.pem
│   ├── permissions_ca.cert.pem
│   └── permissions.p7s
├── cert.pem
├── governance.p7s
├── identity_ca.cert.pem
├── key.pem
├── permissions_ca.cert.pem
└── permissions.p7s
```
### 相对推送的限定飞地路径
对于由命名空间推送的具有限定飞地路径的单个进程节点，限定飞地目录将相应地被推送到相对子文件夹中。

```xml
<launch>
  <group>
    <push_ros_namespace namespace="foo"/>
    <node pkg="demo_nodes_cpp" exec="listener" enclave="bar"/>
  </group>
</launch>
```

```
$ tree --dirsfirst enclaves/
enclaves/
└── foo
    └── bar
        ├── cert.pem
        ├── governance.p7s
        ├── identity_ca.cert.pem
        ├── key.pem
        ├── permissions_ca.cert.pem
        └── permissions.p7s
```
### 完全限定的飞地路径
对于具有绝对飞地路径的单个进程节点，命名空间不会随之推送相对子文件夹。

```xml
<launch>
  <group>
    <push_ros_namespace namespace="foo"/>
    <node pkg="demo_nodes_cpp" exec="listener" enclave="/bar"/>
  </group>
</launch>
```

```
$ tree --dirsfirst enclaves/
enclaves/
└── bar
    ├── cert.pem
    ├── governance.p7s
    ├── identity_ca.cert.pem
    ├── key.pem
    ├── permissions_ca.cert.pem
    └── permissions.p7s
```

## 替代方案
#### `<push_ros_namespace namespace="..." enclave="foo"/>`
- 通过向`push_ros_namespace`元素添加飞地属性来实现这样的方法。这也使得推送飞地与推送命名空间保持接近/易于阅读。
#### `<push_ros_enclave enclave="foo"/>`
- 另一种替代方法是添加一个全新的`push_ros_enclave`元素。这可以确保推送飞地路径与命名空间独立/灵活。

## 关注点 
### 上下文中的多个命名空间 
当用户可能将不同命名空间的多个节点组合到单个上下文中时，用户仍需随后指定适用于所有组合节点的共同飞地路径。对于飞地路径与节点命名空间正交的情况，使用完全限定的所有相关飞地路径可能很繁琐，但可能仍然可以通过使用 `<var/>` 和 `<arg/>` 替换和扩展来参数化。

### 在进程中对节点的权限建模与中间件参与者的权限 
在使用上下文之前，多个节点被组合到一个单独的进程中，每个节点都映射到一个单独的参与者。每个参与者随后加载了适用于其相应节点的安全身份和访问控制凭证。然而，该进程中的所有节点共享相同的内存空间，因此可以访问其他节点的数据。加载的中间件凭证/权限与进程内可访问资源之间存在不匹配。

通过使用飞地，上下文中的所有节点共享相同的安全身份和访问控制凭证。这不可避免地意味着编译到节点foo的代码可以访问仅受信任于节点bar的凭证/权限。这种组合的后果可能会无意中破坏由策略设计者构建的或通过 ROS 2 工具/IDL 测量/生成的最小跨度策略。

随着飞地的引入，通过在特定飞地内定义一系列 SROS 2 策略配置文件作为元素，成为可能描述访问控制权限的并集。这将允许正式分析工具 [2] 检查在运行时组合节点时潜在的信息流控制违规。如果一个进程加载了一个单一的飞地，这就调和了参与者的权限和进程的权限。

然而，如果每个进程加载了多个飞地，那么由于共享相同的内存空间，这种安全保证再次丧失。因此，应该询问是否应该支持每个进程中的多个飞地。

总之，这里的区分在于以前，多个节点权限的组合无法传达给工具。节点是否能够获得同一进程空间中其他节点的权限并不是注意的关键点；事实上，这些副作用无法由设计者正式建模或考虑。现在通过飞地将成为可能，但允许每个进程加载单独飞地的多个上下文将重新引入并加剧相同的建模不准确。

### 可组合的启动文件包含 
使用带有安全飞地的启动文件的一个特别挑战是保持包含层次结构的可组合性。在编写供下游使用的启动文件时，可能会出现简单性与可配置性之间的固有权衡。作者可以选择性地选择哪些属性作为输入参数公开，而用户可能会隐式覆盖提供的默认值。

在飞地的情况下，无论是包作者还是用户应该采用什么最佳实践来保持可组合和直观的启动文件结构，这并不明显。例如：作者应该为每个节点的飞地路径参数化输入参数吗？用户应该将包含的启动文件的命名空间推送到独特的飞地吗？

可以肯定的是，应该鼓励不要在启动文件内设置安全环境变量，因为这将限制结合节点 IDL 进行程序化策略生成的启动文件的静态分析。

### 容器中的可组合节点
鉴于容器可能是动态的，节点可以在运行时添加或删除，可能会有一些关于容器应如何与安全飞地集成的问题。在 ros2launch 中，可以在容器实例化时使用范围内的命名空间来解析容器指定的相对飞地路径，从而适用于容器内的所有节点/组件。在最终扩展容器的启动 API 时，应该进一步考虑这一点。

### RMW实现的迁移 
由于可能需要时间才能使所有 RMW 实现实现新的上下文系统，应该指定一个明确回退行为。对于这些实现，应按照“ROS 2 DDS-Security 集成”设计文档中指定的，为参与者加载由 RCL 确定的飞地安全目录。这主要避免了在默认查找路径中包含节点名称的做法，从而使用户习惯于为单独的进程创建单独的飞地，或通过启动文件明确指定唯一的飞地路径。

## 参考文献
- [Procedurally Provisioned Access Control for Robotic Systems](https://arxiv.org/pdf/1810.08125.pdf)
```
@inproceedings{White2018,
	title     = {Procedurally Provisioned Access Control for Robotic Systems},
	author    = {White, Ruffin and Caiazza, Gianluca and Christensen, Henrik and Cortesi, Agostino},
	year      = 2018,
	booktitle = {2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
	doi       = {10.1109/IROS.2018.8594462},
	issn      = {2153-0866},
	url       = {https://arxiv.org/pdf/1810.08125.pdf}}
```
-  [Network Reconnaissance and Vulnerability Excavation of Secure DDS Systems](https://doi.org/10.1109/EuroSPW.2019.00013)
```
@inproceedings{White2019,
	title     = {Network Reconnaissance and Vulnerability Excavation of Secure DDS Systems},
	author    = {White, Ruffin and Caiazza, Gianluca and Jiang, Chenxu and Ou, Xinyue and Yang, Zhiyue and Cortesi, Agostino and Christensen, Henrik},
	year      = 2019,
	booktitle = {2019 IEEE European Symposium on Security and Privacy Workshops (EuroS PW)},
	doi       = {10.1109/EuroSPW.2019.00013},
	pages     = {57-66},
	url       = {https://arxiv.org/abs/1908.05310.pdf}}
```
