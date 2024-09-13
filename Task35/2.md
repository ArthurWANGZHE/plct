# ROS 2访问控制策略

*本文规定了在保护ROS子系统时用于访问控制的策略格式。*

## 作者：Ruffin White, Kyle Fazzari

## 撰写日期：2019-08

## 最后修改：2021-06

SROS 2引入了包括加密、认证和授权在内的多项安全属性。通过结合前两个属性和访问控制模型，可以获得授权。这些模型通常被称为访问控制策略。策略作为与属性相关的特权的高级抽象，然后可以转换为个别身份的低级权限，例如在安全DDS网络内的特定ROS节点。

## 概念

在详细说明SROS 2的访问控制策略设计之前，建立一些有助于以安全术语形式化设计方法的概念很重要。在这个设置中，主体可以被视为分布式数据总线上的参与者（例如计算图中的ROS节点），而对象可能是特定子系统的实例（例如ROS主题），访问定义为对该对象采取行动的能力（例如发布或订阅）。

### 强制访问控制
强制访问控制（MAC）指的是只有在存在规则允许给定主体访问资源时，才允许访问对象；术语“强制”表示主体对对象的访问必须始终明确配置。最重要的是，与自由访问控制（DAC）相反，这些策略由一组授权规则执行，主体不能意外或故意覆盖或修改这些规则。这也可以理解为“默认拒绝”。

### 最小特权原则
最小特权原则（PoLP）要求在特定的抽象层中，每个主体只能访问其合法目的所必需的资源。这也被称为最小特权原则或最小权威原则。应用PoLP不仅增强了系统安全性，还可以简化部署并帮助提高系统稳定性。

### 权限分离
权限分离要求将主体的访问权限分割成部分，这些部分仅限于执行特定任务所需的特定特权。这用于减轻安全漏洞的潜在损害。无法遵守此要求的系统可能也无法满足PoLP。

### 关心点分离
关心点分离（SoC）是一种设计原则，用于将系统划分为不同的部分，以便每个部分解决一个单独的问题。在这种情况下，不同的问题可能是系统如何管理加密与如何向主体授权。

## 标准

### 验证
在解释任何用户配置输入之前，例如访问控制策略，应应用数据验证以确保输入符合且格式正确。错误的输入可能会影响大多数程序或工具的完整性，然而防范一般性畸形可能本身需要细致的验证逻辑。使用精确的模式形式化数据描述，允许不同的程序断言输入符合而无需在实现中复制验证逻辑。在XML中，这是通过使用XSD实现的；允许策略标记由可扩展的标准定义而不是规范实现来定义。

### 转换
为了可用性和泛化，访问控制策略可以使用特定于域的抽象表达，例如基于ROS的主体和对象。然而，这些抽象在应用于较低级别的传输和策略执行点时可能会转换为不同的表示。使用转换语言形式化此数据转换允许不同的程序进行转译而无需在实现中复制转换逻辑。在XML中，这是通过使用XSLT实现的；允许策略标记通过简单地交换或扩展转换来轻松转换为各种传输。

### 组合
在制定访问控制策略时，许多主体可能共享基本权限以获取基本访问权限。为了避免可能加剧人为错误或其他差异的不必要重复，策略应具有足够的表达能力以保持DRY（不重复自己）。在XML中，这是通过使用XInclude实现的；允许策略标记轻松包含或替换在不同策略或配置文件中重复的特定配置文件和权限的外部引用。

## 模式

SROS 2策略模式定义为XML。组成策略的元素和属性如下所述。

```xml
<?xml version="1.0" encoding="UTF-8"?>
<xs:schema
    xmlns:xs="http://www.w3.org/2001/XMLSchema"
    xmlns:xml="http://www.w3.org/XML/1998/namespace"
    elementFormDefault="qualified" attributeFormDefault="unqualified">
    <xs:import namespace="http://www.w3.org/XML/1998/namespace"
               schemaLocation="http://www.w3.org/2001/03/xml.xsd" />


    <xs:element name="policy" type="Policy" />
    <xs:complexType name="Policy">
        <xs:sequence minOccurs="1" maxOccurs="1">
            <xs:element name="enclaves" type="Enclaves" />
        </xs:sequence>
        <xs:attribute name="version" type="xs:string" use="required" fixed="0.2.0"/>
    </xs:complexType>

    <xs:complexType name="Enclaves">
        <xs:sequence minOccurs="1" maxOccurs="unbounded">
            <xs:element name="enclave" type="Enclave" />
        </xs:sequence>
    </xs:complexType>

    <xs:complexType name="Enclave">
        <xs:sequence minOccurs="1" maxOccurs="unbounded">
            <xs:element name="profiles" type="Profiles" />
        </xs:sequence>
        <xs:attribute name="path" type="xs:string" use="required" />
    </xs:complexType>

    <xs:complexType name="Profiles">
        <xs:sequence minOccurs="1" maxOccurs="1">
            <xs:sequence minOccurs="1" maxOccurs="unbounded">
                <xs:element name="profile" type="Profile" />
            </xs:sequence>
            <xs:sequence minOccurs="0" maxOccurs="1">
                <xs:element name="metadata" type="xs:anyType" />
            </xs:sequence>
        </xs:sequence>
        <xs:attribute name="type" type="xs:string" use="optional" />
    </xs:complexType>

    <xs:complexType name="Profile">
        <xs:sequence minOccurs="0" maxOccurs="unbounded">
            <xs:choice minOccurs="1" maxOccurs="1">
                <xs:element name="topics" minOccurs="1" type="TopicExpressionList" />
                <xs:element name="services" minOccurs="1" type="ServicesExpressionList" />
                <xs:element name="actions" minOccurs="1" type="ActionsExpressionList" />
            </xs:choice>
        </xs:sequence>
        <xs:attribute name="ns" type="xs:string" use="required" />
        <xs:attribute name="node" type="xs:string" use="required" />
        <xs:attribute ref="xml:base" />
    </xs:complexType>

    <xs:complexType name="TopicExpressionList">
        <xs:sequence minOccurs="1" maxOccurs="unbounded">
            <xs:element name="topic" type="Expression" />
        </xs:sequence>
        <xs:attribute name="publish" type="RuleQualifier" use="optional" />
        <xs:attribute name="subscribe" type="RuleQualifier" use="optional" />
        <xs:attribute ref="xml:base" />
    </xs:complexType>

    <xs:complexType name="ServicesExpressionList">
        <xs:sequence minOccurs="1" maxOccurs="unbounded">
            <xs:element name="service" type="Expression" />
        </xs:sequence>
        <xs:attribute name="reply" type="RuleQualifier" use="optional" />
        <xs:attribute name="request" type="RuleQualifier" use="optional" />
        <xs:attribute ref="xml:base" />
    </xs:complexType>

    <xs:complexType name="ActionsExpressionList">
        <xs:sequence minOccurs="1" maxOccurs="unbounded">
            <xs:element name="action" type="Expression" />
        </xs:sequence>
        <xs:attribute name="call" type="RuleQualifier" use="optional" />
        <xs:attribute name="execute" type="RuleQualifier" use="optional" />
        <xs:attribute ref="xml:base" />
    </xs:complexType>

    <xs:simpleType name="Expression">
        <xs:restriction base="xs:string" />
    </xs:simpleType>

    <xs:simpleType name="RuleQualifier">
        <xs:restriction base="xs:string">
            <xs:enumeration value="ALLOW" />
            <xs:enumeration value="DENY" />
        </xs:restriction>
    </xs:simpleType>

</xs:schema>
```

### `<policy>` 标签
策略文件的根标签。每个策略文件中只能有一个`<policy>`标签。

属性：

- version: 使用中的模式版本的声明
  允许推进未来的模式修订

### `<enclaves>` 标签
封装了一系列独特的飞地。这种嵌套序列的方法允许将额外的标签扩展到`<policy>`根。

### `<enclave>` 标签
封装了一组配置文件。这是特定于由关联属性确定的飞地。

属性：

- path: 完全合格的飞地路径

鉴于多个节点可以组合到一个单独的进程中，飞地用于包含所有相应节点的配置文件集合。因此，飞地可以被认为是包含配置文件的并集。请注意，飞地内配置文件的并集将导致任何配置文件的权限被拒绝优先于所有配置文件的允许权限。例如，如果一个配置文件请求权限，但另一个配置文件在飞地中明确拒绝了匹配的权限，则拒绝规则将优先。有关如何应用MAC的更多信息，请参见`<profile>`标签部分。

### `<profiles>` 标签
封装了一系列独特的配置文件和指定的元数据。这种嵌套序列的方法允许将额外的标签扩展到`<enclave>`根，以及将特定元数据或约束关联到包含的配置文件元素。

属性：

- type: 指定配置文件和元数据的传输类型

### `<profile>` 标签
封装了一组主体权限。这是特定于由关联属性确定的独特节点实例。

属性：

- ns: 节点的命名空间
- node: 节点的名称

根据MAC，必须明确允许权限。此外，像许多其他MAC语言一样，虽然组合权限可能会重叠，但任何特定的拒绝权限将抑制任何类似适用的允许权限。也就是说，拒绝权限的优先级保守地优先于允许权限，避免潜在的PoLP漏洞。这种平坦权限的方法使用户能够为更大一组对象提供一般访问权限，同时同时撤销对较小子集的敏感对象的访问权限。尽管随后阻止了资格的递归，但转换随后简化了，防止了意外访问的潜力。

### `<metadata>` 标签
封装了任意元数据或约束。这可能包括适用于兄弟配置文件元素的特定于传输的权限详细信息。每个配置文件父元素只能有一个元数据元素。

属性：

- 待定义

鉴于桥接接口的使用案例，其中飞地的凭据可能用于跨多个传输或传输特定域的互连，可能需要用特定约束来限定某些配置文件序列，同时对每个飞地的单独配置文件这样做多次。这允许高级用户全面控制跨传输域的权限交集，同时保持安全权限的准确模型保真度。鉴于桥接接口的安全敏感性以及它们暴露的攻击面，确保桥接内的信息流控制对于安全操作是形式上可验证的至关重要。

## 权限
权限被定义为对象访问的规则和权限配置。由于对象可以按子系统类型分类，规则和相应的权限具有相同的结构。鉴于平均配置文件可能会引用具有多个权限的更多独特对象，而不是规则数量，因此选择规则/权限/对象的后续层次结构以最小化冗余。

规则类型 | 权限
--- | ---
actions | call, execute
services | reply, request
topics | publish, subscribe

每个子系统都与给定的规则类型相关联，而权限则在其各自的规则标签中作为属性表达。此序列中规则的唯一性或顺序不是必需的，因为这是通过转换模板考虑的。实际上，配置文件可能包含一个空的权限集；当节点可能不需要子系统权限时特别有用，但仍然必须为DDS中的发现目的提供身份。

每个规则都包括一系列权限适用的对象。对于某些安全传输，例如Secure DDS，还可以使用匹配表达式进一步扩展范围，特别是使用fnmatch支持的globbing模式，具体是POSIX标准中指定的那些。然而，在使用表达式匹配时要小心，如在关注部分进一步讨论。

支持基本的fnmatch风格模式：

| 模式            | 含义           |
| ------------- | ------------ |
| *             | 匹配所有内容       |
| ?             | 匹配任何单个字符     |
| \[sequence\]  | 匹配序列中的所有内容   |
| \[!sequence\] | 匹配不在序列中的任何内容 |


### `<topics>` 标签
一组具有指定权限的`<topic>`标签。

属性：

- publish: 是否允许在此一组主题上发布
  - 即节点是否可以成为主题发布者
  - 有效值为“ALLOW”或“DENY”
- subscribe: 是否允许在此一组主题上订阅
  - 即节点是否可以成为主题订阅者
  - 有效值为“ALLOW”或“DENY”

### `<services>` 标签
一组具有指定权限的`<service>`标签。

属性：

- request: 是否允许请求服务
  - 即节点是否可以成为服务客户端
  - 有效值为“ALLOW”或“DENY”
- reply: 是否允许回复服务请求
  - 即节点是否可以成为服务服务器
  - 有效值为“ALLOW”或“DENY”

### `<actions>` 标签
一组具有指定权限的`<action>`标签。

属性：

- call: 是否允许调用动作
  - 即节点是否可以成为动作客户端
  - 有效值为“ALLOW”或“DENY”
- execute: 是否允许执行动作
  - 即节点是否可以成为动作服务器
  - 有效值为“ALLOW”或“DENY”

## 模板化
要将SROS 2策略转换为针对目标访问控制传输的安全工件，可以使用XSLT模板执行此级别的文档转换。这可能包括针对目标传输的任何数量的优化或调整。例如，针对Secure DDS的目标管道阶段如下：

1.  指定一个以SROS 2策略为根的XML文档
2. 使用XInclude完全展开文档以引用外部元素
3. 使用等效的模式版本验证展开的文档
4. 在这一点上，文档树可能会或可能不会被修剪到特定配置文件
5. 然后使用转换模板转译有效的文档
6. 对于每个配置文件，将匹配的DDS授权追加到权限文档中
  - 将权限和命名空间重新映射为以DDS为中心的表示
  - 将具有匹配属性的权限合并以减少有效载荷大小
  - 修剪在同一权限中的重复对象以减少有效载荷大小
  - 按照使用DDS时定性优先级的顺序对权限进行排序，首先是拒绝，对象也按字母顺序排序，以提高可读性和更改差异

## 替代方案

本节列出了关于所提出的设计和所考虑的替代方案的关注，包括不同的标记语言和策略格式。

### YAML
YAML，“YAML Ain’t Markup Language”的递归缩写，最初在SROS [1]的第一个版本中被采用来指定访问控制策略。尽管SROS 1中使用的策略模型在语义上是等效的，但由于缺乏清晰的元素属性，YAML格式由于重复每个命名空间资源的权限而变得相当冗长。出于以下利弊权衡中的许多原因，我们决定在SROS 2中从YAML转向XML：

#### 优点

- 可读性：最小化行噪声 YAML的语法非常简洁，旨在用于人类可编辑的配置文件，使其易于阅读和手动编写。

- 数据模型：直观的解释 YAML具有非常简单的数据模型，通过键值对字典和列表形成树状结构，使其相当易于接近。


#### 缺点

- 解析性：隐式类型转换 YAML作为一种数据序列化语言，可能会在可能的情况下尝试类型转换。然而，这并不总是产生预期的效果，可能导致意外的行为。布尔值与字符串的解析是模糊性的一个显著例子。

- 解释器：验证和转换 尽管YAML支持许多编程语言，但YAML本身并不提供模式来强制执行文档结构。 每个解释器实现都必须重复验证，这使得它与所使用的编程语言无关。 同样，将策略转换为传输安全工件的转换在不同实现之间也不太通用。
    
- 可组合性：配置文件的重用 YAML通过锚点、别名和扩展支持一定程度的可组合性，允许文档更加DRY（Don't Repeat Yourself），但这些并不扩展到单独的文件或外部资源。

- 表达性：简洁的表示 鉴于YAML的固有数据模型，其表达能力相当有限，需要要么使用冗长的文件结构，要么使用不直观的选项来实现类似的访问控制配置。

### 自定义
作为选择现有标记格式的替代方案，可以为ROS 2定义自己的正式语言，使用自定义文件语法来表达访问控制权限。AppArmor中使用的政策语言就是一个例子。尽管这种方法在简洁地表达配置文件权限的同时提供了灵活性，以最小化一般语法开销，但由于以下利弊权衡中的许多原因，这种方法并未被采纳：

#### 优点

- 表达性：简洁的表示 完全控制语法和解释，允许针对SROS策略表示进行领域特定的优化。

#### 缺点

- 解释器：验证和转换 对自定义策略格式进行解析和解释的规范和实现将被视为一项任务。 每个解释器实现都必须重复验证，这使得它与所使用的编程语言无关。 同样，将策略转换为传输安全工件的转换在不同实现之间也不太通用。
    
- 正确性：策略保持合理 在多种编程语言中维护和同步解析支持可能会影响策略的合理性。

### ComArmor
ComArmor ，SROS 2现在使用的现有XML ROS 2策略格式的前身和灵感来源，本身受到AppArmor策略语言的启发。ComArmor通过使用策略/配置文件/权限原语的嵌套树结构来促进组合。与AppArmor一样，它还支持配置文件的嵌套，即导入子配置文件到父配置文件中。虽然这在嵌套导入子配置文件层次结构的灵活性方面大大扩展了功能，但它也增加了将策略转换为安全传输工件时的转换过程的复杂性。

为了在简单性和灵活性之间取得平衡，SROS 2选择了单级配置文件的平面序列，使策略格式既可以作为更高级策略语言和工具的中间表示，例如XACML或Keymint，同时仍然简洁地表达ROS概念以供一般使用。

## 关注点

### 权限分离
ROS 2子系统，如主题、服务、动作和参数，最终必须映射到传输层接口，例如DDS主题，以充分执行所需的访问控制策略，以保护ROS应用层。然而，子系统映射和权限分离之间的任何差异都可能降低安全性。

### 关心点分离
像DDS这样的中间件传输提供了大量功能和选项，如QoS和安全性。在决定从配置角度公开哪些功能时，划定界限可能很棘手。尽管如此，ROS 2的目标之一是尽可能保持对传输的不可知性。

### 组合性
ROS 2允许在运行时重新映射许多命名空间子系统，例如当重用启动文件来编排更大的应用程序时。虽然从静态配置的权限中期望这种动态灵活性可能是不合理的，但应该能够从组合的启动文件和类似的编排中推断出必要的能力。

## 参考文献
- [SROS1: Using and Developing Secure ROS1 Systems](https://doi.org/10.1007/978-3-319-91590-6_11)
```
@inbook{White2019,
	title     = {SROS1: Using and Developing Secure ROS1 Systems},
	author    = {White, Ruffin and Caiazza, Gianluca and Christensen, Henrik and Cortesi, Agostino},
	year      = 2019,
	booktitle = {Robot Operating System (ROS): The Complete Reference (Volume 3)},
	doi       = {10.1007/978-3-319-91590-6_11},
	isbn      = {978-3-319-91590-6},
	url       = {https://doi.org/10.1007/978-3-319-91590-6_11}}

```
- [Procedurally Provisioned Access Control for Robotic Systems](https://doi.org/10.1109/IROS.2018.8594462)
```
@inproceedings{White2018,
	title     = {Procedurally Provisioned Access Control for Robotic Systems},
	author    = {White, Ruffin and Caiazza, Gianluca and Christensen, Henrik and Cortesi, Agostino},
	year      = 2018,
	booktitle = {2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
	doi       = {10.1109/IROS.2018.8594462},
	issn      = {2153-0866},
	url       = {https://doi.org/10.1109/IROS.2018.8594462}}
```