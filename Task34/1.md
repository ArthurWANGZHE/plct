# IDL - 接口定义和语言映射

本文描述了使用接口定义语言（IDL）的一个子集来定义组件之间的接口。

## 作者：Dirk Thomas

## 撰写日期：2019-03

## 最后修改：2020-07

## 范围
本文确定了可以使用的接口定义语言（IDL）的一个子集，用于描述组件之间的接口。进一步描述了如何使用这些接口在C、C++和Python中生成代码。

## 支持的IDL子集
ROS 2支持OMG IDL 4.2规范的一个子集。下面未列出的内容可能目前不支持（例如 enums）。

## 词法约定
### 7.2.2 注释
支持行注释（`//`）以及块注释（`/* ... */`）。

### 7.2.3 标识符
标识符必须以ASCII字母字符开始，后跟任意数量的ASCII字母字符、数字和下划线( `_`)字符。

### 7.2.6 字面量
支持以下所有字面量：

- 整数
- 字符
- 字符串
- 浮点数
- 定点数

## 预处理
目前，在读取`.idl`文件时没有进行预处理。

## 构建块
### 7.4.6.4.1.4 导入
必须使用导入来引用声明了在此`.idl`文件中使用的其他`.idl`文件中的数据类型。

### 7.4.1.4.2 模块
每个模块必须包含至少一个定义，它们可以嵌套。对于ROS接口，第一级模块通常代表包名，第二级模块区分接口类型（msg, srv, action）。

### 7.4.1.4.3 常量
#### 7.4.1.4.4.4.1 结构体
结构体必须至少包含一个成员。

## 基本类型
### 7.4.1.4.4.2.1 整数类型
IDL类型 | 值范围
--- | ---
short | -2^15 … 2^15 - 1
unsigned short | 0 … 2^16 - 1
long | -2^31 … 2^31 - 1
unsigned long | 0 … 2^32 - 1
long long | -2^63 … 2^63 - 1
unsigned long long | 0 … 2^64 - 1

### 7.4.1.4.4.2.2 浮点数类型
IDL类型 | 格式
--- | ---
float | IEEE单精度浮点数
double | IEEE双精度浮点数
long double | IEEE扩展双精度浮点数

### 7.4.1.4.4.2.3 字符类型
IDL类型 | 值范围
--- | ---
char | 8位单字节字符，数值介于0和255之间（见7.2.6.2.1）
该类型可以存储任何字节导向的代码集的单字节字符，或者在数组中使用时，可以编码多字节代码集的多字节字符。

### 7.4.1.4.4.2.4 宽字符类型
IDL类型 | 值范围
--- | ---
wchar | 16位宽字符
虽然IDL规范仅将wchar的大小定义为实现依赖，但DDS-XTypes规范1.2将其定义为16位。

### 7.4.1.4.4.2.5 布尔类型
IDL类型 | 值范围
--- | ---
boolean | TRUE或FALSE中的一个值

### 7.4.1.4.4.2.6 八位类型
IDL类型 | 值范围
--- | ---
octet | 不透明的8位

### 7.4.13.4.4 8位整数
IDL类型 | 值范围
--- | ---
int8 | -2^7 … 2^7 - 1
uint8 | 0 … 2^8 - 1

### 7.4.13.4.5 显式命名的整数类型
IDL类型 | 等效IDL类型
--- | ---
int16 | short
uint16 | unsigned short
int32 | long
uint32 | unsigned long
int64 | long long
uint64 | unsigned long long

## 模板类型
### 7.4.1.4.4.3.1 序列
| IDL类型                  | 值范围                                      |
| ---------------------- | ---------------------------------------- |
| sequence<type_spec>    | 特定type_spec的项目的序列序列是无界的，没有指定最大大小         |
| sequence<type_spec, N> | 指定type_spec的最多N个项目的序列<br>序列是有界的，包含0到N个项目 |

### 7.4.1.4.4.3.2 字符串
IDL类型 | 值范围
--- | ---
string | 除了空值之外的char序列

### 7.4.1.4.4.3.3 宽字符串
IDL类型 | 值范围
--- | ---
wstring | 除了空值之外的wchar序列

## 构造类型
### 7.4.1.4.4.4.1 结构体
结构体是至少一个成员的组合。

### 7.4.1.4.4.4.3 枚举
枚举类型由有序的枚举器列表组成。

### 7.4.1.4.4.5 数组
通过每个项目的类型和每个维度的显式大小定义多维、固定大小的数组。目前只支持一维、固定大小的数组。

## 注解
支持任意注解的语法。每种注解类型的处理方式取决于下文描述的代码生成。

## 代码生成
    待办事项：所有这些

## 约束检查
只有当消息被发布时，才保证生成的消息代码强制执行约束。例如，当一个字段被限制为固定大小的数组时，分配一个错误大小的数组可能不会出错。这可能因客户端库的实现、语言特性和性能成本而在不同客户端库之间不一致。将来，任何客户端库都可能在字段设置或修改时添加额外的约束检查。使用生成的消息代码的用户应假设约束可能随时被强制执行，以适应此类更改。

## 类型映射
下表定义了IDL类型如何映射到以下编程语言：
- C11 (或更高)
- C++11 (或更高)
- Python 3 (或更高)

| IDL类型       | C11           | Python 3            | C++11         |
| ----------- | ------------- | ------------------- | ------------- |
| float       | float         | float               | float         |
| double      | double        | float               | double        |
| long double | long double   | float               | long double   |
| char        | unsigned char | str with length 1   | unsigned char |
| wchar       | char16_t      | str with length 1   | char16_t      |
| boolean     | _Bool         | bool                | bool          |
| octet       | unsigned char | bytes with length 1 | std::byte     |
| int8        | int8_t        | int                 | int8_t        |
| uint8       | uint8_t       | int                 | uint8_t       |
| int16       | int16_t       | int                 | int16_t       |
| uint16      | uint16_t      | int                 | uint16_t      |
| int32       | int32_t       | int                 | int32_t       |
| uint32      | uint32_t      | int                 | uint32_t      |
| int64       | int64_t       | int                 | int64_t       |
| uint64      | uint64_t      | int                 | uint64_t      |

1. 如果std::byte不可用，则使用unsigned char代替。
2. 用户应研究他们选择的中间件和平台对long double的支持。例如，在使用Visual Studio时，它只有64位。

下表定义了IDL模板和构造类型如何映射到编程语言（除非下面另有说明）。当指定T是上述类型之一或IDL结构体时。N是有界类型的上限。

| IDL类型 | C类型 | C++类型 | Python类型 |
| --- | --- | --- | --- |
| T[N] | T[N] | std::array<T, N> | list |
| sequence<T> | struct {size_t, T * } | std::vector<T> | list |
| sequence<T, N> | struct {size_t, T * }, size_t N | std::vector<T> | list |
| string | char * | std::string | str |
| string<N> | char * | std::string | str |
| wstring | char16_t * | std::u16string | str |
| wstring<N> | char16_t * | std::u16string | str |


以下这些数组列表有特殊的映射关系, 如果单元格为空，则使用默认映射。

|IDL类型|C类型|C++类型|Python类型|
|---|---|---|---|
|T[N]|对于数值类型 T:<br>float, double,<br>int8, uint8,<br>int16, uint16,<br>int32, uint32,<br>int64, uint64|-|-|
|sequence<T><br>sequence<T, N>|对于数值类型 T:<br>float, double,<br>int8, uint8,<br>int16, uint16,<br>int32, uint32,<br>int64, uint64|-|-|
|octet[N]|-|-|bytes|
|sequence<octet>|-|-|bytes|
|sequence<octet, N>|-|-|bytes|

## 标准化注释

### 8.3.2.1 @key 注释

ROS 2 节点通过主题交换某个现实世界对象的信息。每个主题中的消息被称为数据样本，代表对象状态的更新。

主题实例是一种通过同一资源（即主题）复用传输多个相同逻辑类型对象更新的方式。在带键的主题中，每条消息都与一个主题实例相关联，每个主题实例都由一个唯一键标识。从这个意义上说，键可以被视为数据库的主键。此键允许节点更新相同类型的不同状态。因此，所有共享相同键值的数据样本都引用相同的对象。

`@key `注释允许指示数据成员是键的一部分，可以有零个或多个键字段，并且可以应用于各种类型的结构字段。以下示例展示了如何使用 IDL 格式定义带键的消息：



```plaintext
# KeyedMsgName.idl
module package_name {
  module msg {
    struct KeyedMsgName {
      @key long member1;
      string member2;
    };
  };
};
```

>目前，支持 @key 注释的消息接口格式是 .idl。因此，.msg 和 .srv 尚不支持 @key 注释。有关它们之间等效性和转换的更多信息，请参阅转换为 IDL。

包括原始类型、序列、字符串和结构在内的通用 idl 类型可以被注释为键。以下是一些示例：

|类型|键成员|
|---|---|
|struct NoKey|boolean member1;<br>long member2;<br>long member3;|
|struct SimpleKey|@key long member1;<br>long member2;|
|struct ArrayKey|@key long member1[3];|
|struct StringKey|@key string member1;<br>long member2|
|struct NestedNoKey|SimpleKey member1;<br>long member2;|
|struct NestedKey|@key SimpleKey member1;<br>long member2;|
|struct NestedKey2|@key NoKey member1;<br>long member2;|
|struct ComplexNestedKey|@key NestedNoKey member1;<br>long member2;|

### 8.3.3.1 @default 注释

默认值用于初始化结构成员。

### 8.3.5.1 @verbatim 注释

当值注释作为语言参数传递时，文本参数被用作所有支持的编程语言中注释元素的 docblock。

复制再试一次分享