## agibot
>参考文档: 
> 
> https://github.com/AgibotTech/agibot_x1_infer
> 
> https://github.com/microsoft/onnxruntime
## 安装步骤
1. 更新cmake
2. 安装 ONNX Runtime
3. 安装依赖
4. 安装agibot

### 1. 更新cmake
从官网下载 [cmake](https://cmake.org/download/) 安装

### 2. 安装 ONNX Runtime
安装相关依赖后git clone下来仓库进行编译安装
```angular2html
sudo apt update
sudo apt install -y build-essential cmake git libprotobuf-dev protobuf-compiler

git clone --recursive https://github.com/microsoft/onnxruntime

cd onnxruntime
./build.sh --config Release --build_shared_lib --parallel

cd build/Linux/Release/
sudo make install
```

直接编译出现如下报错:
```
/home/arthur/onnxruntime/onnxruntime/core/providers/cpu/math/element_wise_ops.cc:873:85: error: type/value mismatch at argument 1 in template parameter list for ‘template<class OtherDerived> const Eigen::CwiseBinaryOp<Eigen::internal::scalar_max_op<typename Eigen::internal::traits<T>::Scalar, typename Eigen::internal::traits<OtherDerived>::Scalar>, const Derived, const OtherDerived> Eigen::ArrayBase<Derived>::max(const Eigen::ArrayBase<OtherDerived>&) const [with OtherDerived = OtherDerived; Derived = Eigen::ArrayWrapper<Eigen::Map<const Eigen::Matrix<long unsigned int, -1, 1, 0, -1, 1>, 0, Eigen::Stride<0, 0> > >]’
  873 | per_iter_bh.EigenInput0<T>().array().template max<Eigen::PropagateNaN>(
      | ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^

  874 |     per_iter_bh.EigenInput1<T>().array());
      |     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~                              

/home/arthur/onnxruntime/onnxruntime/core/providers/cpu/math/element_wise_ops.cc:873:85: note:   expected a type, got ‘Eigen::PropagateNaN’
```
报错提示在文件`onnxruntime/onnxruntime/core/providers/cpu/math/element_wise_ops.cc使用了Eigen::PropagateNaN`作为模板参数的方式不正确

修改方法 删除所有 `<eigen::PropagateNaN> `后进行编译

![img_2.png](attachment/img_2.png)

### 3. 安装依赖
```
sudo apt install jstest-gtk \
                 ros-humble-xacro \
                 ros-humble-gazebo-ros-pkgs \
                 ros-humble-gazebo-ros2-control \
                 ros-humble-joint-state-publisher \
                 ros-humble-joint-state-broadcaster \
```

### 4. 安装Agibot
根据文档教程提示安装
```
source /opt/ros/humble/setup.bash
source url.bashrc

# Build
./build.sh $DOWNLOAD_FLAGS

# Test
./test.sh $DOWNLOAD_FLAGS
```

出现如下报错
```
e/joy_stick_module/joy.cc:30:
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.h:44:8: error: ‘vector’ in namespace ‘std’ does not name a template type
   44 |   std::vector<double> axis;
      |        ^~~~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.h:38:1: note: ‘std::vector’ is defined in header ‘<vector>’; did you forget to ‘#include <vector>’?
   37 | #include <thread>
  +++ |+#include <vector>
   38 | 
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.h:45:8: error: ‘vector’ in namespace ‘std’ does not name a template type
   45 |   std::vector<int32_t> buttons;
      |        ^~~~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.h:45:3: note: ‘std::vector’ is defined in header ‘<vector>’; did you forget to ‘#include <vector>’?
   45 |   std::vector<int32_t> buttons;
      |   ^~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc: In member function ‘bool xyber_x1_infer::joy_stick_module::Joy::handleJoyAxis(const SDL_Event&)’:
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:145:32: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘axis’
  145 |   if (e.jaxis.axis >= joy_msg_.axis.size()) {
      |                                ^~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:149:36: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘axis’
  149 |   float last_axis_value = joy_msg_.axis.at(e.jaxis.axis);
      |                                    ^~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:150:12: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘axis’
  150 |   joy_msg_.axis.at(e.jaxis.axis) = convertRawAxisValueToROS(e.jaxis.value);
      |            ^~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:151:35: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘axis’
  151 |   if (last_axis_value != joy_msg_.axis.at(e.jaxis.axis)) {
      |                                   ^~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc: In member function ‘bool xyber_x1_infer::joy_stick_module::Joy::handleJoyButtonDown(const SDL_Event&)’:
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:172:36: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘buttons’
  172 |   if (e.jbutton.button >= joy_msg_.buttons.size()) {
      |                                    ^~~~~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:178:14: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘buttons’
  178 |     joy_msg_.buttons.at(e.jbutton.button) = 1 - joy_msg_.buttons.at(e.jbutton.button);
      |              ^~~~~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:178:58: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘buttons’
  178 |     joy_msg_.buttons.at(e.jbutton.button) = 1 - joy_msg_.buttons.at(e.jbutton.button);
      |                                                          ^~~~~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:180:14: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘buttons’
  180 |     joy_msg_.buttons.at(e.jbutton.button) = 1;
      |              ^~~~~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc: In member function ‘bool xyber_x1_infer::joy_stick_module::Joy::handleJoyButtonUp(const SDL_Event&)’:
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:191:36: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘buttons’
  191 |   if (e.jbutton.button >= joy_msg_.buttons.size()) {
      |                                    ^~~~~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:196:14: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘buttons’
  196 |     joy_msg_.buttons.at(e.jbutton.button) = 0;
      |              ^~~~~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc: In member function ‘bool xyber_x1_infer::joy_stick_module::Joy::handleJoyHatMotion(const SDL_Event&)’:
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:223:42: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘axis’
  223 |   if ((axes_start_index + 1) >= joy_msg_.axis.size()) {
      |                                          ^~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:229:14: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘axis’
  229 |     joy_msg_.axis.at(axes_start_index) = 1.0;
      |              ^~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:232:14: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘axis’
  232 |     joy_msg_.axis.at(axes_start_index) = -1.0;
      |              ^~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:235:14: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘axis’
  235 |     joy_msg_.axis.at(axes_start_index + 1) = 1.0;
      |              ^~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:238:14: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘axis’
  238 |     joy_msg_.axis.at(axes_start_index + 1) = -1.0;
      |              ^~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:241:14: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘axis’
  241 |     joy_msg_.axis.at(axes_start_index) = 0.0;
      |              ^~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:242:14: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘axis’
  242 |     joy_msg_.axis.at(axes_start_index + 1) = 0.0;
      |              ^~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc: In member function ‘void xyber_x1_infer::joy_stick_module::Joy::handleJoyDeviceAdded(const SDL_Event&)’:
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:304:12: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘buttons’
  304 |   joy_msg_.buttons.resize(num_buttons);
      |            ^~~~~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:320:12: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘axis’
  320 |   joy_msg_.axis.resize(num_axes + num_hats * 2);
      |            ^~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:326:16: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘axis’
  326 |       joy_msg_.axis.at(i) = convertRawAxisValueToROS(state);
      |                ^~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc: In member function ‘void xyber_x1_infer::joy_stick_module::Joy::handleJoyDeviceRemoved(const SDL_Event&)’:
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:349:12: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘buttons’
  349 |   joy_msg_.buttons.resize(0);
      |            ^~~~~~~
/home/arthur/agibot_x1_infer-main/src/module/joy_stick_module/joy.cc:350:12: error: ‘struct xyber_x1_infer::joy_stick_module::JoyStruct’ has no member named ‘axis’
  350 |   joy_msg_.axis.resize(0);
      |            ^~~~
```
报错提示缺少头文件

修改方法在文件中增加头文件 `#include<vector>`

![img.png](attachment/img.png)

