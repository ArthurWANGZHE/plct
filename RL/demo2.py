
# PD 控制器
def pd_controller(model, data):
    target_angle = 0  # 控制目标：保持摆杆垂直
    kp = 100  # 比例增益
    kd = 10   # 微分增益
    error = data.qpos[1] - target_angle  # 角度误差
    error_dot = data.qvel[1]  # 角速度误差
    control_signal = -kp * error - kd * error_dot  # 控制信号
    data.ctrl[0] = control_signal  # 应用控制信号

# 仿真参数
duration = 5.0  # 仿真时长（秒）
framerate = 60  # 帧率（Hz）
frames = []

# 仿真循环
mujoco.mj_resetData(model, data)
while data.time < duration:
    pd_controller(model, data)  # 调用 PD 控制器
    mujoco.mj_step(model, data)  # 步进仿真
    if len(frames) < data.time * framerate:
        renderer.update_scene(data)  # 更新场景
        pixels = renderer.render()  # 渲染图像
        frames.append(pixels)

# 显示动画
media.show_video(frames, fps=framerate)
