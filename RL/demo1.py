import mujoco
import mediapy as media

# 定义 MJCF 模型
xml = """
<mujoco>
    <option timestep="0.001" gravity="0 0 -9.81"/>
    <asset>
        <texture name="grid" type="2d" builtin="checker" rgb1="0.1 0.2 0.3" rgb2="0.2 0.3 0.4" width="300" height="300"/>
        <material name="grid_material" texture="grid" texrepeat="8 8" reflectance="0.2"/>
    </asset>
    <worldbody>
        <light name="top" pos="0 0 1"/>
        <geom name="floor" type="plane" size="1 1 0.1" rgba="0.5 0.5 0.5 1" material="grid_material"/>
        <body name="box_and_sphere" pos="0 0 1">
            <freejoint/>
            <geom name="box" type="box" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
            <geom name="sphere" type="sphere" size="0.05" rgba="0 1 0 1" pos="0.1 0.1 0.1"/>
        </body>
    </worldbody>
</mujoco>
"""

# 加载模型
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# 创建渲染器
renderer = mujoco.Renderer(model)

# 仿真参数
duration = 5.0  # 仿真时长（秒）
framerate = 60  # 帧率（Hz）
frames = []

# 仿真循环
mujoco.mj_resetData(model, data)
while data.time < duration:
    mujoco.mj_step(model, data)  # 步进仿真
    if len(frames) < data.time * framerate:
        renderer.update_scene(data)  # 更新场景
        pixels = renderer.render()  # 渲染图像
        frames.append(pixels)

# 显示动画
media.show_video(frames, fps=framerate)