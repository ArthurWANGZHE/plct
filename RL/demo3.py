import mujoco
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import random
import gymnasium as gym
from gymnasium.wrappers import RecordVideo

# 设置随机数种子
SEED = 42
random.seed(SEED)
np.random.seed(SEED)
torch.manual_seed(SEED)

# 定义 Q 网络
class QNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, action_dim)
        )

    def forward(self, x):
        return self.fc(x)

# 定义 DQN Agent
class DQNAgent:
    def __init__(self, state_dim, action_dim):
        self.q_net = QNetwork(state_dim, action_dim)  # 当前网络
        self.target_net = QNetwork(state_dim, action_dim)  # 目标网络
        self.target_net.load_state_dict(self.q_net.state_dict())  # 初始化目标网络
        self.optimizer = optim.Adam(self.q_net.parameters(), lr=1e-3)
        self.replay_buffer = deque(maxlen=10000)  # 经验回放缓冲区
        self.batch_size = 64
        self.gamma = 0.99
        self.epsilon = 1.0  # 探索概率
        self.epsilon_end = 0.01
        self.epsilon_decay = 0.995
        self.update_target_freq = 100  # 目标网络更新频率

    def choose_action(self, state):
        if np.random.rand() < self.epsilon:
            action_index = np.random.randint(0, 2)  # 随机选择动作索引
        else:
            state_tensor = torch.FloatTensor(state)
            q_values = self.q_net(state_tensor)
            action_index = q_values.cpu().detach().numpy().argmax()
        return np.array([action_index])  # 返回一个数组

    def store_experience(self, state, action, reward, next_state, done):
        self.replay_buffer.append((state, action, reward, next_state, done))

    def train(self):
        if len(self.replay_buffer) < self.batch_size:
            return

        batch = random.sample(self.replay_buffer, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)

        states = torch.FloatTensor(np.array(states))
        actions = torch.LongTensor(actions)
        rewards = torch.FloatTensor(rewards)
        next_states = torch.FloatTensor(np.array(next_states))
        dones = torch.FloatTensor(dones)

        current_q = self.q_net(states).gather(1, actions.unsqueeze(1)).squeeze()
        with torch.no_grad():
            next_q = self.target_net(next_states).max(1)[0]
            target_q = rewards + self.gamma * next_q * (1 - dones)

        loss = nn.MSELoss()(current_q, target_q)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        if self.epsilon > self.epsilon_end:
            self.epsilon *= self.epsilon_decay

        if len(self.replay_buffer) % self.update_target_freq == 0:
            self.target_net.load_state_dict(self.q_net.state_dict())

# 创建 MuJoCo 环境
xml = """
<mujoco>
  <option integrator="RK4" timestep="0.001" gravity="0 0 -9.81"/>
  <worldbody>
    <light name="top" pos="0 0 1"/>
    <body name="cart" pos="0 0 0.1">
      <geom name="cart" type="box" size="0.1 0.2 0.1" rgba="0.5 0.5 0.5 1"/>
      <joint name="slider" type="slide" axis="1 0 0" pos="0 0 0"/>
      <body name="pole" pos="0 0 0.2">
        <geom name="pole" type="capsule" size="0.01 0.5" rgba="0.8 0.2 0.2 1"/>
        <joint name="hinge" type="hinge" axis="0 1 0" pos="0 0 0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor" joint="slider" gear="1"/>
  </actuator>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model)

# 定义环境
class InvertedPendulumEnv(gym.Env):
    def __init__(self):
        self.model = model
        self.data = data
        self.renderer = renderer
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[1] = np.random.uniform(-0.1, 0.1)  # 初始摆杆角度
        self.data.qvel[1] = np.random.uniform(-0.1, 0.1)  # 初始摆杆角速度
        return self._get_obs()

    def step(self, action):
        self.data.ctrl[0] = action[0]  # 应用控制信号
        mujoco.mj_step(self.model, self.data)
        reward = -np.square(self.data.qpos[1]) - 0.1 * np.square(self.data.qvel[1])
        done = np.abs(self.data.qpos[1]) > np.pi / 2  # 摆杆倾倒
        return self._get_obs(), reward, done, {}

    def _get_obs(self):
        return np.concatenate([self.data.qpos, self.data.qvel])

    def render(self, mode='human'):
        self.renderer.update_scene(self.data)
        return self.renderer.render()

# 初始化环境和 Agent
env = InvertedPendulumEnv()
state_dim = env.observation_space.shape[0]
action_dim = env.action_space.shape[0]
agent = DQNAgent(state_dim, action_dim)

# 训练过程
for episode in range(1000):
    state = env.reset()
    total_reward = 0
    while True:
        action = agent.choose_action(state)
        next_state, reward, done, _ = env.step(action)
        agent.store_experience(state, action, reward, next_state, done)
        agent.train()
        state = next_state
        total_reward += reward
        if done:
            break
    print(f"Episode: {episode}, Total Reward: {total_reward}")

# 测试训练结果
env = RecordVideo(env, "./videos")
state = env.reset()
while True:
    action = agent.choose_action(state)
    next_state, reward, done, _ = env.step(action)
    state = next_state
    if done:
        break
env.close()