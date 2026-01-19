# MotionFlow

基于 C++ 的人形机器人动作播放与仿真引擎，支持 URDF 模型加载、动作文件播放，以及与 gentle-humanoid 仿真环境的实时流式传输。

## 功能特性

- 🤖 **URDF 机器人加载**: 支持 URDF 格式机器人模型，包括 STL 网格渲染
- 🎬 **动作播放**: 支持 `.motion` 和 `.npz` 格式动作文件，可控制播放速度、循环、进度拖动
- 📡 **实时流式传输**: 通过 UDP 将动作数据实时发送到 gentle-humanoid 仿真环境
- 🖥️ **进程管理**: 一键启动/停止 sim2sim 和 deploy 子进程
- 🎮 **交互式 GUI**: 完整的控制面板，支持动作选择、关节调试、流式传输配置

## 技术栈

- **渲染框架**: raylib 5.0
- **GUI**: raygui
- **构建系统**: CMake 3.20+
- **语言标准**: C++17
- **网络**: POSIX Socket (UDP)
- **进程管理**: POSIX fork/exec

## 构建

```bash
# 创建构建目录
mkdir build && cd build

# 配置（首次运行会自动下载 raylib 依赖）
cmake ..

# 编译
cmake --build . -j$(nproc)

# 运行
./bin/motionflow
```

## 项目结构

```
motionflow/
├── CMakeLists.txt                # 主 CMake 配置
├── cmake/
│   └── FetchDependencies.cmake   # 依赖管理（自动下载 raylib）
├── src/
│   ├── main.cpp                  # 入口点 & 演示程序
│   ├── core/
│   │   ├── Application.hpp       # 应用基类（窗口、主循环）
│   │   ├── OrbitCamera.hpp       # 轨道相机
│   │   ├── FlyCamera.hpp         # 飞行相机（UE4 风格）
│   │   └── ProcessManager.hpp    # 子进程管理器
│   ├── scene/
│   │   ├── Transform.hpp         # 变换组件
│   │   ├── Entity.hpp            # 场景实体
│   │   └── Scene.hpp             # 场景管理器
│   ├── robot/
│   │   ├── RobotEntity.hpp/cpp   # 机器人实体
│   │   ├── URDFParser.hpp/cpp    # URDF 解析器
│   │   └── MeshLoader.hpp/cpp    # STL 网格加载器
│   ├── motion/
│   │   ├── MotionData.hpp/cpp    # 动作数据结构
│   │   ├── MotionPlayer.hpp/cpp  # 动作播放器
│   │   └── NpzReader.hpp         # NPZ 文件读取器
│   ├── network/
│   │   ├── UDPStreamer.hpp/cpp   # UDP 流式传输
│   │   └── JointMapper.hpp       # 关节名称映射
│   └── gui/
│       └── GuiPanel.hpp/cpp      # GUI 控制面板
├── assets/
│   ├── G1_jy/                    # Unitree G1 机器人模型
│   │   ├── G1_jy.urdf
│   │   └── meshes/*.STL
│   └── motions/                  # 动作文件
│       ├── *.motion              # .motion 格式
│       └── *.npz                 # .npz 格式
├── gentle-humanoid/              # gentle-humanoid 子模块
├── scripts/                      # 工具脚本
│   ├── bvh_to_npz.py
│   ├── convert_gmr_to_motion.py
│   └── export_motion.py
└── README.md
```

## 操作说明

### 通用快捷键
| 按键 | 功能 |
|------|------|
| `Tab` | 切换相机模式（轨道/飞行） |
| `M` | 切换动作播放器/手动模式 |
| `X` | 显示/隐藏关节坐标轴 |
| `G` | 显示/隐藏网格 |
| `ESC` | 退出程序 |

### 轨道相机 (Orbit Camera)
| 操作 | 功能 |
|------|------|
| 右键拖拽 | 旋转视角 |
| 中键拖拽 | 平移视角 |
| 滚轮 | 缩放 |

### 飞行相机 (Fly Camera - UE4 风格)
| 操作 | 功能 |
|------|------|
| 按住右键 | 激活控制 |
| W/A/S/D | 前/左/后/右移动 |
| Q/E | 下降/上升 |
| Space | 上升 |
| Shift | 加速移动 |
| Ctrl | 减速移动 |
| 滚轮 | 调整移动速度 |

### 动作播放器快捷键
| 按键 | 功能 |
|------|------|
| `Space` | 播放/暂停 |
| `S` | 停止（回到开始） |
| `L` | 切换循环播放 |
| `↑/↓` | 调整播放速度 |
| `←/→` | 逐帧前进/后退 |
| `P` | 切换根位置应用 |
| `O` | 切换根旋转应用 |

### 手动模式快捷键
| 按键 | 功能 |
|------|------|
| `Space` | 开始/停止简单动画 |
| `[/]` | 选择上/下一个关节 |
| `←/→` | 调整关节角度 |
| `R` | 重置所有关节 |

## 核心模块

### RobotEntity
从 URDF 文件加载机器人模型：
- 解析 URDF XML 结构
- 加载 STL 网格文件
- 支持关节位置控制
- 支持关节限位

### MotionPlayer
动作播放器，将动作数据应用到机器人：
- 支持 `.motion` 和 `.npz` 格式
- 播放控制（播放、暂停、停止、跳转）
- 循环播放、速度控制
- 可选应用根节点位置/旋转
- 相对位置模式

### UDPStreamer
UDP 流式传输客户端：
- 将实时动作数据发送到 gentle-humanoid
- 数据包格式：魔数 + 时间戳 + 29关节位置 + 根节点四元数 + 根节点位置
- 默认端口: 28563

### JointMapper
关节名称映射器：
- 将 URDF 关节名 (`leg_l1_joint` 等) 映射到 gentle-humanoid 关节名 (`left_hip_pitch_joint` 等)
- 支持 29 个关节的完整映射

### ProcessManager
子进程管理器：
- 启动/停止 `sim2sim.py` 和 `deploy.py`
- 使用 double fork 避免僵尸进程
- 支持优雅关闭和强制终止

### GuiPanel
交互式 GUI 控制面板：
- 动作文件选择（下拉菜单）
- 播放控制（播放/暂停/停止/进度条）
- 关节调试（选择关节、滑块调整）
- 流式传输配置（连接/断开、状态显示）
- 仿真进程控制（一键启动/停止）

## 动作文件格式

### .motion 格式
JSON 格式的动作数据文件：
```json
{
  "fps": 50,
  "joint_names": ["leg_l1_joint", "leg_l2_joint", ...],
  "frames": [
    {
      "joint_positions": [0.0, 0.1, ...],
      "root_position": [0.0, 0.0, 0.8],
      "root_rotation": [0.0, 0.0, 0.0, 1.0]
    },
    ...
  ]
}
```

### .npz 格式
NumPy 压缩格式，包含以下数组：
- `fps`: 帧率
- `joint_names`: 关节名称列表
- `joint_positions`: 关节位置 [N, num_joints]
- `root_positions`: 根位置 [N, 3] (可选)
- `root_rotations`: 根旋转四元数 [N, 4] (可选)

## gentle-humanoid 集成

MotionFlow 可以与 gentle-humanoid 仿真环境实时交互：

1. **启动仿真**: 点击 GUI 中的 "Start All" 按钮，自动启动 `sim2sim.py` 和 `deploy.py`
2. **连接流式传输**: 仿真启动后自动连接或手动点击 "Connect" 按钮
3. **播放动作**: 播放动作文件，动作数据会实时流式传输到仿真环境
4. **停止仿真**: 点击 "Stop All" 按钮，停止所有进程

## 开发进度

- [x] Step 1: 项目搭建 + 基础渲染窗口
- [x] Step 2: 相机系统 + 场景图
- [x] Step 3: URDF 模型加载 + STL 渲染
- [x] Step 4: 动作数据格式 + 播放器
- [x] Step 5: GUI 控制面板
- [x] Step 6: UDP 流式传输 + 关节映射
- [x] Step 7: gentle-humanoid 进程管理
- [ ] Step 8: 动画混合与过渡
- [ ] Step 9: 实时策略推理集成

## 许可证

MIT License
