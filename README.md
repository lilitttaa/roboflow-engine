# MotionFlow

基于 C++ 的骨骼动画引擎。

## 技术栈

- **渲染框架**: raylib 5.0
- **模型加载**: Assimp (计划中)
- **骨骼动画**: ozz-animation (计划中)
- **构建系统**: CMake 3.20+
- **语言标准**: C++17

## 构建

```bash
# 创建构建目录
mkdir build && cd build

# 配置（首次运行会自动下载依赖）
cmake ..

# 编译
cmake --build . -j$(nproc)

# 运行
./bin/motionflow
```

## 项目结构

```
motionflow/
├── CMakeLists.txt              # 主 CMake 配置
├── cmake/
│   └── FetchDependencies.cmake # 依赖管理（自动下载raylib等）
├── src/
│   ├── main.cpp                # 入口点 & 演示程序
│   ├── core/
│   │   ├── Application.hpp/cpp # 应用基类（窗口、主循环）
│   │   ├── OrbitCamera.hpp     # 轨道相机
│   │   └── FlyCamera.hpp       # 飞行相机（UE4风格）
│   └── scene/
│       ├── Transform.hpp       # 变换组件（位置/旋转/缩放）
│       ├── Entity.hpp          # 场景实体 + 层级结构
│       └── Scene.hpp           # 场景管理器
├── assets/                     # 资源文件
└── README.md
```

## 操作说明

### 通用
| 按键 | 功能 |
|------|------|
| `Tab` | 切换相机模式 |
| `ESC` | 退出程序 |

### 轨道相机 (Orbit Camera)
| 操作 | 功能 |
|------|------|
| 右键拖拽 | 旋转视角 |
| 中键拖拽 | 平移视角 |
| 滚轮 | 缩放 |

### 飞行相机 (Fly Camera - UE4风格)
| 操作 | 功能 |
|------|------|
| 按住右键 | 激活控制 |
| W/A/S/D | 前/左/后/右移动 |
| Q/E | 下降/上升 |
| Space | 上升 |
| Shift | 加速移动 |
| Ctrl | 减速移动 |
| 滚轮 | 调整移动速度 |

## 核心模块

### Application
应用程序基类，封装窗口创建、主循环、帧时间管理。子类通过重写 `onInit()`, `onUpdate()`, `onRender()` 实现业务逻辑。

### Transform
变换组件，支持：
- 位置 (Vector3)
- 旋转 (Quaternion)
- 缩放 (Vector3)
- 矩阵计算、欧拉角转换、lookAt

### Entity & Scene
场景图系统：
- 父子层级关系
- 世界变换矩阵自动计算
- 递归更新和渲染
- MeshEntity 支持基础图元（立方体、球体、圆柱体、平面）

### Camera System
双相机系统：
- **OrbitCamera**: 围绕目标点旋转，适合模型查看
- **FlyCamera**: 自由飞行，适合场景漫游

## 开发进度

- [x] Step 1: 项目搭建 + 基础渲染窗口
- [x] Step 2: 相机系统 + 场景图
- [ ] Step 3: 模型加载 (Assimp)
- [ ] Step 4: 骨骼数据提取
- [ ] Step 5: 动画播放 (ozz-animation)
- [ ] Step 6: 动画混合与过渡

## 许可证

MIT License
