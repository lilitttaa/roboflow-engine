#include "core/Application.hpp"
#include "core/OrbitCamera.hpp"
#include "core/ProcessManager.hpp"
#include "core/ThirdPersonCamera.hpp"
#include "core/CharacterController.hpp"
#include "scene/Scene.hpp"
#include "scene/Entity.hpp"
#include "robot/RobotEntity.hpp"
#include "motion/MotionPlayer.hpp"
#include "gui/GuiPanel.hpp"
#include "network/UDPStreamer.hpp"
#include "network/JointMapper.hpp"
#include <raylib.h>
#include <rlgl.h>
#include <cmath>
#include <iostream>
#include <array>
#include <algorithm>
#include <filesystem>

/**
 * DemoApp - 演示应用
 * 展示 URDF 机器人模型加载和动作播放
 */
class DemoApp : public mf::Application {
public:
    DemoApp() : Application(Config{
        .width = 1280,
        .height = 720,
        .title = "MotionFlow - Motion Playback",
        .targetFPS = 60,
        .vsync = true
    }) {}

protected:
    void onInit() override {
        // 初始化 GUI
        m_guiPanel.init();
        
        // 配置轨道相机 - 拉远一点看到整个机器人
        m_scene.orbitCamera.target = { 0.0f, 0.5f, 0.0f };
        m_scene.orbitCamera.distance = 2.5f;
        m_scene.orbitCamera.yaw = 30.0f;
        m_scene.orbitCamera.pitch = 15.0f;
        
        // 配置飞行相机
        m_scene.flyCamera.position = { 2.0f, 1.5f, 2.0f };
        m_scene.flyCamera.lookAt(m_scene.flyCamera.position, { 0.0f, 0.8f, 0.0f });

        // 创建地板
        auto* floor = m_scene.createEntity<mf::MeshEntity>("Floor");
        floor->primitiveType = mf::MeshEntity::PrimitiveType::Plane;
        floor->size = { 10.0f, 1.0f, 10.0f };
        floor->color = { 40, 40, 45, 255 };
        
        // 初始化进程管理器
        // 获取当前可执行文件路径，推算 gentle-humanoid 路径
        std::filesystem::path exePath = std::filesystem::current_path();
        // 尝试多个可能的路径
        std::filesystem::path ghPath = exePath / "gentle-humanoid";
        if (!std::filesystem::exists(ghPath)) {
            ghPath = exePath.parent_path() / "gentle-humanoid";
        }
        if (!std::filesystem::exists(ghPath)) {
            ghPath = exePath.parent_path().parent_path() / "gentle-humanoid";
        }
        if (std::filesystem::exists(ghPath)) {
            m_processManager.setGentleHumanoidPath(ghPath.string());
            std::cout << "[ProcessManager] gentle-humanoid path: " << ghPath.string() << std::endl;
        } else {
            std::cerr << "[ProcessManager] WARNING: gentle-humanoid directory not found!" << std::endl;
        }
        
        // 加载 G1 机器人
        m_robot = std::make_unique<mf::RobotEntity>();
        if (m_robot->loadFromURDF("assets/G1_jy/G1_jy.urdf")) {
            m_robot->position = { 0.0f, 0.0f, 0.0f };
            m_robot->rotation = { -90.0f, 0.0f, 0.0f };  // URDF Z-up -> Y-up
            m_robot->scale = 1.0f;
            m_robot->showAxes = false;
            m_robotLoaded = true;
            
            // 获取关节列表
            m_jointNames = m_robot->getJointNames();
            m_guiPanel.jointNames = m_jointNames;  // 传递给 GUI
            std::cout << "Loaded robot with " << m_jointNames.size() << " movable joints" << std::endl;
            
            // 初始化动作播放器
            m_motionPlayer = std::make_unique<mf::MotionPlayer>();
            m_motionPlayer->bindRobot(m_robot.get());
            m_motionPlayer->setLoop(true);
            
            // 初始化角色控制器
            m_characterController.targetPosition = &m_robot->position;
            // 注意：机器人的朝向是通过 rotation.y 控制的，但需要转换坐标系
            m_characterController.camera = &m_scene.thirdPersonCamera;
            
            // 设置第三人称相机跟随目标
            m_scene.thirdPersonCamera.setFollowTarget(&m_robot->position);
            
            // 扫描可用的动作文件
            scanMotionFiles();
            
            // 加载第一个动作文件（如果有）
            if (!m_motionFiles.empty()) {
                loadMotionFile(m_motionFiles[0]);
            } else {
                std::cout << "No motion files found. Use simple animation." << std::endl;
            }
        } else {
            std::cerr << "Failed to load robot URDF!" << std::endl;
        }
    }

    void onUpdate(float deltaTime) override {
        m_scene.update(deltaTime);
        m_guiPanel.update();
        
        if (m_robotLoaded) {
            // 第三人称模式下使用角色控制器
            if (m_scene.isThirdPersonMode()) {
                m_useCharacterController = true;
                m_characterController.enabled = true;
                m_characterController.update(deltaTime);
                
                // ========== 坐标系转换 ==========
                // 
                // facingAngle (来自 CharacterController):
                //   - 基于 atan2(x, z)
                //   - 0=面向+Z, 90=面向+X, 180=面向-Z, -90=面向-X
                //
                // yawAngle (给 RobotEntity):
                //   - Ry(yawAngle) 把机器人从默认朝向 +X 旋转
                //   - 0=面向+X, 90=面向-Z, -90=面向+Z, 180=面向-X
                //
                // 映射公式：yawAngle = facingAngle - 90
                //   - facingAngle=0(+Z) → yawAngle=-90 → Ry(-90°)把+X转到+Z ✓
                //   - facingAngle=90(+X) → yawAngle=0 → 面向+X ✓
                //   - facingAngle=180(-Z) → yawAngle=90 → Ry(90°)把+X转到-Z ✓
                //   - facingAngle=-90(-X) → yawAngle=-180 → 面向-X ✓
                //
                m_robot->useYawMode = true;
                m_robot->yawAngle = m_characterController.facingAngle - 90.0f;
                
                // 调试输出
                static int mainDebugCount = 0;
                if (mainDebugCount++ % 120 == 0) {
                    printf("[main.cpp] facingAngle=%.1f → yawAngle=%.1f\n",
                           m_characterController.facingAngle, m_robot->yawAngle);
                }
            } else {
                m_robot->useYawMode = false;
                m_useCharacterController = false;
                m_characterController.enabled = false;
            }
            
            // 键盘快捷键在 update 中处理
            handleKeyboardInput(deltaTime);
            
            // 如果使用动作播放器，更新它
            if (m_motionLoaded && m_useMotionPlayer) {
                m_motionPlayer->update(deltaTime);
            } else if (m_playAnimation) {
                // 简单动画
                m_animTime += deltaTime;
                float armSwing = sinf(m_animTime * 2.0f) * 0.3f;
                m_robot->setJointPosition("arm_l1_joint", armSwing);
                m_robot->setJointPosition("arm_r1_joint", -armSwing);
                
                float legMove = sinf(m_animTime * 3.0f) * 0.1f;
                m_robot->setJointPosition("leg_l1_joint", legMove);
                m_robot->setJointPosition("leg_r1_joint", -legMove);
            }
            
            m_robot->update();
            
            // 流式传输动作数据到 gentle-humanoid
            // 只要连接就持续发送，不管动画是否在播放
            if (m_streamer.isConnected() && m_guiPanel.streamEnabled) {
                sendStreamFrame();
            }
        }
        
        // 即使机器人没加载，只要连接了也发送心跳
        if (m_streamer.isConnected() && m_guiPanel.streamEnabled && !m_robotLoaded) {
            // 发送默认帧保持连接
            std::array<float, mf::JointMapper::NUM_JOINTS> zeros;
            zeros.fill(0.0f);
            float rootPos[3] = {0, 0, 0.8f};
            float rootQuat[4] = {1, 0, 0, 0};
            m_streamer.sendMotion(GetTime(), zeros.data(), 29, rootQuat, rootPos);
        }
        
        // 自动连接流式传输（仿真启动后延迟连接）
        if (m_autoConnectDelay > 0) {
            m_autoConnectDelay -= deltaTime;
            if (m_autoConnectDelay <= 0 && m_guiPanel.autoConnectStream && 
                m_processManager.isAllRunning() && !m_streamer.isConnected()) {
                std::cout << "[Stream] Auto-connecting..." << std::endl;
                if (m_streamer.connect(m_guiPanel.streamHost, m_guiPanel.streamPort)) {
                    m_streamStartTime = GetTime();
                    m_guiPanel.streamEnabled = true;
                    std::cout << "[Stream] Auto-connected to " << m_guiPanel.streamHost 
                              << ":" << m_guiPanel.streamPort << std::endl;
                }
            }
        }
        
        // 同步状态到 GUI（在渲染前）
        syncGuiState();
    }
    
    void sendStreamFrame() {
        // 获取当前关节位置
        std::map<std::string, float> jointMap;
        for (const auto& name : m_jointNames) {
            jointMap[name] = m_robot->getJointPosition(name);
        }
        
        // 映射到 gentle-humanoid 格式 (29 关节)
        std::array<float, mf::JointMapper::NUM_JOINTS> mappedJoints;
        mf::JointMapper::mapJointPositions(jointMap, mappedJoints);
        
        // 获取根节点位置和旋转
        // gentle-humanoid 使用 Z-up 坐标系
        float rootPos[3] = {
            m_robot->position.x,
            m_robot->position.z,
            m_robot->position.y + 0.8f  // 加上机器人高度
        };
        
        // 四元数 (wxyz) - 使用单位四元数
        float rootQuat[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
        
        // 发送
        double timestamp = GetTime() - m_streamStartTime;
        m_streamer.sendMotion(timestamp, mappedJoints.data(), 29, rootQuat, rootPos);
        
        // 每100帧打印一次详细调试信息
        static int debugCounter = 0;
        if (++debugCounter % 100 == 0) {
            // 打印原始 URDF 关节值
            float leg_l1 = jointMap.count("leg_l1_joint") ? jointMap["leg_l1_joint"] : -999;
            float leg_l4 = jointMap.count("leg_l4_joint") ? jointMap["leg_l4_joint"] : -999;
            
            std::cout << "[Stream] Frame " << m_streamer.getFramesSent() << std::endl;
            std::cout << "  URDF: leg_l1=" << leg_l1 << " leg_l4=" << leg_l4 << std::endl;
            std::cout << "  Mapped: [0]=" << mappedJoints[0] 
                      << " [3]=" << mappedJoints[3] 
                      << " [6]=" << mappedJoints[6] << std::endl;
        }
    }
    
    void handleGuiInput() {
        // 模式切换
        if (m_guiPanel.modeToggled) {
            m_useMotionPlayer = !m_useMotionPlayer;
            m_guiPanel.useMotionPlayer = m_useMotionPlayer;
            if (!m_useMotionPlayer) {
                m_robot->resetJoints();
                m_robot->rotation = { -90.0f, 0.0f, 0.0f };
                m_robot->position = { 0.0f, 0.0f, 0.0f };
            }
        }
        
        // 动作文件选择
        if (m_guiPanel.motionSelectionChanged && !m_motionFiles.empty()) {
            int idx = m_guiPanel.selectedMotionIndex;
            if (idx >= 0 && idx < (int)m_motionFiles.size()) {
                loadMotionFile(m_motionFiles[idx]);
            }
        }
        
        if (m_motionLoaded && m_useMotionPlayer) {
            // 动作播放器控制
            if (m_guiPanel.playPressed) {
                m_motionPlayer->togglePlay();
            }
            if (m_guiPanel.stopPressed) {
                m_motionPlayer->stop();
            }
            if (m_guiPanel.loopToggled) {
                m_motionPlayer->setLoop(!m_motionPlayer->isLooping());
            }
            if (m_guiPanel.rootPosToggled) {
                m_motionPlayer->applyRootPosition = !m_motionPlayer->applyRootPosition;
            }
            if (m_guiPanel.rootRotToggled) {
                m_motionPlayer->applyRootRotation = !m_motionPlayer->applyRootRotation;
            }
            if (m_guiPanel.relPosToggled) {
                m_motionPlayer->useRelativePosition = !m_motionPlayer->useRelativePosition;
            }
            
            // 播放速度
            m_motionPlayer->setPlaybackSpeed(m_guiPanel.playbackSpeed);
            
            // 进度拖动
            if (m_guiPanel.seekChanged) {
                float targetTime = m_guiPanel.seekPosition * m_motionPlayer->getDuration();
                m_motionPlayer->seekTo(targetTime);
            }
        } else {
            // 简单动画控制
            if (m_guiPanel.animationToggled) {
                m_playAnimation = !m_playAnimation;
                m_guiPanel.animationPlaying = m_playAnimation;
            }
            if (m_guiPanel.resetJointsPressed) {
                m_robot->resetJoints();
            }
            
            // 关节控制
            if (m_guiPanel.selectedJointIndex != m_selectedJoint) {
                m_selectedJoint = m_guiPanel.selectedJointIndex;
                // 更新关节限位信息
                if (!m_jointNames.empty()) {
                    m_robot->getJointLimits(m_jointNames[m_selectedJoint], 
                                            m_guiPanel.jointLower, m_guiPanel.jointUpper);
                    m_guiPanel.jointPosition = m_robot->getJointPosition(m_jointNames[m_selectedJoint]);
                }
            }
            if (m_guiPanel.jointChanged && !m_jointNames.empty()) {
                m_robot->setJointPosition(m_jointNames[m_selectedJoint], m_guiPanel.jointPosition);
            }
        }
        
        // 显示选项
        if (m_guiPanel.showAxesToggled) {
            m_robot->showAxes = !m_robot->showAxes;
            m_guiPanel.showAxes = m_robot->showAxes;
        }
        if (m_guiPanel.showGridToggled) {
            m_showGrid = !m_showGrid;
            m_guiPanel.showGrid = m_showGrid;
        }
        
        // 流式传输控制
        if (m_guiPanel.streamConnectPressed && !m_streamer.isConnected()) {
            if (m_streamer.connect(m_guiPanel.streamHost, m_guiPanel.streamPort)) {
                m_streamStartTime = GetTime();
                std::cout << "[Stream] Connected to " << m_guiPanel.streamHost 
                          << ":" << m_guiPanel.streamPort << std::endl;
            } else {
                std::cerr << "[Stream] Failed to connect: " << m_streamer.getLastError() << std::endl;
            }
        }
        if (m_guiPanel.streamDisconnectPressed && m_streamer.isConnected()) {
            m_streamer.disconnect();
        }
        
        // 仿真控制
        if (m_guiPanel.simStartAllPressed) {
            std::cout << "[Simulation] Starting sim2sim and deploy..." << std::endl;
            if (m_processManager.startAll(true)) {  // auto mode
                std::cout << "[Simulation] Started successfully" << std::endl;
                // 如果启用了自动连接，等待一段时间后连接流式传输
                if (m_guiPanel.autoConnectStream) {
                    m_autoConnectDelay = 3.0f;  // 3 秒后自动连接
                }
            } else {
                std::cerr << "[Simulation] Failed to start" << std::endl;
            }
        }
        if (m_guiPanel.simStopAllPressed) {
            std::cout << "[Simulation] Stopping all processes..." << std::endl;
            m_processManager.stopAll();
            // 也断开流式传输
            if (m_streamer.isConnected()) {
                m_streamer.disconnect();
            }
        }
    }
    
    void handleKeyboardInput(float deltaTime) {
        // 第三人称模式下禁用与移动冲突的快捷键
        bool isThirdPerson = m_scene.isThirdPersonMode();
        
        // 模式切换 (M 键在第三人称模式下仍可用)
        if (IsKeyPressed(KEY_M) && m_motionLoaded && !isThirdPerson) {
            m_useMotionPlayer = !m_useMotionPlayer;
            m_guiPanel.useMotionPlayer = m_useMotionPlayer;
            if (!m_useMotionPlayer) {
                m_robot->resetJoints();
                m_robot->rotation = { -90.0f, 0.0f, 0.0f };
                m_robot->position = { 0.0f, 0.0f, 0.0f };
            }
        }
        
        // 在第三人称模式下，WASD/方向键被角色控制器使用，跳过动画快捷键
        if (m_motionLoaded && m_useMotionPlayer && !isThirdPerson) {
            // 动作播放器快捷键
            if (IsKeyPressed(KEY_SPACE)) m_motionPlayer->togglePlay();
            if (IsKeyPressed(KEY_S)) m_motionPlayer->stop();
            if (IsKeyPressed(KEY_UP)) m_motionPlayer->setPlaybackSpeed(m_motionPlayer->getPlaybackSpeed() + 0.1f);
            if (IsKeyPressed(KEY_DOWN)) m_motionPlayer->setPlaybackSpeed(std::max(0.1f, m_motionPlayer->getPlaybackSpeed() - 0.1f));
            if (IsKeyPressed(KEY_LEFT)) {
                m_motionPlayer->pause();
                size_t frame = m_motionPlayer->getCurrentFrame();
                if (frame > 0) m_motionPlayer->seekToFrame(frame - 1);
            }
            if (IsKeyPressed(KEY_RIGHT)) {
                m_motionPlayer->pause();
                m_motionPlayer->seekToFrame(m_motionPlayer->getCurrentFrame() + 1);
            }
            if (IsKeyPressed(KEY_L)) m_motionPlayer->setLoop(!m_motionPlayer->isLooping());
            if (IsKeyPressed(KEY_P)) m_motionPlayer->applyRootPosition = !m_motionPlayer->applyRootPosition;
            if (IsKeyPressed(KEY_O)) m_motionPlayer->applyRootRotation = !m_motionPlayer->applyRootRotation;
        } else if (!isThirdPerson) {
            // 简单动画快捷键
            if (IsKeyPressed(KEY_SPACE)) m_playAnimation = !m_playAnimation;
            if (IsKeyPressed(KEY_LEFT_BRACKET)) {
                m_selectedJoint = (m_selectedJoint - 1 + m_jointNames.size()) % m_jointNames.size();
                m_guiPanel.selectedJointIndex = m_selectedJoint;
            }
            if (IsKeyPressed(KEY_RIGHT_BRACKET)) {
                m_selectedJoint = (m_selectedJoint + 1) % m_jointNames.size();
                m_guiPanel.selectedJointIndex = m_selectedJoint;
            }
            if (!m_jointNames.empty()) {
                const auto& jointName = m_jointNames[m_selectedJoint];
                float pos = m_robot->getJointPosition(jointName);
                if (IsKeyDown(KEY_LEFT)) m_robot->setJointPosition(jointName, pos - 1.0f * deltaTime);
                if (IsKeyDown(KEY_RIGHT)) m_robot->setJointPosition(jointName, pos + 1.0f * deltaTime);
            }
            if (IsKeyPressed(KEY_R)) m_robot->resetJoints();
        }
        
        // 显示选项
        if (IsKeyPressed(KEY_X)) {
            m_robot->showAxes = !m_robot->showAxes;
            m_guiPanel.showAxes = m_robot->showAxes;
        }
        if (IsKeyPressed(KEY_G)) {
            m_showGrid = !m_showGrid;
            m_guiPanel.showGrid = m_showGrid;
        }
    }
    
    void syncGuiState() {
        m_guiPanel.useMotionPlayer = m_useMotionPlayer;
        
        if (m_motionLoaded && m_useMotionPlayer) {
            m_guiPanel.isPlaying = m_motionPlayer->isPlaying();
            m_guiPanel.isLooping = m_motionPlayer->isLooping();
            m_guiPanel.applyRootPos = m_motionPlayer->applyRootPosition;
            m_guiPanel.applyRootRot = m_motionPlayer->applyRootRotation;
            m_guiPanel.useRelativePos = m_motionPlayer->useRelativePosition;
            m_guiPanel.currentTime = m_motionPlayer->getCurrentTime();
            m_guiPanel.duration = m_motionPlayer->getDuration();
            m_guiPanel.currentFrame = m_motionPlayer->getCurrentFrame();
            m_guiPanel.totalFrames = m_motionPlayer->getNumFrames();
            m_guiPanel.playbackSpeed = m_motionPlayer->getPlaybackSpeed();
            if (m_motionPlayer->getMotion()) {
                m_guiPanel.fps = m_motionPlayer->getMotion()->getFPS();
            }
            // 只在非拖动时更新进度条位置
            if (!m_guiPanel.seekChanged && m_guiPanel.duration > 0) {
                m_guiPanel.seekPosition = m_guiPanel.currentTime / m_guiPanel.duration;
            }
        } else {
            m_guiPanel.animationPlaying = m_playAnimation;
            m_guiPanel.selectedJointIndex = m_selectedJoint;
            
            // 更新关节信息
            if (!m_jointNames.empty()) {
                m_guiPanel.jointPosition = m_robot->getJointPosition(m_jointNames[m_selectedJoint]);
                m_robot->getJointLimits(m_jointNames[m_selectedJoint], 
                                        m_guiPanel.jointLower, m_guiPanel.jointUpper);
            }
        }
        
        m_guiPanel.showAxes = m_robot->showAxes;
        m_guiPanel.showGrid = m_showGrid;
        
        // 同步角色控制器状态
        m_guiPanel.isThirdPersonMode = m_scene.isThirdPersonMode();
        m_guiPanel.characterControllerEnabled = m_characterController.enabled;
        m_guiPanel.controllerSpeed = m_characterController.speed;
        m_guiPanel.controllerFacingAngle = m_characterController.facingAngle;
        m_guiPanel.controllerIsMoving = m_characterController.isMoving;
        m_guiPanel.controllerIsRunning = m_characterController.isRunning;
        m_guiPanel.controllerWalkSpeed = m_characterController.walkSpeed;
        m_guiPanel.controllerRunSpeed = m_characterController.runSpeed;
        m_guiPanel.controllerMode = (m_characterController.controlMode == mf::ControlMode::FreeRotation) ? 0 : 1;
        
        // 如果 GUI 中修改了速度参数，更新到控制器
        if (m_guiPanel.walkSpeedChanged) {
            m_characterController.walkSpeed = m_guiPanel.controllerWalkSpeed;
        }
        if (m_guiPanel.runSpeedChanged) {
            m_characterController.runSpeed = m_guiPanel.controllerRunSpeed;
        }
        
        // 同步流式传输状态
        m_guiPanel.streamConnected = m_streamer.isConnected();
        m_guiPanel.streamFramesSent = m_streamer.getFramesSent();
        
        // 同步仿真进程状态
        m_guiPanel.sim2simRunning = m_processManager.isSim2SimRunning();
        m_guiPanel.deployRunning = m_processManager.isDeployRunning();
    }

    void onRender() override {
        // 手动控制 3D 模式
        m_scene.begin3D();
        {
            // 渲染场景实体
            m_scene.getRoot()->render();
            
            // 绘制网格
            if (m_showGrid) {
                DrawGrid(20, 0.5f);
            }
            
            // 绘制世界坐标轴
            DrawLine3D({ 0, 0, 0 }, { 1, 0, 0 }, RED);
            DrawLine3D({ 0, 0, 0 }, { 0, 1, 0 }, GREEN);
            DrawLine3D({ 0, 0, 0 }, { 0, 0, 1 }, BLUE);
            
            // 渲染机器人
            if (m_robotLoaded) {
                m_robot->render();
            }
        }
        m_scene.end3D();

        // 绘制左上角信息
        DrawText("MotionFlow - Motion Playback", 10, 10, 20, WHITE);
        
        int y = 40;
        if (m_robotLoaded) {
            DrawText("Unitree G1 Humanoid Robot", 10, y, 16, LIME);
            y += 25;
            DrawText(TextFormat("Mode: %s", m_useMotionPlayer ? "Motion Playback" : "Simple Animation"), 
                     10, y, 14, YELLOW);
        }
        
        // 相机信息
        y = getHeight() - 50;
        DrawText(TextFormat("Camera: %s  [Tab] Switch", m_scene.getCameraModeName()), 10, y, 14, YELLOW);
        
        DrawFPS(10, getHeight() - 25);
        
        // 渲染 GUI 面板
        m_guiPanel.render();
        
        // GUI 输入在渲染后立即处理（raygui 在 render 时检测输入）
        if (m_robotLoaded) {
            handleGuiInput();
        }
    }

    // 扫描 assets/motions 目录下的动画文件 (.motion 和 .npz)
    void scanMotionFiles() {
        m_motionFiles.clear();
        std::filesystem::path motionDir = "assets/motions";
        
        if (std::filesystem::exists(motionDir)) {
            for (const auto& entry : std::filesystem::directory_iterator(motionDir)) {
                if (entry.is_regular_file()) {
                    auto ext = entry.path().extension().string();
                    if (ext == ".motion" || ext == ".npz") {
                        m_motionFiles.push_back(entry.path().string());
                    }
                }
            }
            // 排序使文件列表稳定
            std::sort(m_motionFiles.begin(), m_motionFiles.end());
        }
        
        std::cout << "Found " << m_motionFiles.size() << " motion files:" << std::endl;
        for (const auto& f : m_motionFiles) {
            std::cout << "  - " << f << std::endl;
        }
        
        // 更新 GUI 的动作文件列表
        m_guiPanel.motionFiles = m_motionFiles;
    }
    
    // 加载指定的动作文件
    bool loadMotionFile(const std::string& path) {
        if (m_motionPlayer->loadMotion(path)) {
            std::cout << "Loaded motion: " << path << " (" 
                      << m_motionPlayer->getNumFrames() << " frames)" << std::endl;
            m_motionLoaded = true;
            
            // 自动开始播放
            m_motionPlayer->play();
            
            // 更新选中索引
            for (size_t i = 0; i < m_motionFiles.size(); i++) {
                if (m_motionFiles[i] == path) {
                    m_guiPanel.selectedMotionIndex = i;
                    break;
                }
            }
            return true;
        } else {
            std::cerr << "Failed to load motion: " << path << std::endl;
            return false;
        }
    }

private:
    mf::Scene m_scene;
    mf::GuiPanel m_guiPanel;
    mf::ProcessManager m_processManager;  // 管理 sim2sim/deploy 进程
    mf::CharacterController m_characterController;  // 角色控制器
    std::unique_ptr<mf::RobotEntity> m_robot;
    std::unique_ptr<mf::MotionPlayer> m_motionPlayer;
    mf::UDPStreamer m_streamer;  // UDP 流式传输到 gentle-humanoid
    bool m_robotLoaded = false;
    bool m_motionLoaded = false;
    bool m_useMotionPlayer = true;
    bool m_showGrid = true;
    bool m_useCharacterController = false;  // 是否使用角色控制器模式
    
    std::vector<std::string> m_jointNames;
    std::vector<std::string> m_motionFiles;  // 可用的动作文件列表
    size_t m_selectedJoint = 0;
    
    float m_animTime = 0.0f;
    bool m_playAnimation = true;
    
    // 流式传输时间戳
    double m_streamStartTime = 0.0;
    
    // 自动连接延迟计时器
    float m_autoConnectDelay = 0.0f;
};

int main() {
    DemoApp app;
    app.run();
    return 0;
}
