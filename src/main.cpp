#include "core/Application.hpp"
#include "core/OrbitCamera.hpp"
#include "core/ProcessManager.hpp"
#include "core/ThirdPersonCamera.hpp"
#include "core/CharacterController.hpp"
#include "scene/Scene.hpp"
#include "scene/Entity.hpp"
#include "robot/RobotEntity.hpp"
#include "motion/MotionPlayer.hpp"
#include "motion/MotionMatching.hpp"
#include "motion/MotionMatchingUtils.hpp"
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
        // 尝试多个可能的URDF路径
        std::filesystem::path urdfPath = "assets/G1_jy/G1_jy.urdf";
        if (!std::filesystem::exists(urdfPath)) {
            // 尝试从可执行文件目录
            std::filesystem::path exeDir = std::filesystem::current_path();
            if (std::filesystem::exists(exeDir / "bin" / "assets" / "G1_jy" / "G1_jy.urdf")) {
                urdfPath = exeDir / "bin" / "assets" / "G1_jy" / "G1_jy.urdf";
            } else if (std::filesystem::exists(exeDir / "assets" / "G1_jy" / "G1_jy.urdf")) {
                urdfPath = exeDir / "assets" / "G1_jy" / "G1_jy.urdf";
            } else {
                // 尝试从项目根目录
                std::filesystem::path projectRoot = exeDir.parent_path().parent_path();
                if (std::filesystem::exists(projectRoot / "assets" / "G1_jy" / "G1_jy.urdf")) {
                    urdfPath = projectRoot / "assets" / "G1_jy" / "G1_jy.urdf";
                }
            }
        }
        
        std::cout << "[Main] Trying to load URDF from: " << urdfPath << std::endl;
        m_robot = std::make_unique<mf::RobotEntity>();
        if (m_robot->loadFromURDF(urdfPath.string())) {
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
                
                // 初始化 Motion Matching 数据库
                initMotionMatching();
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
                
                // Motion Matching 模式
                if (m_useMotionMatching && m_motionDatabase.isPreprocessed()) {
                    // 禁用 CharacterController 的位置更新（只用于计算速度和朝向）
                    m_characterController.updatePosition = false;
                    m_characterController.update(deltaTime);
                    
                    // 确保 Motion Player 被禁用
                    if (m_useMotionPlayer) {
                        m_useMotionPlayer = false;
                        m_guiPanel.useMotionPlayer = false;
                    }
                    
                    // 判断控制模式（Free 或 Lock）
                    bool isFreeMode = (m_characterController.controlMode == mf::ControlMode::FreeRotation);
                    
                    // 将角色控制器的速度传递给 Motion Matcher
                    mf::MotionFrame frame = m_motionMatcher.update(
                        deltaTime,
                        m_characterController.velocity,
                        m_characterController.facingAngle,
                        isFreeMode
                    );
                    
                    // 应用 Root Motion（位置和旋转由动画驱动）
                    // frame.rootPos 现在包含的是相对位移（由 MotionMatcher 计算）
                    Vector3 rootDelta = frame.rootPos;
                    
                    // 应用位移（Root Motion 在世界坐标系中）
                    m_robot->position = Vector3Add(m_robot->position, rootDelta);
                    
                    // 应用旋转（由动画驱动，不被 CharacterController 覆盖）
                    Quaternion rootRot = m_motionMatcher.getCurrentRootRotation();
                    float yaw = mf::extractYawFromQuaternion(rootRot);
                    m_robot->useYawMode = true;
                    m_robot->yawAngle = yaw - 90.0f;  // 坐标系转换
                    
                    // 应用动作帧到机器人（应用关节角度）
                    applyMotionFrame(frame);
                } else {
                    // 非 Motion Matching 模式：正常使用 CharacterController
                    m_characterController.updatePosition = true;
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
                }
                
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
            
            // 如果使用动作播放器，更新它（确保不与 Motion Matching 冲突）
            if (m_motionLoaded && m_useMotionPlayer && !m_useMotionMatching) {
                m_motionPlayer->update(deltaTime);
            } else if (m_playAnimation && !m_useMotionMatching) {
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
        
        // Motion Matching 切换 (N 键，仅在第三人称模式下)
        if (IsKeyPressed(KEY_N) && isThirdPerson && m_motionDatabase.isPreprocessed()) {
            m_useMotionMatching = !m_useMotionMatching;
            if (m_useMotionMatching) {
                // 启用 Motion Matching 时禁用 Motion Player
                m_useMotionPlayer = false;
                m_guiPanel.useMotionPlayer = false;
                // 重置 Root Motion 跟踪
                m_lastMotionRootPos = m_motionMatcher.getCurrentRootPosition();
            }
            std::cout << "[MotionMatching] " << (m_useMotionMatching ? "Enabled" : "Disabled") << std::endl;
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
        
        // 同步 Motion Matching 状态（只同步只读状态，不覆盖用户可调参数）
        m_guiPanel.motionMatchingEnabled = m_useMotionMatching;
        if (m_motionDatabase.isPreprocessed()) {
            // 只读状态
            m_guiPanel.mmTotalEntries = m_motionDatabase.getNumEntries();
            m_guiPanel.mmCurrentClip = m_motionMatcher.getCurrentClip();
            m_guiPanel.mmCurrentFrame = m_motionMatcher.getCurrentFrame();
        }
        
        // 处理 GUI 的 Motion Matching 事件（GUI -> Matcher）
        if (m_guiPanel.motionMatchingToggled) {
            m_useMotionMatching = m_guiPanel.motionMatchingEnabled;
            if (m_useMotionMatching) {
                // 启用 Motion Matching 时禁用 Motion Player
                m_useMotionPlayer = false;
                m_guiPanel.useMotionPlayer = false;
                // 重置 Root Motion 跟踪
                m_lastMotionRootPos = m_motionMatcher.getCurrentRootPosition();
            }
            m_guiPanel.motionMatchingToggled = false;
        }
        if (m_guiPanel.mmSearchIntervalChanged) {
            m_motionMatcher.searchInterval = m_guiPanel.mmSearchInterval;
            m_guiPanel.mmSearchIntervalChanged = false;
        }
        if (m_guiPanel.mmBlendDurationChanged) {
            m_motionMatcher.blendDuration = m_guiPanel.mmBlendDuration;
            m_guiPanel.mmBlendDurationChanged = false;
        }
        if (m_guiPanel.mmWeightsChanged) {
            m_motionMatcher.weights().trajectoryPos = m_guiPanel.mmWeightTrajectoryPos;
            m_motionMatcher.weights().trajectoryFacing = m_guiPanel.mmWeightTrajectoryFacing;
            m_motionMatcher.weights().hipVel = m_guiPanel.mmWeightHipVel;
            m_guiPanel.mmWeightsChanged = false;
        }
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
            
            // 绘制 Motion Matching 轨迹可视化
            if (m_guiPanel.mmShowTrajectory && m_useMotionMatching && m_scene.isThirdPersonMode()) {
                drawMotionMatchingTrajectory();
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
        
        // 如果当前目录找不到，尝试其他路径
        if (!std::filesystem::exists(motionDir)) {
            std::filesystem::path exeDir = std::filesystem::current_path();
            if (std::filesystem::exists(exeDir / "bin" / "assets" / "motions")) {
                motionDir = exeDir / "bin" / "assets" / "motions";
            } else if (std::filesystem::exists(exeDir / "assets" / "motions")) {
                motionDir = exeDir / "assets" / "motions";
            } else {
                std::filesystem::path projectRoot = exeDir.parent_path().parent_path();
                if (std::filesystem::exists(projectRoot / "assets" / "motions")) {
                    motionDir = projectRoot / "assets" / "motions";
                }
            }
        }
        
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
    
    // 绘制 Motion Matching 轨迹可视化
    void drawMotionMatchingTrajectory() {
        if (!m_robotLoaded) return;
        
        Vector3 robotPos = m_robot->position;
        float facingAngle = m_characterController.facingAngle;
        float facingRad = facingAngle * DEG2RAD;
        
        // 获取角色的局部前方和右方向
        // facingAngle: 0=+Z, 90=+X, 180=-Z, -90=-X
        Vector3 charForward = { sinf(facingRad), 0.0f, cosf(facingRad) };
        Vector3 charRight = { cosf(facingRad), 0.0f, -sinf(facingRad) };
        
        // 获取输入和速度
        Vector2 input = m_characterController.inputVector;
        float speed = m_characterController.isRunning ? m_characterController.runSpeed : m_characterController.walkSpeed;
        bool isMoving = m_characterController.isMoving;
        
        // 轨迹点时间
        const float times[] = {0.2f, 0.4f, 0.6f, 1.0f};
        Vector3 prevPoint = robotPos;
        prevPoint.y = 0.02f;
        
        // 根据控制模式计算轨迹
        bool isLockMode = (m_characterController.controlMode == mf::ControlMode::LockToCamera);
        
        for (int i = 0; i < 4; ++i) {
            float t = times[i];
            Vector3 predictedPos;
            float predictedFacing = facingAngle;
            
            if (isMoving) {
                if (isLockMode) {
                    // Lock 模式：基于相机方向移动，角色朝向不变
                    float camYaw = m_scene.thirdPersonCamera.yaw;
                    float camYawRad = camYaw * DEG2RAD;
                    Vector3 camForward = { -sinf(camYawRad), 0.0f, -cosf(camYawRad) };
                    Vector3 camRight = { cosf(camYawRad), 0.0f, -sinf(camYawRad) };
                    
                    Vector3 moveDir = {
                        camForward.x * input.y + camRight.x * input.x,
                        0.0f,
                        camForward.z * input.y + camRight.z * input.x
                    };
                    
                    predictedPos = {
                        robotPos.x + moveDir.x * speed * t,
                        0.02f,
                        robotPos.z + moveDir.z * speed * t
                    };
                } else {
                    // Free 模式：角色朝向移动方向，始终往前走
                    // 在 Free 模式下，轨迹就是角色前方
                    predictedPos = {
                        robotPos.x + charForward.x * speed * t,
                        0.02f,
                        robotPos.z + charForward.z * speed * t
                    };
                }
            } else {
                // 不移动时，轨迹就在原地
                predictedPos = {robotPos.x, 0.02f, robotPos.z};
            }
            
            // 绘制轨迹线（绿色 = 期望轨迹）
            DrawLine3D(prevPoint, predictedPos, GREEN);
            
            // 绘制轨迹点
            DrawSphere(predictedPos, 0.03f, GREEN);
            
            // 绘制朝向指示器
            float predictedRad = predictedFacing * DEG2RAD;
            float arrowLen = 0.12f;
            Vector3 arrowEnd = {
                predictedPos.x + sinf(predictedRad) * arrowLen,
                predictedPos.y + 0.01f,
                predictedPos.z + cosf(predictedRad) * arrowLen
            };
            DrawLine3D(predictedPos, arrowEnd, LIME);
            
            prevPoint = predictedPos;
        }
        
        // 绘制当前朝向（黄色箭头）
        Vector3 arrowStart = robotPos;
        arrowStart.y = 0.5f;
        float arrowLen = 0.4f;
        Vector3 arrowEnd = {
            arrowStart.x + charForward.x * arrowLen,
            arrowStart.y,
            arrowStart.z + charForward.z * arrowLen
        };
        DrawLine3D(arrowStart, arrowEnd, YELLOW);
        DrawSphere(arrowEnd, 0.02f, YELLOW);
        
        // 绘制当前速度向量（蓝色）
        Vector3 velocity = m_characterController.velocity;
        float velLen = Vector3Length(velocity);
        if (velLen > 0.1f) {
            Vector3 velEnd = {
                arrowStart.x + velocity.x * 0.25f,
                arrowStart.y,
                arrowStart.z + velocity.z * 0.25f
            };
            DrawLine3D(arrowStart, velEnd, BLUE);
            DrawSphere(velEnd, 0.015f, BLUE);
        }
        
        // 显示当前状态文本
        const char* modeText = isLockMode ? "LOCK" : "FREE";
        const char* movingText = isMoving ? "Moving" : "Idle";
        // DrawText3D 不可用，在 2D 层显示
    }
    
    // 初始化 Motion Matching
    void initMotionMatching() {
        std::cout << "\n[MotionMatching] Initializing..." << std::endl;
        
        // 加载所有动作文件到数据库
        for (const auto& path : m_motionFiles) {
            m_motionDatabase.loadClip(path);
        }
        
        // 预处理数据库
        m_motionDatabase.preprocess();
        
        // 设置 matcher
        m_motionMatcher.setDatabase(&m_motionDatabase);
        
        // 默认禁用 Motion Matching，按 N 键切换
        m_useMotionMatching = false;
        
        std::cout << "[MotionMatching] Ready! Press N to toggle Motion Matching mode." << std::endl;
    }
    
    // 应用动作帧到机器人
    void applyMotionFrame(const mf::MotionFrame& frame) {
        if (!m_robot || !m_robotLoaded) return;
        
        // 应用关节角度
        const auto& jointNames = m_motionDatabase.getJointNames();
        
        // 使用 MotionPlayer 的关节映射
        static const std::map<std::string, std::string> jointMapping = {
            {"left_hip_pitch_joint", "leg_l1_joint"},
            {"left_hip_roll_joint", "leg_l2_joint"},
            {"left_hip_yaw_joint", "leg_l3_joint"},
            {"left_knee_joint", "leg_l4_joint"},
            {"left_ankle_pitch_joint", "leg_l5_joint"},
            {"left_ankle_roll_joint", "leg_l6_joint"},
            {"right_hip_pitch_joint", "leg_r1_joint"},
            {"right_hip_roll_joint", "leg_r2_joint"},
            {"right_hip_yaw_joint", "leg_r3_joint"},
            {"right_knee_joint", "leg_r4_joint"},
            {"right_ankle_pitch_joint", "leg_r5_joint"},
            {"right_ankle_roll_joint", "leg_r6_joint"},
            {"waist_yaw_joint", "waist_joint"},
            {"left_shoulder_pitch_joint", "arm_l1_joint"},
            {"left_shoulder_roll_joint", "arm_l2_joint"},
            {"left_shoulder_yaw_joint", "arm_l3_joint"},
            {"left_elbow_joint", "arm_l4_joint"},
            {"right_shoulder_pitch_joint", "arm_r1_joint"},
            {"right_shoulder_roll_joint", "arm_r2_joint"},
            {"right_shoulder_yaw_joint", "arm_r3_joint"},
            {"right_elbow_joint", "arm_r4_joint"},
        };
        
        for (size_t i = 0; i < std::min(jointNames.size(), frame.jointPos.size()); ++i) {
            const std::string& motionJoint = jointNames[i];
            auto it = jointMapping.find(motionJoint);
            if (it != jointMapping.end()) {
                m_robot->setJointPosition(it->second, frame.jointPos[i]);
            }
        }
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
    
    // Motion Matching
    mf::MotionDatabase m_motionDatabase;
    mf::MotionMatcher m_motionMatcher;
    bool m_useMotionMatching = false;  // 是否使用 Motion Matching
    
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
    
    // Motion Matching Root Motion 跟踪
    Vector3 m_lastMotionRootPos = {0, 0, 0};
};

int main() {
    DemoApp app;
    app.run();
    return 0;
}
