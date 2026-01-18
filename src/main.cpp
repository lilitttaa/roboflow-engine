#include "core/Application.hpp"
#include "core/OrbitCamera.hpp"
#include "scene/Scene.hpp"
#include "scene/Entity.hpp"
#include "robot/RobotEntity.hpp"
#include "motion/MotionPlayer.hpp"
#include "gui/GuiPanel.hpp"
#include <raylib.h>
#include <rlgl.h>
#include <cmath>
#include <iostream>

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
            
            // 尝试加载默认动作文件
            if (m_motionPlayer->loadMotion("assets/motions/walk.motion")) {
                std::cout << "Loaded motion: " << m_motionPlayer->getNumFrames() << " frames" << std::endl;
                m_motionLoaded = true;
            } else {
                std::cout << "No motion file found. Use simple animation." << std::endl;
            }
        } else {
            std::cerr << "Failed to load robot URDF!" << std::endl;
        }
    }

    void onUpdate(float deltaTime) override {
        m_scene.update(deltaTime);
        m_guiPanel.update();
        
        if (m_robotLoaded) {
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
        }
        
        // 同步状态到 GUI（在渲染前）
        syncGuiState();
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
    }
    
    void handleKeyboardInput(float deltaTime) {
        // 模式切换
        if (IsKeyPressed(KEY_M) && m_motionLoaded) {
            m_useMotionPlayer = !m_useMotionPlayer;
            m_guiPanel.useMotionPlayer = m_useMotionPlayer;
            if (!m_useMotionPlayer) {
                m_robot->resetJoints();
                m_robot->rotation = { -90.0f, 0.0f, 0.0f };
                m_robot->position = { 0.0f, 0.0f, 0.0f };
            }
        }
        
        if (m_motionLoaded && m_useMotionPlayer) {
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
        } else {
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

private:
    mf::Scene m_scene;
    mf::GuiPanel m_guiPanel;
    std::unique_ptr<mf::RobotEntity> m_robot;
    std::unique_ptr<mf::MotionPlayer> m_motionPlayer;
    bool m_robotLoaded = false;
    bool m_motionLoaded = false;
    bool m_useMotionPlayer = true;
    bool m_showGrid = true;
    
    std::vector<std::string> m_jointNames;
    size_t m_selectedJoint = 0;
    
    float m_animTime = 0.0f;
    bool m_playAnimation = true;
};

int main() {
    DemoApp app;
    app.run();
    return 0;
}
