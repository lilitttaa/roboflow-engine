#include "core/Application.hpp"
#include "core/OrbitCamera.hpp"
#include "scene/Scene.hpp"
#include "scene/Entity.hpp"
#include "robot/RobotEntity.hpp"
#include "motion/MotionPlayer.hpp"
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
        
        if (m_robotLoaded) {
            // 切换动画模式
            if (IsKeyPressed(KEY_M)) {
                m_useMotionPlayer = !m_useMotionPlayer;
                if (!m_useMotionPlayer) {
                    // 切换回简单动画时重置机器人
                    m_robot->resetJoints();
                    m_robot->rotation = { -90.0f, 0.0f, 0.0f };
                    m_robot->position = { 0.0f, 0.0f, 0.0f };
                }
            }
            
            if (m_motionLoaded && m_useMotionPlayer) {
                // 使用动作播放器
                
                // 播放/暂停
                if (IsKeyPressed(KEY_SPACE)) {
                    m_motionPlayer->togglePlay();
                }
                
                // 停止
                if (IsKeyPressed(KEY_S)) {
                    m_motionPlayer->stop();
                }
                
                // 调整播放速度
                if (IsKeyPressed(KEY_UP)) {
                    m_motionPlayer->setPlaybackSpeed(m_motionPlayer->getPlaybackSpeed() + 0.1f);
                }
                if (IsKeyPressed(KEY_DOWN)) {
                    m_motionPlayer->setPlaybackSpeed(std::max(0.1f, m_motionPlayer->getPlaybackSpeed() - 0.1f));
                }
                
                // 逐帧控制
                if (IsKeyPressed(KEY_LEFT)) {
                    m_motionPlayer->pause();
                    size_t frame = m_motionPlayer->getCurrentFrame();
                    if (frame > 0) m_motionPlayer->seekToFrame(frame - 1);
                }
                if (IsKeyPressed(KEY_RIGHT)) {
                    m_motionPlayer->pause();
                    m_motionPlayer->seekToFrame(m_motionPlayer->getCurrentFrame() + 1);
                }
                
                // 切换循环
                if (IsKeyPressed(KEY_L)) {
                    m_motionPlayer->setLoop(!m_motionPlayer->isLooping());
                }
                
                // 切换根位置/旋转应用
                if (IsKeyPressed(KEY_P)) {
                    m_motionPlayer->applyRootPosition = !m_motionPlayer->applyRootPosition;
                }
                if (IsKeyPressed(KEY_O)) {
                    m_motionPlayer->applyRootRotation = !m_motionPlayer->applyRootRotation;
                }
                
                m_motionPlayer->update(deltaTime);
            } else {
                // 使用简单动画
                m_animTime += deltaTime;
                
                if (m_playAnimation) {
                    // 手臂摆动
                    float armSwing = sinf(m_animTime * 2.0f) * 0.3f;
                    m_robot->setJointPosition("arm_l1_joint", armSwing);
                    m_robot->setJointPosition("arm_r1_joint", -armSwing);
                    
                    // 腿部微动
                    float legMove = sinf(m_animTime * 3.0f) * 0.1f;
                    m_robot->setJointPosition("leg_l1_joint", legMove);
                    m_robot->setJointPosition("leg_r1_joint", -legMove);
                }
                
                // 切换简单动画
                if (IsKeyPressed(KEY_SPACE)) {
                    m_playAnimation = !m_playAnimation;
                }
                
                // 关节控制
                if (IsKeyPressed(KEY_LEFT_BRACKET)) {
                    m_selectedJoint = (m_selectedJoint - 1 + m_jointNames.size()) % m_jointNames.size();
                }
                if (IsKeyPressed(KEY_RIGHT_BRACKET)) {
                    m_selectedJoint = (m_selectedJoint + 1) % m_jointNames.size();
                }
                
                // 调整选中关节
                if (!m_jointNames.empty()) {
                    const auto& jointName = m_jointNames[m_selectedJoint];
                    float pos = m_robot->getJointPosition(jointName);
                    
                    if (IsKeyDown(KEY_LEFT)) {
                        m_robot->setJointPosition(jointName, pos - 1.0f * deltaTime);
                    }
                    if (IsKeyDown(KEY_RIGHT)) {
                        m_robot->setJointPosition(jointName, pos + 1.0f * deltaTime);
                    }
                }
                
                // 重置关节
                if (IsKeyPressed(KEY_R)) {
                    m_robot->resetJoints();
                }
            }
            
            // 切换坐标轴显示
            if (IsKeyPressed(KEY_X)) {
                m_robot->showAxes = !m_robot->showAxes;
            }
            
            m_robot->update();
        }
    }

    void onRender() override {
        // 手动控制 3D 模式，确保机器人也在 3D 中渲染
        m_scene.begin3D();
        {
            // 渲染场景实体
            m_scene.getRoot()->render();
            
            // 绘制网格
            DrawGrid(20, 0.5f);
            
            // 绘制坐标轴
            DrawLine3D({ 0, 0, 0 }, { 1, 0, 0 }, RED);
            DrawLine3D({ 0, 0, 0 }, { 0, 1, 0 }, GREEN);
            DrawLine3D({ 0, 0, 0 }, { 0, 0, 1 }, BLUE);
            
            // 渲染机器人
            if (m_robotLoaded) {
                m_robot->render();
            }
        }
        m_scene.end3D();

        // 绘制UI
        DrawText("MotionFlow - Motion Playback", 10, 10, 20, WHITE);
        
        int y = 40;
        if (m_robotLoaded) {
            DrawText("Unitree G1 Humanoid Robot", 10, y, 16, LIME);
            y += 25;
            
            // 显示当前模式
            if (m_motionLoaded) {
                DrawText(TextFormat("Mode: %s", m_useMotionPlayer ? "Motion Playback" : "Simple Animation"), 10, y, 14, YELLOW);
                y += 20;
            }
            
            if (m_motionLoaded && m_useMotionPlayer) {
                // 动作播放器 UI
                DrawText(TextFormat("Motion: %d frames, %.1f FPS", 
                    (int)m_motionPlayer->getNumFrames(),
                    m_motionPlayer->getMotion() ? m_motionPlayer->getMotion()->getFPS() : 0.0f), 10, y, 14, LIGHTGRAY);
                y += 20;
                
                DrawText(TextFormat("Time: %.2f / %.2f s", 
                    m_motionPlayer->getCurrentTime(), 
                    m_motionPlayer->getDuration()), 10, y, 14, LIGHTGRAY);
                y += 20;
                
                DrawText(TextFormat("Frame: %d / %d", 
                    (int)m_motionPlayer->getCurrentFrame(), 
                    (int)m_motionPlayer->getNumFrames()), 10, y, 14, LIGHTGRAY);
                y += 20;
                
                DrawText(TextFormat("Speed: %.1fx", m_motionPlayer->getPlaybackSpeed()), 10, y, 14, LIGHTGRAY);
                y += 25;
                
                // 进度条
                float progress = m_motionPlayer->getDuration() > 0 ? 
                    m_motionPlayer->getCurrentTime() / m_motionPlayer->getDuration() : 0;
                DrawRectangle(10, y, 200, 10, DARKGRAY);
                DrawRectangle(10, y, (int)(200 * progress), 10, GREEN);
                y += 20;
                
                // 状态
                DrawText(TextFormat("Playing: %s", m_motionPlayer->isPlaying() ? "YES" : "NO"), 10, y, 14, 
                    m_motionPlayer->isPlaying() ? GREEN : RED);
                y += 18;
                DrawText(TextFormat("Loop: %s", m_motionPlayer->isLooping() ? "ON" : "OFF"), 10, y, 14, 
                    m_motionPlayer->isLooping() ? GREEN : GRAY);
                y += 18;
                DrawText(TextFormat("Root Pos: %s", m_motionPlayer->applyRootPosition ? "ON" : "OFF"), 10, y, 14, 
                    m_motionPlayer->applyRootPosition ? GREEN : GRAY);
                y += 18;
                DrawText(TextFormat("Root Rot: %s", m_motionPlayer->applyRootRotation ? "ON" : "OFF"), 10, y, 14, 
                    m_motionPlayer->applyRootRotation ? GREEN : GRAY);
                y += 25;
                
                // 控制说明
                DrawText("[Space] Play/Pause  [S] Stop", 10, y, 14, GRAY);
                y += 18;
                DrawText("[<-][->] Frame Step  [Up][Down] Speed", 10, y, 14, GRAY);
                y += 18;
                DrawText("[L] Loop  [P] Root Pos  [O] Root Rot", 10, y, 14, GRAY);
                y += 18;
                DrawText("[M] Switch Mode  [X] Axes  [Tab] Camera", 10, y, 14, GRAY);
            } else {
                // 简单动画 UI
                DrawText(TextFormat("Joints: %d", (int)m_jointNames.size()), 10, y, 14, LIGHTGRAY);
                y += 20;
                
                // 当前选中的关节
                if (!m_jointNames.empty()) {
                    const auto& jointName = m_jointNames[m_selectedJoint];
                    float pos = m_robot->getJointPosition(jointName);
                    float lower, upper;
                    m_robot->getJointLimits(jointName, lower, upper);
                    
                    DrawText(TextFormat("Selected: %s", jointName.c_str()), 10, y, 14, YELLOW);
                    y += 18;
                    DrawText(TextFormat("Position: %.3f rad (%.1f deg)", pos, pos * RAD2DEG), 10, y, 14, LIGHTGRAY);
                    y += 18;
                    DrawText(TextFormat("Limits: [%.2f, %.2f]", lower, upper), 10, y, 14, GRAY);
                    y += 25;
                }
                
                DrawText("[/] Select Joint  [<-][->] Adjust", 10, y, 14, GRAY);
                y += 18;
                DrawText("[R] Reset  [Space] Toggle Animation", 10, y, 14, GRAY);
                y += 18;
                if (m_motionLoaded) {
                    DrawText("[M] Switch to Motion Playback", 10, y, 14, GRAY);
                    y += 18;
                }
                DrawText("[X] Toggle Axes  [Tab] Camera Mode", 10, y, 14, GRAY);
                y += 25;
                
                DrawText(TextFormat("Animation: %s", m_playAnimation ? "ON" : "OFF"), 10, y, 14, 
                         m_playAnimation ? GREEN : RED);
            }
        } else {
            DrawText("Failed to load robot!", 10, y, 16, RED);
        }
        
        // 相机信息
        y = getHeight() - 80;
        DrawText(TextFormat("Camera: %s", m_scene.getCameraModeName()), 10, y, 14, YELLOW);
        
        DrawFPS(getWidth() - 100, 10);
    }

private:
    mf::Scene m_scene;
    std::unique_ptr<mf::RobotEntity> m_robot;
    std::unique_ptr<mf::MotionPlayer> m_motionPlayer;
    bool m_robotLoaded = false;
    bool m_motionLoaded = false;
    bool m_useMotionPlayer = true;  // 默认使用动作播放器（如果有动作文件）
    
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
