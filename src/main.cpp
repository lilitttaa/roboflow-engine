#include "core/Application.hpp"
#include "core/OrbitCamera.hpp"
#include "scene/Scene.hpp"
#include "scene/Entity.hpp"
#include "robot/RobotEntity.hpp"
#include <raylib.h>
#include <rlgl.h>
#include <cmath>
#include <iostream>

/**
 * DemoApp - 演示应用
 * 展示 URDF 机器人模型加载
 */
class DemoApp : public mf::Application {
public:
    DemoApp() : Application(Config{
        .width = 1280,
        .height = 720,
        .title = "MotionFlow - URDF Robot Viewer",
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
        } else {
            std::cerr << "Failed to load robot URDF!" << std::endl;
        }
    }

    void onUpdate(float deltaTime) override {
        m_scene.update(deltaTime);
        
        if (m_robotLoaded) {
            // 动画时间
            m_animTime += deltaTime;
            
            // 简单的呼吸动画 - 让机器人微微上下运动
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
            
            // 切换动画
            if (IsKeyPressed(KEY_SPACE)) {
                m_playAnimation = !m_playAnimation;
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
        DrawText("MotionFlow - URDF Robot Viewer", 10, 10, 20, WHITE);
        
        int y = 40;
        if (m_robotLoaded) {
            DrawText("Unitree G1 Humanoid Robot", 10, y, 16, LIME);
            y += 25;
            
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
            DrawText("[X] Toggle Axes  [Tab] Camera Mode", 10, y, 14, GRAY);
            y += 25;
            
            DrawText(TextFormat("Animation: %s", m_playAnimation ? "ON" : "OFF"), 10, y, 14, 
                     m_playAnimation ? GREEN : RED);
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
    bool m_robotLoaded = false;
    
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
