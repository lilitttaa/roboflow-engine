#include "core/Application.hpp"
#include "core/OrbitCamera.hpp"
#include "scene/Scene.hpp"
#include "scene/Entity.hpp"
#include <raylib.h>
#include <rlgl.h>

/**
 * DemoApp - 演示应用
 * 展示场景图和轨道相机
 */
class DemoApp : public mf::Application {
public:
    DemoApp() : Application(Config{
        .width = 1280,
        .height = 720,
        .title = "MotionFlow - Step 2 Demo",
        .targetFPS = 60,
        .vsync = true
    }) {}

protected:
    void onInit() override {
        // 配置轨道相机
        m_scene.orbitCamera.target = { 0.0f, 1.0f, 0.0f };
        m_scene.orbitCamera.distance = 8.0f;
        m_scene.orbitCamera.yaw = 45.0f;
        m_scene.orbitCamera.pitch = 25.0f;
        
        // 配置飞行相机
        m_scene.flyCamera.position = { 5.0f, 3.0f, 5.0f };
        m_scene.flyCamera.lookAt(m_scene.flyCamera.position, { 0.0f, 1.0f, 0.0f });

        // 创建地板
        auto* floor = m_scene.createEntity<mf::MeshEntity>("Floor");
        floor->primitiveType = mf::MeshEntity::PrimitiveType::Plane;
        floor->size = { 10.0f, 1.0f, 10.0f };
        floor->color = { 60, 60, 65, 255 };

        // 创建主立方体
        m_cube = m_scene.createEntity<mf::MeshEntity>("MainCube");
        m_cube->transform.position = { 0.0f, 1.0f, 0.0f };
        m_cube->size = { 1.5f, 1.5f, 1.5f };
        m_cube->color = { 80, 160, 220, 255 };

        // 创建子立方体（绕主立方体旋转）
        m_orbitCube = m_cube->createChild<mf::MeshEntity>("OrbitCube");
        m_orbitCube->transform.position = { 2.5f, 0.0f, 0.0f };
        m_orbitCube->size = { 0.6f, 0.6f, 0.6f };
        m_orbitCube->color = { 220, 120, 80, 255 };

        // 创建另一个子立方体
        auto* orbitCube2 = m_cube->createChild<mf::MeshEntity>("OrbitCube2");
        orbitCube2->transform.position = { 0.0f, 0.0f, 2.5f };
        orbitCube2->size = { 0.5f, 0.5f, 0.5f };
        orbitCube2->color = { 120, 220, 80, 255 };

        // 创建球体
        auto* sphere = m_scene.createEntity<mf::MeshEntity>("Sphere");
        sphere->primitiveType = mf::MeshEntity::PrimitiveType::Sphere;
        sphere->transform.position = { -3.0f, 0.8f, 0.0f };
        sphere->size = { 1.6f, 1.6f, 1.6f };
        sphere->color = { 200, 80, 180, 255 };

        // 创建圆柱体
        auto* cylinder = m_scene.createEntity<mf::MeshEntity>("Cylinder");
        cylinder->primitiveType = mf::MeshEntity::PrimitiveType::Cylinder;
        cylinder->transform.position = { 3.0f, 1.0f, 0.0f };
        cylinder->size = { 1.0f, 2.0f, 1.0f };
        cylinder->color = { 80, 200, 180, 255 };
    }

    void onUpdate(float deltaTime) override {
        m_scene.update(deltaTime);

        // 旋转主立方体（子物体会跟随旋转）
        m_cube->transform.rotate({ 0, 1, 0 }, 30.0f * deltaTime);

        // 子立方体自转
        m_orbitCube->transform.rotate({ 1, 0, 0 }, 90.0f * deltaTime);
    }

    void onRender() override {
        m_scene.render();

        // 绘制UI
        DrawText("MotionFlow Engine - Step 2", 10, 10, 20, WHITE);
        DrawText("Scene Graph + Dual Camera System", 10, 35, 16, LIGHTGRAY);
        
        // 相机模式
        int y = 65;
        DrawText(TextFormat("Camera: %s", m_scene.getCameraModeName()), 10, y, 16, YELLOW);
        DrawText("[Tab] Switch Camera Mode", 10, y + 25, 14, GRAY);
        
        y += 55;
        if (m_scene.cameraMode == mf::CameraMode::Orbit) {
            DrawText("-- Orbit Camera --", 10, y, 14, LIGHTGRAY);
            DrawText("Right Mouse: Rotate", 10, y + 20, 14, GRAY);
            DrawText("Middle Mouse: Pan", 10, y + 40, 14, GRAY);
            DrawText("Scroll: Zoom", 10, y + 60, 14, GRAY);
        } else {
            DrawText("-- Fly Camera (UE4) --", 10, y, 14, LIGHTGRAY);
            DrawText("Hold Right Mouse to activate", 10, y + 20, 14, GRAY);
            DrawText("WASD: Move", 10, y + 40, 14, GRAY);
            DrawText("Q/E: Down/Up", 10, y + 60, 14, GRAY);
            DrawText("Shift: Fast | Ctrl: Slow", 10, y + 80, 14, GRAY);
            DrawText("Scroll: Adjust Speed", 10, y + 100, 14, GRAY);
            
            if (m_scene.isFlyModeActive()) {
                DrawText(TextFormat("Speed: %.1f", m_scene.flyCamera.moveSpeed), 10, y + 125, 14, GREEN);
            }
        }
        
        DrawFPS(getWidth() - 100, 10);
    }

private:
    mf::Scene m_scene;
    mf::MeshEntity* m_cube = nullptr;
    mf::MeshEntity* m_orbitCube = nullptr;
};

int main() {
    DemoApp app;
    app.run();
    return 0;
}
