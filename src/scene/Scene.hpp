#pragma once

#include "Entity.hpp"
#include "../core/OrbitCamera.hpp"
#include "../core/FlyCamera.hpp"
#include <memory>

namespace mf {

/**
 * CameraMode - 相机模式
 */
enum class CameraMode {
    Orbit,  // 轨道相机（默认）
    Fly     // 飞行相机（UE4风格）
};

/**
 * Scene - 场景类
 * 管理场景图和相机
 */
class Scene {
public:
    OrbitCamera orbitCamera;
    FlyCamera flyCamera;
    CameraMode cameraMode = CameraMode::Orbit;
    
    Color clearColor = { 30, 30, 35, 255 };
    bool showGrid = true;
    bool showAxes = true;
    int gridSize = 10;
    float gridSpacing = 1.0f;

    Scene() {
        m_root = std::make_unique<Entity>("Root");
        
        // 同步两个相机的初始位置
        syncCameras();
    }

    // 获取根节点
    Entity* getRoot() { return m_root.get(); }

    // 在根节点下创建实体
    template<typename T = Entity, typename... Args>
    T* createEntity(Args&&... args) {
        return m_root->createChild<T>(std::forward<Args>(args)...);
    }

    // 按名称查找实体
    Entity* findEntity(const std::string& name) {
        if (m_root->name == name) return m_root.get();
        return m_root->findChild(name);
    }

    // 切换相机模式
    void setCameraMode(CameraMode mode) {
        if (mode != cameraMode) {
            cameraMode = mode;
            syncCameras();
        }
    }

    // 切换到下一个相机模式
    void toggleCameraMode() {
        if (cameraMode == CameraMode::Orbit) {
            setCameraMode(CameraMode::Fly);
        } else {
            setCameraMode(CameraMode::Orbit);
        }
    }

    // 获取当前相机模式名称
    const char* getCameraModeName() const {
        switch (cameraMode) {
            case CameraMode::Orbit: return "Orbit Camera";
            case CameraMode::Fly: return "Fly Camera (UE4)";
        }
        return "Unknown";
    }

    // 更新场景
    void update(float deltaTime) {
        // 按 Tab 切换相机
        if (IsKeyPressed(KEY_TAB)) {
            toggleCameraMode();
        }
        
        // 更新当前相机
        switch (cameraMode) {
            case CameraMode::Orbit:
                orbitCamera.update();
                break;
            case CameraMode::Fly:
                flyCamera.update(deltaTime);
                break;
        }
        
        m_root->update(deltaTime);
    }

    // 渲染场景（自动管理 3D 模式）
    void render() {
        ClearBackground(clearColor);
        
        // 使用当前相机开始3D渲染
        beginCamera3D();
        {
            // 绘制网格
            if (showGrid) {
                DrawGrid(gridSize, gridSpacing);
            }
            
            // 绘制坐标轴
            if (showAxes) {
                float axisLength = 3.0f;
                DrawLine3D({ 0, 0, 0 }, { axisLength, 0, 0 }, RED);
                DrawLine3D({ 0, 0, 0 }, { 0, axisLength, 0 }, GREEN);
                DrawLine3D({ 0, 0, 0 }, { 0, 0, axisLength }, BLUE);
            }
            
            // 渲染场景图
            m_root->render();
        }
        endCamera3D();
    }

    // 开始 3D 渲染模式（手动控制）
    void begin3D() {
        ClearBackground(clearColor);
        beginCamera3D();
    }

    // 结束 3D 渲染模式（手动控制）
    void end3D() {
        endCamera3D();
    }

    // 检查飞行相机是否激活
    bool isFlyModeActive() const {
        return cameraMode == CameraMode::Fly && flyCamera.isActive();
    }

private:
    std::unique_ptr<Entity> m_root;

    void beginCamera3D() {
        switch (cameraMode) {
            case CameraMode::Orbit:
                orbitCamera.begin3D();
                break;
            case CameraMode::Fly:
                flyCamera.begin3D();
                break;
        }
    }

    void endCamera3D() {
        switch (cameraMode) {
            case CameraMode::Orbit:
                orbitCamera.end3D();
                break;
            case CameraMode::Fly:
                flyCamera.end3D();
                break;
        }
    }

    // 同步相机位置（切换时保持视角连续）
    void syncCameras() {
        if (cameraMode == CameraMode::Fly) {
            // 从轨道相机位置设置飞行相机
            flyCamera.position = orbitCamera.getPosition();
            flyCamera.lookAt(flyCamera.position, orbitCamera.target);
        } else {
            // 从飞行相机位置设置轨道相机
            orbitCamera.target = flyCamera.getTarget();
            orbitCamera.distance = 5.0f;
        }
    }
};

} // namespace mf
