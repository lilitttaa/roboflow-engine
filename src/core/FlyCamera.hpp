#pragma once

#include <raylib.h>
#include <raymath.h>
#include <algorithm>

namespace mf {

/**
 * FlyCamera - UE4风格飞行相机
 * 按住右键时：WASD移动、QE上下、鼠标控制视角
 * 滚轮调整移动速度
 */
class FlyCamera {
public:
    // 相机位置和朝向
    Vector3 position = { 5.0f, 3.0f, 5.0f };
    float yaw = -135.0f;    // 水平旋转 (度)
    float pitch = -20.0f;   // 垂直旋转 (度)
    
    // 移动参数
    float moveSpeed = 5.0f;
    float fastMultiplier = 3.0f;  // Shift加速倍数
    float slowMultiplier = 0.2f;  // Ctrl减速倍数
    
    // 旋转参数
    float lookSensitivity = 0.15f;
    float minPitch = -89.0f;
    float maxPitch = 89.0f;
    
    // 速度调整
    float minSpeed = 0.5f;
    float maxSpeed = 50.0f;
    float speedScrollRate = 0.5f;
    
    // 投影参数
    float fovy = 60.0f;
    float nearPlane = 0.1f;
    float farPlane = 1000.0f;

    FlyCamera() = default;
    
    FlyCamera(Vector3 pos, float yaw = -45.0f, float pitch = -20.0f)
        : position(pos), yaw(yaw), pitch(pitch) {}

    // 处理输入更新
    void update(float deltaTime) {
        // 仅在按住右键时激活
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            // 隐藏鼠标
            if (!m_captured) {
                DisableCursor();
                m_captured = true;
            }
            
            // 鼠标旋转
            Vector2 mouseDelta = GetMouseDelta();
            yaw += mouseDelta.x * lookSensitivity;
            pitch -= mouseDelta.y * lookSensitivity;
            pitch = std::clamp(pitch, minPitch, maxPitch);
            
            // 计算移动速度
            float currentSpeed = moveSpeed;
            if (IsKeyDown(KEY_LEFT_SHIFT)) {
                currentSpeed *= fastMultiplier;
            }
            if (IsKeyDown(KEY_LEFT_CONTROL)) {
                currentSpeed *= slowMultiplier;
            }
            
            // 获取方向向量
            Vector3 forward = getForwardVector();
            Vector3 right = getRightVector();
            Vector3 up = { 0.0f, 1.0f, 0.0f };
            
            // WASD 移动
            Vector3 velocity = { 0, 0, 0 };
            
            if (IsKeyDown(KEY_W)) {
                velocity = Vector3Add(velocity, forward);
            }
            if (IsKeyDown(KEY_S)) {
                velocity = Vector3Subtract(velocity, forward);
            }
            if (IsKeyDown(KEY_D)) {
                velocity = Vector3Add(velocity, right);
            }
            if (IsKeyDown(KEY_A)) {
                velocity = Vector3Subtract(velocity, right);
            }
            if (IsKeyDown(KEY_E) || IsKeyDown(KEY_SPACE)) {
                velocity = Vector3Add(velocity, up);
            }
            if (IsKeyDown(KEY_Q)) {
                velocity = Vector3Subtract(velocity, up);
            }
            
            // 归一化并应用速度
            if (Vector3Length(velocity) > 0.0f) {
                velocity = Vector3Normalize(velocity);
                velocity = Vector3Scale(velocity, currentSpeed * deltaTime);
                position = Vector3Add(position, velocity);
            }
            
            // 滚轮调整移动速度
            float wheel = GetMouseWheelMove();
            if (wheel != 0) {
                moveSpeed += wheel * speedScrollRate * moveSpeed * 0.2f;
                moveSpeed = std::clamp(moveSpeed, minSpeed, maxSpeed);
            }
        } else {
            // 释放右键时显示鼠标
            if (m_captured) {
                EnableCursor();
                m_captured = false;
            }
        }
    }

    // 获取前方向（水平）
    Vector3 getForwardVector() const {
        float yawRad = yaw * DEG2RAD;
        float pitchRad = pitch * DEG2RAD;
        
        return Vector3Normalize({
            cosf(pitchRad) * cosf(yawRad),
            sinf(pitchRad),
            cosf(pitchRad) * sinf(yawRad)
        });
    }

    // 获取右方向
    Vector3 getRightVector() const {
        float yawRad = yaw * DEG2RAD;
        return Vector3Normalize({
            -sinf(yawRad),
            0.0f,
            cosf(yawRad)
        });
    }

    // 获取上方向
    Vector3 getUpVector() const {
        Vector3 forward = getForwardVector();
        Vector3 right = getRightVector();
        return Vector3CrossProduct(right, forward);
    }

    // 获取目标点
    Vector3 getTarget() const {
        return Vector3Add(position, getForwardVector());
    }

    // 转换为 raylib Camera3D
    Camera3D toCamera3D() const {
        return Camera3D{
            .position = position,
            .target = getTarget(),
            .up = { 0.0f, 1.0f, 0.0f },
            .fovy = fovy,
            .projection = CAMERA_PERSPECTIVE
        };
    }

    // 开始3D渲染
    void begin3D() const {
        BeginMode3D(toCamera3D());
    }

    // 结束3D渲染
    void end3D() const {
        EndMode3D();
    }

    // 设置位置并看向目标
    void lookAt(Vector3 pos, Vector3 target) {
        position = pos;
        Vector3 dir = Vector3Normalize(Vector3Subtract(target, pos));
        pitch = asinf(dir.y) * RAD2DEG;
        yaw = atan2f(dir.z, dir.x) * RAD2DEG;
    }

    // 是否处于激活状态（按住右键）
    bool isActive() const { return m_captured; }

private:
    bool m_captured = false;
};

} // namespace mf
