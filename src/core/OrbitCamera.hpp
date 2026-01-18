#pragma once

#include <raylib.h>
#include <raymath.h>
#include <algorithm>

namespace mf {

/**
 * OrbitCamera - 轨道相机
 * 支持鼠标旋转、滚轮缩放、中键平移
 */
class OrbitCamera {
public:
    // 相机参数
    Vector3 target = { 0.0f, 0.0f, 0.0f };  // 看向的目标点
    float distance = 10.0f;                  // 距离目标的距离
    float yaw = 45.0f;                       // 水平旋转角度 (度)
    float pitch = 30.0f;                     // 垂直旋转角度 (度)
    
    // 限制参数
    float minDistance = 1.0f;
    float maxDistance = 100.0f;
    float minPitch = -89.0f;
    float maxPitch = 89.0f;
    
    // 灵敏度
    float rotateSensitivity = 0.3f;
    float zoomSensitivity = 1.0f;
    float panSensitivity = 0.01f;
    
    // 投影参数
    float fovy = 45.0f;
    float nearPlane = 0.1f;
    float farPlane = 1000.0f;

    OrbitCamera() = default;
    
    OrbitCamera(Vector3 target, float distance, float yaw = 45.0f, float pitch = 30.0f)
        : target(target), distance(distance), yaw(yaw), pitch(pitch) {}

    // 处理输入更新
    void update() {
        // 右键拖拽旋转
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 delta = GetMouseDelta();
            yaw -= delta.x * rotateSensitivity;
            pitch -= delta.y * rotateSensitivity;
            pitch = std::clamp(pitch, minPitch, maxPitch);
        }
        
        // 中键拖拽平移
        if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            Vector2 delta = GetMouseDelta();
            Vector3 right = getRightVector();
            Vector3 up = getUpVector();
            
            target = Vector3Add(target, Vector3Scale(right, -delta.x * panSensitivity * distance));
            target = Vector3Add(target, Vector3Scale(up, delta.y * panSensitivity * distance));
        }
        
        // 滚轮缩放
        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            distance -= wheel * zoomSensitivity * (distance * 0.1f);
            distance = std::clamp(distance, minDistance, maxDistance);
        }
    }

    // 获取相机位置
    Vector3 getPosition() const {
        float pitchRad = pitch * DEG2RAD;
        float yawRad = yaw * DEG2RAD;
        
        Vector3 offset = {
            distance * cosf(pitchRad) * sinf(yawRad),
            distance * sinf(pitchRad),
            distance * cosf(pitchRad) * cosf(yawRad)
        };
        
        return Vector3Add(target, offset);
    }

    // 获取前方向
    Vector3 getForwardVector() const {
        return Vector3Normalize(Vector3Subtract(target, getPosition()));
    }

    // 获取右方向
    Vector3 getRightVector() const {
        Vector3 forward = getForwardVector();
        Vector3 worldUp = { 0.0f, 1.0f, 0.0f };
        return Vector3Normalize(Vector3CrossProduct(forward, worldUp));
    }

    // 获取上方向
    Vector3 getUpVector() const {
        Vector3 forward = getForwardVector();
        Vector3 right = getRightVector();
        return Vector3CrossProduct(right, forward);
    }

    // 转换为 raylib Camera3D
    Camera3D toCamera3D() const {
        return Camera3D{
            .position = getPosition(),
            .target = target,
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
};

} // namespace mf
