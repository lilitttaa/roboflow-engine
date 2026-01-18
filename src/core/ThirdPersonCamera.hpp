#pragma once

#include <raylib.h>
#include <raymath.h>
#include <algorithm>
#include <cmath>

namespace mf {

/**
 * ThirdPersonCamera - 第三人称跟随相机
 * 跟随目标对象，支持鼠标旋转视角、滚轮缩放距离
 */
class ThirdPersonCamera {
public:
    // 跟随目标（指向目标位置的指针）
    Vector3* followTarget = nullptr;
    
    // 相机臂参数
    Vector3 armOffset = {0.0f, 1.2f, 0.0f};  // 目标点偏移（看向肩膀高度）
    float armLength = 3.0f;                   // 相机臂长度
    float minArmLength = 1.0f;
    float maxArmLength = 10.0f;
    
    // 旋转参数
    float yaw = 0.0f;      // 水平旋转角度 (度)
    float pitch = 15.0f;   // 垂直旋转角度 (度)
    float minPitch = -60.0f;
    float maxPitch = 80.0f;
    
    // 灵敏度
    float rotateSensitivity = 0.3f;
    float zoomSensitivity = 0.5f;
    float followSpeed = 8.0f;  // 跟随平滑度
    
    // 投影参数
    float fovy = 50.0f;
    float nearPlane = 0.1f;
    float farPlane = 1000.0f;
    
    // 鼠标锁定状态
    bool mouseLocked = true;

    ThirdPersonCamera() {
        m_currentTarget = {0.0f, 1.2f, 0.0f};
    }

    /**
     * 更新相机
     * @param deltaTime 帧时间
     */
    void update(float deltaTime) {
        // 首次进入时锁定鼠标（确保窗口已创建）
        if (!m_initialized && IsWindowReady()) {
            m_initialized = true;
            if (mouseLocked) {
                DisableCursor();
            }
        }
        
        // Alt 键切换鼠标锁定
        if (IsKeyPressed(KEY_LEFT_ALT) || IsKeyPressed(KEY_RIGHT_ALT)) {
            mouseLocked = !mouseLocked;
            if (mouseLocked) {
                DisableCursor();
            } else {
                EnableCursor();
            }
        }
        
        // 鼠标旋转（锁定时才响应）
        if (mouseLocked) {
            Vector2 delta = GetMouseDelta();
            yaw -= delta.x * rotateSensitivity;
            pitch += delta.y * rotateSensitivity;
            pitch = std::clamp(pitch, minPitch, maxPitch);
            
            // 保持 yaw 在 -180 到 180 范围
            while (yaw > 180.0f) yaw -= 360.0f;
            while (yaw < -180.0f) yaw += 360.0f;
        }
        
        // 滚轮缩放
        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            armLength -= wheel * zoomSensitivity;
            armLength = std::clamp(armLength, minArmLength, maxArmLength);
        }
        
        // 平滑跟随目标
        if (followTarget) {
            Vector3 targetPos = Vector3Add(*followTarget, armOffset);
            
            // 使用指数平滑
            float t = 1.0f - std::exp(-followSpeed * deltaTime);
            m_currentTarget = Vector3Lerp(m_currentTarget, targetPos, t);
        }
    }

    /**
     * 获取相机位置
     */
    Vector3 getPosition() const {
        float pitchRad = pitch * DEG2RAD;
        float yawRad = yaw * DEG2RAD;
        
        // 球面坐标计算偏移
        Vector3 offset = {
            armLength * cosf(pitchRad) * sinf(yawRad),
            armLength * sinf(pitchRad),
            armLength * cosf(pitchRad) * cosf(yawRad)
        };
        
        return Vector3Add(m_currentTarget, offset);
    }

    /**
     * 获取相机看向的目标点
     */
    Vector3 getTarget() const {
        return m_currentTarget;
    }

    /**
     * 获取相机前方向（用于渲染）
     */
    Vector3 getForwardVector() const {
        return Vector3Normalize(Vector3Subtract(m_currentTarget, getPosition()));
    }

    /**
     * 获取相机前方向（水平，用于角色移动）
     * 只考虑 yaw 角度，忽略 pitch
     */
    Vector3 getForwardHorizontal() const {
        float yawRad = yaw * DEG2RAD;
        return Vector3Normalize({
            -sinf(yawRad),
            0.0f,
            -cosf(yawRad)
        });
    }

    /**
     * 获取相机右方向（水平，用于角色移动）
     */
    Vector3 getRightHorizontal() const {
        float yawRad = yaw * DEG2RAD;
        return Vector3Normalize({
            cosf(yawRad),
            0.0f,
            -sinf(yawRad)
        });
    }

    /**
     * 获取右方向
     */
    Vector3 getRightVector() const {
        Vector3 forward = getForwardVector();
        Vector3 worldUp = {0.0f, 1.0f, 0.0f};
        return Vector3Normalize(Vector3CrossProduct(forward, worldUp));
    }

    /**
     * 获取上方向
     */
    Vector3 getUpVector() const {
        Vector3 forward = getForwardVector();
        Vector3 right = getRightVector();
        return Vector3CrossProduct(right, forward);
    }

    /**
     * 转换为 raylib Camera3D
     */
    Camera3D toCamera3D() const {
        return Camera3D{
            .position = getPosition(),
            .target = m_currentTarget,
            .up = {0.0f, 1.0f, 0.0f},
            .fovy = fovy,
            .projection = CAMERA_PERSPECTIVE
        };
    }

    /**
     * 开始 3D 渲染
     */
    void begin3D() const {
        BeginMode3D(toCamera3D());
    }

    /**
     * 结束 3D 渲染
     */
    void end3D() const {
        EndMode3D();
    }

    /**
     * 设置跟随目标并初始化当前目标位置
     */
    void setFollowTarget(Vector3* target) {
        followTarget = target;
        if (target) {
            m_currentTarget = Vector3Add(*target, armOffset);
        }
    }

    /**
     * 重置相机角度
     */
    void resetAngles() {
        yaw = 0.0f;
        pitch = 15.0f;
    }

private:
    Vector3 m_currentTarget = {0.0f, 1.2f, 0.0f};  // 平滑后的目标位置
    bool m_initialized = false;  // 是否已初始化（窗口就绪后）
};

} // namespace mf
