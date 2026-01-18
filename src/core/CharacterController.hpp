#pragma once

#include <raylib.h>
#include <raymath.h>
#include <cmath>
#include <cstdio>

namespace mf {

// 前向声明
class ThirdPersonCamera;

/**
 * 控制模式
 */
enum class ControlMode {
    FreeRotation,  // 自由朝向：机器人面向移动方向（按键时旋转）
    LockToCamera   // 锁定相机：机器人始终面向相机前方
};

/**
 * CharacterController - 简化版（仅旋转，无移动）
 * 用于调试坐标系和旋转逻辑
 */
class CharacterController {
public:
    // 绑定对象
    Vector3* targetPosition = nullptr;  // 不再使用，保留兼容性
    ThirdPersonCamera* camera = nullptr;
    
    // 控制模式 - 默认 Lock 模式
    ControlMode controlMode = ControlMode::LockToCamera;
    
    // 旋转参数
    float turnSpeed = 8.0f;  // 转向平滑度
    
    // 移动参数（暂时保留给 GUI，但不使用）
    float walkSpeed = 1.2f;
    float runSpeed = 2.8f;
    float speed = 0.0f;  // 当前速度（不使用）
    
    // 状态输出
    float facingAngle = 0.0f;  // 当前朝向角度 (度)，基于 raylib 坐标系
    Vector2 inputVector = {0, 0};
    bool isMoving = false;
    bool isRunning = false;
    bool enabled = true;
    
    // 调试帧计数
    int debugFrameCount = 0;

    CharacterController() = default;

    /**
     * 获取输入向量
     */
    Vector2 getInputVector() const {
        if (!enabled) return {0, 0};
        
        Vector2 input = {0, 0};
        if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))    input.y += 1.0f;
        if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))  input.y -= 1.0f;
        if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) input.x += 1.0f;
        if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))  input.x -= 1.0f;
        
        float len = Vector2Length(input);
        if (len > 1.0f) {
            input = Vector2Scale(input, 1.0f / len);
        }
        return input;
    }

    /**
     * 切换控制模式
     */
    void toggleControlMode() {
        controlMode = (controlMode == ControlMode::FreeRotation) 
                      ? ControlMode::LockToCamera 
                      : ControlMode::FreeRotation;
        printf("\n[Controller] Mode changed to: %s\n", 
               controlMode == ControlMode::LockToCamera ? "LOCK" : "FREE");
    }

    /**
     * 更新控制器（仅旋转逻辑）
     */
    void update(float deltaTime);
};

} // namespace mf

// 实现部分
#include "ThirdPersonCamera.hpp"

namespace mf {

/**
 * 角度线性插值（处理环绕）
 */
inline float LerpAngle(float from, float to, float t) {
    float diff = to - from;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return from + diff * std::clamp(t, 0.0f, 1.0f);
}

inline void CharacterController::update(float deltaTime) {
    if (!enabled || !camera) return;
    
    // 按 C 键切换模式
    if (IsKeyPressed(KEY_C)) {
        toggleControlMode();
    }
    
    // 读取输入
    inputVector = getInputVector();
    isMoving = Vector2Length(inputVector) > 0.1f;
    
    // 获取相机 yaw
    float cameraYaw = camera->yaw;
    
    // 调试输出标题（每 120 帧一次）
    bool shouldDebug = (debugFrameCount++ % 120 == 0) || isMoving;
    
    if (shouldDebug && debugFrameCount % 30 == 0) {
        printf("\n========== [Debug Frame %d] ==========\n", debugFrameCount);
        printf("[Input] W/S/A/D = (%.2f, %.2f), isMoving=%d\n", 
               inputVector.x, inputVector.y, isMoving);
        printf("[Camera] yaw=%.1f\n", cameraYaw);
        printf("[Mode] %s\n", controlMode == ControlMode::LockToCamera ? "LOCK" : "FREE");
    }
    
    // 目标朝向角度（在 raylib 坐标系中）
    float targetFacingAngle = facingAngle;
    
    if (controlMode == ControlMode::LockToCamera) {
        // ========== LOCK 模式 ==========
        // 机器人始终面向相机看向的方向
        // 
        // 相机前方向 = (-sin(yaw), 0, -cos(yaw))
        //   yaw=0:  (0, 0, -1) = -Z
        //   yaw=90: (-1, 0, 0) = -X
        //
        // facingAngle = atan2(forward.x, forward.z)
        //   yaw=0:  atan2(0, -1) = 180° (面向 -Z) ✓
        //   yaw=90: atan2(-1, 0) = -90° (面向 -X) ✓
        //
        float yawRad = cameraYaw * DEG2RAD;
        targetFacingAngle = atan2f(-sinf(yawRad), -cosf(yawRad)) * RAD2DEG;
        
        // 快速同步（Lock 模式立即跟随）
        float t = 1.0f - std::exp(-20.0f * deltaTime);
        facingAngle = LerpAngle(facingAngle, targetFacingAngle, t);
        
        if (shouldDebug && debugFrameCount % 30 == 0) {
            printf("[LOCK] cameraYaw=%.1f → targetFacing=%.1f, facingAngle=%.1f\n", 
                   cameraYaw, targetFacingAngle, facingAngle);
        }
        
    } else {
        // ========== FREE 模式 ==========
        // 按方向键时，机器人转向相机的相对方向
        if (isMoving) {
            // 计算输入对应的世界方向
            // input.y=1 (W) → 相机前方
            // input.x=1 (D) → 相机右方
            float yawRad = cameraYaw * DEG2RAD;
            
            // 相机前方向（水平）
            Vector3 camForward = { -sinf(yawRad), 0.0f, -cosf(yawRad) };
            // 相机右方向（水平）
            Vector3 camRight = { cosf(yawRad), 0.0f, -sinf(yawRad) };
            
            // 世界空间移动方向
            Vector3 moveDir = {
                camForward.x * inputVector.y + camRight.x * inputVector.x,
                0.0f,
                camForward.z * inputVector.y + camRight.z * inputVector.x
            };
            
            float len = sqrtf(moveDir.x * moveDir.x + moveDir.z * moveDir.z);
            if (len > 0.001f) {
                moveDir.x /= len;
                moveDir.z /= len;
                
                // 目标朝向 = 移动方向的角度
                // atan2(x, z): 0=+Z, 90=+X, 180=-Z, -90=-X
                targetFacingAngle = atan2f(moveDir.x, moveDir.z) * RAD2DEG;
            }
            
            if (shouldDebug && debugFrameCount % 30 == 0) {
                printf("[FREE] camForward=(%.2f,%.2f,%.2f) camRight=(%.2f,%.2f,%.2f)\n",
                       camForward.x, camForward.y, camForward.z,
                       camRight.x, camRight.y, camRight.z);
                printf("[FREE] moveDir=(%.2f,%.2f,%.2f) targetFacing=%.1f\n",
                       moveDir.x, moveDir.y, moveDir.z, targetFacingAngle);
            }
        }
        
        // 平滑转向
        float t = 1.0f - std::exp(-turnSpeed * deltaTime);
        facingAngle = LerpAngle(facingAngle, targetFacingAngle, t);
        
        if (shouldDebug && debugFrameCount % 30 == 0) {
            printf("[FREE] facingAngle=%.1f\n", facingAngle);
        }
    }
    
    // 归一化 facingAngle
    while (facingAngle > 180.0f) facingAngle -= 360.0f;
    while (facingAngle < -180.0f) facingAngle += 360.0f;
    
    if (shouldDebug && debugFrameCount % 30 == 0) {
        printf("[Output] facingAngle=%.1f (this goes to RobotEntity)\n", facingAngle);
        printf("==========================================\n");
    }
}

} // namespace mf
