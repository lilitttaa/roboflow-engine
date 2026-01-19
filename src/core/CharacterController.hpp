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
    FreeRotation,  // 自由朝向：机器人面向移动方向，始终往前走
    LockToCamera   // 锁定相机：机器人始终面向相机前方，可前后左右移动
};

/**
 * CharacterController - 角色控制器
 * 处理键盘输入，控制角色移动和转向
 */
class CharacterController {
public:
    // 绑定对象
    Vector3* targetPosition = nullptr;  // 控制的目标位置
    ThirdPersonCamera* camera = nullptr;
    
    // 控制模式 - 默认 Lock 模式
    ControlMode controlMode = ControlMode::LockToCamera;
    
    // 旋转参数
    float turnSpeed = 8.0f;  // 转向平滑度
    
    // 移动参数
    float walkSpeed = 1.2f;       // 行走速度 (m/s)
    float runSpeed = 2.8f;        // 跑步速度 (m/s)
    float acceleration = 10.0f;   // 加速度
    float deceleration = 15.0f;   // 减速度
    
    // 状态输出
    float facingAngle = 0.0f;     // 当前朝向角度 (度)
    Vector3 velocity = {0, 0, 0}; // 当前速度向量
    float speed = 0.0f;           // 当前速度标量
    Vector2 inputVector = {0, 0}; // 输入向量
    bool isMoving = false;
    bool isRunning = false;
    bool enabled = true;
    bool updatePosition = true;  // 是否更新位置（Motion Matching 模式下设为 false）
    
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
     * 更新控制器
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
    isRunning = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
    isMoving = Vector2Length(inputVector) > 0.1f;
    
    // 获取相机 yaw
    float cameraYaw = camera->yaw;
    float yawRad = cameraYaw * DEG2RAD;
    
    // 相机方向（水平）
    Vector3 camForward = { -sinf(yawRad), 0.0f, -cosf(yawRad) };
    Vector3 camRight = { cosf(yawRad), 0.0f, -sinf(yawRad) };
    
    // 目标朝向角度
    float targetFacingAngle = facingAngle;
    
    // 目标速度向量
    Vector3 targetVelocity = {0, 0, 0};
    float targetSpeed = isRunning ? runSpeed : walkSpeed;
    
    if (controlMode == ControlMode::LockToCamera) {
        // ========== LOCK 模式 ==========
        // 机器人始终面向相机看向的方向
        // 移动方向基于相机的相对方向（前后左右）
        
        // 朝向 = 相机前方
        targetFacingAngle = atan2f(-sinf(yawRad), -cosf(yawRad)) * RAD2DEG;
        
        // 快速同步朝向
        float t = 1.0f - std::exp(-20.0f * deltaTime);
        facingAngle = LerpAngle(facingAngle, targetFacingAngle, t);
        
        // 移动方向基于相机
        if (isMoving) {
            // W/S = 相机前后，A/D = 相机左右
            targetVelocity = {
                (camForward.x * inputVector.y + camRight.x * inputVector.x) * targetSpeed,
                0.0f,
                (camForward.z * inputVector.y + camRight.z * inputVector.x) * targetSpeed
            };
        }
        
    } else {
        // ========== FREE 模式 ==========
        // 按方向键时，机器人转向相机的相对方向
        // 移动方向始终是角色当前朝向的前方
        
        if (isMoving) {
            // 计算输入对应的世界方向（用于确定目标朝向）
            Vector3 inputDir = {
                camForward.x * inputVector.y + camRight.x * inputVector.x,
                0.0f,
                camForward.z * inputVector.y + camRight.z * inputVector.x
            };
            
            float len = sqrtf(inputDir.x * inputDir.x + inputDir.z * inputDir.z);
            if (len > 0.001f) {
                inputDir.x /= len;
                inputDir.z /= len;
                
                // 目标朝向 = 输入方向
                targetFacingAngle = atan2f(inputDir.x, inputDir.z) * RAD2DEG;
            }
        }
        
        // 平滑转向
        float t = 1.0f - std::exp(-turnSpeed * deltaTime);
        facingAngle = LerpAngle(facingAngle, targetFacingAngle, t);
        
        // 移动方向 = 角色当前朝向的前方
        if (isMoving) {
            float facingRad = facingAngle * DEG2RAD;
            // facingAngle: 0=+Z, 90=+X, 180=-Z, -90=-X
            // 前方向 = (sin(facing), 0, cos(facing))
            Vector3 charForward = { sinf(facingRad), 0.0f, cosf(facingRad) };
            targetVelocity = Vector3Scale(charForward, targetSpeed);
        }
    }
    
    // 归一化 facingAngle
    while (facingAngle > 180.0f) facingAngle -= 360.0f;
    while (facingAngle < -180.0f) facingAngle += 360.0f;
    
    // 平滑加减速
    float accelRate = isMoving ? acceleration : deceleration;
    float smoothT = 1.0f - std::exp(-accelRate * deltaTime);
    velocity = Vector3Lerp(velocity, targetVelocity, smoothT);
    
    // 更新速度标量
    speed = Vector3Length(velocity);
    
    // 更新位置（如果启用）
    if (updatePosition && targetPosition && speed > 0.001f) {
        *targetPosition = Vector3Add(*targetPosition, Vector3Scale(velocity, deltaTime));
    }
    
    // 调试输出（减少频率）
    static int debugCount = 0;
    if (isMoving && ++debugCount % 60 == 0) {
        printf("[Controller] mode=%s facing=%.1f speed=%.2f vel=(%.2f,%.2f,%.2f)\n",
               controlMode == ControlMode::LockToCamera ? "LOCK" : "FREE",
               facingAngle, speed, velocity.x, velocity.y, velocity.z);
    }
}

} // namespace mf
