#pragma once

#include <raylib.h>
#include <raymath.h>
#include <cmath>

namespace mf {

/**
 * Motion Matching 工具函数
 * 提供坐标转换、旋转等常用计算
 */

/**
 * 从四元数提取 yaw 角度（度）
 * 假设默认前方向是 +Z 轴
 * facingAngle: 0=+Z, 90=+X, 180=-Z, -90=-X
 */
inline float extractYawFromQuaternion(const Quaternion& quat) {
    Vector3 forward = Vector3RotateByQuaternion({0, 0, 1}, quat);
    return atan2f(forward.x, forward.z) * RAD2DEG;
}

/**
 * 将世界坐标转换到局部坐标系
 * @param worldPos 世界坐标
 * @param yawDeg 角色朝向角度（度）
 * @return 局部坐标 (x=right, y=up, z=forward)
 */
inline Vector3 worldToLocal(const Vector3& worldPos, float yawDeg) {
    float yawRad = yawDeg * DEG2RAD;
    float cosYaw = cosf(yawRad);
    float sinYaw = sinf(yawRad);
    
    return {
        worldPos.x * cosYaw - worldPos.z * sinYaw,  // right
        worldPos.y,                                   // up
        worldPos.x * sinYaw + worldPos.z * cosYaw   // forward
    };
}

/**
 * 将局部坐标转换到世界坐标系
 * @param localPos 局部坐标 (x=right, y=up, z=forward)
 * @param yawDeg 角色朝向角度（度）
 * @return 世界坐标
 */
inline Vector3 localToWorld(const Vector3& localPos, float yawDeg) {
    float yawRad = yawDeg * DEG2RAD;
    float cosYaw = cosf(yawRad);
    float sinYaw = sinf(yawRad);
    
    return {
        localPos.x * cosYaw + localPos.z * sinYaw,  // world x
        localPos.y,                                  // world y
        -localPos.x * sinYaw + localPos.z * cosYaw  // world z
    };
}

/**
 * 归一化角度到 [-180, 180] 范围
 */
inline float normalizeAngle(float angleDeg) {
    while (angleDeg > 180.0f) angleDeg -= 360.0f;
    while (angleDeg < -180.0f) angleDeg += 360.0f;
    return angleDeg;
}

/**
 * 角度差值（处理环绕）
 */
inline float angleDifference(float from, float to) {
    float diff = to - from;
    return normalizeAngle(diff);
}

/**
 * 二阶弹簧阻尼器
 * 用于平滑输入值
 */
class SpringDamper {
public:
    float stiffness = 100.0f;   // 刚度
    float damping = 20.0f;       // 阻尼系数
    
    float value = 0.0f;         // 当前值
    float velocity = 0.0f;       // 当前速度
    
    /**
     * 更新弹簧阻尼器
     * @param target 目标值
     * @param deltaTime 时间增量
     */
    void update(float target, float deltaTime) {
        float error = target - value;
        float force = stiffness * error - damping * velocity;
        velocity += force * deltaTime;
        value += velocity * deltaTime;
    }
    
    /**
     * 重置状态
     */
    void reset(float initialValue = 0.0f) {
        value = initialValue;
        velocity = 0.0f;
    }
};

/**
 * 二阶弹簧阻尼器（向量版本）
 */
class SpringDamperVec3 {
public:
    float stiffness = 100.0f;
    float damping = 20.0f;
    
    Vector3 value = {0, 0, 0};
    Vector3 velocity = {0, 0, 0};
    
    void update(const Vector3& target, float deltaTime) {
        Vector3 error = Vector3Subtract(target, value);
        Vector3 force = Vector3Subtract(
            Vector3Scale(error, stiffness),
            Vector3Scale(velocity, damping)
        );
        velocity = Vector3Add(velocity, Vector3Scale(force, deltaTime));
        value = Vector3Add(value, Vector3Scale(velocity, deltaTime));
    }
    
    void reset(const Vector3& initialValue = {0, 0, 0}) {
        value = initialValue;
        velocity = {0, 0, 0};
    }
};

/**
 * 惯性化插值器
 * 使用二阶临界阻尼系统平滑过渡
 */
class Inertializer {
public:
    float halflife = 0.1f;  // 半衰期（秒）
    
    Vector3 position = {0, 0, 0};
    Vector3 velocity = {0, 0, 0};
    Quaternion rotation = {1, 0, 0, 0};
    Vector3 angularVelocity = {0, 0, 0};
    
    /**
     * 开始惯性化
     * @param currentPos 当前位置
     * @param currentVel 当前速度
     * @param currentRot 当前旋转
     * @param targetPos 目标位置
     * @param targetVel 目标速度
     * @param targetRot 目标旋转
     */
    void start(const Vector3& currentPos, const Vector3& currentVel,
               const Quaternion& currentRot, const Vector3& targetPos,
               const Vector3& targetVel, const Quaternion& targetRot) {
        position = currentPos;
        velocity = currentVel;
        rotation = currentRot;
        
        // 计算位置和速度差异
        Vector3 posError = Vector3Subtract(targetPos, currentPos);
        Vector3 velError = Vector3Subtract(targetVel, currentVel);
        
        // 计算初始速度（用于临界阻尼）
        float damping = 4.0f / halflife;
        float stiffness = damping * damping / 4.0f;
        
        velocity = Vector3Add(velocity, 
            Vector3Add(
                Vector3Scale(posError, stiffness),
                Vector3Scale(velError, damping)
            )
        );
        
        // 旋转差异（简化处理）
        // TODO: 实现旋转的惯性化
    }
    
    /**
     * 更新惯性化
     * @param deltaTime 时间增量
     */
    void update(float deltaTime) {
        // 二阶临界阻尼衰减
        float damping = 4.0f / halflife;
        float stiffness = damping * damping / 4.0f;
        
        Vector3 force = Vector3Subtract(
            Vector3Scale(position, -stiffness),
            Vector3Scale(velocity, damping)
        );
        
        velocity = Vector3Add(velocity, Vector3Scale(force, deltaTime));
        position = Vector3Add(position, Vector3Scale(velocity, deltaTime));
    }
    
    /**
     * 检查是否完成
     */
    bool isComplete(float threshold = 0.001f) const {
        return Vector3Length(position) < threshold && 
               Vector3Length(velocity) < threshold;
    }
};

} // namespace mf
