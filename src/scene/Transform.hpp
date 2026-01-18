#pragma once

#include <raylib.h>
#include <raymath.h>

namespace mf {

/**
 * Transform - 变换组件
 * 存储位置、旋转、缩放，并计算变换矩阵
 */
class Transform {
public:
    Vector3 position = { 0.0f, 0.0f, 0.0f };
    Quaternion rotation = QuaternionIdentity();
    Vector3 scale = { 1.0f, 1.0f, 1.0f };

    Transform() = default;
    
    Transform(Vector3 pos) 
        : position(pos) {}
    
    Transform(Vector3 pos, Quaternion rot) 
        : position(pos), rotation(rot) {}
    
    Transform(Vector3 pos, Quaternion rot, Vector3 scl) 
        : position(pos), rotation(rot), scale(scl) {}

    // 从欧拉角设置旋转 (度数)
    void setEulerAngles(float pitch, float yaw, float roll) {
        rotation = QuaternionFromEuler(
            pitch * DEG2RAD, 
            yaw * DEG2RAD, 
            roll * DEG2RAD
        );
    }

    // 获取欧拉角 (度数)
    Vector3 getEulerAngles() const {
        Vector3 euler = QuaternionToEuler(rotation);
        return { euler.x * RAD2DEG, euler.y * RAD2DEG, euler.z * RAD2DEG };
    }

    // 获取前方向向量
    Vector3 forward() const {
        return Vector3RotateByQuaternion({ 0.0f, 0.0f, -1.0f }, rotation);
    }

    // 获取右方向向量
    Vector3 right() const {
        return Vector3RotateByQuaternion({ 1.0f, 0.0f, 0.0f }, rotation);
    }

    // 获取上方向向量
    Vector3 up() const {
        return Vector3RotateByQuaternion({ 0.0f, 1.0f, 0.0f }, rotation);
    }

    // 计算局部变换矩阵
    Matrix getMatrix() const {
        Matrix matScale = MatrixScale(scale.x, scale.y, scale.z);
        Matrix matRotation = QuaternionToMatrix(rotation);
        Matrix matTranslation = MatrixTranslate(position.x, position.y, position.z);
        
        // 顺序: Scale -> Rotate -> Translate
        return MatrixMultiply(MatrixMultiply(matScale, matRotation), matTranslation);
    }

    // 平移
    void translate(Vector3 delta) {
        position = Vector3Add(position, delta);
    }

    // 沿局部方向平移
    void translateLocal(Vector3 delta) {
        Vector3 worldDelta = Vector3RotateByQuaternion(delta, rotation);
        position = Vector3Add(position, worldDelta);
    }

    // 绕轴旋转 (度数)
    void rotate(Vector3 axis, float angle) {
        Quaternion q = QuaternionFromAxisAngle(axis, angle * DEG2RAD);
        rotation = QuaternionMultiply(rotation, q);
    }

    // 看向目标点
    void lookAt(Vector3 target, Vector3 worldUp = { 0, 1, 0 }) {
        Vector3 forward = Vector3Normalize(Vector3Subtract(target, position));
        Vector3 right = Vector3Normalize(Vector3CrossProduct(worldUp, forward));
        Vector3 up = Vector3CrossProduct(forward, right);
        
        Matrix rotMatrix = {
            right.x, up.x, -forward.x, 0,
            right.y, up.y, -forward.y, 0,
            right.z, up.z, -forward.z, 0,
            0, 0, 0, 1
        };
        rotation = QuaternionFromMatrix(rotMatrix);
    }
};

} // namespace mf
