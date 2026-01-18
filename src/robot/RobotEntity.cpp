#include "RobotEntity.hpp"
#include <rlgl.h>
#include <iostream>
#include <cmath>

namespace mf {

RobotEntity::RobotEntity() {}

RobotEntity::~RobotEntity() {
    for (auto& [name, data] : m_linkData) {
        if (data.hasModel) {
            UnloadModel(data.model);
        }
    }
}

bool RobotEntity::loadFromURDF(const std::string& urdfPath) {
    m_model = URDFParser::load(urdfPath);
    if (!m_model) {
        return false;
    }
    
    int loadedCount = 0;
    
    for (auto& [linkName, link] : m_model->links) {
        LinkData data;
        
        if (!link.meshFile.empty()) {
            std::string meshPath = m_model->basePath + "/" + link.meshFile;
            Mesh mesh = m_meshLoader.load(meshPath);
            
            if (mesh.vertexCount > 0) {
                data.model = LoadModelFromMesh(mesh);
                data.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = link.color;
                data.hasModel = true;
                data.color = link.color;
                loadedCount++;
            }
        }
        
        m_linkData[linkName] = data;
    }
    
    std::cout << "RobotEntity: Loaded " << loadedCount << " models" << std::endl;
    update();
    return true;
}

void RobotEntity::setJointPosition(const std::string& jointName, float position) {
    auto it = m_model->joints.find(jointName);
    if (it != m_model->joints.end()) {
        position = fmaxf(it->second.lowerLimit, fminf(position, it->second.upperLimit));
        it->second.position = position;
    }
}

float RobotEntity::getJointPosition(const std::string& jointName) const {
    auto it = m_model->joints.find(jointName);
    if (it != m_model->joints.end()) {
        return it->second.position;
    }
    return 0.0f;
}

void RobotEntity::resetJoints() {
    for (auto& [name, joint] : m_model->joints) {
        joint.position = 0.0f;
    }
}

std::vector<std::string> RobotEntity::getJointNames() const {
    if (!m_model) return {};
    return m_model->getMovableJoints();
}

void RobotEntity::getJointLimits(const std::string& jointName, float& lower, float& upper) const {
    auto it = m_model->joints.find(jointName);
    if (it != m_model->joints.end()) {
        lower = it->second.lowerLimit;
        upper = it->second.upperLimit;
    } else {
        lower = upper = 0.0f;
    }
}

Matrix RobotEntity::createRotationFromRPY(float roll, float pitch, float yaw) {
    Matrix rotX = MatrixRotateX(roll);
    Matrix rotY = MatrixRotateY(pitch);
    Matrix rotZ = MatrixRotateZ(yaw);
    return MatrixMultiply(MatrixMultiply(rotX, rotY), rotZ);
}

void RobotEntity::update() {
    if (!m_model || m_model->rootLink.empty()) return;
    
    Matrix rootTransform = MatrixIdentity();
    rootTransform = MatrixMultiply(MatrixScale(scale, scale, scale), rootTransform);
    
    if (useYawMode) {
        // ========== 调试：详细打印旋转过程 ==========
        static int debugCount = 0;
        bool shouldDebug = (debugCount++ % 60 == 0);
        
        // URDF 坐标系: Z-up, X-forward, Y-left
        // Raylib 坐标系: Y-up, Z-out, X-right
        
        // 坐标系转换：Rx(-90°) 把 URDF 转换到 Raylib
        // URDF (1,0,0) → (1,0,0)   [X不变]
        // URDF (0,1,0) → (0,0,-1)  [Y→-Z]
        // URDF (0,0,1) → (0,1,0)   [Z→Y]
        
        Matrix coordTransform = MatrixRotateX(-90.0f * DEG2RAD);
        Matrix yawRotation = MatrixRotateY(yawAngle * DEG2RAD);
        
        if (shouldDebug) {
            printf("\n========== [RobotEntity Debug] ==========\n");
            printf("useYawMode=%d, yawAngle=%.1f\n", useYawMode, yawAngle);
            
            // 打印坐标转换矩阵
            printf("\ncoordTransform (Rx(-90)):\n");
            printf("  [%.2f, %.2f, %.2f, %.2f]\n", coordTransform.m0, coordTransform.m4, coordTransform.m8, coordTransform.m12);
            printf("  [%.2f, %.2f, %.2f, %.2f]\n", coordTransform.m1, coordTransform.m5, coordTransform.m9, coordTransform.m13);
            printf("  [%.2f, %.2f, %.2f, %.2f]\n", coordTransform.m2, coordTransform.m6, coordTransform.m10, coordTransform.m14);
            printf("  [%.2f, %.2f, %.2f, %.2f]\n", coordTransform.m3, coordTransform.m7, coordTransform.m11, coordTransform.m15);
            
            // 打印 yaw 旋转矩阵
            printf("\nyawRotation (Ry(%.1f)):\n", yawAngle);
            printf("  [%.2f, %.2f, %.2f, %.2f]\n", yawRotation.m0, yawRotation.m4, yawRotation.m8, yawRotation.m12);
            printf("  [%.2f, %.2f, %.2f, %.2f]\n", yawRotation.m1, yawRotation.m5, yawRotation.m9, yawRotation.m13);
            printf("  [%.2f, %.2f, %.2f, %.2f]\n", yawRotation.m2, yawRotation.m6, yawRotation.m10, yawRotation.m14);
            printf("  [%.2f, %.2f, %.2f, %.2f]\n", yawRotation.m3, yawRotation.m7, yawRotation.m11, yawRotation.m15);
            
            // 验证变换效果（计算 URDF 基向量经过变换后的位置）
            // 使用 coordTransform 只做坐标转换，看机器人是否站立
            Vector3 urdfX = {1, 0, 0};  // URDF 前方
            Vector3 urdfY = {0, 1, 0};  // URDF 左方
            Vector3 urdfZ = {0, 0, 1};  // URDF 上方
            
            // 手动计算 M * v（列向量）
            auto transformVec = [](const Matrix& m, Vector3 v) -> Vector3 {
                return {
                    m.m0*v.x + m.m4*v.y + m.m8*v.z,
                    m.m1*v.x + m.m5*v.y + m.m9*v.z,
                    m.m2*v.x + m.m6*v.y + m.m10*v.z
                };
            };
            
            // 只应用坐标转换
            Vector3 coordX = transformVec(coordTransform, urdfX);
            Vector3 coordY = transformVec(coordTransform, urdfY);
            Vector3 coordZ = transformVec(coordTransform, urdfZ);
            printf("\n只应用 coordTransform (Rx(-90)):\n");
            printf("  URDF +X (前) → (%.2f, %.2f, %.2f)\n", coordX.x, coordX.y, coordX.z);
            printf("  URDF +Y (左) → (%.2f, %.2f, %.2f)\n", coordY.x, coordY.y, coordY.z);
            printf("  URDF +Z (上) → (%.2f, %.2f, %.2f)\n", coordZ.x, coordZ.y, coordZ.z);
        }
        
        // 组合旋转：先 coord 再 yaw
        // 重要发现：raylib 的 MatrixMultiply(A, B) 返回 B * A (不是 A * B!)
        // 所以要得到 yawRotation * coordTransform，需要写成：
        // MatrixMultiply(coordTransform, yawRotation) = yawRotation * coordTransform
        Matrix combinedRotation = MatrixMultiply(coordTransform, yawRotation);
        
        if (shouldDebug) {
            // 打印组合矩阵
            printf("\ncombinedRotation (yaw * coord):\n");
            printf("  [%.2f, %.2f, %.2f, %.2f]\n", combinedRotation.m0, combinedRotation.m4, combinedRotation.m8, combinedRotation.m12);
            printf("  [%.2f, %.2f, %.2f, %.2f]\n", combinedRotation.m1, combinedRotation.m5, combinedRotation.m9, combinedRotation.m13);
            printf("  [%.2f, %.2f, %.2f, %.2f]\n", combinedRotation.m2, combinedRotation.m6, combinedRotation.m10, combinedRotation.m14);
            printf("  [%.2f, %.2f, %.2f, %.2f]\n", combinedRotation.m3, combinedRotation.m7, combinedRotation.m11, combinedRotation.m15);
            
            // 验证最终变换
            auto transformVec = [](const Matrix& m, Vector3 v) -> Vector3 {
                return {
                    m.m0*v.x + m.m4*v.y + m.m8*v.z,
                    m.m1*v.x + m.m5*v.y + m.m9*v.z,
                    m.m2*v.x + m.m6*v.y + m.m10*v.z
                };
            };
            
            Vector3 urdfX = {1, 0, 0};
            Vector3 urdfY = {0, 1, 0};
            Vector3 urdfZ = {0, 0, 1};
            
            Vector3 finalX = transformVec(combinedRotation, urdfX);
            Vector3 finalY = transformVec(combinedRotation, urdfY);
            Vector3 finalZ = transformVec(combinedRotation, urdfZ);
            printf("\n应用 combinedRotation 后:\n");
            printf("  URDF +X (前) → (%.2f, %.2f, %.2f) [期望: 机器人面向的方向]\n", finalX.x, finalX.y, finalX.z);
            printf("  URDF +Y (左) → (%.2f, %.2f, %.2f) [期望: 机器人左边]\n", finalY.x, finalY.y, finalY.z);
            printf("  URDF +Z (上) → (%.2f, %.2f, %.2f) [期望: (0,1,0) 向上]\n", finalZ.x, finalZ.y, finalZ.z);
            printf("==========================================\n");
        }
        
        rootTransform = MatrixMultiply(combinedRotation, rootTransform);
    } else {
        // 普通模式：使用欧拉角
        rootTransform = MatrixMultiply(MatrixRotateXYZ({
            rotation.x * DEG2RAD, 
            rotation.y * DEG2RAD, 
            rotation.z * DEG2RAD
        }), rootTransform);
    }
    
    rootTransform = MatrixMultiply(MatrixTranslate(position.x, position.y, position.z), rootTransform);
    
    updateLinkTransform(m_model->rootLink, rootTransform);
}

void RobotEntity::updateLinkTransform(const std::string& linkName, const Matrix& parentTransform) {
    m_worldTransforms[linkName] = parentTransform;
    
    auto childJoints = m_model->getChildJoints(linkName);
    
    for (const auto& jointName : childJoints) {
        const auto& joint = m_model->joints.at(jointName);
        
        Matrix jointRot = createRotationFromRPY(joint.rpy.x, joint.rpy.y, joint.rpy.z);
        Matrix jointTrans = MatrixTranslate(joint.origin.x, joint.origin.y, joint.origin.z);
        Matrix jointOrigin = MatrixMultiply(jointRot, jointTrans);
        
        Matrix jointMotion = MatrixIdentity();
        if (joint.type == "revolute" || joint.type == "continuous") {
            Vector3 axis = Vector3Normalize(joint.axis);
            jointMotion = MatrixRotate(axis, joint.position);
        } else if (joint.type == "prismatic") {
            Vector3 axis = Vector3Normalize(joint.axis);
            jointMotion = MatrixTranslate(
                axis.x * joint.position,
                axis.y * joint.position,
                axis.z * joint.position
            );
        }
        
        Matrix childTransform = MatrixMultiply(jointOrigin, parentTransform);
        childTransform = MatrixMultiply(jointMotion, childTransform);
        
        updateLinkTransform(joint.childLink, childTransform);
    }
}

void RobotEntity::render() {
    if (!m_model) return;
    
    for (const auto& [linkName, link] : m_model->links) {
        renderLink(linkName);
    }
}

bool RobotEntity::renderLink(const std::string& linkName) {
    auto dataIt = m_linkData.find(linkName);
    auto transformIt = m_worldTransforms.find(linkName);
    
    if (dataIt == m_linkData.end() || transformIt == m_worldTransforms.end()) {
        return false;
    }
    
    const auto& data = dataIt->second;
    const Matrix& worldTransform = transformIt->second;
    
    if (data.hasModel) {
        const auto& link = m_model->links.at(linkName);
        
        Matrix visualRot = createRotationFromRPY(link.visualRPY.x, link.visualRPY.y, link.visualRPY.z);
        Matrix visualTrans = MatrixTranslate(link.visualOrigin.x, link.visualOrigin.y, link.visualOrigin.z);
        Matrix visualTransform = MatrixMultiply(visualRot, visualTrans);
        Matrix finalTransform = MatrixMultiply(visualTransform, worldTransform);
        
        rlDisableBackfaceCulling();
        Model modelCopy = data.model;
        modelCopy.transform = finalTransform;
        DrawModel(modelCopy, {0, 0, 0}, 1.0f, data.color);
        rlEnableBackfaceCulling();
        
        return true;
    }
    
    // 绘制坐标轴（可选）
    if (showAxes) {
        Vector3 pos = { worldTransform.m12, worldTransform.m13, worldTransform.m14 };
        Vector3 xAxis = Vector3Normalize({ worldTransform.m0, worldTransform.m1, worldTransform.m2 });
        Vector3 yAxis = Vector3Normalize({ worldTransform.m4, worldTransform.m5, worldTransform.m6 });
        Vector3 zAxis = Vector3Normalize({ worldTransform.m8, worldTransform.m9, worldTransform.m10 });
        
        DrawLine3D(pos, Vector3Add(pos, Vector3Scale(xAxis, axisLength)), RED);
        DrawLine3D(pos, Vector3Add(pos, Vector3Scale(yAxis, axisLength)), GREEN);
        DrawLine3D(pos, Vector3Add(pos, Vector3Scale(zAxis, axisLength)), BLUE);
    }
    
    return false;
}

} // namespace mf
