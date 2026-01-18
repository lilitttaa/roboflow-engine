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
    
    // ========== 构建变换矩阵 ==========
    // 目标：M = T * R * S（变换应用顺序：先缩放，再旋转，最后平移）
    // 
    // 重要：raylib 的 MatrixMultiply(A, B) 返回 B * A（不是 A * B）
    // 所以要得到 A * B，需要写成 MatrixMultiply(B, A)
    //
    // 构建 T * R * S：
    //   Step 1: M = S
    //   Step 2: M = R * S = MatrixMultiply(S, R)
    //   Step 3: M = T * R * S = MatrixMultiply(R * S, T)
    
    Matrix scaleM = MatrixScale(scale, scale, scale);
    Matrix rotationM;
    
    if (useYawMode) {
        // 第三人称控制器模式
        // combinedRotation = yawRotation * coordTransform
        // = MatrixMultiply(coordTransform, yawRotation)（因为 MatrixMultiply 是反的）
        Matrix coordTransform = MatrixRotateX(-90.0f * DEG2RAD);  // URDF Z-up to Y-up
        Matrix yawRotation = MatrixRotateY(yawAngle * DEG2RAD);   // 水平朝向
        rotationM = MatrixMultiply(coordTransform, yawRotation);
    } else {
        // 普通模式：使用欧拉角
        rotationM = MatrixRotateXYZ({
            rotation.x * DEG2RAD, 
            rotation.y * DEG2RAD, 
            rotation.z * DEG2RAD
        });
    }
    
    Matrix translateM = MatrixTranslate(position.x, position.y, position.z);
    
    // 构建 T * R * S
    // R * S = MatrixMultiply(S, R)
    // T * R * S = MatrixMultiply(R * S, T)
    Matrix rootTransform = MatrixMultiply(scaleM, rotationM);      // = R * S
    rootTransform = MatrixMultiply(rootTransform, translateM);      // = T * R * S
    
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
