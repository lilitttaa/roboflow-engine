#pragma once

#include "URDFParser.hpp"
#include "MeshLoader.hpp"
#include <string>
#include <map>
#include <memory>
#include <raylib.h>
#include <raymath.h>

namespace mf {

/**
 * 机器人实体
 * 从 URDF 加载并渲染机器人模型
 */
class RobotEntity {
public:
    RobotEntity();
    ~RobotEntity();
    
    /**
     * 从 URDF 文件加载机器人
     * @param urdfPath URDF 文件路径
     * @return 是否成功
     */
    bool loadFromURDF(const std::string& urdfPath);
    
    /**
     * 设置关节角度
     * @param jointName 关节名称
     * @param angle 角度（弧度）
     */
    void setJointPosition(const std::string& jointName, float position);
    
    /**
     * 获取关节角度
     */
    float getJointPosition(const std::string& jointName) const;
    
    /**
     * 设置所有关节为零位
     */
    void resetJoints();
    
    /**
     * 更新（计算各 link 的世界变换）
     */
    void update();
    
    /**
     * 渲染机器人
     */
    void render();
    
    /**
     * 获取可动关节列表
     */
    std::vector<std::string> getJointNames() const;
    
    /**
     * 获取关节限位
     */
    void getJointLimits(const std::string& jointName, float& lower, float& upper) const;
    
    // 世界变换
    Vector3 position = {0, 0, 0};
    Vector3 rotation = {0, 0, 0};  // 欧拉角（度）
    float scale = 1.0f;
    
    // 是否显示坐标轴
    bool showAxes = false;
    float axisLength = 0.05f;

private:
    std::unique_ptr<URDFModel> m_model;
    MeshLoader m_meshLoader;
    
    // 每个 link 的模型数据
    struct LinkData {
        Model model;
        Color color = WHITE;
        bool hasModel = false;
    };
    std::map<std::string, LinkData> m_linkData;
    
    // 每个 link 的世界变换矩阵
    std::map<std::string, Matrix> m_worldTransforms;
    
    // 从 RPY（弧度）创建旋转矩阵
    Matrix createRotationFromRPY(float roll, float pitch, float yaw);
    
    // 递归更新 link 的世界变换
    void updateLinkTransform(const std::string& linkName, const Matrix& parentTransform);
    
    // 渲染单个 link，返回是否成功渲染
    bool renderLink(const std::string& linkName);
};

} // namespace mf
