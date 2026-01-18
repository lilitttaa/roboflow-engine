#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <raylib.h>
#include <raymath.h>

namespace mf {

/**
 * URDF Link 数据
 */
struct URDFLink {
    std::string name;
    
    // Visual
    std::string meshFile;           // STL 文件名
    Vector3 visualOrigin = {0, 0, 0};
    Vector3 visualRPY = {0, 0, 0};  // Roll, Pitch, Yaw
    Color color = WHITE;
    
    // Inertial (可选，用于物理模拟)
    float mass = 0.0f;
    Vector3 inertialOrigin = {0, 0, 0};
};

/**
 * URDF Joint 数据
 */
struct URDFJoint {
    std::string name;
    std::string type;       // "revolute", "prismatic", "fixed", "floating"
    std::string parentLink;
    std::string childLink;
    
    Vector3 origin = {0, 0, 0};
    Vector3 rpy = {0, 0, 0};        // Roll, Pitch, Yaw
    Vector3 axis = {0, 0, 1};       // 旋转/平移轴
    
    float lowerLimit = 0.0f;
    float upperLimit = 0.0f;
    float effort = 0.0f;
    float velocity = 0.0f;
    
    // 当前关节角度/位置
    float position = 0.0f;
};

/**
 * URDF 机器人模型数据
 */
struct URDFModel {
    std::string name;
    std::string basePath;           // URDF 文件所在目录
    
    std::map<std::string, URDFLink> links;
    std::map<std::string, URDFJoint> joints;
    
    std::string rootLink;           // 根 link 名称
    
    // 获取 link 的子 joint 列表
    std::vector<std::string> getChildJoints(const std::string& linkName) const;
    
    // 获取 joint 数量
    size_t getJointCount() const { return joints.size(); }
    
    // 获取可动关节列表 (非 fixed)
    std::vector<std::string> getMovableJoints() const;
};

/**
 * URDF 解析器
 */
class URDFParser {
public:
    /**
     * 从文件加载 URDF
     * @param filepath URDF 文件路径
     * @return 解析后的模型数据，失败返回 nullptr
     */
    static std::unique_ptr<URDFModel> load(const std::string& filepath);

private:
    static Vector3 parseVector3(const char* str);
    static Color parseColor(const char* rgba);
};

} // namespace mf
