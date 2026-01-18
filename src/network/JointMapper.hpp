#pragma once

#include <string>
#include <vector>
#include <map>
#include <array>

namespace mf {

/**
 * 关节名称映射器
 * 将 URDF 关节名映射到 gentle-humanoid 的关节名
 */
class JointMapper {
public:
    // gentle-humanoid 使用的 29 个关节名（按顺序）
    static constexpr int NUM_JOINTS = 29;
    
    static const std::vector<std::string>& getTargetJointNames() {
        static const std::vector<std::string> names = {
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint"
        };
        return names;
    }
    
    /**
     * URDF 关节名到 gentle-humanoid 关节名的映射
     */
    static const std::map<std::string, std::string>& getURDFToTargetMap() {
        static const std::map<std::string, std::string> mapping = {
            // 左腿
            {"leg_l1_joint", "left_hip_pitch_joint"},
            {"leg_l2_joint", "left_hip_roll_joint"},
            {"leg_l3_joint", "left_hip_yaw_joint"},
            {"leg_l4_joint", "left_knee_joint"},
            {"leg_l5_joint", "left_ankle_pitch_joint"},
            {"leg_l6_joint", "left_ankle_roll_joint"},
            // 右腿
            {"leg_r1_joint", "right_hip_pitch_joint"},
            {"leg_r2_joint", "right_hip_roll_joint"},
            {"leg_r3_joint", "right_hip_yaw_joint"},
            {"leg_r4_joint", "right_knee_joint"},
            {"leg_r5_joint", "right_ankle_pitch_joint"},
            {"leg_r6_joint", "right_ankle_roll_joint"},
            // 腰部
            {"waist_yaw_joint", "waist_yaw_joint"},
            {"waist_roll_joint", "waist_roll_joint"},
            {"waist_pitch_joint", "waist_pitch_joint"},
            // 左臂
            {"arm_l1_joint", "left_shoulder_pitch_joint"},
            {"arm_l2_joint", "left_shoulder_roll_joint"},
            {"arm_l3_joint", "left_shoulder_yaw_joint"},
            {"arm_l4_joint", "left_elbow_joint"},
            {"arm_l5_joint", "left_wrist_roll_joint"},
            {"arm_l6_joint", "left_wrist_pitch_joint"},
            {"arm_l7_joint", "left_wrist_yaw_joint"},
            // 右臂
            {"arm_r1_joint", "right_shoulder_pitch_joint"},
            {"arm_r2_joint", "right_shoulder_roll_joint"},
            {"arm_r3_joint", "right_shoulder_yaw_joint"},
            {"arm_r4_joint", "right_elbow_joint"},
            {"arm_r5_joint", "right_wrist_roll_joint"},
            {"arm_r6_joint", "right_wrist_pitch_joint"},
            {"arm_r7_joint", "right_wrist_yaw_joint"}
        };
        return mapping;
    }
    
    /**
     * 将 URDF 关节位置映射到 gentle-humanoid 格式
     * @param urdfJointNames URDF 关节名列表
     * @param urdfJointPositions URDF 关节位置
     * @param outPositions 输出的 29 个关节位置（按 gentle-humanoid 顺序）
     */
    static void mapJointPositions(
        const std::vector<std::string>& urdfJointNames,
        const std::vector<float>& urdfJointPositions,
        std::array<float, NUM_JOINTS>& outPositions
    ) {
        // 初始化为 0
        outPositions.fill(0.0f);
        
        const auto& targetNames = getTargetJointNames();
        const auto& urdfToTarget = getURDFToTargetMap();
        
        // 创建目标关节名到索引的映射
        std::map<std::string, int> targetNameToIdx;
        for (int i = 0; i < (int)targetNames.size(); ++i) {
            targetNameToIdx[targetNames[i]] = i;
        }
        
        // 遍历 URDF 关节
        for (size_t i = 0; i < urdfJointNames.size() && i < urdfJointPositions.size(); ++i) {
            const std::string& urdfName = urdfJointNames[i];
            
            // 查找映射
            auto mapIt = urdfToTarget.find(urdfName);
            if (mapIt != urdfToTarget.end()) {
                const std::string& targetName = mapIt->second;
                auto idxIt = targetNameToIdx.find(targetName);
                if (idxIt != targetNameToIdx.end()) {
                    outPositions[idxIt->second] = urdfJointPositions[i];
                }
            }
        }
    }
    
    /**
     * 将 URDF 关节位置映射到 gentle-humanoid 格式（使用 map 输入）
     * @param urdfJointMap URDF 关节名到位置的映射
     * @param outPositions 输出的 29 个关节位置（按 gentle-humanoid 顺序）
     */
    static void mapJointPositions(
        const std::map<std::string, float>& urdfJointMap,
        std::array<float, NUM_JOINTS>& outPositions
    ) {
        outPositions.fill(0.0f);
        
        const auto& targetNames = getTargetJointNames();
        const auto& urdfToTarget = getURDFToTargetMap();
        
        // 创建目标关节名到索引的映射
        std::map<std::string, int> targetNameToIdx;
        for (int i = 0; i < (int)targetNames.size(); ++i) {
            targetNameToIdx[targetNames[i]] = i;
        }
        
        // 遍历映射表中的所有 URDF 关节
        for (const auto& [urdfName, targetName] : urdfToTarget) {
            auto posIt = urdfJointMap.find(urdfName);
            if (posIt != urdfJointMap.end()) {
                auto idxIt = targetNameToIdx.find(targetName);
                if (idxIt != targetNameToIdx.end()) {
                    outPositions[idxIt->second] = posIt->second;
                }
            }
        }
    }
};

} // namespace mf
