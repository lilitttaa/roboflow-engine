#pragma once

#include "MotionData.hpp"
#include "MotionMatchingUtils.hpp"
#include <raylib.h>
#include <raymath.h>
#include <vector>
#include <string>
#include <memory>
#include <cmath>

namespace mf {

// ============================================================================
// 特征定义
// ============================================================================

/**
 * 轨迹点 - 用于预测未来轨迹
 */
struct TrajectoryPoint {
    Vector2 position;    // 水平位置 (x, z)
    float facing;        // 朝向角度 (度)
};

/**
 * 动作特征向量
 * 用于匹配当前状态和数据库中的动作帧
 */
struct MotionFeature {
    // 轨迹特征（未来 0.2s, 0.4s, 0.6s, 1.0s 的位置和朝向）
    static constexpr int TRAJECTORY_POINTS = 4;
    static constexpr float TRAJECTORY_TIMES[4] = {0.2f, 0.4f, 0.6f, 1.0f};
    TrajectoryPoint trajectory[TRAJECTORY_POINTS];
    
    // 脚部特征（相对于根节点）
    Vector3 leftFootPos;
    Vector3 rightFootPos;
    Vector3 leftFootVel;
    Vector3 rightFootVel;
    
    // 髋部速度（用于判断移动状态）
    Vector3 hipVelocity;
    
    /**
     * 计算与另一个特征的距离（加权欧氏距离）
     */
    float distance(const MotionFeature& other, const struct FeatureWeights& weights) const;
    
    /**
     * 将特征转换为浮点数组（用于 KD-Tree 等）
     */
    std::vector<float> toVector() const;
    
    /**
     * 从浮点数组恢复特征
     */
    static MotionFeature fromVector(const std::vector<float>& vec);
    
    /**
     * 获取特征维度
     */
    static constexpr size_t dimension() {
        return TRAJECTORY_POINTS * 3 +  // trajectory: x, z, facing
               3 * 4 +                   // foot pos/vel: 4 * Vector3
               3;                        // hip velocity
    }
};

/**
 * 特征权重
 */
struct FeatureWeights {
    float trajectoryPos = 1.0f;      // 轨迹位置权重
    float trajectoryFacing = 1.0f;   // 轨迹朝向权重
    float footPos = 0.75f;           // 脚部位置权重
    float footVel = 1.0f;            // 脚部速度权重
    float hipVel = 1.0f;             // 髋部速度权重
    
    // 响应性调整：更高 = 更快响应输入变化
    float responsiveness = 1.0f;
};

// ============================================================================
// 动作数据库条目
// ============================================================================

/**
 * 数据库中的单个动作帧
 */
struct MotionDatabaseEntry {
    size_t clipIndex;        // 所属动作片段索引
    size_t frameIndex;       // 在片段中的帧索引
    MotionFeature feature;   // 预计算的特征
    
    // 用于快速访问
    const MotionFrame* frame = nullptr;
};

// ============================================================================
// 动作数据库
// ============================================================================

/**
 * 动作数据库
 * 存储多个动作片段，预计算特征，支持快速搜索
 */
class MotionDatabase {
public:
    MotionDatabase() = default;
    ~MotionDatabase() = default;
    
    /**
     * 添加动作片段
     * @param motion 动作数据
     * @param tag 标签（如 "walk", "run", "idle"）
     * @return 片段索引
     */
    size_t addClip(std::unique_ptr<MotionData> motion, const std::string& tag = "");
    
    /**
     * 从文件加载并添加动作片段
     */
    bool loadClip(const std::string& path, const std::string& tag = "");
    
    /**
     * 预处理：计算所有帧的特征
     * 必须在添加完所有片段后调用
     */
    void preprocess();
    
    /**
     * 搜索最佳匹配帧
     * @param query 查询特征
     * @param weights 特征权重
     * @param excludeClip 排除的片段（-1 表示不排除）
     * @param excludeFrameRange 排除的帧范围（当前帧附近）
     * @param continuityBias 连续性偏置系数（0.0-1.0，越小减分越多）
     * @return 最佳匹配的条目索引
     */
    size_t findBestMatch(
        const MotionFeature& query,
        const FeatureWeights& weights,
        int excludeClip = -1,
        int excludeFrameStart = -1,
        int excludeFrameEnd = -1,
        float continuityBias = 0.5f
    ) const;
    
    /**
     * 获取条目
     */
    const MotionDatabaseEntry& getEntry(size_t index) const { return m_entries[index]; }
    
    /**
     * 获取条目数量
     */
    size_t getNumEntries() const { return m_entries.size(); }
    
    /**
     * 获取片段
     */
    const MotionData* getClip(size_t index) const { 
        return index < m_clips.size() ? m_clips[index].get() : nullptr; 
    }
    
    /**
     * 获取片段数量
     */
    size_t getNumClips() const { return m_clips.size(); }
    
    /**
     * 获取片段标签
     */
    const std::string& getClipTag(size_t index) const { return m_clipTags[index]; }
    
    /**
     * 是否已预处理
     */
    bool isPreprocessed() const { return m_preprocessed; }
    
    /**
     * 获取关节名称（使用第一个片段的）
     */
    const std::vector<std::string>& getJointNames() const {
        static std::vector<std::string> empty;
        return m_clips.empty() ? empty : m_clips[0]->getJointNames();
    }

private:
    std::vector<std::unique_ptr<MotionData>> m_clips;
    std::vector<std::string> m_clipTags;
    std::vector<MotionDatabaseEntry> m_entries;
    bool m_preprocessed = false;
    
    /**
     * 计算单帧的特征
     */
    MotionFeature computeFeature(size_t clipIndex, size_t frameIndex) const;
    
    /**
     * 获取指定关节的位置（相对于根节点）
     */
    Vector3 getJointPosition(const MotionFrame& frame, const std::string& jointName) const;
};

// ============================================================================
// Motion Matcher
// ============================================================================

/**
 * Motion Matcher
 * 核心匹配逻辑，处理状态跟踪和平滑过渡
 */
class MotionMatcher {
public:
    MotionMatcher() = default;
    ~MotionMatcher() = default;
    
    /**
     * 设置动作数据库
     */
    void setDatabase(MotionDatabase* db) { m_database = db; }
    
    /**
     * 更新匹配器
     * @param deltaTime 时间增量
     * @param desiredVelocity 期望速度（来自 CharacterController）
     * @param desiredFacing 期望朝向（来自 CharacterController）
     * @param controlMode 控制模式（Free 或 Lock）
     * @return 当前应该播放的帧
     */
    MotionFrame update(float deltaTime, Vector3 desiredVelocity, float desiredFacing, bool isFreeMode = true);
    
    /**
     * 获取当前根位置（用于 Root Motion）
     */
    Vector3 getCurrentRootPosition() const { return m_currentRootPos; }
    
    /**
     * 获取当前根旋转（用于 Root Motion）
     */
    Quaternion getCurrentRootRotation() const { return m_currentRootRot; }
    
    /**
     * 设置 Motion Warping 目标位置
     */
    void setWarpTarget(const Vector3& targetPos, float duration = 0.5f);
    
    /**
     * 清除 Motion Warping
     */
    void clearWarp() { m_warpActive = false; }
    
    /**
     * 强制跳转到指定帧
     */
    void jumpTo(size_t clipIndex, size_t frameIndex);
    
    /**
     * 获取当前片段索引
     */
    size_t getCurrentClip() const { return m_currentClip; }
    
    /**
     * 获取当前帧索引
     */
    size_t getCurrentFrame() const { return m_currentFrame; }
    
    /**
     * 获取当前播放时间
     */
    float getCurrentTime() const { return m_currentTime; }
    
    /**
     * 获取/设置特征权重
     */
    FeatureWeights& weights() { return m_weights; }
    const FeatureWeights& weights() const { return m_weights; }
    
    /**
     * 搜索间隔（帧数）
     * 更大的值 = 更少的搜索 = 更好的性能，但响应更慢
     */
    int searchInterval = 10;
    
    /**
     * 过渡混合时间（秒）
     */
    float blendDuration = 0.2f;
    
    /**
     * 强制搜索阈值
     * 当输入变化超过此值时强制搜索
     */
    float forceSearchThreshold = 0.5f;
    
    /**
     * 连续性偏置权重
     * 给当前片段后续帧的减分系数（0.0-1.0，越小减分越多）
     */
    float continuityBias = 0.5f;
    
    /**
     * 惯性化半衰期（秒）
     */
    float inertializationHalflife = 0.1f;

private:
    MotionDatabase* m_database = nullptr;
    FeatureWeights m_weights;
    
    // 当前状态
    size_t m_currentClip = 0;
    size_t m_currentFrame = 0;
    float m_currentTime = 0.0f;
    int m_framesSinceLastSearch = 0;
    
    // Root Motion 状态
    Vector3 m_currentRootPos = {0, 0, 0};
    Quaternion m_currentRootRot = {1, 0, 0, 0};
    Vector3 m_lastRootPos = {0, 0, 0};
    Vector3 m_lastFrameRootPos = {0, 0, 0};  // 用于计算相对位移
    bool m_firstFrame = true;  // 是否是第一帧
    
    // 惯性化状态
    Inertializer m_inertializer;
    bool m_inertializing = false;
    
    // Motion Warping 状态
    bool m_warpActive = false;
    Vector3 m_warpTarget = {0, 0, 0};
    float m_warpTime = 0.0f;
    float m_warpDuration = 0.5f;
    Vector3 m_warpStartPos = {0, 0, 0};
    size_t m_warpStartFrame = 0;
    
    // 期望轨迹平滑（二阶弹簧阻尼器）
    SpringDamperVec3 m_smoothVelocity;
    SpringDamper m_smoothFacing;
    
    // 混合状态（已废弃，改用惯性化）
    bool m_blending = false;
    float m_blendTime = 0.0f;
    MotionFrame m_blendFromFrame;
    size_t m_blendToClip = 0;
    size_t m_blendToFrame = 0;
    
    // 输入历史（用于计算期望轨迹）
    Vector3 m_lastDesiredVelocity = {0, 0, 0};
    float m_lastDesiredFacing = 0.0f;
    
    /**
     * 根据当前输入计算查询特征
     * @param isFreeMode true=Free模式, false=Lock模式
     */
    MotionFeature computeQueryFeature(Vector3 desiredVelocity, float desiredFacing, bool isFreeMode) const;
    
    /**
     * 执行搜索（带连续性偏置）
     */
    void performSearch(const MotionFeature& query);
    
    /**
     * 混合两帧（已废弃，改用惯性化）
     */
    MotionFrame blendFrames(const MotionFrame& from, const MotionFrame& to, float t) const;
    
    /**
     * 应用 Motion Warping
     */
    void applyMotionWarping(MotionFrame& frame, float deltaTime);
};

// ============================================================================
// 内联实现
// ============================================================================

constexpr float MotionFeature::TRAJECTORY_TIMES[4];

inline float MotionFeature::distance(const MotionFeature& other, const FeatureWeights& weights) const {
    float dist = 0.0f;
    
    // 轨迹距离
    for (int i = 0; i < TRAJECTORY_POINTS; ++i) {
        float dx = trajectory[i].position.x - other.trajectory[i].position.x;
        float dz = trajectory[i].position.y - other.trajectory[i].position.y;
        dist += (dx * dx + dz * dz) * weights.trajectoryPos;
        
        float df = trajectory[i].facing - other.trajectory[i].facing;
        // 处理角度环绕
        while (df > 180.0f) df -= 360.0f;
        while (df < -180.0f) df += 360.0f;
        dist += df * df * 0.01f * weights.trajectoryFacing;  // 角度缩放
    }
    
    // 脚部位置距离
    dist += Vector3DistanceSqr(leftFootPos, other.leftFootPos) * weights.footPos;
    dist += Vector3DistanceSqr(rightFootPos, other.rightFootPos) * weights.footPos;
    
    // 脚部速度距离
    dist += Vector3DistanceSqr(leftFootVel, other.leftFootVel) * weights.footVel;
    dist += Vector3DistanceSqr(rightFootVel, other.rightFootVel) * weights.footVel;
    
    // 髋部速度距离
    dist += Vector3DistanceSqr(hipVelocity, other.hipVelocity) * weights.hipVel;
    
    return dist;
}

inline std::vector<float> MotionFeature::toVector() const {
    std::vector<float> vec;
    vec.reserve(dimension());
    
    for (int i = 0; i < TRAJECTORY_POINTS; ++i) {
        vec.push_back(trajectory[i].position.x);
        vec.push_back(trajectory[i].position.y);
        vec.push_back(trajectory[i].facing);
    }
    
    vec.push_back(leftFootPos.x);
    vec.push_back(leftFootPos.y);
    vec.push_back(leftFootPos.z);
    vec.push_back(rightFootPos.x);
    vec.push_back(rightFootPos.y);
    vec.push_back(rightFootPos.z);
    vec.push_back(leftFootVel.x);
    vec.push_back(leftFootVel.y);
    vec.push_back(leftFootVel.z);
    vec.push_back(rightFootVel.x);
    vec.push_back(rightFootVel.y);
    vec.push_back(rightFootVel.z);
    
    vec.push_back(hipVelocity.x);
    vec.push_back(hipVelocity.y);
    vec.push_back(hipVelocity.z);
    
    return vec;
}

} // namespace mf
