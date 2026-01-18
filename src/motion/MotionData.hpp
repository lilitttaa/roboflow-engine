#pragma once

#include <raylib.h>
#include <raymath.h>
#include <string>
#include <vector>
#include <cstdint>

namespace mf {

/**
 * 单帧动作数据
 */
struct MotionFrame {
    Vector3 rootPos;           // 根位置
    Quaternion rootQuat;       // 根旋转 (w, x, y, z)
    std::vector<float> jointPos;  // 关节角度
};

/**
 * 动作数据
 * 加载和存储动作序列
 */
class MotionData {
public:
    MotionData() = default;
    ~MotionData() = default;
    
    /**
     * 从 .motion 文件加载
     * @param path 文件路径
     * @return 是否成功
     */
    bool loadFromFile(const std::string& path);
    
    /**
     * 获取帧数
     */
    size_t getNumFrames() const { return m_frames.size(); }
    
    /**
     * 获取帧率
     */
    float getFPS() const { return m_fps; }
    
    /**
     * 获取时长（秒）
     */
    float getDuration() const { return m_frames.empty() ? 0.0f : (m_frames.size() - 1) / m_fps; }
    
    /**
     * 获取关节数量
     */
    size_t getNumJoints() const { return m_jointNames.size(); }
    
    /**
     * 获取关节名称列表
     */
    const std::vector<std::string>& getJointNames() const { return m_jointNames; }
    
    /**
     * 获取指定帧
     */
    const MotionFrame& getFrame(size_t index) const { return m_frames[index]; }
    
    /**
     * 按时间获取插值后的帧
     * @param time 时间（秒）
     * @param loop 是否循环
     * @return 插值后的帧
     */
    MotionFrame getFrameAtTime(float time, bool loop = true) const;
    
    /**
     * 检查是否有数据
     */
    bool isValid() const { return !m_frames.empty(); }
    
    /**
     * 获取关节在数组中的索引（-1 表示未找到）
     */
    int getJointIndex(const std::string& name) const;

private:
    float m_fps = 50.0f;
    std::vector<std::string> m_jointNames;
    std::vector<MotionFrame> m_frames;
    
    // 线性插值辅助函数
    static Vector3 lerpVector3(const Vector3& a, const Vector3& b, float t);
    static Quaternion slerpQuat(const Quaternion& a, const Quaternion& b, float t);
};

} // namespace mf
