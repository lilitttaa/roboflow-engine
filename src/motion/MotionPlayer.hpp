#pragma once

#include "MotionData.hpp"
#include "../robot/RobotEntity.hpp"
#include <memory>
#include <map>

namespace mf {

/**
 * 动作播放器
 * 将动作数据应用到机器人实体
 */
class MotionPlayer {
public:
    MotionPlayer();
    ~MotionPlayer() = default;
    
    /**
     * 加载动作文件
     * @param path .motion 文件路径
     * @return 是否成功
     */
    bool loadMotion(const std::string& path);
    
    /**
     * 绑定机器人实体
     * @param robot 机器人实体指针
     */
    void bindRobot(RobotEntity* robot);
    
    /**
     * 设置关节名称映射
     * 用于将动作数据中的关节名称映射到机器人 URDF 中的关节名称
     * @param motionName 动作数据中的关节名称
     * @param robotName 机器人中的关节名称
     */
    void setJointMapping(const std::string& motionName, const std::string& robotName);
    
    /**
     * 清除所有关节映射
     */
    void clearJointMappings();
    
    /**
     * 更新播放器（每帧调用）
     * @param deltaTime 时间增量
     */
    void update(float deltaTime);
    
    /**
     * 播放
     */
    void play() { m_playing = true; }
    
    /**
     * 暂停
     */
    void pause() { m_playing = false; }
    
    /**
     * 切换播放/暂停
     */
    void togglePlay() { m_playing = !m_playing; }
    
    /**
     * 停止并重置到开始
     */
    void stop();
    
    /**
     * 跳转到指定时间
     * @param time 时间（秒）
     */
    void seekTo(float time);
    
    /**
     * 跳转到指定帧
     * @param frame 帧索引
     */
    void seekToFrame(size_t frame);
    
    /**
     * 获取当前时间
     */
    float getCurrentTime() const { return m_currentTime; }
    
    /**
     * 获取当前帧索引
     */
    size_t getCurrentFrame() const;
    
    /**
     * 获取总时长
     */
    float getDuration() const { return m_motion ? m_motion->getDuration() : 0.0f; }
    
    /**
     * 获取总帧数
     */
    size_t getNumFrames() const { return m_motion ? m_motion->getNumFrames() : 0; }
    
    /**
     * 是否正在播放
     */
    bool isPlaying() const { return m_playing; }
    
    /**
     * 是否循环播放
     */
    bool isLooping() const { return m_loop; }
    
    /**
     * 设置循环播放
     */
    void setLoop(bool loop) { m_loop = loop; }
    
    /**
     * 获取播放速度
     */
    float getPlaybackSpeed() const { return m_playbackSpeed; }
    
    /**
     * 设置播放速度
     */
    void setPlaybackSpeed(float speed) { m_playbackSpeed = speed; }
    
    /**
     * 是否已加载动作
     */
    bool hasMotion() const { return m_motion && m_motion->isValid(); }
    
    /**
     * 获取动作数据
     */
    const MotionData* getMotion() const { return m_motion.get(); }
    
    /**
     * 是否应用根位置
     */
    bool applyRootPosition = false;  // 默认关闭，避免瞬移
    
    /**
     * 是否应用根旋转
     */
    bool applyRootRotation = false;  // 默认关闭
    
    /**
     * 使用相对位置模式
     * true: 位置相对于动作开始时的位置
     * false: 使用动作数据中的绝对位置
     */
    bool useRelativePosition = true;
    
    /**
     * 根位置偏移（用于手动调整）
     */
    Vector3 rootPositionOffset = {0, 0, 0};

private:
    std::unique_ptr<MotionData> m_motion;
    RobotEntity* m_robot = nullptr;
    
    // 关节名称映射：动作数据名称 -> 机器人名称
    std::map<std::string, std::string> m_jointMapping;
    
    // 播放状态
    float m_currentTime = 0.0f;
    float m_playbackSpeed = 1.0f;
    bool m_playing = false;
    bool m_loop = true;
    
    // 相对位置模式的初始值
    Vector3 m_initialRobotPos = {0, 0, 0};     // 播放开始时机器人的位置
    Vector3 m_initialMotionPos = {0, 0, 0};    // 动作第一帧的根位置
    Quaternion m_initialRobotRot = {0, 0, 0, 1};  // 播放开始时机器人的旋转
    Quaternion m_initialMotionRot = {0, 0, 0, 1}; // 动作第一帧的根旋转
    bool m_initialValuesSet = false;
    
    // 将当前帧应用到机器人
    void applyFrame(const MotionFrame& frame);
    
    // 设置相对位置模式的初始值
    void captureInitialValues();
};

} // namespace mf
