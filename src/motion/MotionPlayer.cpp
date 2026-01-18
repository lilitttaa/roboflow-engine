#include "MotionPlayer.hpp"
#include <iostream>
#include <cmath>

namespace mf {

MotionPlayer::MotionPlayer() {
    // 默认的关节名称映射（从动作数据名称到 G1_jy URDF 名称）
    // 可以通过 setJointMapping 覆盖
    setJointMapping("left_hip_pitch_joint", "leg_l1_joint");
    setJointMapping("left_hip_roll_joint", "leg_l2_joint");
    setJointMapping("left_hip_yaw_joint", "leg_l3_joint");
    setJointMapping("left_knee_joint", "leg_l4_joint");
    setJointMapping("left_ankle_pitch_joint", "leg_l5_joint");
    setJointMapping("left_ankle_roll_joint", "leg_l6_joint");
    
    setJointMapping("right_hip_pitch_joint", "leg_r1_joint");
    setJointMapping("right_hip_roll_joint", "leg_r2_joint");
    setJointMapping("right_hip_yaw_joint", "leg_r3_joint");
    setJointMapping("right_knee_joint", "leg_r4_joint");
    setJointMapping("right_ankle_pitch_joint", "leg_r5_joint");
    setJointMapping("right_ankle_roll_joint", "leg_r6_joint");
    
    setJointMapping("waist_yaw_joint", "waist_joint");
    
    setJointMapping("left_shoulder_pitch_joint", "arm_l1_joint");
    setJointMapping("left_shoulder_roll_joint", "arm_l2_joint");
    setJointMapping("left_shoulder_yaw_joint", "arm_l3_joint");
    setJointMapping("left_elbow_joint", "arm_l4_joint");
    
    setJointMapping("right_shoulder_pitch_joint", "arm_r1_joint");
    setJointMapping("right_shoulder_roll_joint", "arm_r2_joint");
    setJointMapping("right_shoulder_yaw_joint", "arm_r3_joint");
    setJointMapping("right_elbow_joint", "arm_r4_joint");
}

bool MotionPlayer::loadMotion(const std::string& path) {
    m_motion = std::make_unique<MotionData>();
    if (!m_motion->loadFromFile(path)) {
        m_motion.reset();
        return false;
    }
    
    // 重置播放状态
    m_currentTime = 0.0f;
    m_playing = false;
    
    return true;
}

void MotionPlayer::bindRobot(RobotEntity* robot) {
    m_robot = robot;
}

void MotionPlayer::setJointMapping(const std::string& motionName, const std::string& robotName) {
    m_jointMapping[motionName] = robotName;
}

void MotionPlayer::clearJointMappings() {
    m_jointMapping.clear();
}

void MotionPlayer::update(float deltaTime) {
    if (!m_motion || !m_motion->isValid() || !m_playing) {
        return;
    }
    
    // 更新时间
    m_currentTime += deltaTime * m_playbackSpeed;
    
    float duration = m_motion->getDuration();
    if (duration > 0) {
        if (m_loop) {
            m_currentTime = std::fmod(m_currentTime, duration);
            if (m_currentTime < 0) m_currentTime += duration;
        } else {
            if (m_currentTime >= duration) {
                m_currentTime = duration;
                m_playing = false;
            } else if (m_currentTime < 0) {
                m_currentTime = 0;
                m_playing = false;
            }
        }
    }
    
    // 获取插值后的帧并应用
    MotionFrame frame = m_motion->getFrameAtTime(m_currentTime, m_loop);
    applyFrame(frame);
}

void MotionPlayer::stop() {
    m_currentTime = 0.0f;
    m_playing = false;
    
    // 应用第一帧
    if (m_motion && m_motion->isValid()) {
        applyFrame(m_motion->getFrame(0));
    }
}

void MotionPlayer::seekTo(float time) {
    if (!m_motion || !m_motion->isValid()) return;
    
    float duration = m_motion->getDuration();
    if (m_loop && duration > 0) {
        time = std::fmod(time, duration);
        if (time < 0) time += duration;
    } else {
        time = std::max(0.0f, std::min(time, duration));
    }
    
    m_currentTime = time;
    
    // 应用当前帧
    MotionFrame frame = m_motion->getFrameAtTime(m_currentTime, m_loop);
    applyFrame(frame);
}

void MotionPlayer::seekToFrame(size_t frame) {
    if (!m_motion || !m_motion->isValid()) return;
    
    float fps = m_motion->getFPS();
    float time = frame / fps;
    seekTo(time);
}

size_t MotionPlayer::getCurrentFrame() const {
    if (!m_motion || !m_motion->isValid()) return 0;
    return static_cast<size_t>(m_currentTime * m_motion->getFPS());
}

void MotionPlayer::applyFrame(const MotionFrame& frame) {
    if (!m_robot) return;
    
    // 应用根位置（需要坐标转换：动作数据是 Z-up，raylib 是 Y-up）
    if (applyRootPosition) {
        // Z-up to Y-up: (x, y, z) -> (x, z, -y)
        // 但由于机器人已经有 rotation = {-90, 0, 0}，这里直接设置即可
        m_robot->position = {
            frame.rootPos.x + rootPositionOffset.x,
            frame.rootPos.z + rootPositionOffset.y,  // Z -> Y
            -frame.rootPos.y + rootPositionOffset.z  // Y -> -Z
        };
    }
    
    // 应用根旋转
    if (applyRootRotation) {
        // 将四元数转换为欧拉角（度）
        // 由于坐标系转换比较复杂，这里先简化处理
        // TODO: 正确处理坐标系转换
        Vector3 euler = QuaternionToEuler(frame.rootQuat);
        // 转换为度并调整坐标系
        m_robot->rotation = {
            euler.x * RAD2DEG - 90.0f,  // 保持 URDF Z-up -> Y-up 的基础旋转
            euler.z * RAD2DEG,
            -euler.y * RAD2DEG
        };
    }
    
    // 应用关节角度
    const auto& jointNames = m_motion->getJointNames();
    for (size_t i = 0; i < jointNames.size() && i < frame.jointPos.size(); ++i) {
        const std::string& motionJointName = jointNames[i];
        
        // 查找映射后的机器人关节名称
        std::string robotJointName = motionJointName;
        auto it = m_jointMapping.find(motionJointName);
        if (it != m_jointMapping.end()) {
            robotJointName = it->second;
        }
        
        // 设置关节角度
        m_robot->setJointPosition(robotJointName, frame.jointPos[i]);
    }
}

} // namespace mf
