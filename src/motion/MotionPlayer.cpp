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
    
    // 首次播放时捕获初始值
    if (!m_initialValuesSet) {
        captureInitialValues();
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

void MotionPlayer::captureInitialValues() {
    if (!m_robot || !m_motion || !m_motion->isValid()) return;
    
    // 保存机器人当前位置
    m_initialRobotPos = m_robot->position;
    
    // 保存动作第一帧的根位置（转换到 Y-up 坐标系）
    const MotionFrame& firstFrame = m_motion->getFrame(0);
    m_initialMotionPos = {
        firstFrame.rootPos.x,
        firstFrame.rootPos.z,   // Z -> Y
        -firstFrame.rootPos.y   // Y -> -Z
    };
    
    m_initialMotionRot = firstFrame.rootQuat;
    
    m_initialValuesSet = true;
    std::cout << "[MotionPlayer] Captured initial position: robot(" 
              << m_initialRobotPos.x << ", " << m_initialRobotPos.y << ", " << m_initialRobotPos.z
              << ") motion(" << m_initialMotionPos.x << ", " << m_initialMotionPos.y << ", " << m_initialMotionPos.z << ")"
              << std::endl;
}

void MotionPlayer::stop() {
    m_currentTime = 0.0f;
    m_playing = false;
    m_initialValuesSet = false;  // 重置初始值，下次播放时重新捕获
    
    // 应用第一帧（但不改变位置）
    if (m_motion && m_motion->isValid() && m_robot) {
        // 保存当前位置
        Vector3 savedPos = m_robot->position;
        Vector3 savedRot = m_robot->rotation;
        
        // 只应用关节角度
        const MotionFrame& frame = m_motion->getFrame(0);
        const auto& jointNames = m_motion->getJointNames();
        for (size_t i = 0; i < jointNames.size() && i < frame.jointPos.size(); ++i) {
            const std::string& motionJointName = jointNames[i];
            std::string robotJointName = motionJointName;
            auto it = m_jointMapping.find(motionJointName);
            if (it != m_jointMapping.end()) {
                robotJointName = it->second;
            }
            m_robot->setJointPosition(robotJointName, frame.jointPos[i]);
        }
        
        // 恢复位置（如果不应用根位置）
        if (!applyRootPosition) {
            m_robot->position = savedPos;
        }
        if (!applyRootRotation) {
            m_robot->rotation = savedRot;
        }
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
        // 当前帧的根位置（转换到 Y-up）
        Vector3 currentMotionPos = {
            frame.rootPos.x,
            frame.rootPos.z,   // Z -> Y
            -frame.rootPos.y   // Y -> -Z
        };
        
        if (useRelativePosition && m_initialValuesSet) {
            // 相对模式：计算相对于第一帧的位移，然后应用到初始机器人位置
            Vector3 delta = {
                currentMotionPos.x - m_initialMotionPos.x,
                currentMotionPos.y - m_initialMotionPos.y,
                currentMotionPos.z - m_initialMotionPos.z
            };
            m_robot->position = {
                m_initialRobotPos.x + delta.x + rootPositionOffset.x,
                m_initialRobotPos.y + delta.y + rootPositionOffset.y,
                m_initialRobotPos.z + delta.z + rootPositionOffset.z
            };
        } else {
            // 绝对模式：直接使用动作数据中的位置
            m_robot->position = {
                currentMotionPos.x + rootPositionOffset.x,
                currentMotionPos.y + rootPositionOffset.y,
                currentMotionPos.z + rootPositionOffset.z
            };
        }
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
