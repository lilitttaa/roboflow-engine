#include "MotionData.hpp"
#include "NpzReader.hpp"
#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>

namespace mf {

bool MotionData::loadFromFile(const std::string& path) {
    // 自动检测格式
    if (path.size() >= 4 && path.substr(path.size() - 4) == ".npz") {
        return loadFromNpz(path);
    }
    return loadFromMotion(path);
}

bool MotionData::loadFromNpz(const std::string& path) {
    NpzReader reader;
    if (!reader.load(path)) {
        std::cerr << "MotionData: Failed to open NPZ file: " << path << std::endl;
        return false;
    }
    
    // 读取 FPS
    if (reader.hasArray("fps")) {
        auto fpsData = reader.getArray("fps").asFloat();
        if (!fpsData.empty()) {
            m_fps = fpsData[0];
        }
    }
    
    // 读取关节名称
    if (reader.hasArray("joint_names")) {
        m_jointNames = reader.getArray("joint_names").asStringArray();
    }
    
    // 读取数据数组
    if (!reader.hasArray("root_pos") || !reader.hasArray("root_rot") || !reader.hasArray("dof_pos")) {
        std::cerr << "MotionData: Missing required arrays in NPZ file" << std::endl;
        return false;
    }
    
    const auto& rootPosArr = reader.getArray("root_pos");
    const auto& rootRotArr = reader.getArray("root_rot");
    const auto& dofPosArr = reader.getArray("dof_pos");
    
    auto rootPosData = rootPosArr.asFloat();
    auto rootRotData = rootRotArr.asFloat();
    auto dofPosData = dofPosArr.asFloat();
    
    size_t numFrames = rootPosArr.shape[0];
    size_t numJoints = dofPosArr.shape.size() > 1 ? dofPosArr.shape[1] : 0;
    
    std::cout << "MotionData: Loading NPZ " << path << std::endl;
    std::cout << "  FPS: " << m_fps << std::endl;
    std::cout << "  Frames: " << numFrames << std::endl;
    std::cout << "  Joints: " << numJoints << std::endl;
    
    // 构建帧数据
    m_frames.resize(numFrames);
    for (size_t i = 0; i < numFrames; ++i) {
        MotionFrame& frame = m_frames[i];
        
        // root_pos (x, y, z)
        frame.rootPos.x = rootPosData[i * 3 + 0];
        frame.rootPos.y = rootPosData[i * 3 + 1];
        frame.rootPos.z = rootPosData[i * 3 + 2];
        
        // root_rot (xyzw -> raylib uses xyzw internally)
        float qx = rootRotData[i * 4 + 0];
        float qy = rootRotData[i * 4 + 1];
        float qz = rootRotData[i * 4 + 2];
        float qw = rootRotData[i * 4 + 3];
        frame.rootQuat = { qx, qy, qz, qw };
        
        // joint positions
        frame.jointPos.resize(numJoints);
        for (size_t j = 0; j < numJoints; ++j) {
            frame.jointPos[j] = dofPosData[i * numJoints + j];
        }
    }
    
    // 如果没有关节名称，使用默认的 G1 关节名称
    if (m_jointNames.empty() && numJoints == 29) {
        m_jointNames = {
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
        };
    }
    
    std::cout << "  Joint names: ";
    for (size_t i = 0; i < std::min(size_t(3), m_jointNames.size()); ++i) {
        std::cout << m_jointNames[i] << " ";
    }
    if (m_jointNames.size() > 3) std::cout << "...";
    std::cout << std::endl;
    
    return true;
}

bool MotionData::loadFromMotion(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "MotionData: Failed to open file: " << path << std::endl;
        return false;
    }
    
    // 读取 magic
    char magic[4];
    file.read(magic, 4);
    if (std::strncmp(magic, "MFMT", 4) != 0) {
        std::cerr << "MotionData: Invalid file format (bad magic)" << std::endl;
        return false;
    }
    
    // 读取 version
    uint32_t version;
    file.read(reinterpret_cast<char*>(&version), sizeof(version));
    if (version != 1) {
        std::cerr << "MotionData: Unsupported version: " << version << std::endl;
        return false;
    }
    
    // 读取 FPS
    file.read(reinterpret_cast<char*>(&m_fps), sizeof(m_fps));
    
    // 读取帧数和关节数
    uint32_t numFrames, numJoints;
    file.read(reinterpret_cast<char*>(&numFrames), sizeof(numFrames));
    file.read(reinterpret_cast<char*>(&numJoints), sizeof(numJoints));
    
    // 读取关节名称偏移
    uint32_t jointNamesOffset;
    file.read(reinterpret_cast<char*>(&jointNamesOffset), sizeof(jointNamesOffset));
    
    std::cout << "MotionData: Loading " << path << std::endl;
    std::cout << "  FPS: " << m_fps << std::endl;
    std::cout << "  Frames: " << numFrames << std::endl;
    std::cout << "  Joints: " << numJoints << std::endl;
    
    // 读取帧数据
    m_frames.resize(numFrames);
    for (uint32_t i = 0; i < numFrames; ++i) {
        MotionFrame& frame = m_frames[i];
        
        // root_pos
        file.read(reinterpret_cast<char*>(&frame.rootPos.x), sizeof(float));
        file.read(reinterpret_cast<char*>(&frame.rootPos.y), sizeof(float));
        file.read(reinterpret_cast<char*>(&frame.rootPos.z), sizeof(float));
        
        // root_quat (wxyz)
        float w, x, y, z;
        file.read(reinterpret_cast<char*>(&w), sizeof(float));
        file.read(reinterpret_cast<char*>(&x), sizeof(float));
        file.read(reinterpret_cast<char*>(&y), sizeof(float));
        file.read(reinterpret_cast<char*>(&z), sizeof(float));
        frame.rootQuat = { x, y, z, w };  // raylib 使用 xyzw 顺序
        
        // joint_pos
        frame.jointPos.resize(numJoints);
        for (uint32_t j = 0; j < numJoints; ++j) {
            file.read(reinterpret_cast<char*>(&frame.jointPos[j]), sizeof(float));
        }
    }
    
    // 读取关节名称
    file.seekg(jointNamesOffset);
    m_jointNames.resize(numJoints);
    for (uint32_t j = 0; j < numJoints; ++j) {
        std::string name;
        char c;
        while (file.get(c) && c != '\0') {
            name += c;
        }
        m_jointNames[j] = name;
    }
    
    std::cout << "  Joint names: ";
    for (size_t i = 0; i < std::min(size_t(3), m_jointNames.size()); ++i) {
        std::cout << m_jointNames[i] << " ";
    }
    if (m_jointNames.size() > 3) std::cout << "...";
    std::cout << std::endl;
    
    return true;
}

MotionFrame MotionData::getFrameAtTime(float time, bool loop) const {
    if (m_frames.empty()) {
        return MotionFrame{};
    }
    
    float duration = getDuration();
    if (duration <= 0) {
        return m_frames[0];
    }
    
    // 处理循环
    if (loop) {
        time = std::fmod(time, duration);
        if (time < 0) time += duration;
    } else {
        time = std::max(0.0f, std::min(time, duration));
    }
    
    // 计算帧索引和插值因子
    float frameFloat = time * m_fps;
    size_t frame0 = static_cast<size_t>(frameFloat);
    size_t frame1 = frame0 + 1;
    float t = frameFloat - frame0;
    
    // 边界检查
    if (frame0 >= m_frames.size() - 1) {
        return m_frames.back();
    }
    
    // 插值
    const MotionFrame& f0 = m_frames[frame0];
    const MotionFrame& f1 = m_frames[frame1];
    
    MotionFrame result;
    result.rootPos = lerpVector3(f0.rootPos, f1.rootPos, t);
    result.rootQuat = slerpQuat(f0.rootQuat, f1.rootQuat, t);
    
    result.jointPos.resize(f0.jointPos.size());
    for (size_t j = 0; j < f0.jointPos.size(); ++j) {
        result.jointPos[j] = f0.jointPos[j] + (f1.jointPos[j] - f0.jointPos[j]) * t;
    }
    
    return result;
}

int MotionData::getJointIndex(const std::string& name) const {
    for (size_t i = 0; i < m_jointNames.size(); ++i) {
        if (m_jointNames[i] == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

Vector3 MotionData::lerpVector3(const Vector3& a, const Vector3& b, float t) {
    return {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    };
}

Quaternion MotionData::slerpQuat(const Quaternion& a, const Quaternion& b, float t) {
    // 使用 raylib 的 QuaternionSlerp，但先检查点积以处理最短路径
    Quaternion bAdjusted = b;
    float dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (dot < 0.0f) {
        bAdjusted = { -b.x, -b.y, -b.z, -b.w };
    }
    return QuaternionSlerp(a, bAdjusted, t);
}

} // namespace mf
