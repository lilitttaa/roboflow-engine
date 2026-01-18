#include "MotionData.hpp"
#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>

namespace mf {

bool MotionData::loadFromFile(const std::string& path) {
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
