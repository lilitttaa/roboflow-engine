#include "UDPStreamer.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace mf {

UDPStreamer::UDPStreamer() {
    // 预分配缓冲区
    // 格式: magic(4) + timestamp(8) + joints(29*4) + quat(4*4) + pos(3*4) = 148 bytes
    m_buffer.resize(4 + 8 + 29*4 + 4*4 + 3*4);
}

UDPStreamer::~UDPStreamer() {
    disconnect();
}

bool UDPStreamer::connect(const std::string& host, int port) {
    if (m_connected) {
        disconnect();
    }
    
    m_host = host;
    m_port = port;
    
    // 创建 UDP socket
    m_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_socket < 0) {
        m_lastError = "Failed to create socket";
        return false;
    }
    
    // 设置目标地址
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    
    if (inet_pton(AF_INET, host.c_str(), &serverAddr.sin_addr) <= 0) {
        m_lastError = "Invalid address: " + host;
        close(m_socket);
        m_socket = -1;
        return false;
    }
    
    // 使用 connect 绑定目标地址（对于 UDP 仅设置默认目标）
    if (::connect(m_socket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        m_lastError = "Failed to connect";
        close(m_socket);
        m_socket = -1;
        return false;
    }
    
    m_connected = true;
    m_framesSent = 0;
    std::cout << "[UDPStreamer] Connected to " << host << ":" << port << std::endl;
    return true;
}

void UDPStreamer::disconnect() {
    if (m_socket >= 0) {
        close(m_socket);
        m_socket = -1;
    }
    m_connected = false;
    std::cout << "[UDPStreamer] Disconnected. Total frames sent: " << m_framesSent << std::endl;
}

void UDPStreamer::packFrame(const StreamFrame& frame) {
    uint8_t* ptr = m_buffer.data();
    
    // Magic (4 bytes)
    uint32_t magic = MAGIC;
    memcpy(ptr, &magic, 4);
    ptr += 4;
    
    // Timestamp (8 bytes)
    memcpy(ptr, &frame.timestamp, 8);
    ptr += 8;
    
    // Joint positions (29 * 4 = 116 bytes)
    memcpy(ptr, frame.jointPositions, 29 * sizeof(float));
    ptr += 29 * sizeof(float);
    
    // Root quaternion (4 * 4 = 16 bytes)
    memcpy(ptr, frame.rootQuaternion, 4 * sizeof(float));
    ptr += 4 * sizeof(float);
    
    // Root position (3 * 4 = 12 bytes)
    memcpy(ptr, frame.rootPosition, 3 * sizeof(float));
}

bool UDPStreamer::sendFrame(const StreamFrame& frame) {
    if (!m_connected) {
        m_lastError = "Not connected";
        return false;
    }
    
    packFrame(frame);
    
    ssize_t sent = send(m_socket, m_buffer.data(), m_buffer.size(), 0);
    if (sent < 0) {
        m_lastError = "Send failed";
        return false;
    }
    
    m_framesSent++;
    return true;
}

bool UDPStreamer::sendMotion(double timestamp, 
                             const float* jointPositions, int numJoints,
                             const float* rootQuat, const float* rootPos) {
    StreamFrame frame;
    frame.timestamp = timestamp;
    
    // 清零
    memset(frame.jointPositions, 0, sizeof(frame.jointPositions));
    
    // 复制关节角度（最多 29 个）
    int copyCount = std::min(numJoints, 29);
    if (jointPositions) {
        memcpy(frame.jointPositions, jointPositions, copyCount * sizeof(float));
    }
    
    // 复制根节点数据
    if (rootQuat) {
        memcpy(frame.rootQuaternion, rootQuat, 4 * sizeof(float));
    } else {
        // 默认单位四元数 (wxyz)
        frame.rootQuaternion[0] = 1.0f;
        frame.rootQuaternion[1] = 0.0f;
        frame.rootQuaternion[2] = 0.0f;
        frame.rootQuaternion[3] = 0.0f;
    }
    
    if (rootPos) {
        memcpy(frame.rootPosition, rootPos, 3 * sizeof(float));
    } else {
        frame.rootPosition[0] = 0.0f;
        frame.rootPosition[1] = 0.0f;
        frame.rootPosition[2] = 0.0f;
    }
    
    return sendFrame(frame);
}

} // namespace mf
