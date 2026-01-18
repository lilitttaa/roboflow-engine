#pragma once

#include <string>
#include <cstdint>
#include <vector>
#include <atomic>

namespace mf {

/**
 * 动作帧数据结构
 * 用于 UDP 传输的数据格式
 */
struct StreamFrame {
    double timestamp;                    // 时间戳 (秒)
    float jointPositions[29];            // 29 个关节角度 (rad)
    float rootQuaternion[4];             // 根节点四元数 (wxyz)
    float rootPosition[3];               // 根节点位置 (xyz)
};

/**
 * UDP 流式传输客户端
 * 将动作数据实时发送到 gentle-humanoid
 */
class UDPStreamer {
public:
    // 数据包魔数标识
    static constexpr uint32_t MAGIC = 0x4D465354;  // "MFST" (MotionFlow STream)
    static constexpr int DEFAULT_PORT = 28563;
    
    UDPStreamer();
    ~UDPStreamer();
    
    /**
     * 连接到服务端
     * @param host 服务端地址
     * @param port 服务端端口
     * @return 是否成功
     */
    bool connect(const std::string& host = "127.0.0.1", int port = DEFAULT_PORT);
    
    /**
     * 断开连接
     */
    void disconnect();
    
    /**
     * 发送一帧数据
     * @param frame 动作帧数据
     * @return 是否成功
     */
    bool sendFrame(const StreamFrame& frame);
    
    /**
     * 发送动作数据（便捷方法）
     * @param timestamp 时间戳
     * @param jointPositions 关节角度数组
     * @param numJoints 关节数量
     * @param rootQuat 根节点四元数 (wxyz)
     * @param rootPos 根节点位置
     * @return 是否成功
     */
    bool sendMotion(double timestamp, 
                    const float* jointPositions, int numJoints,
                    const float* rootQuat, const float* rootPos);
    
    /**
     * 是否已连接
     */
    bool isConnected() const { return m_connected; }
    
    /**
     * 获取发送的帧数
     */
    uint64_t getFramesSent() const { return m_framesSent; }
    
    /**
     * 获取最后一次错误信息
     */
    const std::string& getLastError() const { return m_lastError; }

private:
    int m_socket = -1;
    std::atomic<bool> m_connected{false};
    std::string m_host;
    int m_port = DEFAULT_PORT;
    uint64_t m_framesSent = 0;
    std::string m_lastError;
    
    // 数据包缓冲区
    std::vector<uint8_t> m_buffer;
    
    // 打包数据
    void packFrame(const StreamFrame& frame);
};

} // namespace mf
