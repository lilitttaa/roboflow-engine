#pragma once

#include <raylib.h>
#include <string>
#include <vector>
#include <functional>

// 在包含 raygui.h 之前定义实现宏（只在一个 .cpp 文件中定义）
// 这里只声明，实现在 GuiPanel.cpp 中

namespace mf {

/**
 * GUI 面板类
 * 提供可交互的控制面板
 */
class GuiPanel {
public:
    GuiPanel();
    ~GuiPanel() = default;
    
    /**
     * 初始化 GUI
     */
    void init();
    
    /**
     * 更新 GUI（处理输入）
     */
    void update();
    
    /**
     * 渲染 GUI
     */
    void render();
    
    /**
     * 切换面板显示/隐藏
     */
    void toggle() { m_visible = !m_visible; }
    
    /**
     * 是否可见
     */
    bool isVisible() const { return m_visible; }
    
    /**
     * 设置可见性
     */
    void setVisible(bool visible) { m_visible = visible; }
    
    // ===== 动作文件选择 =====
    std::vector<std::string> motionFiles;      // 可用的动作文件列表
    int selectedMotionIndex = 0;               // 当前选中的动作索引
    bool motionSelectionChanged = false;       // 动作选择改变事件
    
    // ===== 动作播放器控制 =====
    bool playPressed = false;
    bool stopPressed = false;
    bool loopToggled = false;
    bool rootPosToggled = false;
    bool rootRotToggled = false;
    float playbackSpeed = 1.0f;
    float seekPosition = 0.0f;  // 0.0 - 1.0
    bool seekChanged = false;
    
    // 动作播放器状态（由外部设置）
    bool isPlaying = false;
    bool isLooping = true;
    bool applyRootPos = false;     // 默认关闭，避免瞬移
    bool applyRootRot = false;     // 默认关闭
    bool useRelativePos = true;    // 使用相对位置模式
    bool relPosToggled = false;    // 相对位置切换事件
    float currentTime = 0.0f;
    float duration = 1.0f;
    int currentFrame = 0;
    int totalFrames = 100;
    float fps = 50.0f;
    
    // ===== 模式切换 =====
    bool modeToggled = false;
    bool useMotionPlayer = true;
    
    // ===== 简单动画控制 =====
    bool animationToggled = false;
    bool animationPlaying = true;
    bool resetJointsPressed = false;
    
    // ===== 关节控制 =====
    int selectedJointIndex = 0;
    float jointPosition = 0.0f;
    bool jointChanged = false;
    std::vector<std::string> jointNames;
    float jointLower = -3.14f;
    float jointUpper = 3.14f;
    
    // ===== 显示选项 =====
    bool showAxesToggled = false;
    bool showAxes = false;
    bool showGridToggled = false;
    bool showGrid = true;
    
    // ===== 流式传输控制 (gentle-humanoid 集成) =====
    bool streamConnectPressed = false;
    bool streamDisconnectPressed = false;
    bool streamEnabled = false;          // 是否启用流式传输
    bool streamConnected = false;        // 连接状态（由外部设置）
    uint64_t streamFramesSent = 0;       // 已发送帧数
    std::string streamHost = "127.0.0.1";
    int streamPort = 28563;
    
    // ===== 仿真控制 (gentle-humanoid sim2sim + deploy) =====
    bool simStartAllPressed = false;     // 启动全部按钮
    bool simStopAllPressed = false;      // 停止全部按钮
    bool sim2simRunning = false;         // sim2sim 运行状态（由外部设置）
    bool deployRunning = false;          // deploy 运行状态（由外部设置）
    bool autoConnectStream = true;       // 启动后自动连接流式传输
    
    // ===== 角色控制器状态（第三人称模式）=====
    bool isThirdPersonMode = false;      // 是否处于第三人称模式
    bool characterControllerEnabled = false;  // 角色控制器是否启用
    float controllerSpeed = 0.0f;        // 当前移动速度
    float controllerFacingAngle = 0.0f;  // 当前朝向角度
    bool controllerIsMoving = false;     // 是否正在移动
    bool controllerIsRunning = false;    // 是否在跑步
    float controllerWalkSpeed = 1.2f;    // 行走速度
    float controllerRunSpeed = 2.8f;     // 跑步速度
    bool walkSpeedChanged = false;       // 行走速度改变事件
    bool runSpeedChanged = false;        // 跑步速度改变事件
    int controllerMode = 0;              // 0=自由朝向, 1=锁定相机

private:
    bool m_visible = true;
    Rectangle m_panelRect = { 0, 0, 280, 600 };
    Vector2 m_panelScroll = { 0, 0 };
    bool m_motionDropdownActive = false;  // 动作下拉框是否展开
    
    // GUI 样式
    void setupStyle();
};

} // namespace mf
