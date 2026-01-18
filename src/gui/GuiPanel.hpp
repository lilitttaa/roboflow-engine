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
    bool applyRootPos = true;
    bool applyRootRot = true;
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

private:
    bool m_visible = true;
    Rectangle m_panelRect = { 0, 0, 280, 600 };
    Vector2 m_panelScroll = { 0, 0 };
    
    // GUI 样式
    void setupStyle();
};

} // namespace mf
