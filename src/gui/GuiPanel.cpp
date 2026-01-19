#include "GuiPanel.hpp"

// 定义 raygui 需要但 raylib 中缺失的函数
#ifndef TextToFloat
static float TextToFloat(const char *text) {
    if (text == nullptr) return 0.0f;
    return (float)atof(text);
}
#endif

// raygui implementation
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

namespace mf {

GuiPanel::GuiPanel() {
}

void GuiPanel::init() {
    setupStyle();
}

void GuiPanel::setupStyle() {
    // 设置深色主题风格
    GuiSetStyle(DEFAULT, TEXT_SIZE, 14);
    GuiSetStyle(DEFAULT, TEXT_SPACING, 1);
    
    // 深色背景
    GuiSetStyle(DEFAULT, BACKGROUND_COLOR, 0x2d2d2dff);
    GuiSetStyle(DEFAULT, LINE_COLOR, 0x505050ff);
    GuiSetStyle(DEFAULT, BORDER_COLOR_NORMAL, 0x505050ff);
    GuiSetStyle(DEFAULT, BASE_COLOR_NORMAL, 0x3d3d3dff);
    GuiSetStyle(DEFAULT, TEXT_COLOR_NORMAL, 0xccccccff);
    
    // 悬停状态
    GuiSetStyle(DEFAULT, BORDER_COLOR_FOCUSED, 0x60a060ff);
    GuiSetStyle(DEFAULT, BASE_COLOR_FOCUSED, 0x4d4d4dff);
    GuiSetStyle(DEFAULT, TEXT_COLOR_FOCUSED, 0xffffffff);
    
    // 按下状态
    GuiSetStyle(DEFAULT, BORDER_COLOR_PRESSED, 0x80c080ff);
    GuiSetStyle(DEFAULT, BASE_COLOR_PRESSED, 0x60a060ff);
    GuiSetStyle(DEFAULT, TEXT_COLOR_PRESSED, 0xffffffff);
    
    // 按钮圆角
    GuiSetStyle(BUTTON, BORDER_WIDTH, 1);
    
    // 滑块样式
    GuiSetStyle(SLIDER, SLIDER_WIDTH, 16);
    GuiSetStyle(SLIDER, SLIDER_PADDING, 1);
    
    // 进度条样式
    GuiSetStyle(PROGRESSBAR, PROGRESS_PADDING, 1);
}

void GuiPanel::update() {
    // 按 H 键切换面板显示
    if (IsKeyPressed(KEY_H)) {
        toggle();
    }
    
    // 不在这里重置状态，而是在 render() 开始时重置
    // 这样 update() 可以读取上一帧 render() 设置的状态
}

void GuiPanel::render() {
    // 在渲染开始时重置按钮状态
    playPressed = false;
    stopPressed = false;
    loopToggled = false;
    rootPosToggled = false;
    rootRotToggled = false;
    relPosToggled = false;
    seekChanged = false;
    modeToggled = false;
    animationToggled = false;
    resetJointsPressed = false;
    jointChanged = false;
    showAxesToggled = false;
    showGridToggled = false;
    streamConnectPressed = false;
    streamDisconnectPressed = false;
    simStartAllPressed = false;
    simStopAllPressed = false;
    motionSelectionChanged = false;
    walkSpeedChanged = false;
    runSpeedChanged = false;
    
    if (!m_visible) {
        // 显示提示
        DrawText("[H] Show Panel", 10, GetScreenHeight() - 30, 14, GRAY);
        return;
    }
    
    // 面板位置（右侧）
    int panelX = GetScreenWidth() - 290;
    int panelY = 10;
    int panelWidth = 280;
    int panelHeight = GetScreenHeight() - 20;
    
    // 绘制面板背景
    DrawRectangle(panelX, panelY, panelWidth, panelHeight, Color{45, 45, 48, 240});
    DrawRectangleLines(panelX, panelY, panelWidth, panelHeight, Color{80, 80, 80, 255});
    
    int x = panelX + 10;
    int y = panelY + 10;
    int width = panelWidth - 20;
    
    // 标题
    GuiLabel(Rectangle{(float)x, (float)y, (float)width, 24}, "Control Panel");
    y += 30;
    
    // 分隔线
    DrawLine(x, y, x + width, y, Color{80, 80, 80, 255});
    y += 10;
    
    // ===== 模式切换 =====
    GuiLabel(Rectangle{(float)x, (float)y, (float)width, 20}, "Mode");
    y += 22;
    
    if (GuiButton(Rectangle{(float)x, (float)y, (float)width, 28}, 
                  useMotionPlayer ? "Motion Playback" : "Simple Animation")) {
        modeToggled = true;
    }
    y += 35;
    
    // 分隔线
    DrawLine(x, y, x + width, y, Color{80, 80, 80, 255});
    y += 10;
    
    if (useMotionPlayer) {
        // ===== 动作播放器控制 =====
        GuiLabel(Rectangle{(float)x, (float)y, (float)width, 20}, "Motion Playback");
        y += 22;
        
        // 动作文件选择下拉框
        if (!motionFiles.empty()) {
            GuiLabel(Rectangle{(float)x, (float)y, 60, 20}, "Motion:");
            
            // 构建下拉框文本（用分号分隔）
            std::string dropdownText;
            for (size_t i = 0; i < motionFiles.size(); i++) {
                if (i > 0) dropdownText += ";";
                // 只显示文件名，不显示路径
                std::string name = motionFiles[i];
                size_t lastSlash = name.find_last_of("/\\");
                if (lastSlash != std::string::npos) {
                    name = name.substr(lastSlash + 1);
                }
                // 去掉 .motion 后缀
                size_t dotPos = name.find(".motion");
                if (dotPos != std::string::npos) {
                    name = name.substr(0, dotPos);
                }
                dropdownText += name;
            }
            
            int newIndex = selectedMotionIndex;
            if (GuiDropdownBox(Rectangle{(float)(x + 60), (float)y, (float)(width - 60), 24}, 
                              dropdownText.c_str(), &newIndex, m_motionDropdownActive)) {
                m_motionDropdownActive = !m_motionDropdownActive;
            }
            if (!m_motionDropdownActive && newIndex != selectedMotionIndex) {
                selectedMotionIndex = newIndex;
                motionSelectionChanged = true;
            }
            y += 32;
        }
        
        // 播放状态
        DrawText(TextFormat("Frame: %d / %d", currentFrame, totalFrames), x, y, 12, LIGHTGRAY);
        y += 16;
        DrawText(TextFormat("Time: %.2f / %.2f s", currentTime, duration), x, y, 12, LIGHTGRAY);
        y += 16;
        DrawText(TextFormat("FPS: %.0f", fps), x, y, 12, LIGHTGRAY);
        y += 22;
        
        // 进度条（可拖动）
        float newSeek = seekPosition;
        GuiSlider(Rectangle{(float)x, (float)y, (float)width, 16}, nullptr, nullptr, &newSeek, 0.0f, 1.0f);
        if (newSeek != seekPosition) {
            seekPosition = newSeek;
            seekChanged = true;
        }
        y += 24;
        
        // 播放/暂停 和 停止按钮
        int halfWidth = (width - 10) / 2;
        if (GuiButton(Rectangle{(float)x, (float)y, (float)halfWidth, 30}, isPlaying ? "Pause" : "Play")) {
            playPressed = true;
        }
        if (GuiButton(Rectangle{(float)(x + halfWidth + 10), (float)y, (float)halfWidth, 30}, "Stop")) {
            stopPressed = true;
        }
        y += 38;
        
        // 播放速度
        GuiLabel(Rectangle{(float)x, (float)y, 80, 20}, "Speed:");
        GuiSlider(Rectangle{(float)(x + 60), (float)y, (float)(width - 100), 20}, nullptr, 
                  TextFormat("%.1fx", playbackSpeed), &playbackSpeed, 0.1f, 3.0f);
        y += 28;
        
        // 选项复选框
        bool loopVal = isLooping;
        GuiCheckBox(Rectangle{(float)x, (float)y, 20, 20}, "Loop", &loopVal);
        if (loopVal != isLooping) {
            loopToggled = true;
        }
        y += 26;
        
        bool rootPosVal = applyRootPos;
        GuiCheckBox(Rectangle{(float)x, (float)y, 20, 20}, "Apply Root Position", &rootPosVal);
        if (rootPosVal != applyRootPos) {
            rootPosToggled = true;
        }
        y += 26;
        
        bool rootRotVal = applyRootRot;
        GuiCheckBox(Rectangle{(float)x, (float)y, 20, 20}, "Apply Root Rotation", &rootRotVal);
        if (rootRotVal != applyRootRot) {
            rootRotToggled = true;
        }
        y += 26;
        
        // 只有在启用根位置时才显示相对位置选项
        if (applyRootPos) {
            bool relPosVal = useRelativePos;
            GuiCheckBox(Rectangle{(float)x, (float)y, 20, 20}, "  Use Relative Position", &relPosVal);
            if (relPosVal != useRelativePos) {
                relPosToggled = true;
            }
            y += 26;
        }
        y += 4;
        
    } else {
        // ===== 简单动画控制 =====
        GuiLabel(Rectangle{(float)x, (float)y, (float)width, 20}, "Simple Animation");
        y += 22;
        
        // 动画开关
        bool animVal = animationPlaying;
        GuiCheckBox(Rectangle{(float)x, (float)y, 20, 20}, "Animation Playing", &animVal);
        if (animVal != animationPlaying) {
            animationToggled = true;
        }
        y += 28;
        
        // 重置按钮
        if (GuiButton(Rectangle{(float)x, (float)y, (float)width, 28}, "Reset Joints")) {
            resetJointsPressed = true;
        }
        y += 38;
        
        // 分隔线
        DrawLine(x, y, x + width, y, Color{80, 80, 80, 255});
        y += 10;
        
        // ===== 关节控制 =====
        GuiLabel(Rectangle{(float)x, (float)y, (float)width, 20}, "Joint Control");
        y += 22;
        
        // 关节选择下拉框
        if (!jointNames.empty()) {
            // 简化的关节选择 - 使用上下按钮
            int prevIdx = selectedJointIndex;
            
            // 上一个关节
            if (GuiButton(Rectangle{(float)x, (float)y, 30, 24}, "<")) {
                selectedJointIndex = (selectedJointIndex - 1 + jointNames.size()) % jointNames.size();
            }
            
            // 显示当前关节名
            const char* jointName = selectedJointIndex < jointNames.size() ? 
                                    jointNames[selectedJointIndex].c_str() : "None";
            GuiLabel(Rectangle{(float)(x + 35), (float)y, (float)(width - 70), 24}, jointName);
            
            // 下一个关节
            if (GuiButton(Rectangle{(float)(x + width - 30), (float)y, 30, 24}, ">")) {
                selectedJointIndex = (selectedJointIndex + 1) % jointNames.size();
            }
            y += 30;
            
            // 关节角度滑块
            GuiLabel(Rectangle{(float)x, (float)y, (float)width, 16}, 
                     TextFormat("Position: %.3f rad", jointPosition));
            y += 18;
            
            float prevPos = jointPosition;
            GuiSlider(Rectangle{(float)x, (float)y, (float)width, 20}, nullptr, nullptr, 
                      &jointPosition, jointLower, jointUpper);
            if (jointPosition != prevPos) {
                jointChanged = true;
            }
            y += 26;
            
            // 限位信息
            DrawText(TextFormat("Limits: [%.2f, %.2f]", jointLower, jointUpper), x, y, 12, GRAY);
            y += 22;
        }
    }
    
    // 分隔线
    DrawLine(x, y, x + width, y, Color{80, 80, 80, 255});
    y += 10;
    
    // ===== 显示选项 =====
    GuiLabel(Rectangle{(float)x, (float)y, (float)width, 20}, "Display Options");
    y += 22;
    
    bool axesVal = showAxes;
    GuiCheckBox(Rectangle{(float)x, (float)y, 20, 20}, "Show Axes", &axesVal);
    if (axesVal != showAxes) {
        showAxesToggled = true;
    }
    y += 26;
    
    bool gridVal = showGrid;
    GuiCheckBox(Rectangle{(float)x, (float)y, 20, 20}, "Show Grid", &gridVal);
    if (gridVal != showGrid) {
        showGridToggled = true;
    }
    y += 30;
    
    // ===== 角色控制器状态（仅在第三人称模式下显示）=====
    if (isThirdPersonMode) {
        // 分隔线
        DrawLine(x, y, x + width, y, Color{80, 80, 80, 255});
        y += 10;
        
        GuiLabel(Rectangle{(float)x, (float)y, (float)width, 20}, "Character Controller");
        y += 22;
        
        // 控制模式显示
        const char* modeText = (controllerMode == 0) ? "Free Rotation" : "Lock to Camera";
        Color modeColor = (controllerMode == 0) ? Color{100, 180, 255, 255} : Color{255, 180, 100, 255};
        DrawText(TextFormat("Mode: %s [C]", modeText), x, y, 12, modeColor);
        y += 18;
        
        // 状态显示
        Color statusColor = controllerIsMoving ? Color{100, 200, 100, 255} : Color{150, 150, 150, 255};
        const char* moveStatus = controllerIsMoving ? (controllerIsRunning ? "Running" : "Walking") : "Idle";
        DrawText(TextFormat("Status: %s", moveStatus), x, y, 12, statusColor);
        y += 16;
        
        DrawText(TextFormat("Speed: %.2f m/s", controllerSpeed), x, y, 12, LIGHTGRAY);
        y += 16;
        
        DrawText(TextFormat("Facing: %.1f deg", controllerFacingAngle), x, y, 12, LIGHTGRAY);
        y += 22;
        
        // 行走速度滑块
        GuiLabel(Rectangle{(float)x, (float)y, 80, 16}, "Walk Speed:");
        float prevWalk = controllerWalkSpeed;
        GuiSlider(Rectangle{(float)(x + 80), (float)y, (float)(width - 80), 16}, nullptr, 
                  TextFormat("%.1f", controllerWalkSpeed), &controllerWalkSpeed, 0.5f, 3.0f);
        if (controllerWalkSpeed != prevWalk) {
            walkSpeedChanged = true;
        }
        y += 22;
        
        // 跑步速度滑块
        GuiLabel(Rectangle{(float)x, (float)y, 80, 16}, "Run Speed:");
        float prevRun = controllerRunSpeed;
        GuiSlider(Rectangle{(float)(x + 80), (float)y, (float)(width - 80), 16}, nullptr, 
                  TextFormat("%.1f", controllerRunSpeed), &controllerRunSpeed, 1.0f, 5.0f);
        if (controllerRunSpeed != prevRun) {
            runSpeedChanged = true;
        }
        y += 26;
        
        // 提示
        DrawText("WASD: Move | Shift: Run | C: Mode", x, y, 11, GRAY);
        y += 14;
        DrawText("Alt: Unlock Mouse | Tab: Camera", x, y, 11, GRAY);
        y += 14;
        DrawText("N: Toggle Motion Matching", x, y, 11, GRAY);
        y += 20;
    }
    
    // ===== Motion Matching 控制 =====
    if (isThirdPersonMode) {
        // 分隔线
        DrawLine(x, y, x + width, y, Color{80, 80, 80, 255});
        y += 10;
        
        GuiLabel(Rectangle{(float)x, (float)y, (float)width, 20}, "Motion Matching");
        y += 22;
        
        // 启用/禁用开关
        bool prevEnabled = motionMatchingEnabled;
        GuiCheckBox(Rectangle{(float)x, (float)y, 20, 20}, "Enable Motion Matching", &motionMatchingEnabled);
        if (motionMatchingEnabled != prevEnabled) {
            motionMatchingToggled = true;
        }
        y += 24;
        
        if (motionMatchingEnabled) {
            // 状态显示
            DrawText(TextFormat("Clip: %d  Frame: %d", mmCurrentClip, mmCurrentFrame), x, y, 12, LIGHTGRAY);
            y += 16;
            DrawText(TextFormat("Database: %d entries", mmTotalEntries), x, y, 12, LIGHTGRAY);
            y += 20;
            
            // 搜索间隔
            GuiLabel(Rectangle{(float)x, (float)y, 80, 16}, "Search Int:");
            float searchInt = (float)mmSearchInterval;
            float prevSearchInt = searchInt;
            GuiSlider(Rectangle{(float)(x + 80), (float)y, (float)(width - 80), 16}, nullptr, 
                      TextFormat("%d", mmSearchInterval), &searchInt, 1.0f, 30.0f);
            mmSearchInterval = (int)searchInt;
            if (searchInt != prevSearchInt) {
                mmSearchIntervalChanged = true;
            }
            y += 20;
            
            // 混合时长
            GuiLabel(Rectangle{(float)x, (float)y, 80, 16}, "Blend Time:");
            float prevBlend = mmBlendDuration;
            GuiSlider(Rectangle{(float)(x + 80), (float)y, (float)(width - 80), 16}, nullptr, 
                      TextFormat("%.2f", mmBlendDuration), &mmBlendDuration, 0.05f, 0.5f);
            if (mmBlendDuration != prevBlend) {
                mmBlendDurationChanged = true;
            }
            y += 24;
            
            // 特征权重
            DrawText("Feature Weights:", x, y, 12, LIGHTGRAY);
            y += 16;
            
            // 轨迹位置权重
            GuiLabel(Rectangle{(float)x, (float)y, 70, 14}, "Traj Pos:");
            float prevTrajPos = mmWeightTrajectoryPos;
            GuiSlider(Rectangle{(float)(x + 70), (float)y, (float)(width - 70), 14}, nullptr, 
                      TextFormat("%.1f", mmWeightTrajectoryPos), &mmWeightTrajectoryPos, 0.0f, 3.0f);
            if (mmWeightTrajectoryPos != prevTrajPos) mmWeightsChanged = true;
            y += 18;
            
            // 轨迹朝向权重
            GuiLabel(Rectangle{(float)x, (float)y, 70, 14}, "Traj Face:");
            float prevTrajFace = mmWeightTrajectoryFacing;
            GuiSlider(Rectangle{(float)(x + 70), (float)y, (float)(width - 70), 14}, nullptr, 
                      TextFormat("%.1f", mmWeightTrajectoryFacing), &mmWeightTrajectoryFacing, 0.0f, 3.0f);
            if (mmWeightTrajectoryFacing != prevTrajFace) mmWeightsChanged = true;
            y += 18;
            
            // 髋部速度权重
            GuiLabel(Rectangle{(float)x, (float)y, 70, 14}, "Hip Vel:");
            float prevHipVel = mmWeightHipVel;
            GuiSlider(Rectangle{(float)(x + 70), (float)y, (float)(width - 70), 14}, nullptr, 
                      TextFormat("%.1f", mmWeightHipVel), &mmWeightHipVel, 0.0f, 3.0f);
            if (mmWeightHipVel != prevHipVel) mmWeightsChanged = true;
            y += 22;
            
            // 显示轨迹开关
            bool prevShowTraj = mmShowTrajectory;
            GuiCheckBox(Rectangle{(float)x, (float)y, 20, 20}, "Show Trajectory", &mmShowTrajectory);
            if (mmShowTrajectory != prevShowTraj) {
                mmShowTrajectoryToggled = true;
            }
            y += 28;
        }
    }
    
    // 分隔线
    DrawLine(x, y, x + width, y, Color{80, 80, 80, 255});
    y += 10;
    
    // ===== 仿真控制 (Gentle-Humanoid) =====
    GuiLabel(Rectangle{(float)x, (float)y, (float)width, 20}, "Simulation Control");
    y += 22;
    
    // 进程状态
    {
        const char* sim2simStatus = sim2simRunning ? "Running" : "Stopped";
        Color sim2simColor = sim2simRunning ? Color{100, 200, 100, 255} : Color{150, 150, 150, 255};
        DrawText(TextFormat("sim2sim: %s", sim2simStatus), x, y, 12, sim2simColor);
        y += 16;
        
        const char* deployStatus = deployRunning ? "Running" : "Stopped";
        Color deployColor = deployRunning ? Color{100, 200, 100, 255} : Color{150, 150, 150, 255};
        DrawText(TextFormat("deploy: %s", deployStatus), x, y, 12, deployColor);
        y += 20;
    }
    
    // 启动/停止按钮
    bool allRunning = sim2simRunning && deployRunning;
    if (!allRunning) {
        if (GuiButton(Rectangle{(float)x, (float)y, (float)width, 30}, "Start Simulation")) {
            simStartAllPressed = true;
        }
    } else {
        if (GuiButton(Rectangle{(float)x, (float)y, (float)width, 30}, "Stop Simulation")) {
            simStopAllPressed = true;
        }
    }
    y += 38;
    
    // 自动连接选项
    GuiCheckBox(Rectangle{(float)x, (float)y, 20, 20}, "Auto Connect Stream", &autoConnectStream);
    y += 30;
    
    // 分隔线
    DrawLine(x, y, x + width, y, Color{80, 80, 80, 255});
    y += 10;
    
    // ===== 流式传输 =====
    GuiLabel(Rectangle{(float)x, (float)y, (float)width, 20}, "Motion Streaming");
    y += 22;
    
    // 连接状态
    const char* statusText = streamConnected ? "Connected" : "Disconnected";
    Color statusColor = streamConnected ? Color{100, 200, 100, 255} : Color{200, 100, 100, 255};
    DrawText(TextFormat("Status: %s", statusText), x, y, 12, statusColor);
    y += 18;
    
    if (streamConnected) {
        DrawText(TextFormat("Frames sent: %llu", (unsigned long long)streamFramesSent), x, y, 12, LIGHTGRAY);
        y += 18;
    }
    
    // 连接/断开按钮
    if (!streamConnected) {
        if (GuiButton(Rectangle{(float)x, (float)y, (float)width, 28}, "Connect Stream")) {
            streamConnectPressed = true;
        }
    } else {
        if (GuiButton(Rectangle{(float)x, (float)y, (float)width, 28}, "Disconnect Stream")) {
            streamDisconnectPressed = true;
        }
    }
    y += 35;
    
    // 流式传输开关
    bool streamVal = streamEnabled;
    GuiCheckBox(Rectangle{(float)x, (float)y, 20, 20}, "Enable Streaming", &streamVal);
    streamEnabled = streamVal;
    y += 30;
    
    // 底部提示
    DrawText("[H] Hide Panel", x, panelY + panelHeight - 25, 12, GRAY);
}

} // namespace mf
