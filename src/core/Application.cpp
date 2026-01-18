#include "Application.hpp"
#include <raylib.h>

namespace mf {

Application::Application() 
    : m_config{} {
}

Application::Application(const Config& config) 
    : m_config(config) {
}

Application::~Application() {
    if (m_initialized) {
        shutdown();
    }
}

void Application::init() {
    // 配置窗口
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    if (m_config.vsync) {
        SetConfigFlags(FLAG_VSYNC_HINT);
    }
    
    // 初始化窗口
    InitWindow(m_config.width, m_config.height, m_config.title.c_str());
    SetTargetFPS(m_config.targetFPS);
    
    m_initialized = true;
    
    // 调用子类初始化
    onInit();
}

void Application::shutdown() {
    // 调用子类清理
    onShutdown();
    
    // 关闭窗口
    CloseWindow();
    m_initialized = false;
}

void Application::run() {
    init();
    mainLoop();
    shutdown();
}

bool Application::shouldClose() const {
    return WindowShouldClose();
}

void Application::mainLoop() {
    while (!shouldClose()) {
        float deltaTime = GetFrameTime();
        
        // 更新逻辑
        onUpdate(deltaTime);
        
        // 渲染
        BeginDrawing();
        onRender();
        EndDrawing();
    }
}

} // namespace mf
