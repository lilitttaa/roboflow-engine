#pragma once

#include <string>

namespace mf {

/**
 * Application - 应用程序主类
 * 管理窗口、主循环和基础渲染
 */
class Application {
public:
    struct Config {
        int width = 1280;
        int height = 720;
        std::string title = "MotionFlow";
        int targetFPS = 60;
        bool vsync = true;
    };

    Application();
    explicit Application(const Config& config);
    ~Application();

    // 禁止拷贝
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    // 运行主循环
    void run();

    // 检查是否应该关闭
    bool shouldClose() const;

    // 获取窗口尺寸
    int getWidth() const { return m_config.width; }
    int getHeight() const { return m_config.height; }

protected:
    // 子类可重写的方法
    virtual void onInit() {}
    virtual void onUpdate(float deltaTime) {}
    virtual void onRender() {}
    virtual void onShutdown() {}

private:
    Config m_config;
    bool m_initialized = false;

    void init();
    void shutdown();
    void mainLoop();
};

} // namespace mf
