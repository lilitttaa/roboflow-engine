#pragma once

#include <string>
#include <vector>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>

namespace mf {

/**
 * 进程状态
 */
enum class ProcessState {
    Stopped,
    Starting,
    Running,
    Stopping,
    Error
};

/**
 * 管理单个子进程
 */
class ManagedProcess {
public:
    ManagedProcess(const std::string& name, const std::string& workDir, 
                   const std::vector<std::string>& command)
        : m_name(name)
        , m_workDir(workDir)
        , m_command(command)
        , m_pid(-1)
        , m_state(ProcessState::Stopped)
    {}
    
    ~ManagedProcess() {
        stop();
    }
    
    /**
     * 启动进程（使用 double fork 避免僵尸进程）
     */
    bool start() {
        if (m_state == ProcessState::Running) {
            std::cerr << "[ProcessManager] " << m_name << " is already running" << std::endl;
            return false;
        }
        
        m_state = ProcessState::Starting;
        
        // 创建管道用于获取孙进程的 PID
        int pipefd[2];
        if (pipe(pipefd) < 0) {
            std::cerr << "[ProcessManager] Failed to create pipe" << std::endl;
            m_state = ProcessState::Error;
            return false;
        }
        
        pid_t pid = fork();
        if (pid < 0) {
            std::cerr << "[ProcessManager] Failed to fork for " << m_name << std::endl;
            m_state = ProcessState::Error;
            close(pipefd[0]);
            close(pipefd[1]);
            return false;
        }
        
        if (pid == 0) {
            // 第一个子进程 - 立即 fork 然后退出
            close(pipefd[0]);  // 关闭读端
            
            pid_t grandchild = fork();
            if (grandchild < 0) {
                write(pipefd[1], &grandchild, sizeof(grandchild));
                close(pipefd[1]);
                _exit(1);
            }
            
            if (grandchild == 0) {
                // 孙进程 - 实际执行任务
                close(pipefd[1]);
                
                // 创建新会话，脱离控制终端
                setsid();
                
                if (!m_workDir.empty()) {
                    if (chdir(m_workDir.c_str()) != 0) {
                        std::cerr << "[ProcessManager] Failed to change directory to " << m_workDir << std::endl;
                        _exit(1);
                    }
                }
                
                // Prepare arguments
                std::vector<char*> args;
                for (const auto& arg : m_command) {
                    args.push_back(const_cast<char*>(arg.c_str()));
                }
                args.push_back(nullptr);
                
                // Execute
                execvp(args[0], args.data());
                
                // If exec fails
                std::cerr << "[ProcessManager] Failed to execute " << m_command[0] << std::endl;
                _exit(1);
            }
            
            // 第一个子进程：写入孙进程 PID 然后退出
            write(pipefd[1], &grandchild, sizeof(grandchild));
            close(pipefd[1]);
            _exit(0);
        }
        
        // 父进程
        close(pipefd[1]);  // 关闭写端
        
        // 立即等待第一个子进程退出（不会阻塞太久）
        waitpid(pid, nullptr, 0);
        
        // 读取孙进程的 PID
        pid_t grandchild_pid;
        read(pipefd[0], &grandchild_pid, sizeof(grandchild_pid));
        close(pipefd[0]);
        
        if (grandchild_pid <= 0) {
            std::cerr << "[ProcessManager] Failed to start grandchild process" << std::endl;
            m_state = ProcessState::Error;
            return false;
        }
        
        m_pid = grandchild_pid;
        m_state = ProcessState::Running;
        std::cout << "[ProcessManager] Started " << m_name << " (PID: " << m_pid << ")" << std::endl;
        return true;
    }
    
    /**
     * 停止进程
     */
    bool stop() {
        if (m_state != ProcessState::Running || m_pid <= 0) {
            return true;
        }
        
        m_state = ProcessState::Stopping;
        std::cout << "[ProcessManager] Stopping " << m_name << " (PID: " << m_pid << ")..." << std::endl;
        
        // Send SIGTERM first
        kill(m_pid, SIGTERM);
        
        // Wait up to 3 seconds for graceful shutdown
        for (int i = 0; i < 30; i++) {
            int status;
            pid_t result = waitpid(m_pid, &status, WNOHANG);
            if (result == m_pid) {
                // Process has terminated
                m_pid = -1;
                m_state = ProcessState::Stopped;
                std::cout << "[ProcessManager] " << m_name << " stopped gracefully" << std::endl;
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Force kill
        std::cout << "[ProcessManager] Force killing " << m_name << std::endl;
        kill(m_pid, SIGKILL);
        waitpid(m_pid, nullptr, 0);
        m_pid = -1;
        m_state = ProcessState::Stopped;
        return true;
    }
    
    /**
     * 检查进程是否还在运行
     */
    bool checkAlive() {
        if (m_pid <= 0) {
            m_state = ProcessState::Stopped;
            return false;
        }
        
        int status;
        pid_t result = waitpid(m_pid, &status, WNOHANG);
        if (result == m_pid) {
            // Process has terminated
            m_pid = -1;
            m_state = ProcessState::Stopped;
            return false;
        } else if (result == 0) {
            // Process is still running
            return true;
        } else {
            // Error
            m_state = ProcessState::Error;
            return false;
        }
    }
    
    const std::string& getName() const { return m_name; }
    ProcessState getState() const { return m_state; }
    pid_t getPid() const { return m_pid; }
    
    bool isRunning() {
        checkAlive();
        return m_state == ProcessState::Running;
    }
    
private:
    std::string m_name;
    std::string m_workDir;
    std::vector<std::string> m_command;
    pid_t m_pid;
    ProcessState m_state;
};

/**
 * 进程管理器
 * 管理 sim2sim.py 和 deploy.py 子进程
 */
class ProcessManager {
public:
    ProcessManager() = default;
    ~ProcessManager() {
        stopAll();
    }
    
    /**
     * 设置 gentle-humanoid 目录路径
     */
    void setGentleHumanoidPath(const std::string& path) {
        m_ghPath = path;
    }
    
    /**
     * 清理残留的僵尸进程和停止的进程
     */
    void cleanupZombies() {
        std::cout << "[ProcessManager] Cleaning up zombie/stopped processes..." << std::endl;
        
        // 使用 pkill -9 强制清理可能残留的进程
        system("pkill -9 -f 'sim2sim.py' 2>/dev/null");
        system("pkill -9 -f 'deploy.py' 2>/dev/null");
        system("pkill -9 -f 'multiprocessing.resource_tracker' 2>/dev/null");
        
        // 等待子进程被回收
        int status;
        while (waitpid(-1, &status, WNOHANG) > 0) {
            // 回收僵尸进程
        }
        
        // 重置内部状态
        if (m_sim2sim) {
            m_sim2sim.reset();
        }
        if (m_deploy) {
            m_deploy.reset();
        }
        m_startingAll = false;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "[ProcessManager] Cleanup done" << std::endl;
    }
    
    /**
     * 启动 sim2sim.py
     */
    bool startSim2Sim(bool autoMode = true) {
        if (m_sim2sim && m_sim2sim->isRunning()) {
            std::cerr << "[ProcessManager] sim2sim is already running" << std::endl;
            return false;
        }
        
        std::vector<std::string> cmd = {"python3", "src/sim2sim.py"};
        if (autoMode) {
            cmd.push_back("--auto");
        }
        
        m_sim2sim = std::make_unique<ManagedProcess>("sim2sim", m_ghPath, cmd);
        return m_sim2sim->start();
    }
    
    /**
     * 停止 sim2sim.py
     */
    bool stopSim2Sim() {
        if (m_sim2sim) {
            return m_sim2sim->stop();
        }
        return true;
    }
    
    /**
     * 启动 deploy.py
     */
    bool startDeploy(bool autoMode = true, const std::string& net = "lo") {
        if (m_deploy && m_deploy->isRunning()) {
            std::cerr << "[ProcessManager] deploy is already running" << std::endl;
            return false;
        }
        
        std::vector<std::string> cmd = {"python3", "src/deploy.py", "--sim2sim", "--net", net};
        if (autoMode) {
            cmd.push_back("--auto");
        }
        
        m_deploy = std::make_unique<ManagedProcess>("deploy", m_ghPath, cmd);
        return m_deploy->start();
    }
    
    /**
     * 停止 deploy.py
     */
    bool stopDeploy() {
        if (m_deploy) {
            return m_deploy->stop();
        }
        return true;
    }
    
    /**
     * 启动全部（异步启动，不阻塞主线程）
     */
    bool startAll(bool autoMode = true) {
        if (m_startingAll) {
            std::cerr << "[ProcessManager] Already starting..." << std::endl;
            return false;
        }
        
        // 先清理可能残留的僵尸进程
        cleanupZombies();
        
        if (!startSim2Sim(autoMode)) {
            return false;
        }
        
        // 异步启动 deploy（在后台线程中等待后启动）
        m_startingAll = true;
        m_deployAutoMode = autoMode;
        m_deployStartThread = std::thread([this]() {
            // 等待 sim2sim 初始化
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            
            if (m_startingAll) {
                if (!startDeploy(m_deployAutoMode)) {
                    std::cerr << "[ProcessManager] Failed to start deploy" << std::endl;
                    stopSim2Sim();
                }
            }
            m_startingAll = false;
        });
        m_deployStartThread.detach();
        
        return true;
    }
    
    /**
     * 停止全部
     */
    void stopAll() {
        m_startingAll = false;  // 停止异步启动
        stopDeploy();
        stopSim2Sim();
        
        // 确保子进程被清理
        system("pkill -9 -f 'sim2sim.py' 2>/dev/null");
        system("pkill -9 -f 'deploy.py' 2>/dev/null");
        system("pkill -9 -f 'multiprocessing.resource_tracker' 2>/dev/null");
        
        // 回收僵尸进程
        int status;
        while (waitpid(-1, &status, WNOHANG) > 0) {}
    }
    
    /**
     * 检查状态
     */
    bool isSim2SimRunning() {
        return m_sim2sim && m_sim2sim->isRunning();
    }
    
    bool isDeployRunning() {
        return m_deploy && m_deploy->isRunning();
    }
    
    bool isAllRunning() {
        return isSim2SimRunning() && isDeployRunning();
    }
    
    /**
     * 获取状态字符串
     */
    std::string getStatusString() {
        std::string status;
        
        if (m_sim2sim) {
            status += "sim2sim: ";
            status += m_sim2sim->isRunning() ? "Running" : "Stopped";
        } else {
            status += "sim2sim: Not started";
        }
        
        status += " | ";
        
        if (m_deploy) {
            status += "deploy: ";
            status += m_deploy->isRunning() ? "Running" : "Stopped";
        } else {
            status += "deploy: Not started";
        }
        
        return status;
    }
    
private:
    std::string m_ghPath;
    std::unique_ptr<ManagedProcess> m_sim2sim;
    std::unique_ptr<ManagedProcess> m_deploy;
    
    // 异步启动相关
    std::atomic<bool> m_startingAll{false};
    bool m_deployAutoMode = true;
    std::thread m_deployStartThread;
};

} // namespace mf
