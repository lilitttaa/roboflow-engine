# FetchContent 用于下载依赖
include(FetchContent)

# ============================================================================
# raylib - 渲染框架
# ============================================================================
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.0
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)

# raylib 配置选项
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(raylib)
message(STATUS "raylib fetched and configured")

# ============================================================================
# Assimp - 使用系统安装的库
# 安装: sudo apt install libassimp-dev
# ============================================================================
find_package(assimp REQUIRED)
message(STATUS "Assimp found: ${assimp_VERSION}")

# ============================================================================
# tinyxml2 - 使用系统安装的库 (使用 pkg-config)
# 安装: sudo apt install libtinyxml2-dev
# ============================================================================
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)
message(STATUS "tinyxml2 found: ${TINYXML2_VERSION}")
