#pragma once

#include <string>
#include <map>
#include <raylib.h>

namespace mf {

/**
 * 网格加载器
 * 使用 Assimp 加载各种 3D 格式 (STL, OBJ 等)
 * 支持缓存避免重复加载
 */
class MeshLoader {
public:
    ~MeshLoader();
    
    /**
     * 加载网格文件
     * @param filepath 文件路径
     * @return raylib Mesh，失败返回空 Mesh
     */
    Mesh load(const std::string& filepath);
    
    /**
     * 清空缓存
     */
    void clearCache();
    
    /**
     * 获取已缓存的网格数量
     */
    size_t getCacheSize() const { return m_cache.size(); }

private:
    std::map<std::string, Mesh> m_cache;
    
    Mesh loadWithAssimp(const std::string& filepath);
};

} // namespace mf
