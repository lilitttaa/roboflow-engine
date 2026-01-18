#include "MeshLoader.hpp"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>
#include <cstring>

namespace mf {

MeshLoader::~MeshLoader() {
    clearCache();
}

void MeshLoader::clearCache() {
    for (auto& [path, mesh] : m_cache) {
        UnloadMesh(mesh);
    }
    m_cache.clear();
}

Mesh MeshLoader::load(const std::string& filepath) {
    // 检查缓存
    auto it = m_cache.find(filepath);
    if (it != m_cache.end()) {
        return it->second;
    }
    
    // 加载新网格
    Mesh mesh = loadWithAssimp(filepath);
    if (mesh.vertexCount > 0) {
        m_cache[filepath] = mesh;
    }
    
    return mesh;
}

Mesh MeshLoader::loadWithAssimp(const std::string& filepath) {
    Mesh mesh = {0};
    
    Assimp::Importer importer;
    
    const aiScene* scene = importer.ReadFile(filepath,
        aiProcess_Triangulate |
        aiProcess_GenNormals |
        aiProcess_JoinIdenticalVertices |
        aiProcess_OptimizeMeshes
    );
    
    if (!scene || !scene->mRootNode || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) {
        std::cerr << "MeshLoader: Failed to load " << filepath << std::endl;
        std::cerr << "  Error: " << importer.GetErrorString() << std::endl;
        return mesh;
    }
    
    if (scene->mNumMeshes == 0) {
        std::cerr << "MeshLoader: No meshes in " << filepath << std::endl;
        return mesh;
    }
    
    // 合并所有子网格
    int totalVertices = 0;
    int totalTriangles = 0;
    
    for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
        totalVertices += scene->mMeshes[i]->mNumVertices;
        totalTriangles += scene->mMeshes[i]->mNumFaces;
    }
    
    // 分配内存
    mesh.vertexCount = totalVertices;
    mesh.triangleCount = totalTriangles;
    
    mesh.vertices = (float*)RL_MALLOC(totalVertices * 3 * sizeof(float));
    mesh.normals = (float*)RL_MALLOC(totalVertices * 3 * sizeof(float));
    mesh.texcoords = (float*)RL_MALLOC(totalVertices * 2 * sizeof(float));
    mesh.indices = (unsigned short*)RL_MALLOC(totalTriangles * 3 * sizeof(unsigned short));
    
    int vertexOffset = 0;
    int indexOffset = 0;
    int baseVertex = 0;
    
    for (unsigned int m = 0; m < scene->mNumMeshes; m++) {
        const aiMesh* aiMesh = scene->mMeshes[m];
        
        // 复制顶点和法线
        for (unsigned int i = 0; i < aiMesh->mNumVertices; i++) {
            // 顶点位置 - 保持原始单位（URDF 中的单位通常已经是米）
            mesh.vertices[vertexOffset + i * 3 + 0] = aiMesh->mVertices[i].x;
            mesh.vertices[vertexOffset + i * 3 + 1] = aiMesh->mVertices[i].y;
            mesh.vertices[vertexOffset + i * 3 + 2] = aiMesh->mVertices[i].z;
            
            // 法线
            if (aiMesh->HasNormals()) {
                mesh.normals[vertexOffset + i * 3 + 0] = aiMesh->mNormals[i].x;
                mesh.normals[vertexOffset + i * 3 + 1] = aiMesh->mNormals[i].y;
                mesh.normals[vertexOffset + i * 3 + 2] = aiMesh->mNormals[i].z;
            } else {
                mesh.normals[vertexOffset + i * 3 + 0] = 0.0f;
                mesh.normals[vertexOffset + i * 3 + 1] = 1.0f;
                mesh.normals[vertexOffset + i * 3 + 2] = 0.0f;
            }
            
            // UV 坐标
            if (aiMesh->HasTextureCoords(0)) {
                mesh.texcoords[(vertexOffset / 3) * 2 + i * 2 + 0] = aiMesh->mTextureCoords[0][i].x;
                mesh.texcoords[(vertexOffset / 3) * 2 + i * 2 + 1] = aiMesh->mTextureCoords[0][i].y;
            } else {
                mesh.texcoords[(vertexOffset / 3) * 2 + i * 2 + 0] = 0.0f;
                mesh.texcoords[(vertexOffset / 3) * 2 + i * 2 + 1] = 0.0f;
            }
        }
        
        // 复制索引
        for (unsigned int i = 0; i < aiMesh->mNumFaces; i++) {
            const aiFace& face = aiMesh->mFaces[i];
            if (face.mNumIndices == 3) {
                mesh.indices[indexOffset + i * 3 + 0] = (unsigned short)(baseVertex + face.mIndices[0]);
                mesh.indices[indexOffset + i * 3 + 1] = (unsigned short)(baseVertex + face.mIndices[1]);
                mesh.indices[indexOffset + i * 3 + 2] = (unsigned short)(baseVertex + face.mIndices[2]);
            }
        }
        
        vertexOffset += aiMesh->mNumVertices * 3;
        indexOffset += aiMesh->mNumFaces * 3;
        baseVertex += aiMesh->mNumVertices;
    }
    
    // 上传到 GPU
    UploadMesh(&mesh, false);
    
    std::cout << "MeshLoader: Loaded " << filepath << " (" 
              << mesh.vertexCount << " vertices, " 
              << mesh.triangleCount << " triangles)" << std::endl;
    
    return mesh;
}

} // namespace mf
