#pragma once

#include "Transform.hpp"
#include <rlgl.h>
#include <string>
#include <vector>
#include <memory>
#include <functional>

namespace mf {

/**
 * Entity - 场景实体
 * 支持层级结构的场景节点
 */
class Entity {
public:
    std::string name;
    Transform transform;
    bool visible = true;

    Entity(const std::string& name = "Entity") : name(name) {}
    virtual ~Entity() = default;

    // 获取父节点
    Entity* getParent() const { return m_parent; }

    // 获取子节点
    const std::vector<std::unique_ptr<Entity>>& getChildren() const { return m_children; }

    // 添加子节点
    Entity* addChild(std::unique_ptr<Entity> child) {
        child->m_parent = this;
        m_children.push_back(std::move(child));
        return m_children.back().get();
    }

    // 创建并添加子节点
    template<typename T = Entity, typename... Args>
    T* createChild(Args&&... args) {
        auto child = std::make_unique<T>(std::forward<Args>(args)...);
        T* ptr = child.get();
        addChild(std::move(child));
        return ptr;
    }

    // 移除子节点
    bool removeChild(Entity* child) {
        auto it = std::find_if(m_children.begin(), m_children.end(),
            [child](const auto& ptr) { return ptr.get() == child; });
        
        if (it != m_children.end()) {
            (*it)->m_parent = nullptr;
            m_children.erase(it);
            return true;
        }
        return false;
    }

    // 按名称查找子节点（递归）
    Entity* findChild(const std::string& name) {
        for (auto& child : m_children) {
            if (child->name == name) {
                return child.get();
            }
            if (Entity* found = child->findChild(name)) {
                return found;
            }
        }
        return nullptr;
    }

    // 获取世界变换矩阵
    Matrix getWorldMatrix() const {
        Matrix local = transform.getMatrix();
        if (m_parent) {
            return MatrixMultiply(local, m_parent->getWorldMatrix());
        }
        return local;
    }

    // 获取世界位置
    Vector3 getWorldPosition() const {
        Matrix world = getWorldMatrix();
        return { world.m12, world.m13, world.m14 };
    }

    // 更新（子类可重写）
    virtual void onUpdate(float deltaTime) {}

    // 渲染（子类可重写）
    virtual void onRender() {}

    // 递归更新
    void update(float deltaTime) {
        onUpdate(deltaTime);
        for (auto& child : m_children) {
            child->update(deltaTime);
        }
    }

    // 递归渲染
    void render() {
        if (!visible) return;
        
        rlPushMatrix();
        rlMultMatrixf(MatrixToFloat(transform.getMatrix()));
        
        onRender();
        
        for (auto& child : m_children) {
            child->render();
        }
        
        rlPopMatrix();
    }

    // 遍历所有节点
    void traverse(const std::function<void(Entity*)>& callback) {
        callback(this);
        for (auto& child : m_children) {
            child->traverse(callback);
        }
    }

private:
    Entity* m_parent = nullptr;
    std::vector<std::unique_ptr<Entity>> m_children;
};

/**
 * MeshEntity - 带网格渲染的实体
 */
class MeshEntity : public Entity {
public:
    Color color = WHITE;
    bool wireframe = false;

    enum class PrimitiveType {
        Cube,
        Sphere,
        Cylinder,
        Plane
    };

    PrimitiveType primitiveType = PrimitiveType::Cube;
    Vector3 size = { 1.0f, 1.0f, 1.0f };

    MeshEntity(const std::string& name = "MeshEntity") : Entity(name) {}

    void onRender() override {
        switch (primitiveType) {
            case PrimitiveType::Cube:
                if (wireframe) {
                    DrawCubeWiresV(Vector3Zero(), size, color);
                } else {
                    DrawCubeV(Vector3Zero(), size, color);
                }
                break;
            case PrimitiveType::Sphere:
                if (wireframe) {
                    DrawSphereWires(Vector3Zero(), size.x * 0.5f, 16, 16, color);
                } else {
                    DrawSphere(Vector3Zero(), size.x * 0.5f, color);
                }
                break;
            case PrimitiveType::Cylinder:
                if (wireframe) {
                    DrawCylinderWiresEx(
                        { 0, -size.y * 0.5f, 0 },
                        { 0, size.y * 0.5f, 0 },
                        size.x * 0.5f, size.x * 0.5f, 16, color);
                } else {
                    DrawCylinderEx(
                        { 0, -size.y * 0.5f, 0 },
                        { 0, size.y * 0.5f, 0 },
                        size.x * 0.5f, size.x * 0.5f, 16, color);
                }
                break;
            case PrimitiveType::Plane:
                DrawPlane(Vector3Zero(), { size.x, size.z }, color);
                break;
        }
    }
};

} // namespace mf
