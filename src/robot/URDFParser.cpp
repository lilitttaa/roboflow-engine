#include "URDFParser.hpp"
#include <tinyxml2.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <filesystem>

namespace mf {

std::vector<std::string> URDFModel::getChildJoints(const std::string& linkName) const {
    std::vector<std::string> result;
    for (const auto& [name, joint] : joints) {
        if (joint.parentLink == linkName) {
            result.push_back(name);
        }
    }
    return result;
}

std::vector<std::string> URDFModel::getMovableJoints() const {
    std::vector<std::string> result;
    for (const auto& [name, joint] : joints) {
        if (joint.type != "fixed") {
            result.push_back(name);
        }
    }
    return result;
}

Vector3 URDFParser::parseVector3(const char* str) {
    if (!str) return {0, 0, 0};
    
    Vector3 v = {0, 0, 0};
    sscanf(str, "%f %f %f", &v.x, &v.y, &v.z);
    return v;
}

Color URDFParser::parseColor(const char* rgba) {
    if (!rgba) return WHITE;
    
    float r, g, b, a;
    if (sscanf(rgba, "%f %f %f %f", &r, &g, &b, &a) == 4) {
        return {
            (unsigned char)(r * 255),
            (unsigned char)(g * 255),
            (unsigned char)(b * 255),
            (unsigned char)(a * 255)
        };
    }
    return WHITE;
}

std::unique_ptr<URDFModel> URDFParser::load(const std::string& filepath) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(filepath.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "URDFParser: Failed to load " << filepath << std::endl;
        return nullptr;
    }
    
    auto* robotElem = doc.FirstChildElement("robot");
    if (!robotElem) {
        std::cerr << "URDFParser: No <robot> element found" << std::endl;
        return nullptr;
    }
    
    auto model = std::make_unique<URDFModel>();
    model->name = robotElem->Attribute("name") ? robotElem->Attribute("name") : "unnamed";
    model->basePath = std::filesystem::path(filepath).parent_path().string();
    
    std::cout << "URDFParser: Loading robot '" << model->name << "'" << std::endl;
    
    // 解析所有 link
    for (auto* linkElem = robotElem->FirstChildElement("link"); 
         linkElem; 
         linkElem = linkElem->NextSiblingElement("link")) {
        
        URDFLink link;
        link.name = linkElem->Attribute("name") ? linkElem->Attribute("name") : "";
        
        if (link.name.empty()) continue;
        
        // 解析 visual
        if (auto* visual = linkElem->FirstChildElement("visual")) {
            // origin
            if (auto* origin = visual->FirstChildElement("origin")) {
                link.visualOrigin = parseVector3(origin->Attribute("xyz"));
                link.visualRPY = parseVector3(origin->Attribute("rpy"));
            }
            
            // geometry -> mesh
            if (auto* geometry = visual->FirstChildElement("geometry")) {
                if (auto* mesh = geometry->FirstChildElement("mesh")) {
                    const char* filename = mesh->Attribute("filename");
                    if (filename) {
                        link.meshFile = filename;
                    }
                }
            }
            
            // material -> color
            if (auto* material = visual->FirstChildElement("material")) {
                if (auto* color = material->FirstChildElement("color")) {
                    link.color = parseColor(color->Attribute("rgba"));
                }
            }
        }
        
        // 解析 inertial (可选)
        if (auto* inertial = linkElem->FirstChildElement("inertial")) {
            if (auto* mass = inertial->FirstChildElement("mass")) {
                mass->QueryFloatAttribute("value", &link.mass);
            }
            if (auto* origin = inertial->FirstChildElement("origin")) {
                link.inertialOrigin = parseVector3(origin->Attribute("xyz"));
            }
        }
        
        model->links[link.name] = link;
    }
    
    // 解析所有 joint
    std::map<std::string, bool> hasParent;  // 用于找根 link
    
    for (auto* jointElem = robotElem->FirstChildElement("joint"); 
         jointElem; 
         jointElem = jointElem->NextSiblingElement("joint")) {
        
        URDFJoint joint;
        joint.name = jointElem->Attribute("name") ? jointElem->Attribute("name") : "";
        joint.type = jointElem->Attribute("type") ? jointElem->Attribute("type") : "fixed";
        
        if (joint.name.empty()) continue;
        
        // parent & child
        if (auto* parent = jointElem->FirstChildElement("parent")) {
            joint.parentLink = parent->Attribute("link") ? parent->Attribute("link") : "";
        }
        if (auto* child = jointElem->FirstChildElement("child")) {
            joint.childLink = child->Attribute("link") ? child->Attribute("link") : "";
            hasParent[joint.childLink] = true;
        }
        
        // origin
        if (auto* origin = jointElem->FirstChildElement("origin")) {
            joint.origin = parseVector3(origin->Attribute("xyz"));
            joint.rpy = parseVector3(origin->Attribute("rpy"));
        }
        
        // axis
        if (auto* axis = jointElem->FirstChildElement("axis")) {
            joint.axis = parseVector3(axis->Attribute("xyz"));
        }
        
        // limit
        if (auto* limit = jointElem->FirstChildElement("limit")) {
            limit->QueryFloatAttribute("lower", &joint.lowerLimit);
            limit->QueryFloatAttribute("upper", &joint.upperLimit);
            limit->QueryFloatAttribute("effort", &joint.effort);
            limit->QueryFloatAttribute("velocity", &joint.velocity);
        }
        
        model->joints[joint.name] = joint;
    }
    
    // 找到根 link (没有 parent 的 link)
    for (const auto& [name, link] : model->links) {
        if (hasParent.find(name) == hasParent.end()) {
            model->rootLink = name;
            break;
        }
    }
    
    std::cout << "URDFParser: Loaded " << model->links.size() << " links, " 
              << model->joints.size() << " joints" << std::endl;
    std::cout << "URDFParser: Root link: " << model->rootLink << std::endl;
    
    return model;
}

} // namespace mf
