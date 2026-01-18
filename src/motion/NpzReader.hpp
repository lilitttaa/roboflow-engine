#pragma once

#include <string>
#include <vector>
#include <map>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <cstring>
#include <stdexcept>
#include <algorithm>

namespace mf {

/**
 * Simple NPZ reader for loading NumPy compressed archives.
 * 
 * NPZ is a ZIP archive containing .npy files.
 * NPY format: magic + version + header_len + header (dict) + data
 */
class NpzReader {
public:
    struct NpyArray {
        std::vector<size_t> shape;
        std::string dtype;      // e.g., "<f4" for float32 little-endian
        bool fortran_order = false;
        std::vector<char> data;
        
        size_t numElements() const {
            size_t n = 1;
            for (auto s : shape) n *= s;
            return n;
        }
        
        size_t elementSize() const {
            if (dtype.size() >= 2) {
                // Parse the number at the end of dtype (e.g., 'f4' -> 4, 'U26' -> 26)
                size_t numStart = dtype.find_first_of("0123456789");
                if (numStart != std::string::npos) {
                    return std::stoul(dtype.substr(numStart));
                }
            }
            return 4;
        }
        
        // Get data as float array
        std::vector<float> asFloat() const {
            std::vector<float> result(numElements());
            if (dtype.find('f') != std::string::npos && elementSize() == 4) {
                std::memcpy(result.data(), data.data(), result.size() * sizeof(float));
            }
            return result;
        }
        
        // Get data as string array (for joint names)
        std::vector<std::string> asStringArray() const {
            std::vector<std::string> result;
            if (dtype.find('U') != std::string::npos || dtype.find('S') != std::string::npos) {
                size_t strLen = elementSize();  // Number of characters per string
                size_t count = numElements();
                
                // Unicode strings: 4 bytes per character (UTF-32)
                if (dtype.find('U') != std::string::npos) {
                    size_t bytesPerString = strLen * 4;  // 4 bytes per Unicode char
                    for (size_t i = 0; i < count; i++) {
                        std::string s;
                        const char* strStart = data.data() + i * bytesPerString;
                        for (size_t j = 0; j < strLen; j++) {
                            uint32_t c = *reinterpret_cast<const uint32_t*>(strStart + j * 4);
                            if (c == 0) break;
                            if (c < 128) {
                                s += static_cast<char>(c);
                            }
                        }
                        result.push_back(s);
                    }
                } else {
                    // ASCII strings (S type)
                    for (size_t i = 0; i < count; i++) {
                        const char* ptr = data.data() + i * strLen;
                        result.push_back(std::string(ptr, strnlen(ptr, strLen)));
                    }
                }
            }
            return result;
        }
    };
    
    bool load(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open()) {
            return false;
        }
        
        // Read entire file
        file.seekg(0, std::ios::end);
        size_t fileSize = file.tellg();
        file.seekg(0, std::ios::beg);
        
        std::vector<char> buffer(fileSize);
        file.read(buffer.data(), fileSize);
        file.close();
        
        return parseZip(buffer);
    }
    
    bool hasArray(const std::string& name) const {
        return m_arrays.find(name) != m_arrays.end();
    }
    
    const NpyArray& getArray(const std::string& name) const {
        return m_arrays.at(name);
    }
    
    const std::map<std::string, NpyArray>& arrays() const {
        return m_arrays;
    }

private:
    std::map<std::string, NpyArray> m_arrays;
    
    bool parseZip(const std::vector<char>& data) {
        // Simple ZIP parser - looks for local file headers
        size_t pos = 0;
        const uint32_t LOCAL_FILE_HEADER_SIG = 0x04034b50;
        
        while (pos + 30 < data.size()) {
            uint32_t sig = *reinterpret_cast<const uint32_t*>(&data[pos]);
            if (sig != LOCAL_FILE_HEADER_SIG) {
                break;  // End of local file headers
            }
            
            // Parse local file header
            uint16_t compression = *reinterpret_cast<const uint16_t*>(&data[pos + 8]);
            uint32_t compressedSize = *reinterpret_cast<const uint32_t*>(&data[pos + 18]);
            uint32_t uncompressedSize = *reinterpret_cast<const uint32_t*>(&data[pos + 22]);
            uint16_t nameLen = *reinterpret_cast<const uint16_t*>(&data[pos + 26]);
            uint16_t extraLen = *reinterpret_cast<const uint16_t*>(&data[pos + 28]);
            
            std::string filename(&data[pos + 30], nameLen);
            size_t dataStart = pos + 30 + nameLen + extraLen;
            
            // Only support uncompressed files (compression == 0)
            if (compression == 0 && filename.size() > 4 && 
                filename.substr(filename.size() - 4) == ".npy") {
                
                std::string arrayName = filename.substr(0, filename.size() - 4);
                std::vector<char> npyData(data.begin() + dataStart, 
                                          data.begin() + dataStart + uncompressedSize);
                
                NpyArray arr;
                if (parseNpy(npyData, arr)) {
                    m_arrays[arrayName] = std::move(arr);
                }
            }
            
            pos = dataStart + compressedSize;
        }
        
        return !m_arrays.empty();
    }
    
    bool parseNpy(const std::vector<char>& data, NpyArray& arr) {
        // Check magic number
        if (data.size() < 10) return false;
        if (data[0] != '\x93' || data[1] != 'N' || data[2] != 'U' || 
            data[3] != 'M' || data[4] != 'P' || data[5] != 'Y') {
            return false;
        }
        
        uint8_t majorVersion = data[6];
        // uint8_t minorVersion = data[7];
        
        size_t headerLen;
        size_t headerStart;
        if (majorVersion == 1) {
            headerLen = *reinterpret_cast<const uint16_t*>(&data[8]);
            headerStart = 10;
        } else {
            headerLen = *reinterpret_cast<const uint32_t*>(&data[8]);
            headerStart = 12;
        }
        
        // Parse header (Python dict string)
        std::string header(&data[headerStart], headerLen);
        parseNpyHeader(header, arr);
        
        // Copy data
        size_t dataStart = headerStart + headerLen;
        arr.data.assign(data.begin() + dataStart, data.end());
        
        return true;
    }
    
    void parseNpyHeader(const std::string& header, NpyArray& arr) {
        // Simple parser for NumPy header dict
        // Format: {'descr': '<f4', 'fortran_order': False, 'shape': (100, 3), }
        
        // Extract descr
        size_t descrPos = header.find("'descr':");
        if (descrPos != std::string::npos) {
            size_t start = header.find("'", descrPos + 8);
            size_t end = header.find("'", start + 1);
            if (start != std::string::npos && end != std::string::npos) {
                arr.dtype = header.substr(start + 1, end - start - 1);
            }
        }
        
        // Extract fortran_order
        arr.fortran_order = header.find("True") != std::string::npos &&
                           header.find("'fortran_order': True") != std::string::npos;
        
        // Extract shape
        size_t shapePos = header.find("'shape':");
        if (shapePos != std::string::npos) {
            size_t start = header.find("(", shapePos);
            size_t end = header.find(")", start);
            if (start != std::string::npos && end != std::string::npos) {
                std::string shapeStr = header.substr(start + 1, end - start - 1);
                std::stringstream ss(shapeStr);
                std::string token;
                while (std::getline(ss, token, ',')) {
                    // Trim whitespace
                    token.erase(0, token.find_first_not_of(" \t"));
                    token.erase(token.find_last_not_of(" \t") + 1);
                    if (!token.empty()) {
                        arr.shape.push_back(std::stoul(token));
                    }
                }
            }
        }
    }
};

} // namespace mf
