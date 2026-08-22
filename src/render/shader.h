// Tangent - GLSL program wrapper with hot reload.
#pragma once

#include "core/math.h"

#include <cstdint>
#include <string>
#include <unordered_map>

namespace tg {

class Shader {
public:
    ~Shader();
    Shader() = default;
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    // Loads and links vertex + fragment stages from disk. On failure the
    // previously linked program is kept, so a typo during hot reload degrades
    // to "nothing changed" instead of a black viewport.
    bool load(const std::string& vertPath, const std::string& fragPath);
    bool reloadIfChanged();

    // Releases the GL program. Must be called while the context is still
    // current: the destructor runs during member teardown, which for this app
    // happens after the context is gone.
    void destroy();

    void bind() const;
    bool valid() const { return program_ != 0; }

    void set(const char* name, int value);
    void set(const char* name, Real value);
    void set(const char* name, Vec3 v);
    void set(const char* name, Vec4 v);
    void set(const char* name, const Mat4& m);

private:
    int location(const char* name);

    uint32_t    program_ = 0;
    std::string vertPath_, fragPath_;
    int64_t     vertStamp_ = 0, fragStamp_ = 0;
    std::unordered_map<std::string, int> uniforms_;
};

} // namespace tg
