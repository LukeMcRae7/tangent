#include "render/shader.h"

#include <epoxy/gl.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <vector>

namespace tg {
namespace {

int64_t fileStamp(const std::string& path) {
    struct stat st{};
    return stat(path.c_str(), &st) == 0 ? static_cast<int64_t>(st.st_mtime) : 0;
}

bool readFile(const std::string& path, std::string& out) {
    std::ifstream f(path);
    if (!f) { std::fprintf(stderr, "[shader] cannot open %s\n", path.c_str()); return false; }
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

uint32_t compileStage(GLenum type, const std::string& src, const std::string& label) {
    const uint32_t id = glCreateShader(type);
    const char* c = src.c_str();
    glShaderSource(id, 1, &c, nullptr);
    glCompileShader(id);

    GLint ok = 0;
    glGetShaderiv(id, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetShaderiv(id, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> log(static_cast<size_t>(len > 1 ? len : 1));
        glGetShaderInfoLog(id, len, nullptr, log.data());
        std::fprintf(stderr, "[shader] %s failed to compile:\n%s\n", label.c_str(), log.data());
        glDeleteShader(id);
        return 0;
    }
    return id;
}

} // namespace

Shader::~Shader() {
    // Deliberately does not call destroy(): by the time members are destroyed
    // the GL context may already be gone, and deleting a program with no
    // current context is undefined (epoxy aborts outright). Renderer::shutdown
    // is what actually frees this.
    program_ = 0;
}

void Shader::destroy() {
    if (program_) glDeleteProgram(program_);
    program_ = 0;
    uniforms_.clear();
}

bool Shader::load(const std::string& vertPath, const std::string& fragPath) {
    vertPath_ = vertPath;
    fragPath_ = fragPath;
    vertStamp_ = fileStamp(vertPath);
    fragStamp_ = fileStamp(fragPath);

    std::string vsrc, fsrc;
    if (!readFile(vertPath, vsrc) || !readFile(fragPath, fsrc)) return false;

    const uint32_t vs = compileStage(GL_VERTEX_SHADER, vsrc, vertPath);
    if (!vs) return false;
    const uint32_t fs = compileStage(GL_FRAGMENT_SHADER, fsrc, fragPath);
    if (!fs) { glDeleteShader(vs); return false; }

    const uint32_t prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> log(static_cast<size_t>(len > 1 ? len : 1));
        glGetProgramInfoLog(prog, len, nullptr, log.data());
        std::fprintf(stderr, "[shader] link failed (%s + %s):\n%s\n",
                     vertPath.c_str(), fragPath.c_str(), log.data());
        glDeleteProgram(prog);
        return false;
    }

    if (program_) glDeleteProgram(program_);
    program_ = prog;
    uniforms_.clear();
    return true;
}

bool Shader::reloadIfChanged() {
    const int64_t v = fileStamp(vertPath_), f = fileStamp(fragPath_);
    if (v == vertStamp_ && f == fragStamp_) return false;
    vertStamp_ = v;
    fragStamp_ = f;
    const bool ok = load(vertPath_, fragPath_);
    std::fprintf(stderr, "[shader] reloaded %s: %s\n", fragPath_.c_str(), ok ? "ok" : "FAILED");
    return ok;
}

void Shader::bind() const {
    glUseProgram(program_);
}

int Shader::location(const char* name) {
    auto it = uniforms_.find(name);
    if (it != uniforms_.end()) return it->second;
    const int loc = glGetUniformLocation(program_, name);
    uniforms_.emplace(name, loc);
    return loc;
}

void Shader::set(const char* n, int v)          { glUniform1i(location(n), v); }
void Shader::set(const char* n, float v)        { glUniform1f(location(n), v); }
void Shader::set(const char* n, Vec3 v)         { glUniform3f(location(n), v.x, v.y, v.z); }
void Shader::set(const char* n, Vec4 v)         { glUniform4f(location(n), v.x, v.y, v.z, v.w); }
void Shader::set(const char* n, const Mat4& m)  { glUniformMatrix4fv(location(n), 1, GL_FALSE, m.data()); }

} // namespace tg
