#include "ui/theme.h"

#include "core/palette.h"

#include "imgui.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace tg {
namespace {

inline ImVec4 im(Rgb c, float a = 1.0f) { return ImVec4(c.r, c.g, c.b, a); }

bool exists(const char* path) {
    struct stat st{};
    return stat(path, &st) == 0;
}

inline uint32_t beU32(const unsigned char* p) {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
}
inline uint16_t beU16(const unsigned char* p) {
    return uint16_t((uint32_t(p[0]) << 8) | p[1]);
}

// ImGui rasterises with stb_truetype, which handles TrueType ('glyf') and
// Type 2 CFF outlines but not the CFF2 / variable-font formats that ship as
// the default UI face on some distributions (Cantarell-VF being the one on
// this machine). Handing it such a file makes AddFont fail and log an error,
// so screen the table directory first and just move on to the next candidate.
bool isRasterisable(const std::vector<unsigned char>& d, const char*& why) {
    if (d.size() < 12) { why = "too small"; return false; }

    const uint32_t tag = beU32(d.data());
    // 'ttcf' collections are not handled here; the rest must be sfnt.
    if (tag != 0x00010000u && tag != 0x4F54544Fu /*OTTO*/ && tag != 0x74727565u /*true*/) {
        why = "not an sfnt font";
        return false;
    }

    const size_t numTables = beU16(d.data() + 4);
    if (12 + numTables * 16 > d.size()) { why = "truncated table directory"; return false; }

    bool hasGlyf = false, hasCff = false, hasCff2 = false, hasFvar = false;
    for (size_t i = 0; i < numTables; ++i) {
        const uint32_t t = beU32(d.data() + 12 + i * 16);
        if      (t == 0x676C7966u) hasGlyf = true;   // 'glyf'
        else if (t == 0x43464620u) hasCff  = true;   // 'CFF '
        else if (t == 0x43464632u) hasCff2 = true;   // 'CFF2'
        else if (t == 0x66766172u) hasFvar = true;   // 'fvar'
    }

    if (hasCff2) { why = "CFF2 outlines are unsupported"; return false; }
    if (hasFvar) { why = "variable font"; return false; }
    if (!hasGlyf && !hasCff) { why = "no glyf or CFF outlines"; return false; }
    return true;
}

} // namespace

void applyDarkTheme() {
    using namespace palette;

    ImGuiStyle& s = ImGui::GetStyle();
    ImVec4* col = s.Colors;

    const ImVec4 bg        = im(kBackground);
    const ImVec4 panel     = im(kPanel);
    const ImVec4 raised    = im(kRaised);
    const ImVec4 hover     = im(kHover);
    const ImVec4 active    = im(kActive);
    const ImVec4 border    = im(kBorder);
    const ImVec4 text      = im(kText);
    const ImVec4 textDim   = im(kTextDim);
    const ImVec4 accent    = im(kBrand);
    const ImVec4 accentDim = im(kBrand, 0.28f);

    col[ImGuiCol_Text]                  = text;
    col[ImGuiCol_TextDisabled]          = textDim;
    col[ImGuiCol_WindowBg]              = bg;
    col[ImGuiCol_ChildBg]               = ImVec4(0, 0, 0, 0);
    col[ImGuiCol_PopupBg]               = im(kPanel, 0.98f);
    col[ImGuiCol_Border]                = border;
    col[ImGuiCol_BorderShadow]          = ImVec4(0, 0, 0, 0);
    col[ImGuiCol_FrameBg]               = panel;
    col[ImGuiCol_FrameBgHovered]        = hover;
    col[ImGuiCol_FrameBgActive]         = active;
    col[ImGuiCol_TitleBg]               = bg;
    col[ImGuiCol_TitleBgActive]         = panel;
    col[ImGuiCol_TitleBgCollapsed]      = bg;
    col[ImGuiCol_MenuBarBg]             = im(kMenuBar);
    col[ImGuiCol_ScrollbarBg]           = ImVec4(0, 0, 0, 0);
    col[ImGuiCol_ScrollbarGrab]         = raised;
    col[ImGuiCol_ScrollbarGrabHovered]  = hover;
    col[ImGuiCol_ScrollbarGrabActive]   = active;
    col[ImGuiCol_CheckMark]             = accent;
    col[ImGuiCol_SliderGrab]            = im(mix(kTextDim, kText, 0.4f));
    col[ImGuiCol_SliderGrabActive]      = accent;
    col[ImGuiCol_Button]                = raised;
    col[ImGuiCol_ButtonHovered]         = hover;
    col[ImGuiCol_ButtonActive]          = active;
    col[ImGuiCol_Header]                = accentDim;
    col[ImGuiCol_HeaderHovered]         = hover;
    col[ImGuiCol_HeaderActive]          = accentDim;
    col[ImGuiCol_Separator]             = border;
    col[ImGuiCol_SeparatorHovered]      = accentDim;
    col[ImGuiCol_SeparatorActive]       = accent;
    col[ImGuiCol_ResizeGrip]            = ImVec4(0, 0, 0, 0);
    col[ImGuiCol_ResizeGripHovered]     = accentDim;
    col[ImGuiCol_ResizeGripActive]      = accent;
    col[ImGuiCol_Tab]                   = bg;
    col[ImGuiCol_TabHovered]            = hover;
    col[ImGuiCol_TabSelected]           = panel;
    col[ImGuiCol_TabDimmed]             = bg;
    col[ImGuiCol_TabDimmedSelected]     = panel;
    col[ImGuiCol_TabSelectedOverline]   = accent;
    col[ImGuiCol_DockingPreview]        = accentDim;
    col[ImGuiCol_DockingEmptyBg]        = bg;
    col[ImGuiCol_PlotLines]             = textDim;
    col[ImGuiCol_PlotLinesHovered]      = accent;
    col[ImGuiCol_TableHeaderBg]         = panel;
    col[ImGuiCol_TableBorderStrong]     = border;
    col[ImGuiCol_TableBorderLight]      = im(mix(kBackground, kBorder, 0.5f));
    col[ImGuiCol_TextSelectedBg]        = accentDim;
    col[ImGuiCol_NavCursor]             = accent;

    // Tight, squared-off geometry: closer to a CAD tool than to a web app.
    s.WindowPadding     = ImVec2(10, 10);
    s.FramePadding      = ImVec2(8, 4);
    s.CellPadding       = ImVec2(6, 3);
    s.ItemSpacing       = ImVec2(8, 6);
    s.ItemInnerSpacing  = ImVec2(6, 4);
    s.IndentSpacing     = 18.0f;
    s.ScrollbarSize     = 11.0f;
    s.GrabMinSize       = 9.0f;

    s.WindowBorderSize  = 1.0f;
    s.ChildBorderSize   = 1.0f;
    s.PopupBorderSize   = 1.0f;
    s.FrameBorderSize   = 0.0f;
    s.TabBarBorderSize  = 1.0f;

    s.WindowRounding    = 4.0f;
    s.ChildRounding     = 4.0f;
    s.FrameRounding     = 3.0f;
    s.PopupRounding     = 4.0f;
    s.ScrollbarRounding = 6.0f;
    s.GrabRounding      = 3.0f;
    s.TabRounding       = 4.0f;

    s.WindowTitleAlign  = ImVec2(0.0f, 0.5f);
    s.WindowMenuButtonPosition = ImGuiDir_None;
    s.SeparatorTextBorderSize  = 1.0f;
    s.SeparatorTextPadding     = ImVec2(0, 6);
    s.AntiAliasedLines  = true;
    s.AntiAliasedFill   = true;
}

void loadFonts(float sizePx) {
    ImGuiIO& io = ImGui::GetIO();

    // Proportional faces first (better for labels), monospace last.
    static const char* kCandidates[] = {
        "/usr/share/fonts/TTF/InterDisplay-Regular.ttf",
        "/usr/share/fonts/inter/Inter-Regular.ttf",
        "/usr/share/fonts/noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/cantarell/Cantarell-VF.otf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/JetBrainsMonoNerdFontPropo-Regular.ttf",
        "/usr/share/fonts/TTF/JetBrainsMonoNLNerdFont-Regular.ttf",
    };

    // Escape hatch for unusual systems: TANGENT_FONT=default forces ImGui's
    // built-in face, any other value is a path tried ahead of the list.
    const char* override_ = std::getenv("TANGENT_FONT");
    if (override_ && std::string(override_) == "default") {
        std::fprintf(stderr, "[ui] font: built-in (forced by TANGENT_FONT)\n");
        io.Fonts->AddFontDefault();
        return;
    }

    std::vector<const char*> candidates;
    if (override_) candidates.push_back(override_);
    for (const char* p : kCandidates) candidates.push_back(p);

    for (const char* path : candidates) {
        if (!exists(path)) continue;

        std::ifstream f(path, std::ios::binary | std::ios::ate);
        if (!f) continue;
        const std::streamsize size = f.tellg();
        if (size <= 0) continue;
        f.seekg(0, std::ios::beg);

        // The buffer is owned here rather than by the atlas, and outlives it:
        // ImGui's dynamic font system re-bakes glyphs on demand (new sizes, DPI
        // changes), so the source bytes must stay readable for the whole run.
        static std::vector<unsigned char> fontData;
        fontData.assign(static_cast<size_t>(size), 0);
        if (!f.read(reinterpret_cast<char*>(fontData.data()), size)) continue;

        const char* why = "";
        if (!isRasterisable(fontData, why)) {
            std::fprintf(stderr, "[ui] skipping %s (%s)\n", path, why);
            continue;
        }

        ImFontConfig cfg;
        cfg.FontDataOwnedByAtlas = false;
        cfg.OversampleH = 2;
        cfg.OversampleV = 1;
        cfg.PixelSnapH  = true;

        if (io.Fonts->AddFontFromMemoryTTF(fontData.data(),
                                           static_cast<int>(fontData.size()),
                                           sizePx, &cfg)) {
            std::fprintf(stderr, "[ui] font: %s\n", path);
            return;
        }
        std::fprintf(stderr, "[ui] font rejected by ImGui: %s\n", path);
    }

    std::fprintf(stderr, "[ui] no usable system font found, using the built-in face\n");
    io.Fonts->AddFontDefault();
}

} // namespace tg
