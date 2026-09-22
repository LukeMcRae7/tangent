#include "ui/theme.h"

#include "core/palette.h"

#include "imgui.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace tg {
namespace {

inline ImVec4 im(Rgb c, float a = 1.0f) { return ImVec4(c.r, c.g, c.b, a); }

UiFonts g_fonts;

inline uint32_t beU32(const unsigned char* p) {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
}
inline uint16_t beU16(const unsigned char* p) {
    return uint16_t((uint32_t(p[0]) << 8) | p[1]);
}

// ImGui rasterises with stb_truetype, which handles TrueType ('glyf') and
// Type 2 CFF outlines but not the CFF2 / variable-font formats that ship as
// the default UI face on some distributions (Cantarell-VF being one). Handing
// it such a file makes AddFont fail and log an error, so screen the table
// directory first and just move on to the next candidate.
bool isRasterisable(const std::vector<unsigned char>& d, const char*& why) {
    if (d.size() < 12) { why = "too small"; return false; }

    const uint32_t tag = beU32(d.data());
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

// The bytes are owned here rather than by the atlas, and outlive it: ImGui's
// dynamic font system re-bakes glyphs on demand (new sizes, DPI changes), so
// the source must stay readable for the whole run.
std::vector<std::vector<unsigned char>>& fontStore() {
    static std::vector<std::vector<unsigned char>> store;
    return store;
}

ImFont* addFontFile(const std::string& path, float sizePx, bool quiet) {
    std::error_code ec;
    if (!std::filesystem::exists(path, ec)) return nullptr;

    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return nullptr;
    const std::streamsize size = f.tellg();
    if (size <= 0) return nullptr;
    f.seekg(0, std::ios::beg);

    std::vector<unsigned char> data(static_cast<size_t>(size));
    if (!f.read(reinterpret_cast<char*>(data.data()), size)) return nullptr;

    const char* why = "";
    if (!isRasterisable(data, why)) {
        if (!quiet) std::fprintf(stderr, "[ui] skipping %s (%s)\n", path.c_str(), why);
        return nullptr;
    }

    fontStore().push_back(std::move(data));
    std::vector<unsigned char>& kept = fontStore().back();

    ImFontConfig cfg;
    cfg.FontDataOwnedByAtlas = false;
    cfg.OversampleH = 2;
    cfg.OversampleV = 2;
    cfg.PixelSnapH  = false;

    ImFont* font = ImGui::GetIO().Fonts->AddFontFromMemoryTTF(
        kept.data(), static_cast<int>(kept.size()), sizePx, &cfg);
    if (!font) {
        fontStore().pop_back();
        if (!quiet) std::fprintf(stderr, "[ui] font rejected by ImGui: %s\n", path.c_str());
    }
    return font;
}

} // namespace

void applyDarkTheme() {
    using namespace palette;

    ImGuiStyle& s = ImGui::GetStyle();
    ImVec4* col = s.Colors;

    const ImVec4 bg        = im(kBackground);
    const ImVec4 panel     = im(kPanel);
    const ImVec4 field     = im(kField);
    const ImVec4 raised    = im(kRaised);
    const ImVec4 hover     = im(kHover);
    const ImVec4 active    = im(kActive);
    const ImVec4 border    = im(kBorder);
    const ImVec4 text      = im(kText);
    const ImVec4 textDim   = im(kTextDim);
    const ImVec4 accent    = im(kBrand);
    const ImVec4 accentDim = im(kBrand, 0.22f);

    col[ImGuiCol_Text]                  = text;
    col[ImGuiCol_TextDisabled]          = textDim;
    col[ImGuiCol_WindowBg]              = panel;
    col[ImGuiCol_ChildBg]               = ImVec4(0, 0, 0, 0);
    col[ImGuiCol_PopupBg]               = im(kPanel, 0.985f);
    col[ImGuiCol_Border]                = border;
    col[ImGuiCol_BorderShadow]          = ImVec4(0, 0, 0, 0);
    col[ImGuiCol_FrameBg]               = field;
    col[ImGuiCol_FrameBgHovered]        = hover;
    col[ImGuiCol_FrameBgActive]         = active;
    col[ImGuiCol_TitleBg]               = bg;
    col[ImGuiCol_TitleBgActive]         = panel;
    col[ImGuiCol_TitleBgCollapsed]      = bg;
    col[ImGuiCol_MenuBarBg]             = im(kTopBar);
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
    col[ImGuiCol_ModalWindowDimBg]      = ImVec4(0, 0, 0, 0.45f);

    // Quiet, rounded, and roomy enough to read: closer to a modern design
    // tool than to a debug overlay.
    s.WindowPadding     = ImVec2(12, 10);
    s.FramePadding      = ImVec2(9, 5);
    s.CellPadding       = ImVec2(6, 3);
    s.ItemSpacing       = ImVec2(8, 6);
    s.ItemInnerSpacing  = ImVec2(6, 4);
    s.IndentSpacing     = 16.0f;
    s.ScrollbarSize     = 10.0f;
    s.GrabMinSize       = 10.0f;

    s.WindowBorderSize  = 0.0f;
    s.ChildBorderSize   = 0.0f;
    s.PopupBorderSize   = 1.0f;
    s.FrameBorderSize   = 0.0f;
    s.TabBarBorderSize  = 0.0f;
    s.DockingSeparatorSize = 1.0f;

    s.WindowRounding    = 8.0f;
    s.ChildRounding     = 6.0f;
    s.FrameRounding     = 5.0f;
    s.PopupRounding     = 8.0f;
    s.ScrollbarRounding = 6.0f;
    s.GrabRounding      = 4.0f;
    s.TabRounding       = 4.0f;

    s.WindowTitleAlign  = ImVec2(0.0f, 0.5f);
    s.WindowMenuButtonPosition = ImGuiDir_None;
    s.SeparatorTextBorderSize  = 1.0f;
    s.SeparatorTextPadding     = ImVec2(0, 6);
    s.AntiAliasedLines  = true;
    s.AntiAliasedFill   = true;
}

void loadFonts(const std::string& assetDir, float sizePx) {
    ImGuiIO& io = ImGui::GetIO();
    g_fonts = UiFonts{};
    g_fonts.size = sizePx;

    // Escape hatch for unusual systems: TANGENT_FONT=default forces ImGui's
    // built-in face, any other value is a path used for every weight.
    const char* override_ = std::getenv("TANGENT_FONT");
    if (override_ && std::string(override_) == "default") {
        std::fprintf(stderr, "[ui] font: built-in (forced by TANGENT_FONT)\n");
        g_fonts.regular = io.Fonts->AddFontDefault();
        g_fonts.medium = g_fonts.semibold = g_fonts.bold = g_fonts.regular;
        return;
    }
    if (override_) {
        if (ImFont* f = addFontFile(override_, sizePx, false)) {
            std::fprintf(stderr, "[ui] font: %s\n", override_);
            g_fonts.regular = g_fonts.medium = g_fonts.semibold = g_fonts.bold = f;
            return;
        }
    }

    // The bundled face: the same on every platform, which is what makes the
    // interface look like one product rather than like whichever system font
    // happened to be installed.
    {
        const std::string dir = assetDir + "/fonts/";
        ImFont* regular  = addFontFile(dir + "SpaceGrotesk-Regular.ttf",  sizePx, false);
        ImFont* medium   = addFontFile(dir + "SpaceGrotesk-Medium.ttf",   sizePx, false);
        ImFont* semibold = addFontFile(dir + "SpaceGrotesk-SemiBold.ttf", sizePx, false);
        ImFont* bold     = addFontFile(dir + "SpaceGrotesk-Bold.ttf",     sizePx, false);
        if (regular) {
            std::fprintf(stderr, "[ui] font: Space Grotesk from %s\n", dir.c_str());
            g_fonts.regular  = regular;
            g_fonts.medium   = medium   ? medium   : regular;
            g_fonts.semibold = semibold ? semibold : g_fonts.medium;
            g_fonts.bold     = bold     ? bold     : g_fonts.semibold;
            return;
        }
    }

    // A system face, then. Two candidates per entry: the regular weight and a
    // bold one to stand in for the heavier weights.
    struct Pair { const char* regular; const char* bold; };
    static const Pair kSystem[] = {
        {"C:/Windows/Fonts/segoeui.ttf",                         "C:/Windows/Fonts/segoeuib.ttf"},
        {"C:/Windows/Fonts/arial.ttf",                           "C:/Windows/Fonts/arialbd.ttf"},
        {"/usr/share/fonts/TTF/InterDisplay-Regular.ttf",        "/usr/share/fonts/TTF/InterDisplay-SemiBold.ttf"},
        {"/usr/share/fonts/inter/Inter-Regular.ttf",             "/usr/share/fonts/inter/Inter-SemiBold.ttf"},
        {"/usr/share/fonts/noto/NotoSans-Regular.ttf",           "/usr/share/fonts/noto/NotoSans-Bold.ttf"},
        {"/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",  "/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf"},
        {"/usr/share/fonts/liberation/LiberationSans-Regular.ttf", "/usr/share/fonts/liberation/LiberationSans-Bold.ttf"},
        {"/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
         "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"},
        {"/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",      "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"},
        {"/usr/share/fonts/TTF/DejaVuSans.ttf",                  "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf"},
        {"/System/Library/Fonts/Supplemental/Arial.ttf",         "/System/Library/Fonts/Supplemental/Arial Bold.ttf"},
    };
    for (const Pair& p : kSystem) {
        ImFont* regular = addFontFile(p.regular, sizePx, true);
        if (!regular) continue;
        ImFont* bold = addFontFile(p.bold, sizePx, true);
        std::fprintf(stderr, "[ui] font: %s\n", p.regular);
        g_fonts.regular  = regular;
        g_fonts.medium   = regular;
        g_fonts.semibold = bold ? bold : regular;
        g_fonts.bold     = bold ? bold : regular;
        return;
    }

    std::fprintf(stderr, "[ui] no usable font found, using the built-in face\n");
    g_fonts.regular = io.Fonts->AddFontDefault();
    g_fonts.medium = g_fonts.semibold = g_fonts.bold = g_fonts.regular;
}

const UiFonts& uiFonts() { return g_fonts; }

void pushFont(FontWeight weight, float sizePx) {
    ImFont* f = nullptr;
    switch (weight) {
        case FontWeight::Regular:  f = g_fonts.regular;  break;
        case FontWeight::Medium:   f = g_fonts.medium;   break;
        case FontWeight::SemiBold: f = g_fonts.semibold; break;
        case FontWeight::Bold:     f = g_fonts.bold;     break;
    }
    // A null font keeps whatever is in force, which is the right thing to do
    // when the fonts have not been loaded at all (the headless tests).
    ImGui::PushFont(f, sizePx);
}

} // namespace tg
