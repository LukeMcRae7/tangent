// The line icons, and the two ways they can silently be wrong.
//
// Each glyph is a bare codepoint into the bundled Tabler font. One that is
// mistyped, or that a newer release of the font dropped, does not fail: it
// draws nothing, and an empty square in a toolbar is easy to miss. And the
// centring in drawGlyph assumes the font's em box is the icon's 24-unit box;
// a font with other metrics would shift every icon off its button.
#include "ui/glyph.h"

#include "imgui.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

int main(int argc, char** argv) {
    const std::string fonts = argc > 1 ? argv[1] : "assets/fonts";

    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(800, 600);
    // As the real backend does: glyphs are rasterised into the atlas when
    // first asked for, with no up-front build.
    io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
    io.Fonts->AddFontDefault();

    std::printf("--- the font loads ---\n");
    const bool loaded = loadGlyphFont(fonts);
    check(loaded, "tabler-icons.ttf loads from " + fonts);
    if (!loaded) {
        std::printf("\nFAILED (%d failures)\n", failures);
        return 1;
    }
    ImFont* font = io.Fonts->Fonts.back();

    std::printf("--- every glyph is in it ---\n");
    int present = 0;
    for (int i = 0; i < static_cast<int>(Glyph::Count); ++i) {
        const ImWchar c = glyphCodepoint(static_cast<Glyph>(i));
        char what[64];
        std::snprintf(what, sizeof what, "glyph %d (U+%04X) is in the font", i,
                      static_cast<unsigned>(c));
        const bool in = c != 0 && font->IsGlyphInFont(c);
        check(in, what);
        present += in;
    }
    std::printf("  %d of %d\n", present, static_cast<int>(Glyph::Count));

    std::printf("--- the em box is the icon's box ---\n");
    {
        // Tabler's square is stroked from 2 to 22 of its 24 units, so drawn a
        // whole em high its ink has to stand 2 units clear of each edge.
        // Large, so the rasteriser's padding is lost in the rounding. Not the
        // left edge: every glyph in the webfont carries a stray point at the
        // origin, left over from the invisible frame each Tabler SVG has.
        const float size = 240.0f, unit = size / 24.0f;
        ImFontBaked* baked = font->GetFontBaked(size);
        const ImFontGlyph* g = baked->FindGlyphNoFallback(glyphCodepoint(Glyph::Maximize));
        check(g != nullptr, "the square rasterises");
        if (g) {
            std::printf("  advance %.1f, right %.1f, top %.1f, bottom %.1f (want %.0f, 220, 20, 220)\n",
                        g->AdvanceX, g->X1, g->Y0, g->Y1, size);
            check(std::abs(g->AdvanceX - size) < 0.5f, "one em wide");
            check(std::abs(g->X1 - 22.0f * unit) < 2.0f, "right edge at 22 units");
            check(std::abs(g->Y0 - 2.0f * unit) < 2.0f, "top edge at 2 units");
            check(std::abs(g->Y1 - 22.0f * unit) < 2.0f, "bottom edge at 22 units");
        }
    }

    ImGui::DestroyContext();
    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
