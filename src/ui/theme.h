// Tangent - UI styling.
#pragma once

#include <string>

struct ImFont;

namespace tg {

// All colours come from tg::palette (core/palette.h); this only translates
// them into ImGui's style table.
void applyDarkTheme();

// The interface face at its four weights. One ImFont per weight; sizes are
// chosen at the point of use with pushFont, since ImGui rasterises on demand.
struct UiFonts {
    ImFont* regular  = nullptr;
    ImFont* medium   = nullptr;
    ImFont* semibold = nullptr;
    ImFont* bold     = nullptr;
    float   size     = 14.0f;      // the base size everything is measured from
};

enum class FontWeight { Regular, Medium, SemiBold, Bold };

// Loads the bundled face from `assetDir`/fonts. Falls back to a system face
// on either platform, and to ImGui's built-in bitmap font after that, so the
// interface always comes up with something to say.
void loadFonts(const std::string& assetDir, float sizePx = 14.0f);
const UiFonts& uiFonts();

// Pushes a weight at a size; 0 keeps the size in force. Pair with ImGui::PopFont.
void pushFont(FontWeight weight, float sizePx = 0.0f);

} // namespace tg
