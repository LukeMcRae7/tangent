// Tangent - UI styling.
#pragma once

namespace tg {

// All colours come from tg::palette (core/palette.h); this only translates
// them into ImGui's style table.
void applyDarkTheme();

// Loads a UI font, preferring the crisp sans faces typically present on Arch
// and falling back to ImGui's built-in bitmap font if none are found.
void loadFonts(float sizePx = 15.0f);

} // namespace tg
