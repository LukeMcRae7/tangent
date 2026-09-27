// Tangent - every command, findable by name.
//
// Ctrl+K and type. The list is every command the menus hold, with its key --
// so it is also the reference sheet for the keys, which were otherwise only
// learnable by reading the menus one at a time. A command that cannot run on
// what is selected is listed, dimmed, with the reason, rather than left out:
// "why can I not find Fillet" has a worse answer than "select an edge".
#pragma once

#include "ui/panels.h"

namespace tg::ui {

// `query`: what is typed into it already, for a demo.
void openCommandPalette(const char* query = "");
bool commandPaletteOpen();
// Draws it when open, and turns the chosen command into the action it stands
// for in `ctx.actions`.
void drawCommandPalette(UiContext& ctx);


} // namespace tg::ui
