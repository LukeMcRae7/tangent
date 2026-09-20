// Tangent - the pieces every panel is built from.
//
// One place for the things that have to look the same wherever they appear:
// a section title, a row of X / Y / Z fields, a primary button, an eye that
// hides a thing, a menu entry with its picture and its key. A panel that
// draws its own version of any of these is a panel that drifts.
#pragma once

#include "core/palette.h"
#include "ui/glyph.h"

#include "imgui.h"

namespace tg::ui {

inline ImVec4 im(Rgb c, float a = 1.0f) { return ImVec4(c.r, c.g, c.b, a); }
inline ImU32  u32(Rgb c, float a = 1.0f) { return ImGui::GetColorU32(im(c, a)); }

// ---- text ---------------------------------------------------------------
// "Transform": the heading over a group of rows.
void sectionTitle(const char* text);
// A small, spaced, upper-case label -- a caption under a toolbar group, a unit.
void captionText(const char* text, ImU32 colour = 0);
// A rounded tag: "[current]", "off", "failed".
void tag(const char* text, Rgb colour, bool filled = false);

// ---- buttons ------------------------------------------------------------
bool primaryButton(const char* label, ImVec2 size = ImVec2(0, 0), bool enabled = true);
bool quietButton(const char* label, ImVec2 size = ImVec2(0, 0), bool enabled = true);
// A small choice among a few words; `on` draws it as the chosen one.
bool pillButton(const char* label, bool on, ImVec2 size = ImVec2(0, 0), bool enabled = true);
// An eye that opens and closes. Returns true when it changed.
bool eyeToggle(const char* id, bool& visible, float sizePx = 15.0f);
// A small round close, for a row that can be removed.
bool closeButton(const char* id, float sizePx = 13.0f);

// ---- fields -------------------------------------------------------------
// "Position                      [mm]": the line above a row of fields.
void fieldHeader(const char* label, const char* unit);
// Three fields, each with its axis letter in the axis's own colour.
bool axisFields(const char* id, Vec3& v, float speed, const char* fmt, bool readOnly = false);
// One labelled number, the field stretched to the right edge.
bool labelledNumber(const char* label, Real& v, float speed, Real lo, Real hi,
                    const char* fmt = "%.2f mm");
bool labelledInt(const char* label, int& v, int lo, int hi);
// The label column every labelled row shares.
float labelColumn();

// ---- menus --------------------------------------------------------------
bool beginMenuPopup(const char* id, ImGuiWindowFlags extra = 0);
void menuHeader(const char* text);
bool menuEntry(Glyph g, const char* label, const char* shortcut = nullptr,
               bool enabled = true, bool selected = false);
bool menuToggle(Glyph g, const char* label, bool* value, const char* shortcut = nullptr);
// What the entry just submitted is for, shown on hover rather than under it.
// Called straight after the menuEntry or menuToggle it belongs to.
void menuNote(const char* text);

// A line of text in a menu that is data rather than explanation -- the frame
// rate, a count -- which stays on the page.
void menuStat(const char* text);
void menuGap();

// ---- dialogs ------------------------------------------------------------
// A centred card with its own title line, for the few things that block:
// a file's options, an unsaved-work question. Pair with endCard.
bool beginCard(const char* id, const char* title, float width);
void endCard();

// A tooltip in the house style, for the item just laid out.
void hoverTip(const char* text);

} // namespace tg::ui
