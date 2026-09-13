// Tangent - the dialog an operation runs in.
//
// Every parametric modeller worth copying puts a running operation in the same
// shape of box: its name at the top, one labelled row per thing you can change,
// the choices among modes as a row of pictures, and the commit at the bottom.
// Fusion docks it beside the viewport, Plasticity floats it small and close --
// both agree on the structure, and the structure is the part that matters.
//
// What this replaces was a banner across the top of the model that grew a new
// shape at every step: the values appeared twice, once as running text and
// again in fields whose labels ImGui draws on the *right*, so a row read
// "25.00 mm Width 18.40 mm Depth" and you had to work out which number went
// with which word.
//
// Rules the panel keeps, so that every operation looks like the same tool:
//   - labels in a fixed left column, controls filling the rest;
//   - a value the keyboard holds is framed and in the accent colour, one the
//     mouse still drives is plain -- the same language the profile uses;
//   - guidance is one dim line at the bottom, never mixed in with the numbers;
//   - the commit is bottom-right, and Esc always cancels.
#pragma once

#include "ui/icons.h"

#include "imgui.h"

namespace tg::ui {

// Opens the dialog at (x, y), which should be a corner of the viewport rather
// than its middle: a panel over the model is a panel in the way of the thing
// being made.
// `Icon::Count` means the operation has no picture of its own -- better than
// borrowing one that shows something else.
bool beginCommand(const char* id, const char* title, Icon icon, float x, float y);
void endCommand();

// Starts a row and leaves the cursor where its control goes. Everything placed
// until the next row shares the line.
void commandRow(const char* label);

// A row whose value is the tool's to say rather than the user's.
void commandValue(const char* label, const char* value);

// A number the operation is about.
//
// `fixed` means the keyboard is holding it, `editing` that it is the one being
// typed into; `buffer` is what has been typed so far. Returns true when the row
// is clicked, which is how a field is chosen with the mouse rather than by
// tabbing to it.
bool commandNumber(const char* label, double value, const char* unit,
                   bool fixed, bool editing, const char* buffer);

// One dim line of guidance, at the foot of the rows.
void commandHint(const char* text);

// Says the operation has already been applied and the panel is now adjusting
// it rather than building it. In the accent colour, because it is a change of
// what the panel means and not another piece of guidance.
void commandApplied(const char* what);

// The commit and the cancel. Returns 1 for commit, -1 for cancel, 0 for
// neither. The commit is drawn in the brand colour, since it is the one thing
// on the panel that finishes the operation. A null `cancelLabel` leaves the
// cancel out, for a panel whose operation has already been applied.
int commandFooter(const char* commitLabel, bool commitEnabled = true,
                  const char* cancelLabel = "Cancel  (Esc)");

// Width of the label column, so callers can line other things up with it.
float commandLabelWidth();

} // namespace tg::ui
