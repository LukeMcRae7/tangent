// Tangent - the dialog an operation runs in.
//
// Every parametric modeller worth copying puts a running operation in the same
// shape of box: what it is acting on and what it is doing at the top, one
// labelled row per thing you can change, the choices among modes as a row of
// pictures, and the commit at the bottom. This one sits in the viewport's top
// left corner, under the status line, where Onshape and SolidWorks keep theirs:
// the corner is the one place a view framed on the model leaves empty, and the
// eye starts there anyway.
//
// It is a layer behind the viewport's pointer, not a wall in front of it. Only
// its controls take the pointer; over the rest of it the tool under way still
// follows, and once the pointer has settled there -- or the view is being
// orbited or dragged across it -- the panel fades back so the part and the
// preview show through, and a click there goes to the model.
//
// Rules the panel keeps, so that every operation looks like the same tool:
//   - labels in a fixed left column, controls filling the rest;
//   - a number is a bar you can drag as well as a value you can type. What
//     the keyboard holds is framed in the accent colour; what the pointer
//     still drives is plain;
//   - a choice among modes is a row of equal buttons, each with its picture,
//     its word and the key that picks it;
//   - guidance is one dim line at the foot, never mixed in with the numbers;
//   - Finish is bottom-right and in the brand colour, Cancel beside it, and
//     Esc always cancels.
#pragma once

#include "core/math.h"
#include "ui/glyph.h"

#include "imgui.h"

namespace tg::ui {

// Where the viewport is this frame, so the panel knows where its bottom edge
// is. Set once per frame by the application.
void setCommandAnchor(float x, float y, float w, float h);

// How far down the viewport the panel starts, below whatever the viewport
// already says in that corner. Pixels from the viewport's top.
void setCommandTopInset(float y);

// What the viewport is doing with the pointer this frame, so the panel knows
// when to step back. `gesture` is an orbit or a drag that began in the view;
// `tracking` is a tool that follows the pointer -- one for which the pointer
// resting on the panel means "I am still working on the model".
void setCommandRecede(bool gesture, bool tracking);

// Whether the viewport should hear the pointer although it is over the panel:
// it is over the panel but not over anything in it that can be used. Moving
// and the wheel pass then; a click passes only once the panel has faded back,
// so a click that misses a button by a hair does not land on the model.
bool commandPassesPointer();
bool commandPassesClicks();

// Where the panel was last drawn, in screen pixels; empty when none was. For
// the viewport's own notes, so they can keep out of its way.
ImVec4 commandPanelRect();

// Opens the panel. `context` is what the operation is acting on ("Box 01"),
// shown before the title as a breadcrumb; null leaves it out.
bool beginCommand(const char* id, const char* title, Glyph glyph, const char* context = nullptr);
void endCommand();

// Starts a row and leaves the cursor where its control goes. Everything placed
// until the next row shares the line.
void commandRow(const char* label);

// A row whose value is the tool's to say rather than the user's.
void commandValue(const char* label, const char* value);

// A number the operation is about, as a bar.
//
// `fixed` means the keyboard is holding it, `editing` that it is the one being
// typed into; `buffer` is what has been typed so far. `lo` and `hi` give the
// bar its extent; equal means unknown and the bar shows no fill. A signed
// range fills from the middle.
//
// `clicked` is a plain click on the bar, which is how a field is handed back
// to the pointer. `dragged` means the bar was pulled this frame and `value`
// is what it was pulled to; `dragging` stays set for the whole pull, and
// `released` marks the frame it let go -- for an operation too slow to rebuild
// on every frame of a drag.
//
// The range is held fixed from the moment a pull starts until it ends, so a
// caller whose range is derived from the value itself does not chase its own
// tail while the bar is being pulled.
struct NumberEdit {
    bool   clicked = false;
    bool   dragged = false;
    bool   dragging = false;
    bool   released = false;
    double value = 0.0;
};
NumberEdit commandNumber(const char* label, double value, const char* unit,
                         bool fixed, bool editing, const char* buffer,
                         double lo = 0.0, double hi = 0.0, bool signedRange = false);

// A choice among a few ways of doing the operation: Cut, Join, Intersect, New.
// Returns the index clicked, or -1. `compact` draws a single row of small
// buttons with words only, for choices that do not deserve the big pictures.
struct Choice {
    Glyph       glyph;
    const char* label;
    const char* key;      // "D", or null
    const char* tip;
    // A choice that would do nothing here is shown, dimmed, and cannot be
    // picked: which of them those are is part of what the panel says, and
    // leaving one out instead would move the others about as things change.
    bool        enabled = true;
};
int commandChoices(const char* label, const Choice* choices, int count, int active,
                   bool compact = false);

// Before each pill after the first in a row of them: beside the last, or on
// the next line under the first when it would run off the panel. The panel is
// narrow, and a row of screw sizes is not.
void commandNextPill(const char* label, float gap = 4.0f);

// What the operation would say if asked: what it does, what the keys are, what
// it is for. It is not drawn in the panel -- it goes behind the ? in the
// panel's top right corner and comes out on hover. Several calls stack up into
// one tooltip, in the order they were made.
void commandHint(const char* text);

// Says the operation has already been applied and the panel is now adjusting
// it rather than building it. Two words; the rest goes behind the ?.
void commandApplied(const char* what);

// The other answer: it was asked for and the kernel would not. The reason is
// said in the panel that asked -- it is the one thing the user has to read --
// and what to do about it goes behind the ?.
void commandRefused(const char* why);

// The commit and the cancel. Returns 1 for commit, -1 for cancel, 0 for
// neither. A null `cancelLabel` leaves the cancel out, for a panel whose
// operation has already been applied; a null `commitLabel` leaves that out.
int commandFooter(const char* commitLabel, bool commitEnabled = true,
                  const char* cancelLabel = "Cancel");

// Width of the label column, so callers can line other things up with it.
float commandLabelWidth();

} // namespace tg::ui
