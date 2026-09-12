// Tangent - the operation icons.
//
// Every one of these is a picture of the operation it names, rendered from the
// kernel's own output by tools/bake_icons: the fillet icon is a filleted cube,
// the shell icon is a shelled one. Nothing was drawn by hand, so nothing can
// drift away from what the operation actually does -- and a part that changes
// shape changes its icon with it.
//
// They go in one texture and are used sparingly: on the operations in the
// toolbar and the menus, and on the rows of the history, where a column of them
// is what makes a chain of ten features readable at a glance. Not on ordinary
// controls, where a word is clearer than a picture of one.
#pragma once

#include "imgui.h"

#include <string>
#include <vector>

namespace tg {

enum class Icon {
    Box, Cylinder, Sphere, Cone, Torus,
    Union, Difference, Intersection,
    Extrude, Fillet, Chamfer, Shell, Inset,
    Count
};

// Reads one baked icon into 8-bit RGBA.
//
// Exposed because it is the only part of this file that can be wrong in a way
// a test can catch: it understands exactly what tools/icon_raster.cpp writes
// and nothing else, and the two have to agree.
bool readIconImage(const std::string& path, int& w, int& h,
                   std::vector<unsigned char>& rgba);

// Reads the baked images and uploads them as one texture. Safe to call when
// there are none: everything below then draws nothing and the interface falls
// back to its words, which is what it had before there were icons at all.
bool loadIcons(const std::string& assetDir);
void unloadIcons();
bool iconsReady();

// The icon on its own, at `sizePx` square. Draws a placeholder gap if the
// icons could not be loaded, so layout does not move.
void iconImage(Icon icon, float sizePx, float alpha = 1.0f);

// A square button. `active` draws it held down, for a choice among several.
bool iconButton(Icon icon, const char* id, float sizePx, const char* tooltip,
                bool active = false, bool enabled = true);

} // namespace tg
