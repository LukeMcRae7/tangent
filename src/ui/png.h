// Tangent - reading an ordinary PNG.
//
// The baked operation icons are read by ui/icons.cpp, which understands only
// the stored-block files the baker writes. The logo and the application icon
// are ordinary compressed PNGs from a drawing tool, so they need a real
// inflate -- which zlib, already linked for the 3MF writer, provides.
//
// Eight-bit greyscale, greyscale+alpha, RGB and RGBA, non-interlaced, every
// filter type. Everything comes out as RGBA.
#pragma once

#include <string>
#include <vector>

namespace tg {

bool readPng(const std::string& path, int& width, int& height,
             std::vector<unsigned char>& rgba);

} // namespace tg
