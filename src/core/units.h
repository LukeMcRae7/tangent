// Tangent - the units lengths are shown and typed in.
//
// Everything inside is millimetres: the geometry, the files, the history. A
// unit is only how a number is shown and how a typed one is read, so it lives
// here, at the edge, and nowhere else has to know about it. Change the unit
// and every panel, readout and field follows, because they all come through
// these few functions instead of writing "mm" themselves.
#pragma once

#include "core/math.h"

#include <string>

namespace tg::units {

enum class Length { Millimetre, Centimetre, Inch };

Length current();
void setCurrent(Length u);

// "mm", "cm", "in".
const char* suffix();
const char* suffixOf(Length u);
// "Millimetres", ...
const char* nameOf(Length u);

// Millimetres to what is shown, and back.
Real toShown(Real mm);
Real fromShown(Real shown);

// How many decimals a length is shown with: a hundredth of a millimetre, a
// thousandth of a centimetre or an inch -- about the same size of step.
int decimals();

// "12.00 mm", "1.200 cm", "0.472 in". `places` < 0 uses decimals().
std::string length(Real mm, int places = -1);
// The number alone, without the unit.
std::string number(Real mm, int places = -1);
// An area in the unit squared ("mm²"), and a volume as material is reckoned:
// cubic centimetres for metric, cubic inches for inches.
std::string area(Real mm2);
std::string volume(Real mm3);

// A typed length: a number, in the unit shown, or with a unit of its own --
// "12", "12mm", "1.5 cm", "0.5in", "0.5\"". False when it is not a number.
bool parse(const std::string& text, Real& mm);

} // namespace tg::units
