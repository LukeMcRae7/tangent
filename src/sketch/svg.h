// Tangent - an SVG drawing, read into a sketch.
//
// A logo, a gasket outline or a panel cutout is usually already drawn, in
// something that saves SVG. Reading it in means it does not have to be drawn a
// second time by hand -- and because an SVG path is made of lines, circular and
// elliptical arcs and Bezier curves, and a sketch is made of lines, arcs and
// cubic Beziers, nothing is approximated that did not have to be:
//
//   a line is a line; a quadratic curve is raised to the cubic it exactly is;
//   a circle, and an arc of one, stay a circle and an arc, so they can be
//   dimensioned by radius; only an ellipse, which a sketch has no curve for,
//   becomes the cubics every drawing program approximates one with.
//
// Sizes are real. An SVG says how big it is -- width="100mm", or a viewBox in
// pixels at 96 to the inch -- and the drawing arrives at that size, in
// millimetres, the right way up: SVG's y runs down the page, a sketch's up.
//
// No dependency: SVG is XML, and the part of XML a drawing uses is small enough
// to read here rather than link a parser for. Text, images, gradients and the
// rest of what is not geometry are counted and left out, so the import can say
// what it did not bring rather than bringing it wrongly.
#pragma once

#include "core/math.h"
#include "sketch/sketch.h"

#include <memory>
#include <string>
#include <vector>

namespace tg {

// One piece of a path, in millimetres with y up.
struct SvgSegment {
    enum class Kind { Line, Cubic, Arc };
    Kind kind = Kind::Line;

    // Line: from -> to. Cubic: from, c1, c2, to. Arc: a circular arc from ->
    // to about `centre`, counter-clockwise when `ccw` is set.
    Vec2 from, c1, c2, to;
    Vec2 centre;
    Real radius = 0;
    bool ccw = true;
};

// How an element is painted, as far as its shape goes: whether its inside is
// filled at all, and by which rule -- even-odd, or nonzero, SVG's default.
struct SvgFill {
    bool filled = true;
    bool evenOdd = false;
    int element = -1;   // which element drew it, and when: later is on top
    int colour = -1;    // into SvgDrawing::colours
};

// A colour something in the drawing is filled with, and whether it is ink --
// what becomes the part -- or paper, which what is painted in it wipes out.
struct SvgColour {
    std::string css;        // as normalised: "#1b1c1e"
    Vec3 rgb{0, 0, 0};
    int shapes = 0;         // how many things are filled with it
    bool ink = true;
};

// One subpath: segments end to end, and whether it comes back to its start.
struct SvgPath {
    std::vector<SvgSegment> segments;
    bool closed = false;
    SvgFill fill;
};

struct SvgCircle {
    Vec2 centre;
    Real radius = 0;
    SvgFill fill;
};

struct SvgDrawing {
    std::vector<SvgPath> paths;
    std::vector<SvgCircle> circles;

    // Every fill colour in the drawing, and which are ink. By default all but
    // the near-white: white painted over black is how drawings cut a highlight
    // or a counter, and on the page it is paper.
    std::vector<SvgColour> colours;

    // The drawing as read, before its outlines were made to bound the ink, so
    // they can be made again when which colours are ink changes.
    std::shared_ptr<const SvgDrawing> source;

    // Around everything that was read, in millimetres.
    Vec2 min{0, 0}, max{0, 0};

    // What was left out, so it can be said.
    int text = 0;      // <text>: convert it to paths first
    int images = 0;    // <image>: a picture has no outline to read
    int unreadable = 0;// a path whose data stopped making sense part way

    // What was put right on the way in, so it can be said.
    int background = 0;  // a rectangle behind the whole page, left out
    int crossings = 0;   // places outlines crossed, themselves or each other
    int covered = 0;     // outlines inside what was already filled, which bound nothing
    int specks = 0;      // loops too small to be anything, dropped
    int unresolved = 0;  // outlines whose crossings could not be worked out, kept as drawn

    bool ok = false;
    std::string error;

    bool empty() const { return paths.empty() && circles.empty(); }
    Vec2 size() const { return max - min; }

    // How many sketch entities it will make.
    size_t entityCount() const;
};

// Reads SVG text. `ok` is false, with `error` said for a person, when it is not
// an SVG at all or nothing in it is geometry.
//
// Two things downloaded drawings nearly all do, and a solid cannot, are put
// right as they come in:
//
//   A rectangle filling the page behind everything -- the paper, as drawing
//   programs export it -- is left out. Kept, it would be the part, and the
//   drawing the holes in it.
//
//   The outlines are made to bound what is filled, which is not what they
//   are drawn as. Shapes overlap, a path crosses itself where a pen tool
//   turned sharply, a dot is drawn on top of the shape it sits in -- a
//   browser fills all of it and does not mind, but a solid needs outlines
//   that neither cross nor sit inside filled ground. So every filled outline
//   is cut where it crosses any other, exactly on the curves it was drawn
//   with, and a piece is kept only where it runs between filled and not --
//   judged by each element's own fill rule, as a browser judges it. What
//   is kept is joined up again into loops that do not cross. A loop too
//   small to be anything is dropped. Outlines that are not filled -- a
//   stroke, an open line -- come through as they were.
SvgDrawing parseSvg(const std::string& text);

// The same, from a file.
SvgDrawing readSvgFile(const std::string& path);

// The same drawing with other colours as ink: `ink` is parallel to its
// colours. Its outlines are made again from the drawing as read.
SvgDrawing recolourSvg(const SvgDrawing& drawing, const std::vector<bool>& ink);

// ---- Crossings --------------------------------------------------------------------

// How many places the outlines cross -- one another, or themselves -- not
// counting where pieces of one outline meet end to end. Exact, on the curves:
// found by halving both until they part or meet, and settled by Newton's
// method. What checks a sketch's regions before they are swept, where the
// kernel's own check compares every curve with every other and took seconds
// on a drawing of a few thousand.
size_t outlineCrossings(const std::vector<std::vector<SvgSegment>>& loops);

// A loop of a sketch as curves, for outlineCrossings: a circle as two halves.
std::vector<SvgSegment> sketchLoopCurves(const Sketch& sketch, const SketchLoop& loop);

// ---- Into a sketch ------------------------------------------------------------

// What an import added to a sketch, remembered so it can still be sized, moved
// and turned after it lands -- until something else is drawn.
struct SvgInsert {
    struct Point { SketchId id; Vec2 at; };        // `at`: about the drawing's centre, at size 1
    struct Radius { SketchId entity; Real radius; };
    struct Level { SketchId constraint; bool horizontal; };

    std::vector<Point> points;
    std::vector<Radius> radii;
    std::vector<Level> levels;                     // lines held level or plumb
    std::vector<SketchId> entities;
    Vec2 size{0, 0};                               // of the drawing, at size 1

    bool empty() const { return entities.empty(); }
};

// Where an import sits in the sketch: its centre, how big it is against the
// size the file gave, and a turn in quarter turns counter-clockwise.
struct SvgPlacement {
    Vec2 centre{0, 0};
    Real scale = 1.0;
    int quarterTurns = 0;
};

// Adds the drawing to `sketch` at `placement`. Each subpath is a chain of
// entities on shared points, so a closed one is a loop the sketch can extrude.
// Lines that run exactly level or plumb are held that way, as a line drawn
// that way by hand is; nothing else is constrained.
SvgInsert insertSvg(Sketch& sketch, const SvgDrawing& drawing, const SvgPlacement& placement);

// Moves what `insert` added to a new placement. The sketch must not have been
// changed in between in any way that removed what was added.
void placeSvg(Sketch& sketch, const SvgInsert& insert, const SvgPlacement& placement);

} // namespace tg
