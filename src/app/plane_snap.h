// Tangent - where a sketch point actually goes.
//
// Snapping to a feature is the easy half. The half that decides whether a tool
// feels precise is what happens when the cursor is *not* on anything: in every
// CAD package worth copying, the point still lands somewhere meaningful --
// level with a hole, in line with a corner, on a grid line you can see -- and
// the reason is shown rather than left to be inferred.
//
// So this answers one question: given the cursor, where does the point go and
// why. The "why" is not decoration. It is what the overlay draws and what the
// status line says, and without it a snap is indistinguishable from the tool
// being imprecise -- or, worse, from a snap to the wrong thing.
//
// Ranked strongest first:
//
//   1. on a feature          the cursor is sitting on something that exists
//   2. a crossing            two references line up, one along each axis
//   3. in line with one      the other axis falls back to the grid
//   4. a visible grid line   only the levels actually drawn
//
// Nothing below that is reported at all. Landing between two drawn lines is not
// a snap; it is a small step, and claiming otherwise would make the indicator
// meaningless by having it always on.
//
// What is deliberately not here: holding to the extension of a slanted edge.
// Every alignment runs along one of the plane's own axes, which covers what a
// person means by "level with that hole" on the axis-aligned geometry a sketch
// is nearly always drawn over -- and a slanted edge still offers its ends and
// its middle as references. Lining up along an arbitrary direction would mean
// intersecting two general lines rather than two axes, and would put a crossing
// glyph in places nobody was aiming at.
#pragma once

#include "app/camera.h"
#include "app/snap.h"
#include "scene/scene.h"

#include <string>
#include <vector>

namespace tg {

// The three grid spacings the viewport is drawing at this zoom.
//
// The same law as shaders/grid.frag, which is the point: a tool that snaps to a
// step of its own choosing puts the point somewhere there is no line, and the
// user is left holding a number that disagrees with what they can see.
struct GridLevels {
    Real fine  = 0.1;
    Real main  = 1.0;
    Real major = 10.0;
};
GridLevels gridLevelsAt(const Camera& camera, Vec3 at, Real spacing = 1.0,
                        Real subdivide = 10.0);

// Where an alignment came from, so a line can be drawn back to it.
struct SnapRef {
    Vec3     from{};
    SnapKind kind = SnapKind::None;
    bool     alongU = false;   // the reference fixes u (its line runs along v)
};

struct PlaneSnap {
    Vec2     uv{};                    // where the point goes, in plane coordinates
    Vec3     point{};                 // the same, in world
    SnapKind kind = SnapKind::None;   // the strongest reason it went there
    Real     radius = 0.0;            // when it landed on something round

    SnapRef refs[2];
    int     refCount = 0;

    // The step each axis was quantised to, or 0 where a reference fixed it.
    // The overlay uses these to draw the lines the point actually landed on.
    Real stepU = 0.0, stepV = 0.0;

    bool valid() const { return kind != SnapKind::None; }
};

struct PlaneSnapConfig {
    SnapConfig points;             // what counts as a feature

    // How close the cursor has to be to a reference's line, in pixels. Tighter
    // than a direct hit: an alignment covers a whole line across the screen, so
    // a generous tolerance would have the cursor caught by something on the far
    // side of the model most of the time.
    Real alignRadiusPx = 7.0;

    // How close to a drawn grid line counts as being on it.
    Real gridRadiusPx = 7.0;

    bool alignments = true;
    bool grid       = true;

    // A cap on references considered, so a dense part cannot make this
    // unbounded. Reached only on geometry far denser than a sketch needs.
    size_t maxRefs = 64;
};

// What to tell the user, as a phrase rather than a label: a snap has a reason,
// and the reason is the part worth reading. "In line with a corner" is a
// different claim from "a corner", and confusing the two is how a point ends up
// somewhere nobody meant.
std::string describeSnap(const PlaneSnap& snap);

// `freeUV` is where the cursor is on the plane with no snapping at all.
// `extra` are references the caller knows about that the scene does not -- the
// profile's own first point, most usefully, which is the thing a second point
// most often wants to line up with.
PlaneSnap snapOnPlane(const Scene& scene, const Camera& camera,
                      const PlaneFrame& plane, Vec2 mousePx, Vec2 freeUV,
                      const PlaneSnapConfig& config = {},
                      const std::vector<SnapPoint>& extra = {});

} // namespace tg
