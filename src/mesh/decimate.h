// Tangent - making a mesh smaller without making it wrong.
//
// A scan or a sculpt arrives with far more triangles than its shape needs, and
// every one of them is a face if it is turned into a solid. Sixty thousand
// faces is past what the exact kernel can work on at interactive speed -- a
// fillet on one took nine seconds and failed -- so the way to make a large mesh
// something you can model on is to give it fewer, better-placed triangles
// first.
//
// Edge collapse, ordered by quadric error (Garland and Heckbert, 1997): each
// vertex carries the planes of the triangles it has absorbed, and the edge that
// moves the surface least is collapsed first. Flat regions go at no cost at
// all, which is what returns a CAD part's faces; curved regions go last and
// least.
//
// What this adds to the textbook algorithm is that the tolerance is measured,
// not estimated. The quadric only decides the order and where the new vertex
// sits. Whether a collapse is allowed is decided by measuring distances:
//
//   points of the ORIGINAL mesh -- every vertex, edge midpoint, and a few
//   points inside every triangle -- must stay near the reduced surface,
//   re-checked against the triangles a collapse changes each time it changes
//   them; and
//
//   the REDUCED surface must stay near the original, checked along every edge
//   that moves and across every triangle that changes by searching for where
//   the distance peaks, not only at chosen points.
//
// So sharp edges and corners stay where they were -- moving one further than
// the tolerance is exactly what the first check sees -- without a separate
// notion of what a feature is.
//
// Neither check is a proof: a point between samples can sit a little further
// out. So the finished result is verified -- both surfaces searched again,
// thoroughly, in both directions -- and if anything is past the tolerance the
// reduction runs again with its checks held tighter. What is returned has been
// measured to be within the tolerance, or says that it could not be.
//
// Topology is kept too. A collapse is refused if it would pinch the surface,
// join two boundaries, fold a triangle over, or leave two triangles on the
// same three vertices, so a closed mesh stays closed and a hole stays a hole.
// The result is rebuilt through Mesh::build and its Euler characteristic
// compared with the input's; a reduction that changed either is not returned.
#pragma once

#include "mesh/halfedge.h"

#include <string>

namespace tg {

struct ReduceOptions {
    // How far the surface may move, in millimetres. Never exceeded at the
    // sampled points described above.
    Real toleranceMm = 0.05;

    // Stop once there are this many triangles or fewer. Zero means go as far
    // as the tolerance allows. When the tolerance runs out first, the result
    // says so rather than pretending the target was met.
    size_t targetTriangles = 0;

    // Edges sharper than this weigh the quadric toward keeping them. It changes
    // which collapses come first and where vertices land; the tolerance, not
    // this, is what guarantees the edge survives.
    Real creaseAngleDeg = 30.0;

    // How far inside the tolerance the first pass holds its checks, overall and
    // then again across sharp creases. The defaults are measured -- see
    // reduceMesh -- and are here so a test can set them to 1 and watch a
    // second, locally tightened pass do its job. Leave them alone otherwise:
    // looser means more second passes, tighter means less reduction.
    Real firstPassMargin = 0.98;
    Real sharpCreaseMargin = 0.9;
};

struct ReduceResult {
    bool ok = false;
    std::string error;

    size_t trianglesBefore = 0, trianglesAfter = 0;
    size_t verticesBefore = 0, verticesAfter = 0;

    // Measured on the finished result, both ways, by the verification pass.
    Real deviationMm = 0.0;

    // Whether that measurement is within the tolerance. Only false after every
    // tightened pass still came out over, which the result then says rather
    // than hiding.
    bool withinTolerance = false;

    // How many times the reduction ran: 1 unless verification sent it back.
    int passes = 0;

    // True when targetTriangles was asked for and reached.
    bool reachedTarget = false;

    double milliseconds = 0.0;
};

// Reduces `in` into `out`. `in` is left alone. Faces of more than three sides
// are triangulated first; the result is all triangles.
//
// `salt` names what is made, so two reductions in one chain do not hand their
// faces the same names.
ReduceResult reduceMesh(const Mesh& in, const ReduceOptions& options, Mesh& out,
                        ElementId salt);

} // namespace tg
