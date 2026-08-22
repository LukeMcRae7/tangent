// Tangent - parametric primitive generators.
//
// World units are millimetres and the up axis is +Z, matching both Fusion 360
// and Blender. Every generator takes a plain parameter struct so that once the
// feature history lands, a primitive can be re-evaluated from its parameters
// instead of being frozen at creation time.
#pragma once

#include "mesh/halfedge.h"

namespace tg {

struct BoxParams {
    Real width  = 20.0f;  // X
    Real depth  = 20.0f;  // Y
    Real height = 20.0f;  // Z
};

struct CylinderParams {
    Real radius   = 10.0f;
    Real height   = 20.0f;
    int   segments = 32;
};

struct SphereParams {
    Real radius   = 10.0f;
    int   segments = 32;   // around Z
    int   rings    = 16;   // pole to pole
};

struct ConeParams {
    Real bottomRadius = 10.0f;
    Real topRadius    = 0.0f;   // 0 gives a true apex
    Real height       = 20.0f;
    int   segments     = 32;
};

struct TorusParams {
    Real majorRadius = 12.0f;
    Real minorRadius = 4.0f;
    int   majorSegments = 40;
    int   minorSegments = 20;
};

struct PlaneParams {
    Real width = 20.0f;
    Real depth = 20.0f;
};

// Which generator a shape comes from, and the parameters for every one of
// them. They travel together: the struct is small, and this keeps the feature
// history and serialisation free of variant plumbing.
enum class PrimitiveKind { Box, Cylinder, Sphere, Cone, Torus, Plane, Custom };

const char* primitiveName(PrimitiveKind k);

struct PrimitiveSpec {
    PrimitiveKind  kind = PrimitiveKind::Box;
    BoxParams      box;
    CylinderParams cylinder;
    SphereParams   sphere;
    ConeParams     cone;
    TorusParams    torus;
    PlaneParams    plane;
};

// Each builds a closed, outward-wound, manifold mesh centred on the origin
// (the plane being the one open surface). All return false only if the
// parameters are degenerate.
bool makeBox     (Mesh& out, const BoxParams&      p = {});
bool makeCylinder(Mesh& out, const CylinderParams& p = {});
bool makeSphere  (Mesh& out, const SphereParams&   p = {});
bool makeCone    (Mesh& out, const ConeParams&     p = {});
bool makeTorus   (Mesh& out, const TorusParams&    p = {});
bool makePlane   (Mesh& out, const PlaneParams&    p = {});

} // namespace tg
