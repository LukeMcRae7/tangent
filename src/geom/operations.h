// Tangent - modelling operations, over a Body rather than a mesh.
//
// Every operation the feature history can run appears here exactly once, taking
// and returning Body. For a mesh body each one forwards straight to the
// half-edge implementation in src/mesh; when a second backend arrives, this is
// the file that dispatches to it, and nothing above changes.
//
// Two things are deliberate. Operations take handles, never raw indices dressed
// up as handles -- see the note in body.h. And every one that can refuse takes a
// `reason` out-parameter, because "it did not work" is not something an
// interface can put in front of a person.
#pragma once

#include "geom/body.h"
#include "mesh/boolean.h"
#include "mesh/operations.h"
#include "mesh/primitives.h"

#include <string>
#include <vector>

namespace tg {

// Which kernel a body is made of. Chosen per body rather than per project: a
// mesh imported from an STL and a parametric bracket can sit in the same scene,
// and the operations below say plainly when one of them cannot do something.
enum class Backend { Mesh, Brep };

// ---- Construction ---------------------------------------------------------
// Returns false, leaving `out` untouched, for degenerate parameters -- and for
// a Brep body when the project was built without OpenCASCADE, or when the kind
// has no exact form (a Plane is a surface, not a solid). Refusing is deliberate:
// quietly handing back a mesh body would put a body in the scene that cannot do
// what the caller asked for.
bool makePrimitive(const PrimitiveSpec& spec, Body& out, Backend backend = Backend::Mesh);

// ---- Modelling ------------------------------------------------------------
// Pushes faces along the region's area-weighted normal. `newFaces` reports
// where the moved faces ended up, in the order they were given.
bool extrudeFaces(Body& body, const std::vector<FaceId>& faces, Real distance,
                  std::vector<FaceId>* newFaces = nullptr, ElementId salt = 0,
                  ExtrudeOp op = ExtrudeOp::Auto);

bool insetFaces(Body& body, const std::vector<FaceId>& faces, Real amount,
                std::vector<FaceId>* newFaces = nullptr, ElementId salt = 0);

// Rounds edges. `reason` gets a short phrase on refusal; see the note above.
bool filletEdges(Body& body, const FilletSpec& spec, std::string* reason = nullptr);

bool booleanOp(const Body& a, const Body& b, BooleanOp op, Body& out,
               ElementId salt = 0, bool trustBNames = false);

// Largest radius the whole body can take before a face collapses. For clamping
// a slider to a range that always produces valid geometry.
Real maxFilletRadius(const Body& body);

// ---- Analysis -------------------------------------------------------------
// Extends a selection along tangent-continuous edges, the way F does in Fusion.
std::vector<EdgeId> extendTangentChain(const Body& body, const std::vector<EdgeId>& edges);

// Splits into connected bodies, largest first. A body that is already one piece
// yields itself, so a caller can always use the result.
size_t splitBodies(const Body& body, std::vector<Body>& out);

bool splitByPlane(const Body& body, Vec3 planePoint, Vec3 planeNormal,
                  Body& a, Body& b);

} // namespace tg
