#include "ui/glyph.h"

#include "core/palette.h"

#include <cmath>
#include <initializer_list>
#include <vector>

namespace tg {
namespace {

constexpr float kGlyphPi = 3.14159265358979f;

// A pen over the 24-unit box every glyph is drawn in.
struct Pen {
    ImDrawList* dl;
    ImVec2 origin;      // top-left of the box in pixels
    float  unit;        // pixels per glyph unit
    ImU32  col;
    float  stroke;

    ImVec2 at(float x, float y) const { return {origin.x + x * unit, origin.y + y * unit}; }

    void line(float x0, float y0, float x1, float y1) const {
        dl->AddLine(at(x0, y0), at(x1, y1), col, stroke);
    }
    void poly(std::initializer_list<float> xy, bool closed) const {
        std::vector<ImVec2> p;
        auto it = xy.begin();
        while (it != xy.end()) { const float x = *it++; const float y = *it++; p.push_back(at(x, y)); }
        dl->AddPolyline(p.data(), static_cast<int>(p.size()), col,
                        closed ? ImDrawFlags_Closed : ImDrawFlags_None, stroke);
    }
    void fill(std::initializer_list<float> xy, ImU32 c) const {
        std::vector<ImVec2> p;
        auto it = xy.begin();
        while (it != xy.end()) { const float x = *it++; const float y = *it++; p.push_back(at(x, y)); }
        dl->AddConvexPolyFilled(p.data(), static_cast<int>(p.size()), c);
    }
    void circle(float cx, float cy, float r) const {
        dl->AddCircle(at(cx, cy), r * unit, col, 0, stroke);
    }
    void disc(float cx, float cy, float r, ImU32 c) const {
        dl->AddCircleFilled(at(cx, cy), r * unit, c);
    }
    void arc(float cx, float cy, float r, float a0, float a1) const {
        dl->PathArcTo(at(cx, cy), r * unit, a0, a1, 24);
        dl->PathStroke(col, ImDrawFlags_None, stroke);
    }
    // An ellipse, in whole or in part. Angles in radians, clockwise from +x.
    void ellipse(float cx, float cy, float rx, float ry, float a0 = 0.0f, float a1 = 2.0f * kGlyphPi) const {
        const int n = 28;
        std::vector<ImVec2> p;
        for (int i = 0; i <= n; ++i) {
            const float a = a0 + (a1 - a0) * static_cast<float>(i) / n;
            p.push_back(at(cx + std::cos(a) * rx, cy + std::sin(a) * ry));
        }
        dl->AddPolyline(p.data(), static_cast<int>(p.size()), col, ImDrawFlags_None, stroke);
    }
    void bezier(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3) const {
        dl->AddBezierCubic(at(x0, y0), at(x1, y1), at(x2, y2), at(x3, y3), col, stroke);
    }
    void dot(float cx, float cy, float r = 1.1f) const { disc(cx, cy, r, col); }

    // The shape a plane makes seen from a little above: a parallelogram.
    void slab(float x, float y, float w, float h, float skew) const {
        poly({x + skew, y, x + w + skew, y, x + w, y + h, x, y + h}, true);
    }
};

ImU32 faded(ImU32 c, float alpha) {
    const float a = static_cast<float>((c >> IM_COL32_A_SHIFT) & 0xFF) / 255.0f * alpha;
    return (c & ~IM_COL32_A_MASK) | (static_cast<ImU32>(a * 255.0f + 0.5f) << IM_COL32_A_SHIFT);
}

// Two circles, and which parts of them are solid. The lens is bounded by the
// two arcs between the intersection points, so the fills are polygons made of
// exactly those arcs rather than a clip that ImGui does not have.
void venn(const Pen& pen, bool fillLeft, bool fillRight, bool fillLens, bool holeLens) {
    const float r = 5.6f, cx0 = 9.2f, cx1 = 14.8f, cy = 12.0f;
    const float d = (cx1 - cx0) * 0.5f;
    const float th = std::acos(d / r);
    const ImU32 soft = faded(pen.col, 0.45f);
    auto arcPts = [&](float cx, float a0, float a1, std::vector<ImVec2>& out) {
        const int n = 20;
        for (int i = 0; i <= n; ++i) {
            const float a = a0 + (a1 - a0) * static_cast<float>(i) / n;
            out.push_back(pen.at(cx + std::cos(a) * r, cy + std::sin(a) * r));
        }
    };
    std::vector<ImVec2> lens;
    arcPts(cx0, -th, th, lens);                 // the right-hand arc of the left circle
    arcPts(cx1, kGlyphPi - th, kGlyphPi + th, lens);      // the left-hand arc of the right circle
    if (fillLeft) {
        std::vector<ImVec2> p;
        arcPts(cx0, th, 2.0f * kGlyphPi - th, p);    // the outside of the left circle
        arcPts(cx1, kGlyphPi + th, kGlyphPi - th, p);     // back along the lens boundary
        pen.dl->AddConcavePolyFilled(p.data(), static_cast<int>(p.size()), soft);
    }
    if (fillRight) {
        std::vector<ImVec2> p;
        arcPts(cx1, kGlyphPi + th, 3.0f * kGlyphPi - th, p);
        arcPts(cx0, th, -th, p);
        pen.dl->AddConcavePolyFilled(p.data(), static_cast<int>(p.size()), soft);
    }
    if (fillLens) pen.dl->AddConvexPolyFilled(lens.data(), static_cast<int>(lens.size()), soft);
    if (holeLens) {
        // Cut away: the lens drawn in the panel colour so it reads as removed.
        pen.dl->AddConvexPolyFilled(lens.data(), static_cast<int>(lens.size()),
                                    ImGui::GetColorU32(ImGuiCol_WindowBg, 0.0f));
    }
    pen.circle(cx0, cy, r);
    pen.circle(cx1, cy, r);
}

void cube(const Pen& pen, bool shadeTop) {
    // Front square, top and side as parallelograms.
    pen.poly({5, 9, 15, 9, 15, 19, 5, 19}, true);
    pen.poly({5, 9, 10, 4, 20, 4, 15, 9}, true);
    pen.poly({15, 9, 20, 4, 20, 14, 15, 19}, true);
    if (shadeTop) pen.fill({5, 9, 10, 4, 20, 4, 15, 9}, faded(pen.col, 0.35f));
}

void arrowHead(const Pen& pen, float x, float y, float dx, float dy, float size) {
    // A filled head pointing along (dx, dy).
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-6f) return;
    dx /= len; dy /= len;
    const float px = -dy, py = dx;
    pen.fill({x, y, x - dx * size + px * size * 0.55f, y - dy * size + py * size * 0.55f,
              x - dx * size - px * size * 0.55f, y - dy * size - py * size * 0.55f}, pen.col);
}

void meshGrid(const Pen& pen) {
    pen.poly({4, 5, 20, 5, 20, 19, 4, 19}, true);
    pen.line(4, 19, 20, 5);
    pen.line(12, 5, 12, 19);
    pen.line(4, 12, 20, 12);
}

} // namespace

void drawGlyph(ImDrawList* dl, Glyph g, ImVec2 centre, float sizePx, ImU32 colour, float strokePx) {
    if (!dl || sizePx <= 0.0f) return;
    const float unit = sizePx / 24.0f;
    Pen pen{dl, ImVec2(centre.x - sizePx * 0.5f, centre.y - sizePx * 0.5f), unit, colour,
            strokePx > 0.0f ? strokePx : std::max(1.2f, sizePx / 13.0f)};
    const ImU32 soft = faded(colour, 0.35f);

    switch (g) {
    // ---- files ----------------------------------------------------------
    case Glyph::New:
        pen.poly({6, 3, 14, 3, 19, 8, 19, 21, 6, 21}, true);
        pen.poly({14, 3, 14, 8, 19, 8}, false);
        pen.line(12.5f, 12, 12.5f, 18);
        pen.line(9.5f, 15, 15.5f, 15);
        break;
    case Glyph::Open:
        pen.poly({3, 6, 9, 6, 11, 8.5f, 21, 8.5f, 21, 20, 3, 20}, true);
        pen.line(3, 11.5f, 21, 11.5f);
        break;
    case Glyph::Save:
        pen.poly({4, 4, 17, 4, 20, 7, 20, 20, 4, 20}, true);
        pen.poly({8, 4, 8, 9, 15, 9, 15, 4}, false);
        pen.poly({7.5f, 20, 7.5f, 14, 16.5f, 14, 16.5f, 20}, false);
        break;
    case Glyph::Settings: {
        pen.circle(12, 12, 3.2f);
        for (int i = 0; i < 8; ++i) {
            const float a = static_cast<float>(i) * kGlyphPi / 4.0f;
            pen.line(12 + std::cos(a) * 6.2f, 12 + std::sin(a) * 6.2f,
                     12 + std::cos(a) * 9.4f, 12 + std::sin(a) * 9.4f);
        }
        pen.circle(12, 12, 6.2f);
        break;
    }
    case Glyph::Import:
        pen.poly({4, 14, 4, 20, 20, 20, 20, 14}, false);
        pen.line(12, 3, 12, 15);
        pen.poly({7.5f, 10.5f, 12, 15, 16.5f, 10.5f}, false);
        break;
    case Glyph::Export:
        pen.poly({4, 14, 4, 20, 20, 20, 20, 14}, false);
        pen.line(12, 15, 12, 3);
        pen.poly({7.5f, 7.5f, 12, 3, 16.5f, 7.5f}, false);
        break;

    // ---- shapes ---------------------------------------------------------
    case Glyph::Box:
    case Glyph::Body:
        cube(pen, false);
        break;
    case Glyph::Cube:
        cube(pen, true);
        break;
    case Glyph::Cylinder:
        pen.ellipse(12, 6.5f, 6.5f, 2.6f);
        pen.line(5.5f, 6.5f, 5.5f, 17.5f);
        pen.line(18.5f, 6.5f, 18.5f, 17.5f);
        pen.ellipse(12, 17.5f, 6.5f, 2.6f, 0.0f, kGlyphPi);
        break;
    case Glyph::Sphere:
        pen.circle(12, 12, 8);
        pen.ellipse(12, 12, 8, 3);
        pen.ellipse(12, 12, 3, 8);
        break;
    case Glyph::Cone:
        pen.line(12, 3.5f, 5, 17.5f);
        pen.line(12, 3.5f, 19, 17.5f);
        pen.ellipse(12, 17.5f, 7, 2.6f);
        break;
    case Glyph::Torus:
        pen.ellipse(12, 12, 9, 5.5f);
        pen.ellipse(12, 11.2f, 3.6f, 1.8f, 0.0f, kGlyphPi);
        pen.ellipse(12, 12.8f, 3.6f, 1.8f, kGlyphPi, 2.0f * kGlyphPi);
        break;
    case Glyph::Plane:
        pen.slab(3, 8, 14, 9, 4);
        break;
    case Glyph::Sketch:
        pen.bezier(3, 18, 7, 2, 13, 24, 21, 6);
        pen.dot(21, 6, 1.5f);
        break;

    // ---- transforms and combining --------------------------------------
    case Glyph::Move:
        pen.line(12, 3, 12, 21);
        pen.line(3, 12, 21, 12);
        pen.poly({9, 6, 12, 3, 15, 6}, false);
        pen.poly({9, 18, 12, 21, 15, 18}, false);
        pen.poly({6, 9, 3, 12, 6, 15}, false);
        pen.poly({18, 9, 21, 12, 18, 15}, false);
        break;
    case Glyph::Rotate:
        pen.arc(12, 12, 7.5f, -0.35f * kGlyphPi, 1.35f * kGlyphPi);
        arrowHead(pen, 12 + std::cos(-0.35f * kGlyphPi) * 7.5f, 12 + std::sin(-0.35f * kGlyphPi) * 7.5f,
                  0.55f, 0.85f, 3.4f);
        break;
    case Glyph::RotateFace:
        pen.poly({4, 19, 14, 19, 14, 9, 4, 9}, true);
        pen.arc(4, 19, 14, -0.5f * kGlyphPi, -0.17f * kGlyphPi);
        arrowHead(pen, 4 + std::cos(-0.17f * kGlyphPi) * 14, 19 + std::sin(-0.17f * kGlyphPi) * 14,
                  0.5f, 0.87f, 3.0f);
        break;
    case Glyph::Scale:
        pen.poly({4, 10, 4, 20, 14, 20}, false);
        pen.poly({10, 4, 20, 4, 20, 14}, false);
        pen.line(20, 4, 9, 15);
        pen.poly({4, 15, 4, 20, 9, 20}, true);
        break;
    case Glyph::ScaleFace:
        pen.poly({6, 18, 18, 18, 15, 8, 9, 8}, true);
        pen.line(12, 3, 12, 8);
        pen.poly({9.5f, 5.5f, 12, 3, 14.5f, 5.5f}, false);
        break;
    case Glyph::Boolean:
        venn(pen, false, false, true, false);
        break;
    case Glyph::Union:
        venn(pen, true, true, true, false);
        break;
    case Glyph::Difference:
        venn(pen, true, false, false, true);
        break;
    case Glyph::Intersect:
        venn(pen, false, false, true, false);
        break;
    case Glyph::NewBody:
        pen.circle(12, 12, 7.5f);
        pen.line(12, 8.5f, 12, 15.5f);
        pen.line(8.5f, 12, 15.5f, 12);
        break;

    // ---- modelling ------------------------------------------------------
    case Glyph::Extrude:
        pen.poly({4, 13, 12, 9, 20, 13, 12, 17}, true);
        pen.fill({4, 13, 12, 9, 20, 13, 12, 17}, soft);
        pen.line(4, 13, 4, 18); pen.line(20, 13, 20, 18); pen.line(12, 17, 12, 22);
        pen.poly({4, 18, 12, 22, 20, 18}, false);
        pen.line(12, 9, 12, 2.5f);
        pen.poly({9.2f, 5.3f, 12, 2.5f, 14.8f, 5.3f}, false);
        break;
    case Glyph::PushPull:
        pen.poly({4, 14, 12, 10, 20, 14, 12, 18}, true);
        pen.fill({4, 14, 12, 10, 20, 14, 12, 18}, soft);
        pen.line(12, 10, 12, 3);
        pen.poly({9.2f, 5.8f, 12, 3, 14.8f, 5.8f}, false);
        pen.line(12, 18, 12, 21.5f);
        break;
    case Glyph::Fillet:
        // A square whose top-left corner has been rounded.
        pen.line(4, 20, 20, 20); pen.line(20, 20, 20, 4); pen.line(20, 4, 12, 4);
        pen.arc(12, 12, 8, kGlyphPi, 1.5f * kGlyphPi);
        pen.line(4, 12, 4, 20);
        pen.line(4, 4, 9, 4); pen.line(4, 4, 4, 9);
        break;
    case Glyph::Chamfer:
        pen.poly({4, 20, 20, 20, 20, 4, 12, 4, 4, 12}, true);
        pen.line(4, 4, 9, 4); pen.line(4, 4, 4, 9);
        break;
    case Glyph::Shell:
        pen.poly({4, 5, 4, 20, 20, 20, 20, 5}, false);
        pen.poly({8, 5, 8, 16, 16, 16, 16, 5}, false);
        break;
    case Glyph::Inset:
        pen.poly({4, 4, 20, 4, 20, 20, 4, 20}, true);
        pen.poly({8.5f, 8.5f, 15.5f, 8.5f, 15.5f, 15.5f, 8.5f, 15.5f}, true);
        pen.fill({8.5f, 8.5f, 15.5f, 8.5f, 15.5f, 15.5f, 8.5f, 15.5f}, soft);
        break;
    case Glyph::Divide:
        pen.poly({4, 4, 20, 4, 20, 20, 4, 20}, true);
        for (int i = 0; i < 4; ++i) pen.line(12, 5.5f + i * 4.0f, 12, 7.5f + i * 4.0f);
        break;
    case Glyph::Merge:
        // A funnel: many things in at the top, one thing out at the bottom.
        pen.poly({3, 4, 21, 4, 14, 13, 14, 20, 10, 18, 10, 13}, true);
        break;
    case Glyph::Pattern:
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 2; ++j)
                pen.poly({4.0f + i * 6.0f, 6.0f + j * 7.5f, 8.0f + i * 6.0f, 6.0f + j * 7.5f,
                          8.0f + i * 6.0f, 10.0f + j * 7.5f, 4.0f + i * 6.0f, 10.0f + j * 7.5f}, true);
        break;
    case Glyph::Mirror:
        pen.poly({3, 19, 10, 19, 10, 6}, true);
        pen.poly({21, 19, 14, 19, 14, 6}, true);
        pen.fill({21, 19, 14, 19, 14, 6}, soft);
        for (int i = 0; i < 5; ++i) pen.line(12, 3.5f + i * 4.0f, 12, 5.5f + i * 4.0f);
        break;
    case Glyph::Split:
        pen.poly({4, 5, 11, 5, 11, 20, 4, 20}, true);
        pen.poly({14, 4, 21, 4, 21, 19, 14, 19}, true);
        break;
    case Glyph::Convert:
        meshGrid(pen);
        pen.fill({4, 5, 20, 5, 20, 19, 4, 19}, soft);
        break;
    case Glyph::Reduce:
        pen.poly({4, 5, 20, 5, 20, 19, 4, 19}, true);
        pen.line(4, 19, 20, 5);
        break;
    case Glyph::Mesh:
        meshGrid(pen);
        break;

    // ---- inspecting -----------------------------------------------------
    case Glyph::Measure:
        pen.poly({3, 8, 21, 8, 21, 16, 3, 16}, true);
        for (int i = 1; i < 6; ++i) pen.line(3.0f + i * 3.0f, 8, 3.0f + i * 3.0f, i % 2 ? 11.0f : 12.5f);
        break;
    case Glyph::Alert:
        // A bell.
        pen.arc(12, 11, 6, kGlyphPi, 2.0f * kGlyphPi);
        pen.line(6, 11, 6, 16); pen.line(18, 11, 18, 16);
        pen.line(4, 17, 20, 17);
        pen.line(6, 16, 4, 17); pen.line(18, 16, 20, 17);
        pen.arc(12, 17.5f, 2.6f, 0.0f, kGlyphPi);
        pen.line(12, 3, 12, 5);
        break;
    case Glyph::Check:
        pen.poly({5, 12.5f, 10, 17.5f, 19.5f, 7}, false);
        break;
    case Glyph::Eye:
        pen.bezier(3, 12, 8, 5, 16, 5, 21, 12);
        pen.bezier(3, 12, 8, 19, 16, 19, 21, 12);
        pen.circle(12, 12, 3.2f);
        pen.dot(12, 12, 1.4f);
        break;
    case Glyph::EyeOff:
        pen.bezier(3, 12, 8, 5, 16, 5, 21, 12);
        pen.bezier(3, 12, 8, 19, 16, 19, 21, 12);
        pen.circle(12, 12, 3.2f);
        pen.line(5, 20, 19, 4);
        break;
    case Glyph::Grid:
        pen.poly({4, 4, 20, 4, 20, 20, 4, 20}, true);
        pen.line(9.3f, 4, 9.3f, 20); pen.line(14.6f, 4, 14.6f, 20);
        pen.line(4, 9.3f, 20, 9.3f); pen.line(4, 14.6f, 20, 14.6f);
        break;
    case Glyph::Wire:
        cube(pen, false);
        pen.line(5, 9, 20, 4); pen.line(5, 19, 15, 9);
        break;
    case Glyph::Frame:
        pen.poly({4, 9, 4, 4, 9, 4}, false);
        pen.poly({15, 4, 20, 4, 20, 9}, false);
        pen.poly({20, 15, 20, 20, 15, 20}, false);
        pen.poly({9, 20, 4, 20, 4, 15}, false);
        pen.circle(12, 12, 3);
        break;
    case Glyph::Camera:
        pen.poly({3, 7, 15, 7, 15, 18, 3, 18}, true);
        pen.poly({15, 10.5f, 21, 7, 21, 18, 15, 14.5f}, true);
        break;

    // ---- chrome ---------------------------------------------------------
    case Glyph::ChevronDown:
        pen.poly({6, 9.5f, 12, 15.5f, 18, 9.5f}, false);
        break;
    case Glyph::ChevronRight:
        pen.poly({9.5f, 6, 15.5f, 12, 9.5f, 18}, false);
        break;
    case Glyph::ChevronUp:
        pen.poly({6, 14.5f, 12, 8.5f, 18, 14.5f}, false);
        break;
    case Glyph::Close:
        pen.line(6.5f, 6.5f, 17.5f, 17.5f);
        pen.line(17.5f, 6.5f, 6.5f, 17.5f);
        break;
    case Glyph::Minimize:
        pen.line(6, 12.5f, 18, 12.5f);
        break;
    case Glyph::Maximize:
        pen.poly({6.5f, 6.5f, 17.5f, 6.5f, 17.5f, 17.5f, 6.5f, 17.5f}, true);
        break;
    case Glyph::Restore:
        pen.poly({6, 9, 15, 9, 15, 18, 6, 18}, true);
        pen.poly({9, 9, 9, 6, 18, 6, 18, 15, 15, 15}, false);
        break;
    case Glyph::Undo:
        pen.arc(13, 13, 6, -0.5f * kGlyphPi, 0.85f * kGlyphPi);
        pen.poly({10.5f, 3.5f, 6.5f, 7, 10.5f, 10.5f}, false);
        pen.line(6.5f, 7, 13, 7);
        break;
    case Glyph::Redo:
        pen.arc(11, 13, 6, 0.15f * kGlyphPi, 1.5f * kGlyphPi);
        pen.poly({13.5f, 3.5f, 17.5f, 7, 13.5f, 10.5f}, false);
        pen.line(17.5f, 7, 11, 7);
        break;
    case Glyph::Plus:
        pen.line(12, 5, 12, 19);
        pen.line(5, 12, 19, 12);
        break;
    case Glyph::Dot:
        pen.dot(12, 12, 2.6f);
        break;
    case Glyph::Clock:
        pen.circle(12, 12, 8);
        pen.poly({12, 7, 12, 12, 15.5f, 14}, false);
        break;
    case Glyph::Lock:
        pen.poly({6, 11, 18, 11, 18, 20, 6, 20}, true);
        pen.arc(12, 11, 4, kGlyphPi, 2.0f * kGlyphPi);
        pen.line(8, 11, 8, 7.5f); pen.line(16, 11, 16, 7.5f);
        break;
    case Glyph::Trash:
        pen.line(4.5f, 7, 19.5f, 7);
        pen.poly({6.5f, 7, 7.5f, 20, 16.5f, 20, 17.5f, 7}, false);
        pen.poly({9.5f, 7, 9.5f, 4.5f, 14.5f, 4.5f, 14.5f, 7}, false);
        pen.line(10.3f, 10, 10.6f, 17); pen.line(13.7f, 10, 13.4f, 17);
        break;

    // ---- sketching ------------------------------------------------------
    case Glyph::Select:
        pen.poly({6, 4, 6, 18, 10, 14.5f, 13, 20.5f, 15.5f, 19.5f, 12.5f, 13.5f, 17.5f, 13}, true);
        break;
    case Glyph::Line:
        pen.line(5, 19, 19, 5);
        pen.dot(5, 19, 1.6f); pen.dot(19, 5, 1.6f);
        break;
    case Glyph::Rect:
        pen.poly({4, 6, 20, 6, 20, 18, 4, 18}, true);
        pen.dot(4, 6, 1.5f); pen.dot(20, 18, 1.5f);
        break;
    case Glyph::Circle:
        pen.circle(12, 12, 8);
        pen.dot(12, 12, 1.4f);
        break;
    case Glyph::Arc:
        pen.arc(12, 14, 8, kGlyphPi, 2.0f * kGlyphPi);
        pen.dot(4, 14, 1.5f); pen.dot(20, 14, 1.5f); pen.dot(12, 14, 1.2f);
        break;
    case Glyph::Dimension:
        pen.line(4, 12, 20, 12);
        pen.line(4, 8, 4, 16); pen.line(20, 8, 20, 16);
        pen.poly({7, 9.5f, 4, 12, 7, 14.5f}, false);
        pen.poly({17, 9.5f, 20, 12, 17, 14.5f}, false);
        break;

    case Glyph::Count:
        break;
    }
}

void glyphItem(Glyph g, float sizePx, ImU32 colour) {
    const float h = ImGui::GetFrameHeight();
    const ImVec2 at = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(sizePx, std::max(h, sizePx)));
    drawGlyph(ImGui::GetWindowDrawList(), g,
              ImVec2(at.x + sizePx * 0.5f, at.y + std::max(h, sizePx) * 0.5f), sizePx, colour);
}

bool glyphButton(const char* id, Glyph g, float sizePx, const char* tooltip, bool active,
                 bool enabled, ImU32 tint) {
    const ImGuiStyle& st = ImGui::GetStyle();
    const ImVec2 size(sizePx + st.FramePadding.x * 2.0f, sizePx + st.FramePadding.y * 2.0f);

    ImGui::PushID(id);
    ImGui::BeginDisabled(!enabled);
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton("##g", size);
    const bool hovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
    const bool held = ImGui::IsItemActive();
    ImGui::EndDisabled();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 lo = at, hi = ImVec2(at.x + size.x, at.y + size.y);
    if (active) {
        dl->AddRectFilled(lo, hi, ImGui::GetColorU32(ImVec4(palette::kBrand.r, palette::kBrand.g,
                                                            palette::kBrand.b, 1.0f)), st.FrameRounding);
    } else if (held && enabled) {
        dl->AddRectFilled(lo, hi, ImGui::GetColorU32(ImGuiCol_ButtonActive), st.FrameRounding);
    } else if (hovered && enabled) {
        dl->AddRectFilled(lo, hi, ImGui::GetColorU32(ImGuiCol_ButtonHovered), st.FrameRounding);
    }

    ImU32 col = tint ? tint : ImGui::GetColorU32(ImGuiCol_Text);
    if (active) col = IM_COL32(255, 255, 255, 255);
    if (!enabled) col = faded(col, 0.35f);
    drawGlyph(dl, g, ImVec2(at.x + size.x * 0.5f, at.y + size.y * 0.5f), sizePx, col);

    // Not while a menu is open: a button that opens one is still hovered while
    // it is open, and its tooltip lands exactly on top of what it opened.
    if (tooltip && hovered &&
        !ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel))
        ImGui::SetTooltip("%s", tooltip);
    ImGui::PopID();
    return clicked && enabled;
}

} // namespace tg
