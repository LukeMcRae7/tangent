#include "ui/drag_guide.h"

#include "core/palette.h"
#include "ui/theme.h"
#include "ui/widgets.h"

#include <algorithm>
#include <cmath>

namespace tg::ui {

void drawDragGuide(const DragAxis& axis, const Camera& camera, ImVec2 viewportOrigin,
                   Real value, Real step, Real limit, const char* label) {
    if (!axis.valid) return;

    Vec2 originPx{};
    if (!camera.projectToPixel(axis.origin, originPx)) return;
    const ImVec2 o(viewportOrigin.x + static_cast<float>(originPx.x),
                   viewportOrigin.y + static_cast<float>(originPx.y));

    // The axis as it appears on screen: the same sample DragAxis::offsetPx
    // measures along, so what is drawn is what is measured.
    const Real pxWorld = static_cast<Real>(camera.pixelWorldSize(axis.origin));
    Vec2 aheadPx{};
    ImVec2 dir(1.0f, 0.0f);
    bool endOn = true;
    if (camera.projectToPixel(axis.origin + axis.direction * (pxWorld * 60.0), aheadPx)) {
        const Vec2 along = aheadPx - originPx;
        if (lengthSq(along) > 4.0) {
            const Vec2 unit = along / length(along);
            dir = ImVec2(static_cast<float>(unit.x), static_cast<float>(unit.y));
            endOn = false;
        }
    }
    const ImVec2 across(-dir.y, dir.x);

    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const ImU32 ticks  = u32(palette::kBrand, 0.45f);
    const ImU32 track  = u32(palette::kBrand, 0.65f);
    const ImU32 arrow  = u32(palette::kBrand, 1.0f);
    const ImU32 ring   = u32(palette::kCommand, 0.9f);

    auto at = [&](float along, float side = 0.0f) {
        return ImVec2(o.x + dir.x * along + across.x * side, o.y + dir.y * along + across.y * side);
    };

    const bool bounded = !axis.signedRange && axis.spanValue > axis.baseValue;
    const float trackPx = static_cast<float>(DragAxis::kTrackPx);

    // Where a value sits along the track, in pixels from the origin. The
    // inverse of DragAxis::valueAt, so the head is under the cursor.
    auto place = [&](Real v) -> float {
        if (axis.signedRange)
            return axis.spanValue > 0.0 ? static_cast<float>(v / axis.spanValue) * trackPx
                                        : static_cast<float>(v / std::max(pxWorld, Real(1e-9)));
        if (!bounded) return static_cast<float>(std::max(Real(0), v - axis.baseValue) / pxWorld);
        const Real t = std::clamp((v - axis.baseValue) / (axis.spanValue - axis.baseValue),
                                  Real(0), Real(1));
        return static_cast<float>(t) * trackPx;
    };

    if (endOn) {
        // The axis points at the eye: there is no direction on screen to draw,
        // so the guide is a ring around the origin and the number beside it.
        dl->AddCircle(o, 18.0f, track, 0, 2.0f);
        dl->AddCircleFilled(o, 5.0f, arrow);
        if (label) {
            const ImVec2 ts = ImGui::CalcTextSize(label);
            const ImVec2 p(o.x + 24.0f, o.y - ts.y * 0.5f);
            dl->AddRectFilled(ImVec2(p.x - 7.0f, p.y - 4.0f), ImVec2(p.x + ts.x + 7.0f, p.y + ts.y + 4.0f),
                              ring, 5.0f);
            dl->AddText(p, arrow, label);
        }
        return;
    }

    const float headPx = place(value);

    // The track: the same length on screen every time. That is the whole
    // point of it -- the gesture is the same size for a 2mm wall and a 200mm
    // plate, and the ticks along it are spaced the same in both. A signed
    // drag runs both ways from the start, and grows to hold whatever has been
    // asked for rather than stopping short of the value.
    const float reach = axis.signedRange ? std::max(trackPx, std::fabs(headPx) + 24.0f) : trackPx;
    const ImVec2 t0 = axis.signedRange ? at(-reach) : o;
    const ImVec2 t1 = at(reach);
    // A dark halo under the track first, so it reads over a light face as
    // well as over the dark ground.
    dl->AddLine(t0, t1, u32(palette::kCommand, 0.55f), 4.5f);
    dl->AddLine(t0, t1, track, 2.5f);
    dl->AddCircleFilled(t1, 3.0f, track);
    if (axis.signedRange) dl->AddCircleFilled(t0, 3.0f, track);

    // Ticks, one per step. Nothing behind the start on a one-way drag: the
    // gesture cannot go there, so a scale there would describe travel that
    // does not exist.
    if (step > 0.0) {
        Real first = std::ceil(axis.baseValue / step) * step;
        Real last = bounded ? axis.spanValue : axis.baseValue + reach * pxWorld;
        if (axis.signedRange) {
            const Real span = std::max(std::fabs(value), axis.spanValue) * 1.15;
            first = -std::ceil(span / step) * step;
            last = -first;
        }
        int drawn = 0;
        for (Real v = first; v <= last + 1e-9 && drawn < 120; v += step, ++drawn) {
            const float p = place(v);
            if (std::fabs(p) > reach + 0.5f) continue;
            const bool major = std::fabs(std::fmod(v / step, 4.0)) < 1e-6;
            const float len = major ? 6.0f : 3.5f;
            dl->AddLine(at(p, -len), at(p, len), ticks, 1.0f);
        }
    }

    // The cap: where the shape gives up, which is not always the end of the
    // track. The track carries everything the body could hold; the limit is
    // what it will actually take.
    if (bounded && limit > axis.baseValue) {
        const float p = place(limit);
        dl->AddLine(at(p, -9.0f), at(p, 9.0f), u32(palette::kBrand, 0.7f), 3.0f);
    }

    // The origin, on a drag that can go either way: zero is a value like any
    // other here -- it means "leave it where it is" -- and it has to be found.
    if (axis.signedRange) dl->AddLine(at(0.0f, -7.0f), at(0.0f, 7.0f), arrow, 2.5f);

    // The arrow: from the start to the value, with a filled head and a handle
    // that says this is the thing under the hand.
    const float barb = 13.0f;
    const ImVec2 head = at(headPx);
    const float travelled = std::fabs(headPx);
    const float sign = headPx < 0.0f ? -1.0f : 1.0f;
    if (travelled > barb * 0.9f) {
        dl->AddLine(o, at(headPx - sign * barb * 0.8f), arrow, 3.0f);
    }
    if (travelled > 2.0f) {
        const ImVec2 back = at(headPx - sign * barb);
        const ImVec2 tri[3] = {head,
                               ImVec2(back.x + across.x * barb * 0.42f, back.y + across.y * barb * 0.42f),
                               ImVec2(back.x - across.x * barb * 0.42f, back.y - across.y * barb * 0.42f)};
        dl->AddTriangleFilled(tri[0], tri[1], tri[2], arrow);
    }
    dl->AddCircleFilled(head, 7.0f, ring);
    dl->AddCircleFilled(head, 5.0f, arrow);

    // The number, beside the head and off the axis, so the hand does not
    // cover it. Kept on the side of the axis that faces away from the origin
    // of the screen so it tends to stay inside the viewport.
    if (label && *label) {
        pushFont(FontWeight::Medium);
        const ImVec2 ts = ImGui::CalcTextSize(label);
        const float side = across.y < 0.0f ? -1.0f : 1.0f;    // below the axis where possible
        ImVec2 p(head.x + across.x * 18.0f * side, head.y + across.y * 18.0f * side);
        p.x -= ts.x * 0.5f;
        p.y -= ts.y * 0.5f;
        dl->AddRectFilled(ImVec2(p.x - 8.0f, p.y - 4.0f), ImVec2(p.x + ts.x + 8.0f, p.y + ts.y + 4.0f),
                          ring, 6.0f);
        dl->AddRect(ImVec2(p.x - 8.0f, p.y - 4.0f), ImVec2(p.x + ts.x + 8.0f, p.y + ts.y + 4.0f),
                    u32(palette::kBrand, 0.6f), 6.0f);
        dl->AddText(p, u32(palette::kText), label);
        ImGui::PopFont();
    }
}

} // namespace tg::ui
