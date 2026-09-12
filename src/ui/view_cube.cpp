#include "ui/view_cube.h"

#include "core/palette.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>

namespace tg {

namespace {

// How much of the widget the cube fills. A unit cube seen corner-on measures
// sqrt(3) across, so leaving room for that is what stops the isometric view
// from clipping against the edge of the box.
constexpr float kCubeFit = 1.82f;

ImU32 toU32(Rgb c, float a) {
    return ImGui::GetColorU32(ImVec4(c.r, c.g, c.b, a));
}

// The two in-plane axes of a face, chosen so the pair is right-handed with the
// normal. Which pair does not matter for hit-testing -- the zone is the sum of
// signed axes either way -- but it has to be consistent or the labels end up
// mirrored on half the faces.
void faceAxes(Vec3 n, Vec3& a, Vec3& b) {
    if (std::fabs(n.z) > 0.5f) { a = {1, 0, 0}; b = {0, n.z > 0 ? 1.0f : -1.0f, 0}; }
    else if (std::fabs(n.y) > 0.5f) { a = {n.y > 0 ? -1.0f : 1.0f, 0, 0}; b = {0, 0, 1}; }
    else { a = {0, n.x > 0 ? 1.0f : -1.0f, 0}; b = {0, 0, 1}; }
}

const Vec3 kFaces[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

const char* faceLabel(Vec3 n) {
    if (n.x > 0.5f)  return "RIGHT";
    if (n.x < -0.5f) return "LEFT";
    if (n.y > 0.5f)  return "BACK";
    if (n.y < -0.5f) return "FRONT";
    if (n.z > 0.5f)  return "TOP";
    return "BOTTOM";
}

// Where a point of the cube lands in the widget, in pixels from its centre.
ImVec2 project(const Camera& camera, Vec3 v, float scale, ImVec2 centre) {
    return {centre.x + static_cast<float>(dot(v, camera.right())) * scale,
            centre.y - static_cast<float>(dot(v, camera.up())) * scale};
}

} // namespace

Vec3 viewCubeZoneAt(const Camera& camera, Vec2 offsetFromCentrePx, float sizePx,
                    float bandFraction) {
    const float scale = sizePx * 0.5f / kCubeFit;
    if (scale <= 0.0f) return {};

    // The widget is an orthographic view along the camera's own axis, so a
    // pixel names a line through the cube rather than a point: everything with
    // these two components, at any depth.
    const float a = static_cast<float>(offsetFromCentrePx.x) / scale;
    const float b = -static_cast<float>(offsetFromCentrePx.y) / scale;
    const Vec3 dir = camera.forward();
    const Vec3 origin = camera.right() * a + camera.up() * b - dir * 4.0f;

    // Slabs, against the cube itself.
    float t0 = -1e30f, t1 = 1e30f;
    const float o[3] = {static_cast<float>(origin.x), static_cast<float>(origin.y),
                        static_cast<float>(origin.z)};
    const float d[3] = {static_cast<float>(dir.x), static_cast<float>(dir.y),
                        static_cast<float>(dir.z)};
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(d[i]) < 1e-9f) {
            if (o[i] < -1.0f || o[i] > 1.0f) return {};
            continue;
        }
        float lo = (-1.0f - o[i]) / d[i];
        float hi = (1.0f - o[i]) / d[i];
        if (lo > hi) std::swap(lo, hi);
        t0 = std::max(t0, lo);
        t1 = std::min(t1, hi);
        if (t0 > t1) return {};
    }

    // The near face is the one being looked at, so the entry point is the hit.
    const float band = 1.0f - clampf(bandFraction, 0.05f, 0.49f);
    Real z[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i) {
        const float hit = o[i] + d[i] * t0;
        z[i] = std::fabs(hit) >= band ? (hit > 0.0f ? 1.0 : -1.0) : 0.0;
    }
    const Vec3 out{z[0], z[1], z[2]};
    if (lengthSq(out) < 0.5) return {};      // the entry face always contributes one
    return normalize(out);
}

std::string viewCubeZoneName(Vec3 zone) {
    // Named the way the views themselves are, and in the order a person says
    // them: the face you are mostly looking at first.
    std::string s;
    auto add = [&](const char* word) {
        if (!s.empty()) s += ' ';
        s += word;
    };
    if (zone.y < -0.3f) add("Front");
    if (zone.y >  0.3f) add("Back");
    if (zone.z >  0.3f) add("Top");
    if (zone.z < -0.3f) add("Bottom");
    if (zone.x >  0.3f) add("Right");
    if (zone.x < -0.3f) add("Left");
    return s;
}

void drawViewCube(UiContext& ctx, float x, float y, float w, float h,
                  const ViewCubeStyle& style) {
    if (!ctx.camera) return;
    Camera& camera = *ctx.camera;
    (void)h;

    const float box = style.sizePx;

    // Pinned by its top-right corner rather than its top-left, so the widget
    // stays in the corner whatever it ends up measuring.
    ImGui::SetNextWindowPos(ImVec2(x + w - style.marginPx, y + style.marginPx),
                            ImGuiCond_Always, ImVec2(1.0f, 0.0f));
    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking |
        ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoBringToFrontOnFocus |
        ImGuiWindowFlags_NoBackground;

    bool over = false;      // only to decide what this widget itself draws
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    if (ImGui::Begin("##viewcube", nullptr, flags)) {
        ImGui::InvisibleButton("cube", ImVec2(box, box));
        const ImVec2 topLeft = ImGui::GetItemRectMin();
        const ImVec2 centre{topLeft.x + box * 0.5f, topLeft.y + box * 0.5f};
        over = ImGui::IsItemHovered();

        // A press that moves is an orbit; a press that does not is a view.
        // Deciding on release rather than on press is what lets one gesture be
        // both, without the cube twitching before it knows which. There is one
        // view cube, so one flag is enough to remember which it turned out to
        // be -- ImGui's drag threshold decides, so a shaky click still counts
        // as a click.
        static bool dragged = false;
        if (ImGui::IsItemActivated()) dragged = false;
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            const ImVec2 d = ImGui::GetIO().MouseDelta;
            if (d.x != 0.0f || d.y != 0.0f) {
                dragged = true;
                camera.orbit(d.x, d.y);
            }
        }

        const ImVec2 mouse = ImGui::GetIO().MousePos;
        const Vec3 hovered =
            over && !ImGui::IsItemActive()
                ? viewCubeZoneAt(camera, Vec2{mouse.x - centre.x, mouse.y - centre.y}, box,
                                 style.bandFraction)
                : Vec3{};

        if (ImGui::IsItemDeactivated() && !dragged && lengthSq(hovered) > 0.5f)
            camera.setViewDirection(hovered);

        // ---- the cube ------------------------------------------------------
        const float scale = box * 0.5f / kCubeFit;
        const float band = 1.0f - clampf(style.bandFraction, 0.05f, 0.49f);
        ImDrawList* dl = ImGui::GetWindowDrawList();

        const ImU32 faceCol   = toU32(palette::kRaised, 0.95f);
        const ImU32 faceLit   = toU32(palette::kHover, 0.97f);
        const ImU32 hoverCol  = toU32(palette::kBrand, 0.90f);
        const ImU32 lineCol   = toU32(palette::kBorder, 1.0f);
        const ImU32 textCol   = toU32(palette::kText, 0.92f);

        for (Vec3 n : kFaces) {
            const float facing = static_cast<float>(dot(n, camera.forward()));
            if (facing > -0.02f) continue;                 // turned away from the eye

            Vec3 a{}, b{};
            faceAxes(n, a, b);

            // Nine cells, so the bands that belong to the edges and corners are
            // the ones drawn rather than an invisible overlay on a plain face.
            const float edges[4] = {-1.0f, -band, band, 1.0f};
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    const int si = i - 1, sj = j - 1;
                    Vec3 zone = n + a * static_cast<float>(si) + b * static_cast<float>(sj);
                    zone = normalize(zone);

                    ImVec2 quad[4];
                    const float u0 = edges[i], u1 = edges[i + 1];
                    const float v0 = edges[j], v1 = edges[j + 1];
                    quad[0] = project(camera, n + a * u0 + b * v0, scale, centre);
                    quad[1] = project(camera, n + a * u1 + b * v0, scale, centre);
                    quad[2] = project(camera, n + a * u1 + b * v1, scale, centre);
                    quad[3] = project(camera, n + a * u0 + b * v1, scale, centre);

                    const bool lit = lengthSq(hovered) > 0.5f &&
                                     dot(zone, hovered) > 0.999f;
                    // Faces turned towards the eye are lighter, so the cube
                    // reads as a solid rather than as three flat panels.
                    const ImU32 fill = lit ? hoverCol
                                           : (-facing > 0.8f ? faceLit : faceCol);
                    dl->AddConvexPolyFilled(quad, 4, fill);
                }
            }

            // The face outline last, over its own cells.
            ImVec2 outline[4] = {
                project(camera, n - a - b, scale, centre),
                project(camera, n + a - b, scale, centre),
                project(camera, n + a + b, scale, centre),
                project(camera, n - a + b, scale, centre)};
            dl->AddPolyline(outline, 4, lineCol, ImDrawFlags_Closed, 1.4f);

            // The label, if there is room for it to be read. Not rotated with
            // the face: upright text is legible at every orientation, and a
            // cube that spins its lettering is harder to read than one that
            // does not.
            const ImVec2 c = project(camera, n, scale, centre);
            const float span = std::min(
                std::hypot(outline[1].x - outline[0].x, outline[1].y - outline[0].y),
                std::hypot(outline[2].x - outline[1].x, outline[2].y - outline[1].y));
            const char* label = faceLabel(n);
            if (span > 34.0f) {
                const ImVec2 ts = ImGui::CalcTextSize(label);
                dl->AddText(ImVec2(c.x - ts.x * 0.5f, c.y - ts.y * 0.5f), textCol, label);
            }
        }

        // Where a click would take you, said in words. On the widget itself it
        // would have to be a name long enough to push the cube out of the
        // corner -- "Front Bottom Right" is wider than the cube is -- and a
        // hover is when the question is being asked anyway.
        if (over && lengthSq(hovered) > 0.5f)
            ImGui::SetTooltip("%s", viewCubeZoneName(hovered).c_str());

        // ---- the one control that changes what is drawn --------------------
        const char* proj = camera.orthographic ? "Ortho" : "Persp";
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
        ImGui::PushStyleColor(ImGuiCol_Text,
                              camera.orthographic
                                  ? ImVec4(palette::kText.r, palette::kText.g, palette::kText.b, 0.75f)
                                  : ImVec4(palette::kBrand.r, palette::kBrand.g, palette::kBrand.b, 1.0f));
        const float tw = ImGui::CalcTextSize(proj).x + ImGui::GetStyle().FramePadding.x * 2.0f;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, (box - tw) * 0.5f));
        if (ImGui::Button(proj)) camera.toggleProjection();
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            over = true;
            ImGui::SetTooltip("%s", camera.orthographic
                                        ? "Orthographic. Click for perspective."
                                        : "Perspective. Click for orthographic.");
        }
    }
    ImGui::End();
    ImGui::PopStyleVar(2);
}

} // namespace tg
