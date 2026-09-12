#include "icon_raster.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <string>

namespace tg::icons {
namespace {

struct View {
    Vec3 right, up, forward;   // orthonormal, forward points from the eye into the scene
    Vec3 centre;
    Real scale = 1.0;          // world units per half-frame
};

View fitView(const Body& body, Vec3 eye, Real margin) {
    View v;
    v.forward = normalize(-eye);
    Vec3 worldUp{0, 0, 1};
    if (std::fabs(dot(v.forward, worldUp)) > 0.99) worldUp = {0, 1, 0};
    v.right = normalize(cross(v.forward, worldUp));
    v.up = normalize(cross(v.right, v.forward));

    // Fit the body's own corners, not its bounding sphere: a box seen corner-on
    // would otherwise sit in the middle of a lot of empty frame.
    const AABB b = body.bounds();
    v.centre = (b.min + b.max) * 0.5;
    Real half = 1e-6;
    for (int i = 0; i < 8; ++i) {
        const Vec3 p{i & 1 ? b.max.x : b.min.x, i & 2 ? b.max.y : b.min.y,
                     i & 4 ? b.max.z : b.min.z};
        const Vec3 d = p - v.centre;
        half = std::max(half, std::fabs(dot(d, v.right)));
        half = std::max(half, std::fabs(dot(d, v.up)));
    }
    v.scale = half / (1.0 - margin);
    return v;
}

struct Projected {
    Real x = 0, y = 0, depth = 0;
};

Projected project(const View& v, Vec3 p, int size) {
    const Vec3 d = p - v.centre;
    Projected out;
    out.x = (dot(d, v.right) / v.scale * 0.5 + 0.5) * size;
    out.y = (0.5 - dot(d, v.up) / v.scale * 0.5) * size;
    out.depth = dot(d, v.forward);
    return out;
}

Rgba mix(Rgba c, Real k) {
    auto ch = [&](uint8_t x) {
        return static_cast<uint8_t>(std::clamp(std::lround(x * k), 0L, 255L));
    };
    return {ch(c.r), ch(c.g), ch(c.b), c.a};
}

Real edgeFn(Real ax, Real ay, Real bx, Real by, Real px, Real py) {
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
}

// A line, as a capsule, tested per pixel. Slow and obviously correct, which is
// the right trade for something that runs once at build time.
void strokeLine(Image& img, std::vector<Real>& depth, const View& v, int size,
                Vec3 a, Vec3 b, Rgba colour, Real widthPx) {
    const Projected pa = project(v, a, size);
    const Projected pb = project(v, b, size);

    const Real minX = std::min(pa.x, pb.x) - widthPx - 1;
    const Real maxX = std::max(pa.x, pb.x) + widthPx + 1;
    const Real minY = std::min(pa.y, pb.y) - widthPx - 1;
    const Real maxY = std::max(pa.y, pb.y) + widthPx + 1;

    const Real dx = pb.x - pa.x, dy = pb.y - pa.y;
    const Real len2 = dx * dx + dy * dy;
    const Real half = widthPx * 0.5;

    for (int y = std::max(0, (int)std::floor(minY)); y <= std::min(size - 1, (int)std::ceil(maxY)); ++y) {
        for (int x = std::max(0, (int)std::floor(minX)); x <= std::min(size - 1, (int)std::ceil(maxX)); ++x) {
            const Real px = x + 0.5, py = y + 0.5;
            const Real t = len2 > 1e-9 ? std::clamp(((px - pa.x) * dx + (py - pa.y) * dy) / len2, Real(0), Real(1)) : Real(0);
            const Real cx = pa.x + dx * t, cy = pa.y + dy * t;
            const Real dist = std::hypot(px - cx, py - cy);
            if (dist > half) continue;

            // Just in front of the surface, so an edge on the far side of the
            // body stays hidden but its own face does not swallow it.
            const Real at = pa.depth + (pb.depth - pa.depth) * t - 1e-3;
            const size_t idx = static_cast<size_t>(y) * size + x;
            if (at > depth[idx] + 1e-4) continue;
            img.pixels[idx] = colour;
        }
    }
}

}  // namespace

Image render(const Subject& subject, int size, const Style& style, int supersample) {
    Image out;
    out.width = out.height = size;
    out.pixels.assign(static_cast<size_t>(size) * size, Rgba{});
    if (!subject.body || subject.body->empty()) return out;

    const int big = size * supersample;
    Image hi;
    hi.width = hi.height = big;
    hi.pixels.assign(static_cast<size_t>(big) * big, Rgba{});
    std::vector<Real> depth(static_cast<size_t>(big) * big, 1e30);

    const Body& body = *subject.body;
    const View view = fitView(body, subject.eye, style.margin);

    RenderMesh mesh;
    TessellationQuality q;
    // Finer than the screen ever needs: this runs once at build time, and a
    // curve that is a hair short at 256 pixels is visible once it is boxed down.
    q.deviationMm = length(body.bounds().size()) / 4000.0;
    q.angleRad = 0.12;
    q.independent = true;
    body.tessellate(mesh, q);

    const std::set<FaceId> accent(subject.accentFaces.begin(), subject.accentFaces.end());

    for (size_t t = 0; t * 3 + 2 < mesh.triangles.size(); ++t) {
        const Vec3 a = mesh.positions[mesh.triangles[t * 3 + 0]];
        const Vec3 b = mesh.positions[mesh.triangles[t * 3 + 1]];
        const Vec3 c = mesh.positions[mesh.triangles[t * 3 + 2]];

        Vec3 n = cross(b - a, c - a);
        if (lengthSq(n) < 1e-18) continue;
        n = normalize(n);
        if (dot(n, view.forward) > 0.0) continue;          // back faces

        // Lit in view space, so the shading does not change when the icon's
        // subject is oriented differently.
        const Vec3 lightWorld = normalize(view.right * style.light.x + view.up * style.light.y -
                                          view.forward * style.light.z);
        const Real lambert = std::max(Real(0), dot(n, lightWorld));
        const Real k = style.ambient + (1.0 - style.ambient) * lambert;

        const bool isAccent =
            t < mesh.triangleFace.size() && accent.count(mesh.triangleFace[t]) > 0;
        const Rgba colour = mix(isAccent ? style.accent : style.solid, k);

        const Projected pa = project(view, a, big);
        const Projected pb = project(view, b, big);
        const Projected pc = project(view, c, big);

        const int minX = std::max(0, (int)std::floor(std::min({pa.x, pb.x, pc.x})));
        const int maxX = std::min(big - 1, (int)std::ceil(std::max({pa.x, pb.x, pc.x})));
        const int minY = std::max(0, (int)std::floor(std::min({pa.y, pb.y, pc.y})));
        const int maxY = std::min(big - 1, (int)std::ceil(std::max({pa.y, pb.y, pc.y})));

        const Real area = edgeFn(pa.x, pa.y, pb.x, pb.y, pc.x, pc.y);
        if (std::fabs(area) < 1e-9) continue;

        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                const Real px = x + 0.5, py = y + 0.5;
                Real w0 = edgeFn(pb.x, pb.y, pc.x, pc.y, px, py) / area;
                Real w1 = edgeFn(pc.x, pc.y, pa.x, pa.y, px, py) / area;
                Real w2 = edgeFn(pa.x, pa.y, pb.x, pb.y, px, py) / area;
                if (w0 < 0 || w1 < 0 || w2 < 0) continue;

                const Real at = w0 * pa.depth + w1 * pb.depth + w2 * pc.depth;
                const size_t idx = static_cast<size_t>(y) * big + x;
                if (at >= depth[idx]) continue;
                depth[idx] = at;
                hi.pixels[idx] = colour;
            }
        }
    }

    // The silhouette, found from the tessellation: an edge with a triangle
    // facing the eye on one side and away on the other is where the body turns
    // out of view. A box gets its outline from its own edges, but a sphere has
    // no edges at all -- and without this it renders as a soft grey ball beside
    // a set of crisp ones, which reads as a different icon set.
    {
        std::map<std::pair<int, int>, int> seen;   // welded edge -> facing count
        std::map<std::pair<int, int>, std::pair<Vec3, Vec3>> ends;
        std::map<std::string, int> weld;
        std::vector<int> id(mesh.positions.size(), -1);
        for (size_t i = 0; i < mesh.positions.size(); ++i) {
            const Vec3 p = mesh.positions[i];
            char key[96];
            std::snprintf(key, sizeof key, "%lld,%lld,%lld",
                          (long long)std::llround(p.x * 2048.0),
                          (long long)std::llround(p.y * 2048.0),
                          (long long)std::llround(p.z * 2048.0));
            const auto [it, fresh] = weld.emplace(key, (int)weld.size());
            id[i] = it->second;
        }

        for (size_t t = 0; t * 3 + 2 < mesh.triangles.size(); ++t) {
            const Vec3 a = mesh.positions[mesh.triangles[t * 3 + 0]];
            const Vec3 b = mesh.positions[mesh.triangles[t * 3 + 1]];
            const Vec3 c = mesh.positions[mesh.triangles[t * 3 + 2]];
            Vec3 nrm = cross(b - a, c - a);
            if (lengthSq(nrm) < 1e-18) continue;
            const bool towards = dot(normalize(nrm), view.forward) <= 0.0;
            for (int k = 0; k < 3; ++k) {
                const int u = id[mesh.triangles[t * 3 + k]];
                const int v = id[mesh.triangles[t * 3 + (k + 1) % 3]];
                if (u == v) continue;
                const auto key = std::make_pair(std::min(u, v), std::max(u, v));
                seen[key] += towards ? 1 : -1;
                ends[key] = {mesh.positions[mesh.triangles[t * 3 + k]],
                             mesh.positions[mesh.triangles[t * 3 + (k + 1) % 3]]};
            }
        }
        for (const auto& [key, balance] : seen) {
            // Zero means one triangle each way: the body turns here.
            if (balance != 0) continue;
            const auto& e = ends.at(key);
            strokeLine(hi, depth, view, big, e.first, e.second, style.outline,
                       style.outlineWidthPx * supersample * 0.5);
        }
    }

    // Then the model's own edges. A box is carried by these; a sphere has none,
    // which is what the pass above is for.
    for (size_t i = 0; i + 1 < mesh.edgeLines.size(); i += 2)
        strokeLine(hi, depth, view, big, mesh.positions[mesh.edgeLines[i]],
                   mesh.positions[mesh.edgeLines[i + 1]], style.outline,
                   style.outlineWidthPx * supersample * 0.5);

    // Box down. Averaging in straight alpha would darken the fringe against the
    // transparent background, so the colour is weighted by coverage.
    const Real n = static_cast<Real>(supersample * supersample);
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            Real r = 0, g = 0, b = 0, a = 0;
            for (int sy = 0; sy < supersample; ++sy) {
                for (int sx = 0; sx < supersample; ++sx) {
                    const Rgba p = hi.at(x * supersample + sx, y * supersample + sy);
                    const Real w = p.a / 255.0;
                    r += p.r * w; g += p.g * w; b += p.b * w; a += w;
                }
            }
            Rgba& o = out.at(x, y);
            if (a > 1e-9) {
                o.r = static_cast<uint8_t>(std::clamp(std::lround(r / a), 0L, 255L));
                o.g = static_cast<uint8_t>(std::clamp(std::lround(g / a), 0L, 255L));
                o.b = static_cast<uint8_t>(std::clamp(std::lround(b / a), 0L, 255L));
                o.a = static_cast<uint8_t>(std::clamp(std::lround(a / n * 255.0), 0L, 255L));
            }
        }
    }
    return out;
}

// ---- PNG -------------------------------------------------------------------
namespace {

uint32_t crc32Of(const uint8_t* data, size_t n, uint32_t crc = 0xFFFFFFFFu) {
    static uint32_t table[256];
    static bool built = false;
    if (!built) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            table[i] = c;
        }
        built = true;
    }
    for (size_t i = 0; i < n; ++i) crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return crc;
}

void be32(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back(static_cast<uint8_t>(v >> 24));
    out.push_back(static_cast<uint8_t>(v >> 16));
    out.push_back(static_cast<uint8_t>(v >> 8));
    out.push_back(static_cast<uint8_t>(v));
}

void chunk(std::vector<uint8_t>& out, const char tag[5], const std::vector<uint8_t>& body) {
    be32(out, static_cast<uint32_t>(body.size()));
    const size_t start = out.size();
    out.insert(out.end(), tag, tag + 4);
    out.insert(out.end(), body.begin(), body.end());
    be32(out, crc32Of(out.data() + start, out.size() - start) ^ 0xFFFFFFFFu);
}

}  // namespace

bool writePng(const Image& image, const std::string& path) {
    // Raw scanlines, each with a zero filter byte.
    std::vector<uint8_t> raw;
    raw.reserve(static_cast<size_t>(image.height) * (image.width * 4 + 1));
    for (int y = 0; y < image.height; ++y) {
        raw.push_back(0);
        for (int x = 0; x < image.width; ++x) {
            const Rgba p = image.at(x, y);
            raw.push_back(p.r); raw.push_back(p.g); raw.push_back(p.b); raw.push_back(p.a);
        }
    }

    // zlib around stored deflate blocks: no compressor needed, and an icon is a
    // few kilobytes either way.
    std::vector<uint8_t> z;
    z.push_back(0x78); z.push_back(0x01);
    size_t at = 0;
    while (at < raw.size()) {
        const size_t n = std::min<size_t>(65535, raw.size() - at);
        z.push_back(at + n >= raw.size() ? 1 : 0);
        z.push_back(static_cast<uint8_t>(n & 0xFF));
        z.push_back(static_cast<uint8_t>(n >> 8));
        z.push_back(static_cast<uint8_t>(~n & 0xFF));
        z.push_back(static_cast<uint8_t>((~n >> 8) & 0xFF));
        z.insert(z.end(), raw.begin() + at, raw.begin() + at + n);
        at += n;
    }
    uint32_t s1 = 1, s2 = 0;
    for (uint8_t byte : raw) { s1 = (s1 + byte) % 65521; s2 = (s2 + s1) % 65521; }
    be32(z, (s2 << 16) | s1);

    std::vector<uint8_t> png = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    std::vector<uint8_t> ihdr;
    be32(ihdr, static_cast<uint32_t>(image.width));
    be32(ihdr, static_cast<uint32_t>(image.height));
    ihdr.push_back(8);      // bit depth
    ihdr.push_back(6);      // RGBA
    ihdr.push_back(0); ihdr.push_back(0); ihdr.push_back(0);
    chunk(png, "IHDR", ihdr);
    chunk(png, "IDAT", z);
    chunk(png, "IEND", {});

    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const bool ok = std::fwrite(png.data(), 1, png.size(), f) == png.size();
    std::fclose(f);
    return ok;
}

} // namespace tg::icons
