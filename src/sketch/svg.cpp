#include "sketch/svg.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <unordered_map>

namespace tg {

namespace {

// ---- XML ---------------------------------------------------------------------
//
// Elements, attributes and nesting; comments, processing instructions, the
// doctype and CDATA skipped. Character data is not kept: no part of a drawing's
// geometry lives in it.

struct XmlNode {
    std::string name;   // without a namespace prefix: "svg:path" is "path"
    std::vector<std::pair<std::string, std::string>> attrs;
    std::vector<std::unique_ptr<XmlNode>> children;

    const std::string* attr(const char* key) const {
        for (const auto& [k, v] : attrs)
            if (k == key) return &v;
        return nullptr;
    }
};

std::string localName(const std::string& n) {
    const size_t colon = n.find(':');
    return colon == std::string::npos ? n : n.substr(colon + 1);
}

std::string decodeEntities(const std::string& s) {
    if (s.find('&') == std::string::npos) return s;
    std::string out;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] != '&') { out += s[i]; continue; }
        const size_t semi = s.find(';', i);
        if (semi == std::string::npos) { out += s[i]; continue; }
        const std::string e = s.substr(i + 1, semi - i - 1);
        if (e == "amp") out += '&';
        else if (e == "lt") out += '<';
        else if (e == "gt") out += '>';
        else if (e == "quot") out += '"';
        else if (e == "apos") out += '\'';
        else if (!e.empty() && e[0] == '#') {
            const long c = e.size() > 1 && (e[1] == 'x' || e[1] == 'X')
                               ? std::strtol(e.c_str() + 2, nullptr, 16)
                               : std::strtol(e.c_str() + 1, nullptr, 10);
            if (c > 0 && c < 128) out += static_cast<char>(c);   // nothing geometric is beyond ASCII
        } else {
            out += s.substr(i, semi - i + 1);
        }
        i = semi;
    }
    return out;
}

class XmlReader {
public:
    explicit XmlReader(const std::string& text) : s_(text) {}

    std::unique_ptr<XmlNode> root(std::string& error) {
        auto doc = std::make_unique<XmlNode>();
        std::vector<XmlNode*> open{doc.get()};
        while (i_ < s_.size()) {
            const size_t lt = s_.find('<', i_);
            if (lt == std::string::npos) break;
            i_ = lt;
            if (starts("<!--")) { skipPast("-->"); continue; }
            if (starts("<![CDATA[")) { skipPast("]]>"); continue; }
            if (starts("<?")) { skipPast("?>"); continue; }
            if (starts("<!")) { skipDoctype(); continue; }
            if (starts("</")) {
                const size_t gt = s_.find('>', i_);
                if (gt == std::string::npos) { error = "the file ends inside a tag"; return nullptr; }
                i_ = gt + 1;
                if (open.size() > 1) open.pop_back();
                continue;
            }
            ++i_;   // '<'
            auto node = std::make_unique<XmlNode>();
            node->name = localName(name());
            if (node->name.empty()) { error = "a tag has no name"; return nullptr; }
            bool selfClosing = false;
            for (;;) {
                spaces();
                if (i_ >= s_.size()) { error = "the file ends inside a tag"; return nullptr; }
                if (s_[i_] == '>') { ++i_; break; }
                if (starts("/>")) { i_ += 2; selfClosing = true; break; }
                std::string key = name();
                if (key.empty()) { ++i_; continue; }   // stray character: step over it
                spaces();
                std::string value;
                if (i_ < s_.size() && s_[i_] == '=') {
                    ++i_;
                    spaces();
                    if (i_ < s_.size() && (s_[i_] == '"' || s_[i_] == '\'')) {
                        const char q = s_[i_++];
                        const size_t end = s_.find(q, i_);
                        if (end == std::string::npos) { error = "an attribute is never closed"; return nullptr; }
                        value = decodeEntities(s_.substr(i_, end - i_));
                        i_ = end + 1;
                    } else {
                        value = name();
                    }
                }
                // xlink:href and href mean the same; everything else by its
                // local name too, so an Inkscape prefix does not hide it.
                node->attrs.emplace_back(localName(key), std::move(value));
            }
            XmlNode* raw = node.get();
            open.back()->children.push_back(std::move(node));
            if (!selfClosing) open.push_back(raw);
        }
        for (auto& c : doc->children)
            if (c->name == "svg") return std::move(c);
        error = "there is no <svg> element in it";
        return nullptr;
    }

private:
    const std::string& s_;
    size_t i_ = 0;

    bool starts(const char* p) const { return s_.compare(i_, std::strlen(p), p) == 0; }
    void skipPast(const char* p) {
        const size_t at = s_.find(p, i_);
        i_ = at == std::string::npos ? s_.size() : at + std::strlen(p);
    }
    void skipDoctype() {
        int depth = 0;
        for (; i_ < s_.size(); ++i_) {
            if (s_[i_] == '[') ++depth;
            else if (s_[i_] == ']') --depth;
            else if (s_[i_] == '>' && depth <= 0) { ++i_; return; }
        }
    }
    void spaces() {
        while (i_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[i_]))) ++i_;
    }
    std::string name() {
        const size_t start = i_;
        while (i_ < s_.size()) {
            const char c = s_[i_];
            if (std::isspace(static_cast<unsigned char>(c)) || c == '=' || c == '>' || c == '/' ||
                c == '"' || c == '\'' || c == '<')
                break;
            ++i_;
        }
        return s_.substr(start, i_ - start);
    }
};

// ---- Numbers, lengths and transforms --------------------------------------------

// Scans SVG number lists, where "1.5.5" is two numbers, "-1-2" is two, and a
// flag in an arc may be written with nothing after it: "a1 1 0 0010 10".
class Numbers {
public:
    explicit Numbers(const std::string& s) : s_(s) {}

    void separators() {
        while (i_ < s_.size() && (std::isspace(static_cast<unsigned char>(s_[i_])) || s_[i_] == ','))
            ++i_;
    }
    bool atEnd() { separators(); return i_ >= s_.size(); }
    char peek() { separators(); return i_ < s_.size() ? s_[i_] : '\0'; }
    char take() { separators(); return i_ < s_.size() ? s_[i_++] : '\0'; }
    bool startsNumber() {
        const char c = peek();
        return std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+' || c == '.';
    }

    bool number(Real& out) {
        separators();
        const size_t start = i_;
        if (i_ < s_.size() && (s_[i_] == '-' || s_[i_] == '+')) ++i_;
        bool digits = false, dot = false;
        while (i_ < s_.size()) {
            const char c = s_[i_];
            if (std::isdigit(static_cast<unsigned char>(c))) { digits = true; ++i_; }
            else if (c == '.' && !dot) { dot = true; ++i_; }
            else break;
        }
        if (!digits) { i_ = start; return false; }
        if (i_ < s_.size() && (s_[i_] == 'e' || s_[i_] == 'E')) {
            size_t j = i_ + 1;
            if (j < s_.size() && (s_[j] == '-' || s_[j] == '+')) ++j;
            if (j < s_.size() && std::isdigit(static_cast<unsigned char>(s_[j]))) {
                i_ = j;
                while (i_ < s_.size() && std::isdigit(static_cast<unsigned char>(s_[i_]))) ++i_;
            }
        }
        out = std::strtod(s_.substr(start, i_ - start).c_str(), nullptr);
        return std::isfinite(out);
    }

    bool flag(bool& out) {
        separators();
        if (i_ < s_.size() && (s_[i_] == '0' || s_[i_] == '1')) {
            out = s_[i_++] == '1';
            return true;
        }
        return false;
    }

    // What follows a number in a length: "mm", "px", "%".
    std::string unit() {
        const size_t start = i_;
        while (i_ < s_.size() && (std::isalpha(static_cast<unsigned char>(s_[i_])) || s_[i_] == '%')) ++i_;
        return s_.substr(start, i_ - start);
    }

private:
    const std::string& s_;
    size_t i_ = 0;
};

constexpr Real kMmPerPx = 25.4 / 96.0;

// A length in user units -- px -- or in millimetres when `mm` is asked for.
// Percentages and font-relative units have nothing to be relative to here and
// read as absent.
bool readLength(const std::string* text, Real& out, bool mm = false) {
    if (!text) return false;
    Numbers n(*text);
    Real v = 0;
    if (!n.number(v)) return false;
    const std::string u = n.unit();
    Real px = 0;
    if (u.empty() || u == "px") px = v;
    else if (u == "mm") px = v / kMmPerPx;
    else if (u == "cm") px = v * 10.0 / kMmPerPx;
    else if (u == "in") px = v * 96.0;
    else if (u == "pt") px = v * 96.0 / 72.0;
    else if (u == "pc") px = v * 16.0;
    else return false;
    out = mm ? px * kMmPerPx : px;
    return true;
}

Real lengthOr(const XmlNode& n, const char* key, Real fallback) {
    Real v = fallback;
    return readLength(n.attr(key), v) ? v : fallback;
}

// x' = a x + c y + e, y' = b x + d y + f -- SVG's own order.
struct Affine {
    Real a = 1, b = 0, c = 0, d = 1, e = 0, f = 0;

    Vec2 operator()(Vec2 p) const { return {a * p.x + c * p.y + e, b * p.x + d * p.y + f}; }
    Vec2 linear(Vec2 v) const { return {a * v.x + c * v.y, b * v.x + d * v.y}; }

    // This, then `t` first: (this * t)(p) = this(t(p)).
    Affine operator*(const Affine& t) const {
        return {a * t.a + c * t.b, b * t.a + d * t.b, a * t.c + c * t.d, b * t.c + d * t.d,
                a * t.e + c * t.f + e, b * t.e + d * t.f + f};
    }

    // Whether circles stay circles under it, and by how much they grow.
    bool similarity(Real& factor) const {
        const Real sx = std::hypot(a, b), sy = std::hypot(c, d);
        const Real skew = a * c + b * d;
        if (sx < 1e-12 || std::fabs(sx - sy) > 1e-9 * sx || std::fabs(skew) > 1e-9 * sx * sx) return false;
        factor = sx;
        return true;
    }
    Real det() const { return a * d - b * c; }
};

Affine translate(Real x, Real y) { return {1, 0, 0, 1, x, y}; }
Affine scaling(Real x, Real y) { return {x, 0, 0, y, 0, 0}; }

Affine parseTransform(const std::string* text) {
    Affine out;
    if (!text) return out;
    const std::string& s = *text;
    size_t i = 0;
    while (i < s.size()) {
        while (i < s.size() && !std::isalpha(static_cast<unsigned char>(s[i]))) ++i;
        const size_t nameStart = i;
        while (i < s.size() && std::isalpha(static_cast<unsigned char>(s[i]))) ++i;
        const std::string fn = s.substr(nameStart, i - nameStart);
        const size_t open = s.find('(', i), close = s.find(')', i);
        if (fn.empty() || open == std::string::npos || close == std::string::npos || close < open) break;
        const std::string argText = s.substr(open + 1, close - open - 1);
        i = close + 1;
        std::vector<Real> v;
        Numbers n(argText);
        for (Real x; n.number(x);) v.push_back(x);

        Affine t;
        if (fn == "matrix" && v.size() == 6) {
            t = {v[0], v[1], v[2], v[3], v[4], v[5]};
        } else if (fn == "translate" && !v.empty()) {
            t = translate(v[0], v.size() > 1 ? v[1] : 0.0);
        } else if (fn == "scale" && !v.empty()) {
            t = scaling(v[0], v.size() > 1 ? v[1] : v[0]);
        } else if (fn == "rotate" && !v.empty()) {
            const Real r = v[0] * kDeg2Rad, cs = std::cos(r), sn = std::sin(r);
            t = {cs, sn, -sn, cs, 0, 0};
            if (v.size() >= 3) t = translate(v[1], v[2]) * t * translate(-v[1], -v[2]);
        } else if (fn == "skewX" && !v.empty()) {
            t = {1, 0, std::tan(v[0] * kDeg2Rad), 1, 0, 0};
        } else if (fn == "skewY" && !v.empty()) {
            t = {1, std::tan(v[0] * kDeg2Rad), 0, 1, 0, 0};
        } else {
            continue;
        }
        out = out * t;
    }
    return out;
}

// The part of `style="..."` that decides whether anything is drawn.
std::string styleValue(const XmlNode& n, const char* prop) {
    if (const std::string* st = n.attr("style")) {
        const std::string& s = *st;
        size_t i = 0;
        while (i < s.size()) {
            const size_t semi = std::min(s.find(';', i), s.size());
            const std::string decl = s.substr(i, semi - i);
            i = semi + 1;
            const size_t colon = decl.find(':');
            if (colon == std::string::npos) continue;
            auto trim = [](std::string t) {
                const size_t a = t.find_first_not_of(" \t\r\n");
                const size_t b = t.find_last_not_of(" \t\r\n");
                return a == std::string::npos ? std::string() : t.substr(a, b - a + 1);
            };
            if (trim(decl.substr(0, colon)) == prop) return trim(decl.substr(colon + 1));
        }
    }
    if (const std::string* v = n.attr(prop)) return *v;
    return {};
}

bool hidden(const XmlNode& n) {
    return styleValue(n, "display") == "none" || styleValue(n, "visibility") == "hidden" ||
           styleValue(n, "visibility") == "collapse";
}

// What a viewBox and a size make of user units, into the viewport the element
// establishes. `w` and `h` are that viewport's size in the parent's units.
Affine viewBoxTransform(const XmlNode& n, Real x, Real y, Real w, Real h) {
    const std::string* vbText = n.attr("viewBox");
    if (!vbText) return translate(x, y);
    std::vector<Real> vb;
    Numbers num(*vbText);
    for (Real v; num.number(v);) vb.push_back(v);
    if (vb.size() != 4 || vb[2] <= 0 || vb[3] <= 0) return translate(x, y);

    Real sx = w / vb[2], sy = h / vb[3];
    std::string par = n.attr("preserveAspectRatio") ? *n.attr("preserveAspectRatio") : "xMidYMid meet";
    Real tx = 0, ty = 0;
    if (par.find("none") == std::string::npos) {
        const bool slice = par.find("slice") != std::string::npos;
        const Real s = slice ? std::max(sx, sy) : std::min(sx, sy);
        sx = sy = s;
        const Real spareX = w - vb[2] * s, spareY = h - vb[3] * s;
        if (par.find("xMid") != std::string::npos) tx = spareX * 0.5;
        else if (par.find("xMax") != std::string::npos) tx = spareX;
        if (par.find("YMid") != std::string::npos) ty = spareY * 0.5;
        else if (par.find("YMax") != std::string::npos) ty = spareY;
    }
    return translate(x + tx, y + ty) * scaling(sx, sy) * translate(-vb[0], -vb[1]);
}

// ---- Geometry, as read ------------------------------------------------------------

// A CSS colour, as the few ways a drawing program writes one: #rgb, #rrggbb,
// rgb(), and the names people type. Anything else -- a gradient, a name not
// here -- is a colour of its own, mid-grey.
bool parseColour(const std::string& raw, std::string& css, Vec3& rgb) {
    std::string c;
    for (char ch : raw)
        if (!std::isspace(static_cast<unsigned char>(ch))) c += static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (c.empty() || c == "none" || c == "transparent") return false;
    auto hex = [](char h) { return std::isdigit(static_cast<unsigned char>(h)) ? h - '0' : h - 'a' + 10; };
    if (c[0] == '#' && (c.size() == 4 || c.size() == 7) &&
        std::all_of(c.begin() + 1, c.end(), [](char h) { return std::isxdigit(static_cast<unsigned char>(h)); })) {
        int v[3];
        for (int i = 0; i < 3; ++i)
            v[i] = c.size() == 4 ? hex(c[1 + i]) * 17 : hex(c[1 + 2 * i]) * 16 + hex(c[2 + 2 * i]);
        char b[8];
        std::snprintf(b, sizeof b, "#%02x%02x%02x", v[0], v[1], v[2]);
        css = b;
        rgb = {v[0] / 255.0, v[1] / 255.0, v[2] / 255.0};
        return true;
    }
    if (c.rfind("rgb(", 0) == 0) {
        Numbers n(c.substr(4));
        Real v[3] = {0, 0, 0};
        for (int i = 0; i < 3; ++i) {
            if (!n.number(v[i])) break;
            if (n.peek() == '%') { n.take(); v[i] *= 2.55; }
        }
        char b[8];
        std::snprintf(b, sizeof b, "#%02x%02x%02x", static_cast<int>(std::clamp(v[0], 0.0, 255.0)),
                      static_cast<int>(std::clamp(v[1], 0.0, 255.0)), static_cast<int>(std::clamp(v[2], 0.0, 255.0)));
        return parseColour(b, css, rgb);
    }
    static const std::pair<const char*, const char*> kNames[] = {
        {"black", "#000000"}, {"white", "#ffffff"}, {"red", "#ff0000"}, {"green", "#008000"},
        {"blue", "#0000ff"}, {"yellow", "#ffff00"}, {"orange", "#ffa500"}, {"purple", "#800080"},
        {"gray", "#808080"}, {"grey", "#808080"}, {"silver", "#c0c0c0"}, {"lime", "#00ff00"},
        {"navy", "#000080"}, {"maroon", "#800000"}, {"teal", "#008080"}, {"olive", "#808000"},
        {"currentcolor", "#000000"}};
    for (const auto& [name, value] : kNames)
        if (c == name) return parseColour(value, css, rgb);
    css = c;
    rgb = {0.5, 0.5, 0.5};
    return true;
}

class Reader {
public:
    SvgDrawing out;

    void read(const XmlNode& root) {
        index(root);

        // The outermost viewport: its size, in millimetres, is the drawing's
        // real size. Without one, a user unit is a pixel at 96 to the inch.
        Real w = 0, h = 0;
        const bool hasW = readLength(root.attr("width"), w);
        const bool hasH = readLength(root.attr("height"), h);
        Affine page;
        if (root.attr("viewBox")) {
            std::vector<Real> vb;
            Numbers num(*root.attr("viewBox"));
            for (Real v; num.number(v);) vb.push_back(v);
            if (vb.size() == 4 && vb[2] > 0 && vb[3] > 0) {
                if (!hasW && !hasH) { w = vb[2]; h = vb[3]; }
                else if (!hasW) w = h * vb[2] / vb[3];
                else if (!hasH) h = w * vb[3] / vb[2];
                page = viewBoxTransform(root, 0, 0, w, h);
            }
        }
        // Pixels to millimetres, and y turned the right way up.
        const Affine toMm = scaling(kMmPerPx, -kMmPerPx);
        if (w > 0 && h > 0) {
            havePage_ = true;
            pageLo_ = {0.0, -h * kMmPerPx};
            pageHi_ = {w * kMmPerPx, 0.0};
        }
        walkChildren(root, toMm * page, 0);

        // The paper behind the drawing is left out -- unless it is all there
        // is, in which case it is the drawing.
        if (background_) {
            if (out.empty()) out.paths.push_back(std::move(*background_));
            else ++out.background;
        }
    }

private:
    std::unordered_map<std::string, const XmlNode*> ids_;
    SvgFill paint_;
    int nextElement_ = 0;

    int colourOf(const std::string& css, Vec3 rgb) {
        for (size_t i = 0; i < out.colours.size(); ++i)
            if (out.colours[i].css == css) return static_cast<int>(i);
        SvgColour c;
        c.css = css;
        c.rgb = rgb;
        out.colours.push_back(c);
        return static_cast<int>(out.colours.size()) - 1;
    }
    // SVG's default fill, black, the first time something relies on it.
    SvgFill painted() {
        SvgFill f = paint_;
        if (f.filled && f.colour < 0) f.colour = colourOf("#000000", {0, 0, 0});
        if (f.filled) ++out.colours[static_cast<size_t>(f.colour)].shapes;
        return f;
    }
    bool havePage_ = false;
    Vec2 pageLo_, pageHi_;
    std::unique_ptr<SvgPath> background_;

    void index(const XmlNode& n) {
        if (const std::string* id = n.attr("id")) ids_.emplace(*id, &n);
        for (const auto& c : n.children) index(*c);
    }

    void walkChildren(const XmlNode& n, const Affine& t, int depth) {
        for (const auto& c : n.children) walk(*c, t, depth + 1);
    }

    void walk(const XmlNode& n, const Affine& parent, int depth) {
        if (depth > 64 || hidden(n)) return;
        const std::string& k = n.name;

        // Fill is inherited: set on a group, it is how everything in it is
        // filled unless something inside says otherwise. Put back on the way out.
        const SvgFill inherited = paint_;
        struct Restore {
            SvgFill& into;
            SvgFill was;
            ~Restore() { into = was; }
        } restore{paint_, inherited};
        {
            const std::string fill = styleValue(n, "fill");
            if (!fill.empty()) {
                std::string css;
                Vec3 rgb;
                paint_.filled = parseColour(fill, css, rgb);
                if (paint_.filled) paint_.colour = colourOf(css, rgb);
            }
            const std::string opacity = styleValue(n, "fill-opacity");
            if (!opacity.empty() && std::strtod(opacity.c_str(), nullptr) <= 0.0) paint_.filled = false;
            const std::string rule = styleValue(n, "fill-rule");
            if (rule == "evenodd") paint_.evenOdd = true;
            else if (rule == "nonzero") paint_.evenOdd = false;
            paint_.element = nextElement_++;
        }

        // Not drawn where they stand: definitions, and what only decorates.
        static const char* kSkip[] = {"defs", "clipPath", "mask", "marker", "pattern", "symbol",
                                      "metadata", "title", "desc", "style", "script",
                                      "linearGradient", "radialGradient", "filter",
                                      "foreignObject", "namedview", "switch"};
        for (const char* s : kSkip)
            if (k == s) return;
        if (k == "text") { ++out.text; return; }
        if (k == "image") { ++out.images; return; }

        const Affine t = parent * parseTransform(n.attr("transform"));

        if (k == "g" || k == "a") { walkChildren(n, t, depth); return; }
        if (k == "svg") {
            const Real x = lengthOr(n, "x", 0), y = lengthOr(n, "y", 0);
            const Real w = lengthOr(n, "width", 0), h = lengthOr(n, "height", 0);
            walkChildren(n, t * (w > 0 && h > 0 ? viewBoxTransform(n, x, y, w, h) : translate(x, y)), depth);
            return;
        }
        if (k == "use") {
            const std::string* href = n.attr("href");
            if (!href || href->size() < 2 || (*href)[0] != '#') return;
            auto it = ids_.find(href->substr(1));
            if (it == ids_.end()) return;
            const Affine at = t * translate(lengthOr(n, "x", 0), lengthOr(n, "y", 0));
            const XmlNode& ref = *it->second;
            if (ref.name == "symbol") {
                const Real w = lengthOr(n, "width", 0), h = lengthOr(n, "height", 0);
                walkChildren(ref, w > 0 && h > 0 ? at * viewBoxTransform(ref, 0, 0, w, h) : at, depth);
            } else {
                walk(ref, at, depth + 1);
            }
            return;
        }

        if (k == "path") {
            if (const std::string* d = n.attr("d")) path(*d, t);
        } else if (k == "rect") {
            rect(n, t);
        } else if (k == "circle") {
            ellipse(Vec2{lengthOr(n, "cx", 0), lengthOr(n, "cy", 0)}, lengthOr(n, "r", 0),
                    lengthOr(n, "r", 0), t);
        } else if (k == "ellipse") {
            Real rx = lengthOr(n, "rx", -1), ry = lengthOr(n, "ry", -1);
            if (rx < 0) rx = ry;
            if (ry < 0) ry = rx;
            ellipse(Vec2{lengthOr(n, "cx", 0), lengthOr(n, "cy", 0)}, rx, ry, t);
        } else if (k == "line") {
            SvgPath p;
            addLine(p, t({lengthOr(n, "x1", 0), lengthOr(n, "y1", 0)}),
                    t({lengthOr(n, "x2", 0), lengthOr(n, "y2", 0)}));
            keep(std::move(p));
        } else if (k == "polyline" || k == "polygon") {
            std::vector<Vec2> pts;
            if (const std::string* s = n.attr("points")) {
                Numbers num(*s);
                for (Real x, y; num.number(x) && num.number(y);) pts.push_back(t({x, y}));
            }
            SvgPath p;
            for (size_t i = 1; i < pts.size(); ++i) addLine(p, pts[i - 1], pts[i]);
            if (k == "polygon" && pts.size() > 2) {
                addLine(p, pts.back(), pts.front());
                p.closed = true;
            }
            keep(std::move(p));
        } else {
            // Something unknown: its children may still be drawings.
            walkChildren(n, t, depth);
        }
    }

    // ---- Pieces -------------------------------------------------------------------

    static bool same(Vec2 a, Vec2 b) { return lengthSq(a - b) < 1e-18; }

    static void addLine(SvgPath& p, Vec2 a, Vec2 b) {
        if (same(a, b)) return;
        SvgSegment s;
        s.kind = SvgSegment::Kind::Line;
        s.from = a;
        s.to = b;
        p.segments.push_back(s);
    }

    // A cubic, already in millimetres. One whose handles lie on the straight
    // line between its ends, inside it, is that line: drawing programs write
    // straight edges this way, and a line can be dimensioned where a curve
    // cannot.
    static void addCubic(SvgPath& p, Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
        const Vec2 chord = d - a;
        const Real len2 = lengthSq(chord);
        if (len2 < 1e-18 && same(a, b) && same(a, c)) return;
        if (len2 > 1e-18) {
            auto onChord = [&](Vec2 q) {
                const Vec2 r = q - a;
                const Real along = dot(r, chord) / len2;
                const Real off = std::fabs(r.x * chord.y - r.y * chord.x) / std::sqrt(len2);
                return off < 1e-7 * std::sqrt(len2) + 1e-9 && along >= -1e-9 && along <= 1 + 1e-9;
            };
            if (onChord(b) && onChord(c)) { addLine(p, a, d); return; }
        }
        SvgSegment s;
        s.kind = SvgSegment::Kind::Cubic;
        s.from = a;
        s.c1 = b;
        s.c2 = c;
        s.to = d;
        p.segments.push_back(s);
    }

    // An elliptical arc in user space: centre, radii, the ellipse's tilt, where
    // it starts and how far it sweeps. A circle under a transform that keeps
    // circles is kept as an arc; anything else becomes cubics, a quarter turn
    // or less each, which is how every drawing program draws an ellipse.
    static void addArc(SvgPath& p, const Affine& t, Vec2 c, Real rx, Real ry, Real phi, Real theta,
                       Real sweep) {
        const Real cp = std::cos(phi), sp = std::sin(phi);
        auto at = [&](Real a) {
            const Real x = rx * std::cos(a), y = ry * std::sin(a);
            return Vec2{c.x + cp * x - sp * y, c.y + sp * x + cp * y};
        };
        Real factor = 0;
        if (std::fabs(rx - ry) < 1e-9 * std::max(rx, ry) && t.similarity(factor) &&
            std::fabs(sweep) < kTwoPi - 1e-9) {
            SvgSegment s;
            s.kind = SvgSegment::Kind::Arc;
            s.from = t(at(theta));
            s.to = t(at(theta + sweep));
            s.centre = t(c);
            s.radius = rx * factor;
            // The turn it makes after the transform, which may mirror it --
            // and the page-to-sketch transform always does.
            s.ccw = (sweep > 0) == (t.det() > 0);
            if (same(s.from, s.to)) return;
            p.segments.push_back(s);
            return;
        }
        const int pieces = std::max(1, static_cast<int>(std::ceil(std::fabs(sweep) / (kPi * 0.5) - 1e-9)));
        const Real step = sweep / pieces;
        const Real k = 4.0 / 3.0 * std::tan(step / 4.0);
        for (int i = 0; i < pieces; ++i) {
            const Real a0 = theta + step * i, a1 = a0 + step;
            auto tangent = [&](Real a) {
                const Real x = -rx * std::sin(a), y = ry * std::cos(a);
                return Vec2{cp * x - sp * y, sp * x + cp * y};
            };
            const Vec2 p0 = at(a0), p3 = at(a1);
            addCubic(p, t(p0), t(p0 + tangent(a0) * k), t(p3 - tangent(a1) * k), t(p3));
        }
    }

    void keep(SvgPath p) {
        if (p.segments.empty()) return;
        p.fill = painted();
        // A path that comes back to where it began is closed, said or not;
        // and a filled one is filled as if it were, which is how a browser
        // draws a shape whose author left off the z.
        if (!p.closed && same(p.segments.front().from, p.segments.back().to)) p.closed = true;
        if (!p.closed && p.fill.filled) {
            Real twice = 0;
            for (const SvgSegment& sg : p.segments) twice += sg.from.x * sg.to.y - sg.to.x * sg.from.y;
            twice += p.segments.back().to.x * p.segments.front().from.y -
                     p.segments.front().from.x * p.segments.back().to.y;
            if (std::fabs(twice) > 1e-12) p.closed = true;
        }
        if (p.closed && !same(p.segments.front().from, p.segments.back().to))
            addLine(p, p.segments.back().to, p.segments.front().from);
        out.paths.push_back(std::move(p));
    }

    // ---- Elements -----------------------------------------------------------------

    void ellipse(Vec2 c, Real rx, Real ry, const Affine& t) {
        if (rx <= 0 || ry <= 0) return;
        Real factor = 0;
        if (std::fabs(rx - ry) < 1e-9 * rx && t.similarity(factor)) {
            out.circles.push_back({t(c), rx * factor, painted()});
            return;
        }
        SvgPath p;
        addArc(p, t, c, rx, ry, 0, 0, kPi);
        addArc(p, t, c, rx, ry, 0, kPi, kPi);
        p.closed = true;
        keep(std::move(p));
    }

    void rect(const XmlNode& n, const Affine& t) {
        const Real x = lengthOr(n, "x", 0), y = lengthOr(n, "y", 0);
        const Real w = lengthOr(n, "width", 0), h = lengthOr(n, "height", 0);
        if (w <= 0 || h <= 0) return;
        Real rx = lengthOr(n, "rx", -1), ry = lengthOr(n, "ry", -1);
        if (rx < 0) rx = ry;
        if (ry < 0) ry = rx;
        rx = std::clamp(rx, 0.0, w * 0.5);
        ry = std::clamp(ry, 0.0, h * 0.5);

        // Before anything else is drawn, and covering the whole page: the
        // paper, as a drawing program exports it.
        if (havePage_ && out.empty() && !background_) {
            Vec2 lo{1e300, 1e300}, hi{-1e300, -1e300};
            for (Vec2 c : {t({x, y}), t({x + w, y}), t({x + w, y + h}), t({x, y + h})}) {
                lo = {std::min(lo.x, c.x), std::min(lo.y, c.y)};
                hi = {std::max(hi.x, c.x), std::max(hi.y, c.y)};
            }
            const Vec2 page = pageHi_ - pageLo_;
            const Real slack = 0.01 * std::max(page.x, page.y);
            if (lo.x <= pageLo_.x + slack && lo.y <= pageLo_.y + slack && hi.x >= pageHi_.x - slack &&
                hi.y >= pageHi_.y - slack) {
                SvgPath paper;
                const Vec2 c[4] = {t({x, y}), t({x + w, y}), t({x + w, y + h}), t({x, y + h})};
                for (int i = 0; i < 4; ++i) addLine(paper, c[i], c[(i + 1) % 4]);
                paper.closed = true;
                background_ = std::make_unique<SvgPath>(std::move(paper));
                return;
            }
        }

        SvgPath p;
        if (rx <= 0 || ry <= 0) {
            const Vec2 c[4] = {t({x, y}), t({x + w, y}), t({x + w, y + h}), t({x, y + h})};
            for (int i = 0; i < 4; ++i) addLine(p, c[i], c[(i + 1) % 4]);
        } else {
            // Round the corners clockwise on the page, the way the rectangle
            // itself runs: top edge, top-right corner, and so on.
            const Real q = kPi * 0.5;
            addLine(p, t({x + rx, y}), t({x + w - rx, y}));
            addArc(p, t, {x + w - rx, y + ry}, rx, ry, 0, -q, q);
            addLine(p, t({x + w, y + ry}), t({x + w, y + h - ry}));
            addArc(p, t, {x + w - rx, y + h - ry}, rx, ry, 0, 0, q);
            addLine(p, t({x + w - rx, y + h}), t({x + rx, y + h}));
            addArc(p, t, {x + rx, y + h - ry}, rx, ry, 0, q, q);
            addLine(p, t({x, y + h - ry}), t({x, y + ry}));
            addArc(p, t, {x + rx, y + ry}, rx, ry, 0, 2 * q, q);
        }
        p.closed = true;
        keep(std::move(p));
    }

    // The path mini-language. Absolute and relative, every command, implicit
    // repeats; a subpath ends at the next moveto. Data that stops making sense
    // keeps what came before it, as the SVG specification asks a renderer to.
    void path(const std::string& d, const Affine& t) {
        Numbers n(d);
        Vec2 cur{0, 0}, start{0, 0};
        Vec2 lastCtrl{0, 0};       // for S and T: the handle to reflect
        char prev = 0;
        SvgPath p;
        char cmd = 0;
        bool broken = false;

        auto finish = [&](bool closed) {
            if (!p.segments.empty()) {
                p.closed = closed;
                keep(std::move(p));
            }
            p = SvgPath{};
        };

        while (!n.atEnd()) {
            if (std::isalpha(static_cast<unsigned char>(n.peek()))) {
                cmd = n.take();
            } else if (cmd == 0) {
                broken = true;
                break;
            }
            // A number where a command was expected repeats the last one --
            // except after a moveto, where it is a lineto.
            const bool rel = std::islower(static_cast<unsigned char>(cmd));
            const char up = static_cast<char>(std::toupper(static_cast<unsigned char>(cmd)));
            const Vec2 base = rel ? cur : Vec2{0, 0};
            auto pt = [&](Vec2& out) {
                Real x, y;
                if (!n.number(x) || !n.number(y)) return false;
                out = base + Vec2{x, y};
                return true;
            };

            bool ok = true;
            switch (up) {
            case 'M': {
                Vec2 q;
                if (!(ok = pt(q))) break;
                finish(false);
                cur = start = q;
                cmd = rel ? 'l' : 'L';
                prev = 'M';
                continue;
            }
            case 'Z':
                if (!p.segments.empty() || !same(cur, start)) addLine(p, t(cur), t(start));
                finish(true);
                cur = start;
                prev = 'Z';
                // Nothing may follow a closepath without a command of its own.
                if (!n.atEnd() && !std::isalpha(static_cast<unsigned char>(n.peek()))) { ok = false; break; }
                continue;
            case 'L': {
                Vec2 q;
                if (!(ok = pt(q))) break;
                addLine(p, t(cur), t(q));
                cur = q;
                break;
            }
            case 'H': {
                Real x;
                if (!(ok = n.number(x))) break;
                const Vec2 q{rel ? cur.x + x : x, cur.y};
                addLine(p, t(cur), t(q));
                cur = q;
                break;
            }
            case 'V': {
                Real y;
                if (!(ok = n.number(y))) break;
                const Vec2 q{cur.x, rel ? cur.y + y : y};
                addLine(p, t(cur), t(q));
                cur = q;
                break;
            }
            case 'C': {
                Vec2 a, b, q;
                if (!(ok = pt(a) && pt(b) && pt(q))) break;
                addCubic(p, t(cur), t(a), t(b), t(q));
                lastCtrl = b;
                cur = q;
                break;
            }
            case 'S': {
                Vec2 b, q;
                if (!(ok = pt(b) && pt(q))) break;
                const Vec2 a = (prev == 'C' || prev == 'S') ? cur * 2.0 - lastCtrl : cur;
                addCubic(p, t(cur), t(a), t(b), t(q));
                lastCtrl = b;
                cur = q;
                break;
            }
            case 'Q':
            case 'T': {
                Vec2 c, q;
                if (up == 'Q') {
                    if (!(ok = pt(c) && pt(q))) break;
                } else {
                    if (!(ok = pt(q))) break;
                    c = (prev == 'Q' || prev == 'T') ? cur * 2.0 - lastCtrl : cur;
                }
                // A quadratic is a cubic with its handles two thirds of the
                // way to the one control point: exact, not an approximation.
                addCubic(p, t(cur), t(cur + (c - cur) * (2.0 / 3.0)), t(q + (c - q) * (2.0 / 3.0)), t(q));
                lastCtrl = c;
                cur = q;
                break;
            }
            case 'A': {
                Real rx, ry, rot;
                bool large, sweep;
                Vec2 q;
                if (!(ok = n.number(rx) && n.number(ry) && n.number(rot) && n.flag(large) &&
                           n.flag(sweep) && pt(q)))
                    break;
                arcTo(p, t, cur, q, std::fabs(rx), std::fabs(ry), rot * kDeg2Rad, large, sweep);
                cur = q;
                break;
            }
            default:
                ok = false;
                break;
            }
            if (!ok) { broken = true; break; }
            prev = up;
        }
        finish(false);
        if (broken) ++out.unreadable;
    }

    // SVG's endpoint arc, turned into a centre and angles (SVG 1.1, F.6.5).
    static void arcTo(SvgPath& p, const Affine& t, Vec2 from, Vec2 to, Real rx, Real ry, Real phi,
                      bool large, bool sweep) {
        if (same(from, to)) return;
        if (rx < 1e-12 || ry < 1e-12) { addLine(p, t(from), t(to)); return; }
        const Real cp = std::cos(phi), sp = std::sin(phi);
        const Vec2 h = (from - to) * 0.5;
        const Vec2 x1{cp * h.x + sp * h.y, -sp * h.x + cp * h.y};
        const Real lambda = (x1.x * x1.x) / (rx * rx) + (x1.y * x1.y) / (ry * ry);
        if (lambda > 1) {
            const Real s = std::sqrt(lambda);
            rx *= s;
            ry *= s;
        }
        const Real num = rx * rx * ry * ry - rx * rx * x1.y * x1.y - ry * ry * x1.x * x1.x;
        const Real den = rx * rx * x1.y * x1.y + ry * ry * x1.x * x1.x;
        Real co = den > 0 ? std::sqrt(std::max(0.0, num / den)) : 0.0;
        if (large == sweep) co = -co;
        const Vec2 c1{co * rx * x1.y / ry, -co * ry * x1.x / rx};
        const Vec2 mid = (from + to) * 0.5;
        const Vec2 c{cp * c1.x - sp * c1.y + mid.x, sp * c1.x + cp * c1.y + mid.y};

        auto angle = [](Vec2 u, Vec2 v) {
            return std::atan2(u.x * v.y - u.y * v.x, u.x * v.x + u.y * v.y);
        };
        const Vec2 u{(x1.x - c1.x) / rx, (x1.y - c1.y) / ry};
        const Vec2 v{(-x1.x - c1.x) / rx, (-x1.y - c1.y) / ry};
        const Real theta = angle({1, 0}, u);
        Real dtheta = angle(u, v);
        if (!sweep && dtheta > 0) dtheta -= kTwoPi;
        if (sweep && dtheta < 0) dtheta += kTwoPi;

        // The ends are where the path says, not where the arithmetic lands.
        SvgPath piece;
        addArc(piece, t, c, rx, ry, phi, theta, dtheta);
        if (!piece.segments.empty()) {
            piece.segments.front().from = t(from);
            piece.segments.back().to = t(to);
        }
        for (SvgSegment& s : piece.segments) p.segments.push_back(s);
    }
};

// ---- Untangling -------------------------------------------------------------------
//
// A closed path that crosses itself bounds nothing a solid can be made of, and
// drawings downloaded from anywhere are full of them: the twist a pen tool
// leaves at a sharp turn, a lobe folded back over its outline. Each is split
// where it crosses, into loops that do not, with the curves cut exactly --
// a Bezier by de Casteljau, an arc by its angle -- so nothing moves that was
// not at the crossing.

// Where along an arc its parameter runs: from `a0`, turning by `sweep`.
void arcSpan(const SvgSegment& s, Real& a0, Real& sweep) {
    a0 = std::atan2(s.from.y - s.centre.y, s.from.x - s.centre.x);
    Real a1 = std::atan2(s.to.y - s.centre.y, s.to.x - s.centre.x);
    if (s.ccw) {
        sweep = a1 - a0;
        while (sweep <= 0) sweep += kTwoPi;
    } else {
        sweep = a1 - a0;
        while (sweep >= 0) sweep -= kTwoPi;
    }
}

Vec2 pointAt(const SvgSegment& s, Real u) {
    switch (s.kind) {
    case SvgSegment::Kind::Line:
        return s.from + (s.to - s.from) * u;
    case SvgSegment::Kind::Cubic: {
        const Real v = 1 - u;
        return s.from * (v * v * v) + s.c1 * (3 * v * v * u) + s.c2 * (3 * v * u * u) + s.to * (u * u * u);
    }
    case SvgSegment::Kind::Arc: {
        Real a0, sweep;
        arcSpan(s, a0, sweep);
        const Real a = a0 + sweep * u;
        return s.centre + Vec2{std::cos(a), std::sin(a)} * s.radius;
    }
    }
    return s.from;
}

Vec2 tangentAt(const SvgSegment& s, Real u) {
    switch (s.kind) {
    case SvgSegment::Kind::Line:
        return s.to - s.from;
    case SvgSegment::Kind::Cubic: {
        const Real v = 1 - u;
        return (s.c1 - s.from) * (3 * v * v) + (s.c2 - s.c1) * (6 * v * u) + (s.to - s.c2) * (3 * u * u);
    }
    case SvgSegment::Kind::Arc: {
        Real a0, sweep;
        arcSpan(s, a0, sweep);
        const Real a = a0 + sweep * u;
        return Vec2{-std::sin(a), std::cos(a)} * (s.radius * sweep);
    }
    }
    return {};
}

// The part of `s` from u0 to u1, its ends put exactly at `a` and `b`.
SvgSegment piece(const SvgSegment& s, Real u0, Real u1, Vec2 a, Vec2 b) {
    SvgSegment out = s;
    if (s.kind == SvgSegment::Kind::Cubic) {
        auto lerp = [](Vec2 p, Vec2 q, Real t) { return p + (q - p) * t; };
        // Right of u0, then the left of what is left up to u1.
        auto splitRight = [&](Vec2 p[4], Real t) {
            const Vec2 p01 = lerp(p[0], p[1], t), p12 = lerp(p[1], p[2], t), p23 = lerp(p[2], p[3], t);
            const Vec2 p012 = lerp(p01, p12, t), p123 = lerp(p12, p23, t);
            const Vec2 m = lerp(p012, p123, t);
            p[0] = m; p[1] = p123; p[2] = p23;
        };
        auto splitLeft = [&](Vec2 p[4], Real t) {
            const Vec2 p01 = lerp(p[0], p[1], t), p12 = lerp(p[1], p[2], t), p23 = lerp(p[2], p[3], t);
            const Vec2 p012 = lerp(p01, p12, t), p123 = lerp(p12, p23, t);
            const Vec2 m = lerp(p012, p123, t);
            p[1] = p01; p[2] = p012; p[3] = m;
        };
        Vec2 p[4] = {s.from, s.c1, s.c2, s.to};
        if (u0 > 0) splitRight(p, u0);
        if (u1 < 1) splitLeft(p, (u1 - u0) / (1 - u0));
        out.c1 = p[1];
        out.c2 = p[2];
    }
    out.from = a;
    out.to = b;
    return out;
}

// Where two pieces cross, refined from a guess by Newton's method on the
// curves themselves. False when it does not settle inside both.
bool refineCrossing(const SvgSegment& p, const SvgSegment& q, Real& s, Real& t) {
    for (int it = 0; it < 20; ++it) {
        const Vec2 f = pointAt(p, s) - pointAt(q, t);
        if (lengthSq(f) < 1e-24) break;
        const Vec2 dp = tangentAt(p, s), dq = tangentAt(q, t);
        // [dp  -dq] [ds dt]^T = -f
        const Real det = dp.x * -dq.y - dp.y * -dq.x;
        if (std::fabs(det) < 1e-18) return false;
        const Real ds = (-f.x * -dq.y - -f.y * -dq.x) / det;
        const Real dt = (dp.x * -f.y - dp.y * -f.x) / det;
        s += ds;
        t += dt;
    }
    return s > 1e-9 && s < 1 - 1e-9 && t > 1e-9 && t < 1 - 1e-9 &&
           lengthSq(pointAt(p, s) - pointAt(q, t)) < 1e-16;
}

Real loopArea(const std::vector<SvgSegment>& loop) {
    Real twice = 0;
    for (const SvgSegment& sg : loop) {
        const int steps = sg.kind == SvgSegment::Kind::Line ? 1 : 16;
        Vec2 prev = pointAt(sg, 0);
        for (int k = 1; k <= steps; ++k) {
            const Vec2 q = pointAt(sg, static_cast<Real>(k) / steps);
            twice += prev.x * q.y - q.x * prev.y;
            prev = q;
        }
    }
    return twice * 0.5;
}

// ---- Filled or not: the winding number, exactly ------------------------------------
//
// A ray from the point to the right, and every piece of an outline it passes
// counted up or down by which way the piece runs. Counted on the curves
// themselves -- each cut where it turns back in y, and the crossing found by
// bisection -- not on a flattening of them: a point a hair inside a curve has
// to count as inside, and a flattening would put it on either side.

// Where a piece turns back in y, in its own parameter: between these it only
// rises or only falls.
std::vector<Real> monotoneBreaks(const SvgSegment& s, int axis) {
    std::vector<Real> out{0.0};
    if (s.kind == SvgSegment::Kind::Cubic) {
        // d/du = a u^2 + b u + c, along `axis`
        const Real p0 = s.from[axis], p1 = s.c1[axis], p2 = s.c2[axis], p3 = s.to[axis];
        const Real qa = 3 * (-p0 + 3 * p1 - 3 * p2 + p3), qb = 6 * (p0 - 2 * p1 + p2), qc = 3 * (p1 - p0);
        std::vector<Real> r;
        if (std::fabs(qa) < 1e-15) {
            if (std::fabs(qb) > 1e-15) r.push_back(-qc / qb);
        } else {
            const Real disc = qb * qb - 4 * qa * qc;
            if (disc >= 0) {
                const Real sq = std::sqrt(disc);
                r.push_back((-qb - sq) / (2 * qa));
                r.push_back((-qb + sq) / (2 * qa));
            }
        }
        std::sort(r.begin(), r.end());
        for (Real u : r)
            if (u > 1e-12 && u < 1 - 1e-12) out.push_back(u);
    } else if (s.kind == SvgSegment::Kind::Arc) {
        // y turns back where the angle passes a quarter or three quarters,
        // x where it passes nought or a half.
        Real a0, sweep;
        arcSpan(s, a0, sweep);
        for (int k = -8; k <= 8; ++k) {
            const Real a = (axis == 1 ? kPi * 0.5 : 0.0) + kPi * k;
            const Real u = (a - a0) / sweep;
            if (u > 1e-12 && u < 1 - 1e-12) out.push_back(u);
        }
        std::sort(out.begin(), out.end());
    }
    out.push_back(1.0);
    return out;
}

// A stretch of one segment that only rises or only falls across a ray: met by
// the ray's line once at most. Its ends, and how far it reaches along the ray,
// are kept so most stretches are settled without looking at the curve.
struct Stretch {
    size_t seg;
    Real ua, ub;
    Vec2 A, B;
    Real alongLo, alongHi;
};

struct InkLoop {
    std::vector<SvgSegment> segs;
    std::vector<Stretch> stretches[2];   // for a ray along x, and along y
    Vec2 lo, hi;
    int element = -1;
    bool evenOdd = false;
    int colour = -1;
};

// The winding number of `p`, by a ray along +x (`axis` 0) or +y (`axis` 1).
// `skip`, when given, is where on this loop the point lies -- segment `seg` at
// parameter `u` -- and the crossing there is left out: the caller knows which
// side of it the point is taken to be. Only that one: a curve that folds back
// can cross the ray again further along, and that crossing counts.
struct Skip { size_t loop = SIZE_MAX, seg = SIZE_MAX; Real u = 0; };

int winding(const InkLoop& loop, Vec2 p, int axis, Skip skip = {}) {
    const int along = axis, across = 1 - axis;   // the ray runs along, and is crossed across
    if (p[across] < loop.lo[across] || p[across] > loop.hi[across] || p[along] > loop.hi[along]) return 0;
    int w = 0;
    for (const Stretch& st : loop.stretches[axis]) {
        // Half-open, as a polygon's edges are counted: a ray through a joint
        // counts it once.
        const bool up = st.A[across] <= p[across] && p[across] < st.B[across];
        const bool down = st.B[across] <= p[across] && p[across] < st.A[across];
        if (!up && !down) continue;
        if (st.alongHi <= p[along]) continue;
        // The stretch the point itself is on meets the ray's line once, at
        // the point.
        if (st.seg == skip.seg && skip.u >= st.ua && skip.u <= st.ub) continue;
        bool beyond = st.alongLo > p[along];
        if (!beyond) {
            // Where it crosses the ray's line, by bisection, and which side
            // of the point that is.
            const SvgSegment& s = loop.segs[st.seg];
            Real lo = st.ua, hi = st.ub;
            const bool rising = st.B[across] > st.A[across];
            for (int it = 0; it < 52; ++it) {
                const Real mid = (lo + hi) * 0.5;
                if ((pointAt(s, mid)[across] < p[across]) == rising) lo = mid; else hi = mid;
            }
            beyond = pointAt(s, (lo + hi) * 0.5)[along] > p[along];
        }
        if (!beyond) continue;
        // Counter-clockwise about the point counts one: rising in y to its
        // right, or running back in x above it.
        w += axis == 0 ? (up ? 1 : -1) : (up ? -1 : 1);
    }
    return w;
}

bool fills(const InkLoop& first, int w) { return first.evenOdd ? (w % 2) != 0 : w != 0; }

void inkBounds(InkLoop& l) {
    l.lo = {1e300, 1e300};
    l.hi = {-1e300, -1e300};
    l.stretches[0].clear();
    l.stretches[1].clear();
    for (size_t si = 0; si < l.segs.size(); ++si) {
        const SvgSegment& s = l.segs[si];
        for (int axis = 0; axis < 2; ++axis) {
            const int along = axis, across = 1 - axis;
            const std::vector<Real> br = monotoneBreaks(s, across);
            for (size_t k = 0; k + 1 < br.size(); ++k) {
                Stretch st;
                st.seg = si;
                st.ua = br[k];
                st.ub = br[k + 1];
                st.A = pointAt(s, st.ua);
                st.B = pointAt(s, st.ub);
                // How far along the ray it can reach: a cubic within its
                // control points, an arc within its chord and bow.
                st.alongLo = std::min(st.A[along], st.B[along]);
                st.alongHi = std::max(st.A[along], st.B[along]);
                if (s.kind == SvgSegment::Kind::Cubic) {
                    const SvgSegment part = piece(s, st.ua, st.ub, st.A, st.B);
                    for (Vec2 c : {part.c1, part.c2}) {
                        st.alongLo = std::min(st.alongLo, c[along]);
                        st.alongHi = std::max(st.alongHi, c[along]);
                    }
                } else if (s.kind == SvgSegment::Kind::Arc) {
                    st.alongLo -= s.radius;
                    st.alongHi += s.radius;
                }
                l.stretches[axis].push_back(st);
            }
        }
        const int steps = s.kind == SvgSegment::Kind::Line ? 1 : 16;
        for (int k = 0; k <= steps; ++k) {
            const Vec2 q = pointAt(s, static_cast<Real>(k) / steps);
            l.lo = {std::min(l.lo.x, q.x), std::min(l.lo.y, q.y)};
            l.hi = {std::max(l.hi.x, q.x), std::max(l.hi.y, q.y)};
        }
    }
    // A curve's bulge between samples: a little room either way.
    const Vec2 pad = (l.hi - l.lo) * 0.02 + Vec2{1e-9, 1e-9};
    l.lo -= pad;
    l.hi += pad;
}

// ---- Crossings, between any two outlines ------------------------------------------

struct Crossing { size_t i, j; Real s, t; Vec2 at; };

// `segs` flattened for finding where they cross; a guess from the flattening,
// refined on the curves.
std::vector<Crossing> findCrossings(const std::vector<const SvgSegment*>& segs,
                                    const std::vector<std::pair<size_t, size_t>>& neighbours) {
    struct Flat { std::vector<Vec2> pts; std::vector<Real> us; Vec2 lo, hi; };
    const size_t n = segs.size();
    std::vector<Flat> flat(n);
    Vec2 lo{1e300, 1e300}, hi{-1e300, -1e300};
    for (size_t i = 0; i < n; ++i) {
        const SvgSegment& sg = *segs[i];
        int steps = 1;
        if (sg.kind == SvgSegment::Kind::Cubic) steps = 24;
        if (sg.kind == SvgSegment::Kind::Arc) {
            Real a0, sweep;
            arcSpan(sg, a0, sweep);
            steps = std::max(8, static_cast<int>(std::ceil(std::fabs(sweep) / (kPi / 16))));
        }
        Flat& f = flat[i];
        f.lo = {1e300, 1e300};
        f.hi = {-1e300, -1e300};
        for (int k = 0; k <= steps; ++k) {
            const Real u = static_cast<Real>(k) / steps;
            const Vec2 q = pointAt(sg, u);
            f.pts.push_back(q);
            f.us.push_back(u);
            f.lo = {std::min(f.lo.x, q.x), std::min(f.lo.y, q.y)};
            f.hi = {std::max(f.hi.x, q.x), std::max(f.hi.y, q.y)};
        }
        lo = {std::min(lo.x, f.lo.x), std::min(lo.y, f.lo.y)};
        hi = {std::max(hi.x, f.hi.x), std::max(hi.y, f.hi.y)};
    }
    if (n == 0) return {};

    // Pairs worth looking at: sharing a cell of a grid over the drawing.
    const int g = std::clamp(static_cast<int>(std::sqrt(static_cast<Real>(n))), 1, 256);
    const Vec2 span{std::max(hi.x - lo.x, 1e-12), std::max(hi.y - lo.y, 1e-12)};
    auto cell = [&](Real v, Real l, Real sp) {
        return std::clamp(static_cast<int>((v - l) / sp * g), 0, g - 1);
    };
    std::vector<std::vector<uint32_t>> grid(static_cast<size_t>(g * g));
    for (size_t i = 0; i < n; ++i) {
        const int x0 = cell(flat[i].lo.x, lo.x, span.x), x1 = cell(flat[i].hi.x, lo.x, span.x);
        const int y0 = cell(flat[i].lo.y, lo.y, span.y), y1 = cell(flat[i].hi.y, lo.y, span.y);
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) grid[static_cast<size_t>(y * g + x)].push_back(static_cast<uint32_t>(i));
    }
    std::vector<uint64_t> pairs;
    for (const auto& c : grid)
        for (size_t a = 0; a < c.size(); ++a)
            for (size_t b = a + 1; b < c.size(); ++b) {
                const uint64_t i = std::min(c[a], c[b]), j = std::max(c[a], c[b]);
                pairs.push_back(i << 32 | j);
            }
    std::sort(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
    std::sort(const_cast<std::vector<std::pair<size_t, size_t>>&>(neighbours).begin(),
              const_cast<std::vector<std::pair<size_t, size_t>>&>(neighbours).end());

    // Each pair by halving: both curves cut in two, and cut again, wherever
    // their boxes still overlap, until the boxes are too small to hold two
    // crossings apart -- then Newton's method on the curves themselves. A
    // twist where two curves cross twice inside one step of a flattening is
    // found this way, where comparing two flattenings misses it.
    const Real small = std::max(std::max(span.x, span.y) * 1e-7, 1e-10);
    struct Box { Vec2 lo, hi; };
    auto boxOf = [](const SvgSegment& sg, Real u0, Real u1) {
        Box b{{1e300, 1e300}, {-1e300, -1e300}};
        auto grow = [&b](Vec2 q) {
            b.lo = {std::min(b.lo.x, q.x), std::min(b.lo.y, q.y)};
            b.hi = {std::max(b.hi.x, q.x), std::max(b.hi.y, q.y)};
        };
        const Vec2 a = pointAt(sg, u0), c = pointAt(sg, u1);
        switch (sg.kind) {
        case SvgSegment::Kind::Line:
            grow(a);
            grow(c);
            break;
        case SvgSegment::Kind::Cubic: {
            // Inside the hull of its own control points.
            const SvgSegment p = piece(sg, u0, u1, a, c);
            grow(p.from); grow(p.c1); grow(p.c2); grow(p.to);
            break;
        }
        case SvgSegment::Kind::Arc: {
            // Its chord, widened by how far the arc bows from it.
            Real a0, sweep;
            arcSpan(sg, a0, sweep);
            const Real bow = sg.radius * (1 - std::cos(std::min(std::fabs(sweep * (u1 - u0)), kPi) * 0.5));
            grow(a);
            grow(c);
            grow(pointAt(sg, (u0 + u1) * 0.5));
            b.lo -= Vec2{bow, bow};
            b.hi += Vec2{bow, bow};
            break;
        }
        }
        return b;
    };
    auto overlap = [](const Box& p, const Box& q) {
        return !(p.hi.x < q.lo.x || q.hi.x < p.lo.x || p.hi.y < q.lo.y || q.hi.y < p.lo.y);
    };

    std::vector<Crossing> out;
    for (uint64_t key : pairs) {
        const size_t i = static_cast<size_t>(key >> 32), j = static_cast<size_t>(key & 0xffffffffu);
        const Flat &A = flat[i], &B = flat[j];
        if (A.hi.x < B.lo.x || B.hi.x < A.lo.x || A.hi.y < B.lo.y || B.hi.y < A.lo.y) continue;
        const SvgSegment& P = *segs[i];
        const SvgSegment& Q = *segs[j];
        // Two lines cross once at most, where they cross.
        struct Span { Real u0, u1, t0, t1; int depth; };
        std::vector<Span> stack{{0, 1, 0, 1, 0}};
        int budget = 20000;   // curves that run along each other: not crossings
        const size_t firstHere = out.size();
        while (!stack.empty() && budget-- > 0) {
            const Span sp = stack.back();
            stack.pop_back();
            const Box ba = boxOf(P, sp.u0, sp.u1), bb = boxOf(Q, sp.t0, sp.t1);
            if (!overlap(ba, bb)) continue;
            const Real sa = length(ba.hi - ba.lo), sb = length(bb.hi - bb.lo);
            const bool bothLines = P.kind == SvgSegment::Kind::Line && Q.kind == SvgSegment::Kind::Line;
            if (bothLines || (sa < small && sb < small) || sp.depth > 48 ||
                (sa < std::max(sb, small) * 0.05 && sb < small * 64 && sa < small * 64)) {
                Real s = (sp.u0 + sp.u1) * 0.5, t = (sp.t0 + sp.t1) * 0.5;
                if (!refineCrossing(P, Q, s, t)) continue;
                bool seen = false;
                for (size_t k = firstHere; k < out.size(); ++k)
                    if (std::fabs(out[k].s - s) < 1e-7 && std::fabs(out[k].t - t) < 1e-7) seen = true;
                if (!seen) out.push_back({i, j, s, t, pointAt(P, s)});
                continue;
            }
            // Halve whichever is bigger; both when they are alike.
            const Real um = (sp.u0 + sp.u1) * 0.5, tm = (sp.t0 + sp.t1) * 0.5;
            if (sa >= sb * 2) {
                stack.push_back({sp.u0, um, sp.t0, sp.t1, sp.depth + 1});
                stack.push_back({um, sp.u1, sp.t0, sp.t1, sp.depth + 1});
            } else if (sb >= sa * 2) {
                stack.push_back({sp.u0, sp.u1, sp.t0, tm, sp.depth + 1});
                stack.push_back({sp.u0, sp.u1, tm, sp.t1, sp.depth + 1});
            } else {
                stack.push_back({sp.u0, um, sp.t0, tm, sp.depth + 1});
                stack.push_back({sp.u0, um, tm, sp.t1, sp.depth + 1});
                stack.push_back({um, sp.u1, sp.t0, tm, sp.depth + 1});
                stack.push_back({um, sp.u1, tm, sp.t1, sp.depth + 1});
            }
        }
    }
    return out;
}

// ---- What is filled, outlined ---------------------------------------------------------

void outlineInk(SvgDrawing& d, Real speckArea) {
    // Whether a colour is ink; one never named -- the paper, kept because it
    // was all there was -- is.
    auto isInk = [&](int colour) {
        return colour < 0 || colour >= static_cast<int>(d.colours.size()) ||
               d.colours[static_cast<size_t>(colour)].ink;
    };
    // Every filled closed outline, a circle as two half circles so it can be
    // cut like anything else. Everything else goes through as it was.
    std::vector<InkLoop> loops;
    std::vector<SvgPath> kept;
    std::vector<SvgCircle> keptCircles;
    for (SvgPath& p : d.paths) {
        if (!p.closed || !p.fill.filled || p.segments.empty()) { kept.push_back(std::move(p)); continue; }
        InkLoop l;
        l.segs = std::move(p.segments);
        l.element = p.fill.element;
        l.evenOdd = p.fill.evenOdd;
        l.colour = p.fill.colour;
        loops.push_back(std::move(l));
    }
    for (const SvgCircle& c : d.circles) {
        if (!c.fill.filled) { keptCircles.push_back(c); continue; }
        InkLoop l;
        SvgSegment h;
        h.kind = SvgSegment::Kind::Arc;
        h.centre = c.centre;
        h.radius = c.radius;
        h.ccw = true;
        h.from = c.centre + Vec2{c.radius, 0};
        h.to = c.centre - Vec2{c.radius, 0};
        l.segs.push_back(h);
        std::swap(h.from, h.to);
        l.segs.push_back(h);
        l.element = c.fill.element;
        l.colour = c.fill.colour;
        loops.push_back(std::move(l));
    }
    if (loops.empty()) return;
    for (InkLoop& l : loops) inkBounds(l);

    // Subpaths of one element are filled together.
    std::vector<std::vector<size_t>> byElement;
    {
        std::unordered_map<int, size_t> at;
        for (size_t i = 0; i < loops.size(); ++i) {
            const int e = loops[i].element >= 0 ? loops[i].element : -1 - static_cast<int>(i);
            auto [it, fresh] = at.emplace(e, byElement.size());
            if (fresh) byElement.emplace_back();
            byElement[it->second].push_back(i);
        }
    }

    // Every segment, and the points where they cross.
    std::vector<const SvgSegment*> segs;
    std::vector<std::pair<size_t, size_t>> where;   // loop, index
    for (size_t li = 0; li < loops.size(); ++li)
        for (size_t k = 0; k < loops[li].segs.size(); ++k) {
            segs.push_back(&loops[li].segs[k]);
            where.push_back({li, k});
        }
    const std::vector<Crossing> xs = findCrossings(segs, {});
    d.crossings += static_cast<int>(xs.size());


    // Which outlines are tangled up with which: what is worked out together,
    // and what is put back together if the working out fails.
    std::vector<size_t> group(loops.size());
    for (size_t i = 0; i < group.size(); ++i) group[i] = i;
    auto root = [&](size_t x) {
        while (group[x] != x) x = group[x] = group[group[x]];
        return x;
    };
    for (const Crossing& c : xs) group[root(where[c.i].first)] = root(where[c.j].first);

    // Nodes: every corner of every loop, and every crossing. Corners that
    // coincide are one node, found below.
    std::vector<size_t> cornerBase(loops.size());
    size_t nodes = 0;
    for (size_t li = 0; li < loops.size(); ++li) {
        cornerBase[li] = nodes;
        nodes += loops[li].segs.size();
    }
    const size_t crossingBase = nodes;
    nodes += xs.size();
    std::vector<size_t> same(nodes);
    for (size_t i = 0; i < nodes; ++i) same[i] = i;
    auto node = [&](size_t x) {
        while (same[x] != x) x = same[x] = same[same[x]];
        return x;
    };

    Vec2 allLo{1e300, 1e300}, allHi{-1e300, -1e300};
    for (const InkLoop& l : loops) {
        allLo = {std::min(allLo.x, l.lo.x), std::min(allLo.y, l.lo.y)};
        allHi = {std::max(allHi.x, l.hi.x), std::max(allHi.y, l.hi.y)};
    }
    // Closer than this is touching: far under anything that prints, and over
    // what a file's rounding to three places leaves between two shapes drawn
    // to meet.
    const Real touch = std::max(std::max(allHi.x - allLo.x, allHi.y - allLo.y) * 1e-7, 1e-12);

    struct Cut { Real u; size_t node; Vec2 at; };
    std::vector<std::vector<Cut>> cuts(segs.size());
    for (size_t c = 0; c < xs.size(); ++c) {
        cuts[xs[c].i].push_back({xs[c].s, crossingBase + c, xs[c].at});
        cuts[xs[c].j].push_back({xs[c].t, crossingBase + c, xs[c].at});
    }

    // Where a corner of one outline lies on another -- two squares side by
    // side, a shape drawn to meet the next -- the other is cut there, at that
    // corner, so the two meet at a node rather than passing each other; and
    // corners that coincide are made one.
    {
        const int g = std::clamp(static_cast<int>(std::sqrt(static_cast<Real>(segs.size()))), 1, 256);
        const Vec2 span{std::max(allHi.x - allLo.x, 1e-12), std::max(allHi.y - allLo.y, 1e-12)};
        auto cellOf = [&](Real v, Real lo, Real sp) {
            return std::clamp(static_cast<int>((v - lo) / sp * g), 0, g - 1);
        };
        std::vector<std::vector<uint32_t>> grid(static_cast<size_t>(g * g));
        std::vector<std::pair<Vec2, Vec2>> segBox(segs.size());
        for (size_t si = 0; si < segs.size(); ++si) {
            const SvgSegment& sg = *segs[si];
            Vec2 lo{1e300, 1e300}, hi{-1e300, -1e300};
            const int steps = sg.kind == SvgSegment::Kind::Line ? 1 : 16;
            for (int k = 0; k <= steps; ++k) {
                const Vec2 q = pointAt(sg, static_cast<Real>(k) / steps);
                lo = {std::min(lo.x, q.x), std::min(lo.y, q.y)};
                hi = {std::max(hi.x, q.x), std::max(hi.y, q.y)};
            }
            const Vec2 pad = (hi - lo) * 0.05 + Vec2{touch, touch};
            lo -= pad;
            hi += pad;
            segBox[si] = {lo, hi};
            for (int y = cellOf(lo.y, allLo.y, span.y); y <= cellOf(hi.y, allLo.y, span.y); ++y)
                for (int x = cellOf(lo.x, allLo.x, span.x); x <= cellOf(hi.x, allLo.x, span.x); ++x)
                    grid[static_cast<size_t>(y * g + x)].push_back(static_cast<uint32_t>(si));
        }
        for (size_t li = 0; li < loops.size(); ++li) {
            const size_t n = loops[li].segs.size();
            for (size_t k = 0; k < n; ++k) {
                const Vec2 P = loops[li].segs[k].from;
                const size_t v = cornerBase[li] + k;
                const auto& cell = grid[static_cast<size_t>(cellOf(P.y, allLo.y, span.y) * g +
                                                            cellOf(P.x, allLo.x, span.x))];
                for (uint32_t si : cell) {
                    const auto [lj, kj] = where[si];
                    if (lj == li && (kj == k || kj == (k + n - 1) % n)) continue;   // its own two sides
                    const auto& [lo, hi] = segBox[si];
                    if (P.x < lo.x || P.x > hi.x || P.y < lo.y || P.y > hi.y) continue;
                    // The nearest point of that segment: the best of a few
                    // samples, then Newton on the distance.
                    const SvgSegment& sg = *segs[si];
                    Real t = 0, best = 1e300;
                    for (int q = 0; q <= 32; ++q) {
                        const Real d = lengthSq(pointAt(sg, q / 32.0) - P);
                        if (d < best) { best = d; t = q / 32.0; }
                    }
                    for (int it = 0; it < 30; ++it) {
                        const Vec2 f = pointAt(sg, t) - P, dt = tangentAt(sg, t);
                        const Real den = lengthSq(dt);
                        if (den < 1e-30) break;
                        t = std::clamp(t - dot(f, dt) / den, 0.0, 1.0);
                    }
                    if (length(pointAt(sg, t) - P) > touch) continue;
                    const size_t nj = loops[lj].segs.size();
                    if (length(sg.from - P) <= touch) {
                        same[node(v)] = node(cornerBase[lj] + kj);
                    } else if (length(sg.to - P) <= touch) {
                        same[node(v)] = node(cornerBase[lj] + (kj + 1) % nj);
                    } else {
                        cuts[si].push_back({t, v, P});
                    }
                }
            }
        }
    }

    // Pieces: each segment cut at its crossings and where others touch it,
    // from node to node.
    struct Piece { SvgSegment seg; size_t from, to, loop, index; Real u0, u1; };
    std::vector<Piece> pieces;
    for (size_t si = 0; si < segs.size(); ++si) {
        const auto [li, k] = where[si];
        const SvgSegment& sg = *segs[si];
        const size_t n = loops[li].segs.size();
        std::vector<Cut>& cs = cuts[si];
        std::sort(cs.begin(), cs.end(), [](const Cut& a, const Cut& b) { return a.u < b.u; });
        Real u0 = 0;
        Vec2 a = sg.from;
        size_t from = cornerBase[li] + k;
        for (const Cut& c : cs) {
            if (c.u - u0 < 1e-12) { same[node(c.node)] = node(from); continue; }
            pieces.push_back({piece(sg, u0, c.u, a, c.at), from, c.node, li, k, u0, c.u});
            u0 = c.u;
            a = c.at;
            from = c.node;
        }
        pieces.push_back({piece(sg, u0, 1, a, sg.to), from, cornerBase[li] + (k + 1) % n, li, k, u0, 1.0});
    }
    for (Piece& pc : pieces) {
        pc.from = node(pc.from);
        pc.to = node(pc.to);
    }

    // Pieces lying one on another -- the shared side of two squares side by
    // side -- are judged together, once: each is part of its own shape's
    // outline, and the side of it each shape fills is only known from all of
    // them at once.
    std::vector<std::vector<size_t>> coincident;
    {
        std::unordered_map<uint64_t, std::vector<size_t>> byEnds;
        for (size_t i = 0; i < pieces.size(); ++i) {
            const uint64_t lo = std::min(pieces[i].from, pieces[i].to), hi = std::max(pieces[i].from, pieces[i].to);
            byEnds[lo << 32 | hi].push_back(i);
        }
        for (auto& [key, list] : byEnds) {
            (void)key;
            std::vector<bool> taken(list.size(), false);
            for (size_t a = 0; a < list.size(); ++a) {
                if (taken[a]) continue;
                std::vector<size_t> set{list[a]};
                const Vec2 ma = pointAt(pieces[list[a]].seg, 0.5);
                for (size_t b2 = a + 1; b2 < list.size(); ++b2)
                    if (!taken[b2] && length(pointAt(pieces[list[b2]].seg, 0.5) - ma) <= touch * 10) {
                        taken[b2] = true;
                        set.push_back(list[b2]);
                    }
                coincident.push_back(std::move(set));
            }
        }
    }

    // A piece stays where it runs between painted and not, turned so the
    // paint is on its left. Judged exactly, at a point on the piece: the
    // winding there counted from everything but the pieces lying there, and
    // each of their own crossings added for the side it would be crossed
    // from. No step to either side, which a twist a thousandth of a
    // millimetre across is thinner than.
    std::vector<size_t> elementOf(loops.size());
    for (size_t g = 0; g < byElement.size(); ++g)
        for (size_t li : byElement[g]) elementOf[li] = g;

    // Which outlines a ray can meet: banded across the ray, so a point asks
    // only the outlines whose band it is in -- a drawing of thousands of
    // shapes, asked about every piece of every one, otherwise asks them all.
    constexpr int kBands = 256;
    auto band = [&](int across, Real v) {
        const Real span = std::max(allHi[across] - allLo[across], 1e-12);
        return std::clamp(static_cast<int>((v - allLo[across]) / span * kBands), 0, kBands - 1);
    };
    std::vector<std::vector<size_t>> bands[2];   // by the axis the ray runs along
    for (int axis = 0; axis < 2; ++axis) {
        const int across = 1 - axis;
        bands[axis].resize(kBands);
        for (size_t li = 0; li < loops.size(); ++li)
            for (int b2 = band(across, loops[li].lo[across]); b2 <= band(across, loops[li].hi[across]); ++b2)
                bands[axis][static_cast<size_t>(b2)].push_back(li);
    }

    std::vector<Piece> edges;
    std::vector<bool> bounds(loops.size(), false);
    std::unordered_map<size_t, std::array<int, 3>> byGroup;   // winding, and what each side adds
    std::vector<Skip> skips;
    for (const std::vector<size_t>& set : coincident) {
        const Piece& ref = pieces[set.front()];
        const Real um = (ref.u0 + ref.u1) * 0.5;
        const SvgSegment& whole = loops[ref.loop].segs[ref.index];
        const Vec2 m = pointAt(whole, um);
        const Vec2 tan = tangentAt(whole, um);
        if (lengthSq(tan) < 1e-30) continue;
        // Across the piece, as squarely as can be.
        const int axis = std::fabs(tan.y) >= std::fabs(tan.x) ? 0 : 1;
        // The ray starts on the low side of the pieces and crosses them all;
        // from the high side it crosses none. Which of those is the
        // reference piece's left follows from which way it runs.
        const bool leftIsLow = axis == 0 ? tan.y > 0 : tan.x < 0;

        byGroup.clear();
        skips.clear();
        for (size_t i : set) {
            const Piece& pc = pieces[i];
            const Real ui = (pc.u0 + pc.u1) * 0.5;
            skips.push_back({pc.loop, pc.index, ui});
            const Vec2 ti = tangentAt(loops[pc.loop].segs[pc.index], ui);
            const int sign = axis == 0 ? (ti.y > 0 ? 1 : -1) : (ti.x < 0 ? 1 : -1);
            auto& w = byGroup[elementOf[pc.loop]];
            (leftIsLow ? w[1] : w[2]) += sign;
        }
        for (size_t li : bands[axis][static_cast<size_t>(band(1 - axis, m[1 - axis]))]) {
            Skip sk;
            for (const Skip& k : skips)
                if (k.loop == li) sk = k;
            byGroup[elementOf[li]][0] += winding(loops[li], m, axis, sk);
        }
        // What shows on each side is whatever was painted there last; it is
        // ink if its colour is. Paper-coloured paint on top of ink wipes it
        // out, as it does on the page.
        int topL = INT_MIN, topR = INT_MIN;
        bool l = false, r = false;
        for (const auto& [g, w] : byGroup) {
            const InkLoop& rule = loops[byElement[g].front()];
            const int order = rule.element;
            if (fills(rule, w[0] + w[1]) && order >= topL) { topL = order; l = isInk(rule.colour); }
            if (fills(rule, w[0] + w[2]) && order >= topR) { topR = order; r = isInk(rule.colour); }
        }
        if (l == r) continue;
        for (size_t i : set) bounds[pieces[i].loop] = true;
        if (l) {
            edges.push_back(ref);
        } else {
            Piece rev = ref;
            std::swap(rev.from, rev.to);
            std::swap(rev.seg.from, rev.seg.to);
            std::swap(rev.seg.c1, rev.seg.c2);
            rev.seg.ccw = !ref.seg.ccw;
            edges.push_back(rev);
        }
    }

    // Joined up again. Leaving a node, the way on is the first edge met turning
    // clockwise from the way in: the one that keeps the same paint on the left
    // where two painted shapes only touch.
    std::unordered_map<size_t, std::vector<size_t>> leaving;
    for (size_t e = 0; e < edges.size(); ++e) leaving[edges[e].from].push_back(e);
    std::vector<bool> used(edges.size(), false);
    std::vector<std::vector<Piece>> out;
    std::vector<bool> failed(loops.size(), false);   // by group root
    for (size_t start = 0; start < edges.size(); ++start) {
        if (used[start]) continue;
        std::vector<Piece> loop;
        size_t e = start;
        bool closed = false;
        for (size_t guard = 0; guard <= edges.size(); ++guard) {
            used[e] = true;
            loop.push_back(edges[e]);
            if (edges[e].to == edges[start].from) { closed = true; break; }
            const std::vector<size_t>& options = leaving[edges[e].to];
            const Vec2 in = normalize(tangentAt(edges[e].seg, 1.0));
            size_t best = SIZE_MAX;
            Real bestTurn = 1e9;
            for (size_t o : options) {
                if (used[o]) continue;
                const Vec2 dir = normalize(tangentAt(edges[o].seg, 0.0));
                // Clockwise angle from the way back to the way on.
                const Vec2 back = -in;
                Real turn = std::atan2(back.x * dir.y - back.y * dir.x, back.x * dir.x + back.y * dir.y);
                turn = turn <= 0 ? -turn : kTwoPi - turn;
                if (turn < bestTurn) { bestTurn = turn; best = o; }
            }
            if (best == SIZE_MAX) break;
            e = best;
        }
        if (closed) out.push_back(std::move(loop));
        else failed[root(edges[start].loop)] = true;
    }
    // A group that would not join up comes through as it was drawn, rather
    // than losing what would not close: better an outline the kernel may
    // refuse, and say so, than one quietly missing.
    out.erase(std::remove_if(out.begin(), out.end(),
                             [&](const std::vector<Piece>& l) { return failed[root(l.front().loop)]; }),
              out.end());
    for (size_t li = 0; li < loops.size(); ++li) {
        if (!failed[root(li)]) continue;
        ++d.unresolved;
        SvgPath p;
        p.closed = true;
        p.segments = loops[li].segs;
        p.fill.filled = true;
        p.fill.evenOdd = loops[li].evenOdd;
        p.fill.element = loops[li].element;
        kept.push_back(std::move(p));
        bounds[li] = true;   // not covered: kept
    }

    // A loop that passes one node twice is two loops touching there.
    std::vector<std::vector<Piece>> simple;
    while (!out.empty()) {
        std::vector<Piece> l = std::move(out.back());
        out.pop_back();
        std::unordered_map<size_t, size_t> first;
        bool split = false;
        for (size_t k = 0; k < l.size() && !split; ++k) {
            auto [it, fresh] = first.emplace(l[k].to, k);
            if (fresh) continue;
            const size_t p = it->second;
            std::vector<Piece> inner(l.begin() + static_cast<long>(p) + 1, l.begin() + static_cast<long>(k) + 1);
            std::vector<Piece> rest(l.begin(), l.begin() + static_cast<long>(p) + 1);
            rest.insert(rest.end(), l.begin() + static_cast<long>(k) + 1, l.end());
            out.push_back(std::move(inner));
            out.push_back(std::move(rest));
            split = true;
        }
        if (!split) simple.push_back(std::move(l));
    }

    // What was drawn in ink and is now no boundary at all -- a shape inside
    // another filled one, a hole painted over -- is counted, so it can be
    // said. A paper-coloured shape that bounds nothing is just paper.
    for (size_t li = 0; li < loops.size(); ++li)
        if (!bounds[li] && isInk(loops[li].colour)) ++d.covered;

    for (std::vector<Piece>& l : simple) {
        if (l.empty()) continue;
        std::vector<SvgSegment> segsOut;
        for (Piece& pc : l) segsOut.push_back(pc.seg);
        if (std::fabs(loopArea(segsOut)) <= speckArea) { ++d.specks; continue; }
        // A circle nothing cut is a circle again, to be sized by its radius.
        if (segsOut.size() == 2 && segsOut[0].kind == SvgSegment::Kind::Arc &&
            segsOut[1].kind == SvgSegment::Kind::Arc && lengthSq(segsOut[0].centre - segsOut[1].centre) < 1e-24 &&
            std::fabs(segsOut[0].radius - segsOut[1].radius) < 1e-12 && segsOut[0].ccw == segsOut[1].ccw &&
            lengthSq(segsOut[0].from - segsOut[1].to) < 1e-24 && lengthSq(segsOut[0].to - segsOut[1].from) < 1e-24 &&
            lengthSq(segsOut[0].from - segsOut[0].to - (segsOut[0].from - segsOut[0].centre) * 2.0) < 1e-18) {
            SvgCircle c;
            c.centre = segsOut[0].centre;
            c.radius = segsOut[0].radius;
            keptCircles.push_back(c);
            continue;
        }
        SvgPath p;
        p.closed = true;
        p.segments = std::move(segsOut);
        kept.push_back(std::move(p));
    }
    d.paths = std::move(kept);
    d.circles = std::move(keptCircles);
}

void grow(Vec2& lo, Vec2& hi, Vec2 p) {
    lo.x = std::min(lo.x, p.x);
    lo.y = std::min(lo.y, p.y);
    hi.x = std::max(hi.x, p.x);
    hi.y = std::max(hi.y, p.y);
}

} // namespace

size_t SvgDrawing::entityCount() const {
    size_t n = circles.size();
    for (const SvgPath& p : paths) n += p.segments.size();
    return n;
}

namespace {

// Outlines made to bound the ink, and the drawing measured around them.
void finish(SvgDrawing& d) {
    // Twists are measured against the drawing: a loop a millionth of its
    // area is a pen's slip, not a shape.
    {
        Vec2 lo{1e300, 1e300}, hi{-1e300, -1e300};
        for (const SvgPath& p : d.paths)
            for (const SvgSegment& sg : p.segments) {
                grow(lo, hi, sg.from);
                grow(lo, hi, sg.to);
            }
        const Vec2 span = hi - lo;
        outlineInk(d, span.x > 0 && span.y > 0 ? 1e-6 * span.x * span.y : 0.0);
    }

    if (d.empty()) {
        d.error = "There is no outline in it to import";
        if (d.text > 0) d.error += ": its text is still text -- convert it to paths first";
        return;
    }

    // Around what the geometry actually reaches, curves sampled rather than
    // their handles taken at their word.
    Vec2 lo{1e300, 1e300}, hi{-1e300, -1e300};
    for (const SvgCircle& c : d.circles) {
        grow(lo, hi, c.centre - Vec2{c.radius, c.radius});
        grow(lo, hi, c.centre + Vec2{c.radius, c.radius});
    }
    for (const SvgPath& p : d.paths) {
        for (const SvgSegment& s : p.segments) {
            grow(lo, hi, s.from);
            grow(lo, hi, s.to);
            if (s.kind == SvgSegment::Kind::Cubic) {
                for (int i = 1; i < 16; ++i) {
                    const Real u = i / 16.0, v = 1 - u;
                    grow(lo, hi, s.from * (v * v * v) + s.c1 * (3 * v * v * u) + s.c2 * (3 * v * u * u) +
                                     s.to * (u * u * u));
                }
            } else if (s.kind == SvgSegment::Kind::Arc) {
                Real a0 = std::atan2(s.from.y - s.centre.y, s.from.x - s.centre.x);
                Real a1 = std::atan2(s.to.y - s.centre.y, s.to.x - s.centre.x);
                if (!s.ccw) std::swap(a0, a1);
                if (a1 <= a0) a1 += kTwoPi;
                for (int i = 1; i < 16; ++i) {
                    const Real a = a0 + (a1 - a0) * i / 16.0;
                    grow(lo, hi, s.centre + Vec2{std::cos(a), std::sin(a)} * s.radius);
                }
            }
        }
    }
    d.min = lo;
    d.max = hi;
    d.ok = true;
}

} // namespace

SvgDrawing parseSvg(const std::string& text) {
    std::string error;
    XmlReader xml(text);
    std::unique_ptr<XmlNode> root = xml.root(error);
    if (!root) {
        SvgDrawing d;
        d.error = "Not an SVG drawing: " + error;
        return d;
    }
    Reader r;
    r.read(*root);
    SvgDrawing d = std::move(r.out);

    // Ink by default is everything not near white: on the page, white is the
    // paper, and white painted on top of black is a highlight cut out of it.
    // A drawing all in pale colours is all ink, or there would be nothing.
    // And where there are several colours, the lightest is paper too if it
    // is pale and plainly lighter than the darkest: dark work on a pale
    // backing, which is how a badge or a sign is drawn.
    auto luminance = [](const SvgColour& c) { return 0.2126 * c.rgb.x + 0.7152 * c.rgb.y + 0.0722 * c.rgb.z; };
    size_t lightest = 0, darkest = 0;
    for (size_t i = 0; i < d.colours.size(); ++i) {
        if (luminance(d.colours[i]) > luminance(d.colours[lightest])) lightest = i;
        if (luminance(d.colours[i]) < luminance(d.colours[darkest])) darkest = i;
    }
    bool anyInk = false;
    for (size_t i = 0; i < d.colours.size(); ++i) {
        SvgColour& c = d.colours[i];
        c.ink = luminance(c) < 0.95;
        if (d.colours.size() > 1 && i == lightest && luminance(c) >= 0.75 &&
            luminance(c) - luminance(d.colours[darkest]) >= 0.3)
            c.ink = false;
        anyInk = anyInk || c.ink;
    }
    if (!anyInk)
        for (SvgColour& c : d.colours) c.ink = true;

    auto source = std::make_shared<SvgDrawing>(d);
    d.source = source;
    finish(d);
    return d;
}

SvgDrawing recolourSvg(const SvgDrawing& drawing, const std::vector<bool>& ink) {
    if (!drawing.source) return drawing;
    SvgDrawing d = *drawing.source;
    d.source = drawing.source;
    for (size_t i = 0; i < d.colours.size() && i < ink.size(); ++i) d.colours[i].ink = ink[i];
    finish(d);
    if (!d.ok && d.error.empty()) d.error = "None of the colours left is ink";
    return d;
}

SvgDrawing readSvgFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        SvgDrawing d;
        d.error = "Could not open " + path;
        return d;
    }
    std::stringstream ss;
    ss << in.rdbuf();
    return parseSvg(ss.str());
}

// ---- Into a sketch ------------------------------------------------------------

namespace {

Vec2 turned(Vec2 v, int quarterTurns) {
    switch (((quarterTurns % 4) + 4) % 4) {
        case 1: return {-v.y, v.x};
        case 2: return {-v.x, -v.y};
        case 3: return {v.y, -v.x};
        default: return v;
    }
}

} // namespace

size_t outlineCrossings(const std::vector<std::vector<SvgSegment>>& loops) {
    std::vector<const SvgSegment*> segs;
    for (const auto& l : loops)
        for (const SvgSegment& s : l) segs.push_back(&s);
    return findCrossings(segs, {}).size();
}

std::vector<SvgSegment> sketchLoopCurves(const Sketch& sk, const SketchLoop& loop) {
    std::vector<SvgSegment> out;
    auto at = [&](SketchId id) {
        const SketchPoint* p = sk.point(id);
        return p ? p->at : Vec2{};
    };
    for (SketchId id : loop.entities) {
        const SketchEntity* e = sk.entity(id);
        if (!e) continue;
        SvgSegment s;
        switch (e->curve) {
        case SketchCurve::Line:
            s.kind = SvgSegment::Kind::Line;
            s.from = at(e->a);
            s.to = at(e->b);
            out.push_back(s);
            break;
        case SketchCurve::Bezier:
            s.kind = SvgSegment::Kind::Cubic;
            s.from = at(e->a);
            s.c1 = at(e->b);
            s.c2 = at(e->c);
            s.to = at(e->d);
            out.push_back(s);
            break;
        case SketchCurve::Arc:
            s.kind = SvgSegment::Kind::Arc;
            s.centre = at(e->a);
            s.from = at(e->b);
            s.to = at(e->c);
            s.radius = length(s.from - s.centre);
            s.ccw = true;
            out.push_back(s);
            break;
        case SketchCurve::Circle:
            s.kind = SvgSegment::Kind::Arc;
            s.centre = at(e->a);
            s.radius = e->radius;
            s.ccw = true;
            s.from = s.centre + Vec2{e->radius, 0};
            s.to = s.centre - Vec2{e->radius, 0};
            out.push_back(s);
            std::swap(s.from, s.to);
            out.push_back(s);
            break;
        }
    }
    return out;
}

SvgInsert insertSvg(Sketch& sk, const SvgDrawing& d, const SvgPlacement& placement) {
    SvgInsert ins;
    ins.size = d.size();
    const Vec2 mid = (d.min + d.max) * 0.5;
    // Points closer than this are one point: far below anything printable,
    // and far above the rounding a file's decimals leave.
    const Real tol = std::max(1e-9, 1e-9 * std::max(ins.size.x, ins.size.y));

    auto point = [&](Vec2 at) {
        const Vec2 rel = at - mid;
        const SketchId id = sk.addPoint(rel);
        ins.points.push_back({id, rel});
        return id;
    };

    for (const SvgCircle& c : d.circles) {
        const SketchId e = sk.addCircle(point(c.centre), c.radius);
        ins.entities.push_back(e);
        ins.radii.push_back({e, c.radius});
    }

    for (const SvgPath& path : d.paths) {
        if (path.segments.empty()) continue;
        const SketchId first = point(path.segments.front().from);
        SketchId at = first;
        for (size_t i = 0; i < path.segments.size(); ++i) {
            const SvgSegment& s = path.segments[i];
            // The last piece of a closed path ends on the point it began at:
            // that shared point is what makes it a loop.
            const bool last = i + 1 == path.segments.size();
            const SketchId end = (last && path.closed) ? first : point(s.to);
            SketchId e = kNoSketchId;
            switch (s.kind) {
            case SvgSegment::Kind::Line: {
                e = sk.addLine(at, end);
                const Vec2 v = s.to - s.from;
                if (std::fabs(v.y) <= tol && std::fabs(v.x) > tol) {
                    ins.levels.push_back({sk.constrain(SketchRule::Horizontal, e), true});
                } else if (std::fabs(v.x) <= tol && std::fabs(v.y) > tol) {
                    ins.levels.push_back({sk.constrain(SketchRule::Vertical, e), false});
                }
                break;
            }
            case SvgSegment::Kind::Cubic:
                e = sk.addBezier(at, point(s.c1), point(s.c2), end);
                break;
            case SvgSegment::Kind::Arc: {
                const SketchId c = point(s.centre);
                e = s.ccw ? sk.addArc(c, at, end) : sk.addArc(c, end, at);
                ins.radii.push_back({e, s.radius});
                break;
            }
            }
            ins.entities.push_back(e);
            at = end;
        }
    }
    placeSvg(sk, ins, placement);
    return ins;
}

void placeSvg(Sketch& sk, const SvgInsert& ins, const SvgPlacement& placement) {
    const Real s = placement.scale;
    for (const SvgInsert::Point& p : ins.points)
        if (SketchPoint* q = sk.point(p.id)) q->at = placement.centre + turned(p.at * s, placement.quarterTurns);
    for (const SvgInsert::Radius& r : ins.radii)
        if (SketchEntity* e = sk.entity(r.entity)) e->radius = r.radius * s;
    // A quarter turn makes what was level plumb.
    const bool swap = ((placement.quarterTurns % 2) + 2) % 2 == 1;
    for (const SvgInsert::Level& l : ins.levels)
        if (SketchConstraint* k = sk.constraint(l.constraint))
            k->rule = (l.horizontal != swap) ? SketchRule::Horizontal : SketchRule::Vertical;
}

} // namespace tg
