#include "scene/trials.h"

#include "scene/serialize.h"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace tg {
namespace {

void put(std::string& out, const void* bytes, size_t n) {
    out.append(static_cast<const char*>(bytes), n);
}
void putU32(std::string& out, uint32_t v) {
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
}
void putU64(std::string& out, uint64_t v) {
    for (int i = 0; i < 8; ++i) out.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
}
void putF64(std::string& out, double v) {
    uint64_t bits = 0;
    std::memcpy(&bits, &v, sizeof bits);
    putU64(out, bits);
}

struct Cursor {
    const std::string& bytes;
    size_t at = 0;
    bool bad = false;

    bool need(size_t n) {
        if (bad || bytes.size() - at < n) { bad = true; return false; }
        return true;
    }
    uint64_t u(int width) {
        if (!need(static_cast<size_t>(width))) return 0;
        uint64_t v = 0;
        for (int i = 0; i < width; ++i)
            v |= static_cast<uint64_t>(static_cast<unsigned char>(bytes[at + i])) << (8 * i);
        at += static_cast<size_t>(width);
        return v;
    }
    double f64() {
        const uint64_t bits = u(8);
        double v = 0;
        std::memcpy(&v, &bits, sizeof v);
        return v;
    }
    std::string take(size_t n) {
        if (!need(n)) return {};
        std::string s = bytes.substr(at, n);
        at += n;
        return s;
    }
};

bool runSelfTest(SelfTest behaviour) {
    switch (behaviour) {
        case SelfTest::Build:  return true;
        case SelfTest::Refuse: return false;
        case SelfTest::Throw:  throw std::runtime_error("a self-test that throws");
        case SelfTest::Crash: {
            volatile int* nowhere = nullptr;
            return *nowhere == 0;
        }
        case SelfTest::Chatter:
            // Shaped like an answer on purpose, and through both of the ways
            // things get printed.
            {
                static constexpr unsigned char lookalike[8] = {'T', 'T', 'G', 'A', 1, 0, 0, 0};
                std::fwrite(lookalike, 1, sizeof lookalike, stdout);
                std::printf(" chatter from inside a trial\n");
                std::fflush(stdout);
            }
            std::cout << "TTGA more chatter" << std::endl;
            return true;
    }
    return false;
}

bool runFillet(const Body& body, const FilletSpec& spec) {
    Body test = body;
    return filletEdges(test, spec);
}

bool runChain(std::vector<Feature>& features) {
    Body out;
    return evaluateFeatures(features, out);
}

} // namespace

GuardedWork selfTestTrial(SelfTest behaviour) {
    GuardedWork w;
    w.kind = static_cast<uint32_t>(TrialKind::SelfTest);
    w.run = [behaviour] { return runSelfTest(behaviour); };
    w.encode = [behaviour] {
        std::string out;
        putU32(out, static_cast<uint32_t>(behaviour));
        return out;
    };
    return w;
}

GuardedWork filletTrial(Body body, FilletSpec spec) {
    GuardedWork w;
    w.kind = static_cast<uint32_t>(TrialKind::Fillet);
    w.run = [body, spec] { return runFillet(body, spec); };
    w.encode = [body, spec] {
        // The body goes as a chain of one, which is the format that already
        // knows how to carry either kind of body with its names.
        Feature holder;
        holder.kind = FeatureKind::BaseMesh;
        holder.backend = body.isMesh() ? Backend::Mesh : Backend::Brep;
        holder.bakedBody = body;
        const std::string chain = encodeFeatures({holder});

        std::string out;
        putU64(out, chain.size());
        put(out, chain.data(), chain.size());
        putU64(out, spec.salt);
        putU32(out, spec.chamfer ? 1u : 0u);
        putU32(out, static_cast<uint32_t>(spec.edges.size()));
        for (const FilletEdge& e : spec.edges) {
            putU32(out, static_cast<uint32_t>(e.edge));
            putF64(out, e.radius);
            putF64(out, e.endRadius);
        }
        return out;
    };
    return w;
}

GuardedWork chainTrial(std::vector<Feature> features) {
    GuardedWork w;
    w.kind = static_cast<uint32_t>(TrialKind::Chain);
    w.run = [features]() mutable { return runChain(features); };
    w.encode = [features] { return encodeFeatures(features); };
    return w;
}

bool runEncodedTrial(uint32_t kind, const std::string& payload) {
    switch (static_cast<TrialKind>(kind)) {
        case TrialKind::SelfTest: {
            Cursor c{payload};
            const uint32_t behaviour = static_cast<uint32_t>(c.u(4));
            if (c.bad || behaviour > static_cast<uint32_t>(SelfTest::Chatter)) return false;
            return runSelfTest(static_cast<SelfTest>(behaviour));
        }
        case TrialKind::Fillet: {
            Cursor c{payload};
            const uint64_t chainSize = c.u(8);
            if (c.bad || chainSize > payload.size()) return false;
            std::vector<Feature> holder;
            if (!decodeFeatures(c.take(static_cast<size_t>(chainSize)), holder) ||
                holder.size() != 1)
                return false;
            FilletSpec spec;
            spec.salt = c.u(8);
            spec.chamfer = c.u(4) != 0;
            const uint64_t n = c.u(4);
            if (c.bad || n > payload.size()) return false;
            spec.edges.resize(static_cast<size_t>(n));
            for (FilletEdge& e : spec.edges) {
                e.edge = static_cast<EdgeId>(static_cast<int32_t>(c.u(4)));
                e.radius = c.f64();
                e.endRadius = c.f64();
            }
            if (c.bad || c.at != payload.size()) return false;
            return runFillet(holder.front().bakedBody, spec);
        }
        case TrialKind::Chain: {
            std::vector<Feature> features;
            if (!decodeFeatures(payload, features)) return false;
            return runChain(features);
        }
    }
    return false;
}

} // namespace tg
