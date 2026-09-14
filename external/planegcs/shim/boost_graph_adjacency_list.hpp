// Tangent shim: the one part of Boost.Graph planegcs uses -- an undirected graph
// built by adding vertices and edges, and its connected components -- so that
// Boost is not a dependency for the sake of a union-find.
//
// Components are numbered the way boost::connected_components numbers them: in
// order of the lowest vertex each contains, starting at zero.
#pragma once

#include <cstddef>
#include <numeric>
#include <utility>
#include <vector>

namespace boost {

struct vecS {};
struct undirectedS {};

template <typename OutEdgeList = vecS, typename VertexList = vecS, typename Directed = undirectedS>
class adjacency_list {
public:
    std::size_t vertices = 0;
    std::vector<std::pair<std::size_t, std::size_t>> edges;
};

template <typename O, typename V, typename D>
std::size_t add_vertex(adjacency_list<O, V, D>& g) { return g.vertices++; }

template <typename O, typename V, typename D>
void add_edge(std::size_t a, std::size_t b, adjacency_list<O, V, D>& g) {
    g.edges.emplace_back(a, b);
}

template <typename O, typename V, typename D>
std::size_t num_vertices(const adjacency_list<O, V, D>& g) { return g.vertices; }

template <typename O, typename V, typename D, typename Component>
int connected_components(const adjacency_list<O, V, D>& g, Component* out) {
    std::vector<std::size_t> parent(g.vertices);
    std::iota(parent.begin(), parent.end(), std::size_t{0});
    auto find = [&](std::size_t v) {
        while (parent[v] != v) {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        return v;
    };
    for (const auto& [a, b] : g.edges) {
        const std::size_t ra = find(a), rb = find(b);
        if (ra != rb) parent[ra < rb ? rb : ra] = ra < rb ? ra : rb;
    }
    std::vector<int> id(g.vertices, -1);
    int count = 0;
    for (std::size_t v = 0; v < g.vertices; ++v) {
        const std::size_t root = find(v);
        if (id[root] < 0) id[root] = count++;
        out[v] = static_cast<Component>(id[root]);
    }
    return count;
}

} // namespace boost
