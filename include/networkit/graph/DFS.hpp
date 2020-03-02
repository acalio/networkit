// networkit-format

#ifndef NETWORKIT_GRAPH_DFS_HPP_
#define NETWORKIT_GRAPH_DFS_HPP_

#include <stack>
#include <vector>
#include <utility>

#include <networkit/graph/Graph.hpp>

namespace NetworKit {

namespace Traversal {

/**
 * Iterate over nodes in depth-first search order starting from the given source node.
 *
 * @param G The input graph.
 * @param source The source node.
 * @param handle Takes a node as input parameter.
 */
template <typename L>
void DFSfrom(const Graph &G, node source, L handle) {
    std::vector<bool> marked(G.upperNodeIdBound());
    std::stack<node> s;
    s.push(source); // enqueue root
    marked[source] = true;
    do {
        const auto u = s.top();
        s.pop();
        // apply function
        handle(u);
        G.forNeighborsOf(u, [&](node v) {
            if (!marked[v]) {
                s.push(v);
                marked[v] = true;
            }
        });
    } while (!s.empty());
}


/**
 * Iterate over nodes in depth-first search order starting from the given source node.
 *
 * @param G The input graph.
 * @param source The source node.
 * @param limit Depth limit
 * @param handle Takes a node and its distance from the source as input parameters.
 */
template <typename L>
void DFSfromWithLimit(const Graph &G, node source, int limit, L handle) {
    std::vector<bool> marked(G.upperNodeIdBound());
    std::stack<std::pair<node, int>> s;
    s.push(source); // enqueue root
    marked[source] = true;
    node u;
    int depth;
    do {

        std::tie(u, depth) = s.top();
        s.pop();
        
        // apply function
        handle(u, depth);
        
        if(depth>=limit) continue;
        
        G.forNeighborsOf(u, [&](node v) {
            if (!marked[v]) {
                s.push(std::make_pair(v, depth+1));
                marked[v] = true;
            }
        });
    } while (!s.empty());
}

/**
 * Iterate over edges in depth-first search order starting from the given source node.
 *
 * @param G The input graph.
 * @param source The source node.
 * @param handle Takes a node as input parameter.
 */
template <typename L>
void DFSEdgesFrom(const Graph &G, node source, L handle) {
    std::vector<bool> marked(G.upperNodeIdBound());
    std::stack<node> s;
    s.push(source); // enqueue root
    marked[source] = true;
    do {
        const auto u = s.top();
        s.pop();
        // apply function
        G.forNeighborsOf(u, [&](node, node v, edgeweight w, edgeid eid) {
            if (!marked[v]) {
                handle(u, v, w, eid);
                s.push(v);
                marked[v] = true;
            }
        });
    } while (!s.empty());
}

} // namespace Traversal

} // namespace NetworKit
#endif // NETWORKIT_GRAPH_DFS_HPP_
