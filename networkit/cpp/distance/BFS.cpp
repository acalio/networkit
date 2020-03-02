/*
 * BFS.cpp
 *
 *  Created on: Jul 23, 2013
 *      Author: Henning
 */

#include <queue>

#include <networkit/distance/BFS.hpp>

namespace NetworKit {

BFS::BFS(const Graph &G, node source, bool storePaths,
         bool storeNodesSortedByDistance, node target)
    : SSSP(G, source, storePaths, storeNodesSortedByDistance, target) {}

void BFS::run() {
    count z = G->upperNodeIdBound();
    reachedNodes = 1;
    sumDist = 0.;

    if (distances.size() < z) {
        distances.resize(z, std::numeric_limits<edgeweight>::max());
        visited.resize(z, ts);
    }

    if (ts++ == 255) {
        ts = 1;
        std::fill(visited.begin(), visited.end(), 0);
    }

    if (storePaths) {
        previous.clear();
        previous.resize(z);
        npaths.clear();
        npaths.resize(z, 0);
        npaths[source] = 1;
    }

    if (storeNodesSortedByDistance) {
        std::vector<node> empty;
        std::swap(nodesSortedByDistance, empty);
    }

    std::queue<node> q;
    q.push(source);
    distances[source] = 0.;
    visited[source] = ts;

    bool breakWhenFound = (target != none);
    while (!q.empty()) {
        node u = q.front();
        q.pop();

        if (storeNodesSortedByDistance) {
            nodesSortedByDistance.push_back(u);
        }
        if (breakWhenFound && target == u) {
            break;
        }

        // insert untouched neighbors into queue
        G->forNeighborsOf(u, [&](node v) {
            if (ts != visited[v]) {
                q.push(v);
                visited[v] = ts;
                distances[v] = distances[u] + 1.;
                sumDist += distances[v];
                ++reachedNodes;
                if (storePaths) {
                    previous[v] = {u};
                    npaths[v] = npaths[u];
                }
            } else if (storePaths && (distances[v] == distances[u] + 1.)) {
                previous[v].push_back(u); // additional predecessor
                npaths[v] += npaths[u]; // all the shortest paths to u are also
                                        // shortest paths to v now
            }
        });
    }

    hasRun = true;
}
} // namespace NetworKit
