#pragma once
#include "graph.hpp"
#include <chrono>
#include <queue>
#include <stack>
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <cmath>

namespace pathora {

class PathfindingAlgorithms {
public:
    // Helper to reconstruct path from parent map
    static std::vector<int> reconstructPath(int start, int target, const std::unordered_map<int, int>& parent) {
        std::vector<int> path;
        int curr = target;
        while (curr != -1) {
            path.push_back(curr);
            auto it = parent.find(curr);
            if (it == parent.end() || curr == start) break;
            curr = it->second;
        }
        std::reverse(path.begin(), path.end());
        if (!path.empty() && path.front() == start) {
            return path;
        }
        return {};
    }

    // Helper to compute path metrics (distance, risk, congestion, composite cost)
    static void computePathMetrics(const Graph& graph, const std::vector<int>& path, const PathfindingRequest& req, AlgorithmMetrics& metrics) {
        metrics.totalPathNodes = static_cast<int>(path.size());
        if (path.size() < 2) return;

        double totalDist = 0.0;
        double totalRisk = 0.0;
        double totalCrowd = 0.0;

        for (size_t i = 0; i < path.size() - 1; ++i) {
            int u = path[i];
            int v = path[i+1];
            const auto& neighbors = graph.getNeighbors(u);
            const Edge* edgePtr = nullptr;
            for (const auto& e : neighbors) {
                if (e.to == v) {
                    edgePtr = &e;
                    break;
                }
            }
            if (edgePtr) {
                totalDist += edgePtr->baseDistance;
                totalRisk += graph.getNode(v).hazardLevel;
                totalCrowd += graph.getNode(v).crowdDensity;
            }
        }
        metrics.pathDistance = totalDist;
        metrics.pathRisk = totalRisk;
        metrics.pathCongestion = totalCrowd;
        metrics.compositeCost = (req.weightDistance * totalDist) +
                                (req.weightRisk * totalRisk * 10.0) +
                                (req.weightCongestion * totalCrowd * 10.0);
    }

    // 1. Breadth-First Search (BFS)
    static PathResult runBFS(const Graph& graph, const PathfindingRequest& req) {
        auto startTime = std::chrono::high_resolution_clock::now();
        PathResult res;
        res.metrics.algorithmName = "Breadth-First Search (BFS)";
        res.metrics.theoreticalTimeComplexity = "O(V + E)";
        res.metrics.theoreticalSpaceComplexity = "O(V)";

        int start = req.startNodeId;
        int target = req.targetNodeId;

        if (!graph.hasNode(start) || !graph.hasNode(target)) {
            res.statusMessage = "Start or Target node does not exist in graph.";
            return res;
        }

        std::queue<int> q;
        std::unordered_set<int> visited;
        std::unordered_map<int, int> parent;

        q.push(start);
        visited.insert(start);
        parent[start] = -1;

        bool found = false;

        while (!q.empty()) {
            int curr = q.front();
            q.pop();
            res.exploredNodeSequence.push_back(curr);
            res.metrics.nodesExpanded++;

            if (curr == target) {
                found = true;
                break;
            }

            for (const auto& edge : graph.getNeighbors(curr)) {
                int next = edge.to;
                if (!graph.getNode(next).isWalkable) continue;
                if (req.avoidStairs && edge.isStairs) continue;
                if (req.profile == MobilityProfile::WHEELCHAIR_ACCESSIBLE && edge.isStairs) continue;
                if (graph.getNode(next).hazardLevel > req.maxHazardTolerance) continue;

                if (visited.find(next) == visited.end()) {
                    visited.insert(next);
                    parent[next] = curr;
                    q.push(next);
                }
            }
        }

        res.metrics.nodesVisited = static_cast<int>(visited.size());

        if (found) {
            res.path = reconstructPath(start, target, parent);
            res.success = !res.path.empty();
            computePathMetrics(graph, res.path, req, res.metrics);
            res.statusMessage = "Path successfully computed via BFS (minimal hop count).";
        } else {
            res.statusMessage = "No reachable path found between start and target under current constraints.";
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        res.metrics.executionTimeMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
        return res;
    }

    // 2. Depth-First Search (DFS)
    static PathResult runDFS(const Graph& graph, const PathfindingRequest& req) {
        auto startTime = std::chrono::high_resolution_clock::now();
        PathResult res;
        res.metrics.algorithmName = "Depth-First Search (DFS)";
        res.metrics.theoreticalTimeComplexity = "O(V + E)";
        res.metrics.theoreticalSpaceComplexity = "O(V)";

        int start = req.startNodeId;
        int target = req.targetNodeId;

        if (!graph.hasNode(start) || !graph.hasNode(target)) {
            res.statusMessage = "Start or Target node does not exist in graph.";
            return res;
        }

        std::stack<int> s;
        std::unordered_set<int> visited;
        std::unordered_map<int, int> parent;

        s.push(start);
        parent[start] = -1;
        bool found = false;

        while (!s.empty()) {
            int curr = s.top();
            s.pop();

            if (visited.find(curr) != visited.end()) continue;
            visited.insert(curr);
            res.exploredNodeSequence.push_back(curr);
            res.metrics.nodesExpanded++;

            if (curr == target) {
                found = true;
                break;
            }

            for (const auto& edge : graph.getNeighbors(curr)) {
                int next = edge.to;
                if (!graph.getNode(next).isWalkable) continue;
                if (req.avoidStairs && edge.isStairs) continue;
                if (req.profile == MobilityProfile::WHEELCHAIR_ACCESSIBLE && edge.isStairs) continue;
                if (graph.getNode(next).hazardLevel > req.maxHazardTolerance) continue;

                if (visited.find(next) == visited.end()) {
                    parent[next] = curr;
                    s.push(next);
                }
            }
        }

        res.metrics.nodesVisited = static_cast<int>(visited.size());

        if (found) {
            res.path = reconstructPath(start, target, parent);
            res.success = !res.path.empty();
            computePathMetrics(graph, res.path, req, res.metrics);
            res.statusMessage = "Path found via DFS (structural exploration).";
        } else {
            res.statusMessage = "DFS was unable to reach target node.";
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        res.metrics.executionTimeMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
        return res;
    }

    // 3. Dijkstra's Algorithm
    static PathResult runDijkstra(const Graph& graph, const PathfindingRequest& req) {
        auto startTime = std::chrono::high_resolution_clock::now();
        PathResult res;
        res.metrics.algorithmName = "Dijkstra's Algorithm";
        res.metrics.theoreticalTimeComplexity = "O((V + E) log V)";
        res.metrics.theoreticalSpaceComplexity = "O(V)";

        int start = req.startNodeId;
        int target = req.targetNodeId;

        if (!graph.hasNode(start) || !graph.hasNode(target)) {
            res.statusMessage = "Start or Target node does not exist in graph.";
            return res;
        }

        // Min-heap: pair<cost, nodeId>
        using QueueItem = std::pair<double, int>;
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;

        std::unordered_map<int, double> dist;
        std::unordered_map<int, int> parent;
        std::unordered_set<int> visited;

        dist[start] = 0.0;
        parent[start] = -1;
        pq.push({0.0, start});

        bool found = false;

        while (!pq.empty()) {
            auto [currDist, curr] = pq.top();
            pq.pop();

            if (visited.find(curr) != visited.end()) continue;
            visited.insert(curr);
            res.exploredNodeSequence.push_back(curr);
            res.metrics.nodesExpanded++;

            if (curr == target) {
                found = true;
                break;
            }

            for (const auto& edge : graph.getNeighbors(curr)) {
                int next = edge.to;
                if (visited.find(next) != visited.end()) continue;

                double cost = graph.computeEdgeCost(edge, req);
                if (std::isinf(cost)) continue;

                double newDist = currDist + cost;
                if (dist.find(next) == dist.end() || newDist < dist[next]) {
                    dist[next] = newDist;
                    parent[next] = curr;
                    pq.push({newDist, next});
                }
            }
        }

        res.metrics.nodesVisited = static_cast<int>(visited.size());

        if (found) {
            res.path = reconstructPath(start, target, parent);
            res.success = !res.path.empty();
            computePathMetrics(graph, res.path, req, res.metrics);
            res.statusMessage = "Optimal cost path found via Dijkstra's Algorithm.";
        } else {
            res.statusMessage = "Target is unreachable via Dijkstra under current constraints.";
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        res.metrics.executionTimeMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
        return res;
    }

    // 4. A* Search
    static PathResult runAStar(const Graph& graph, const PathfindingRequest& req) {
        auto startTime = std::chrono::high_resolution_clock::now();
        PathResult res;
        res.metrics.algorithmName = "A* Search";
        res.metrics.theoreticalTimeComplexity = "O(E) best, O(b^d) worst";
        res.metrics.theoreticalSpaceComplexity = "O(V)";

        int start = req.startNodeId;
        int target = req.targetNodeId;

        if (!graph.hasNode(start) || !graph.hasNode(target)) {
            res.statusMessage = "Start or Target node does not exist in graph.";
            return res;
        }

        // Min-heap: pair<fScore, nodeId>
        using QueueItem = std::pair<double, int>;
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> openSet;

        std::unordered_map<int, double> gScore;
        std::unordered_map<int, double> fScore;
        std::unordered_map<int, int> parent;
        std::unordered_set<int> closedSet;

        gScore[start] = 0.0;
        double hStart = graph.computeHeuristic(start, target);
        fScore[start] = hStart;
        parent[start] = -1;

        openSet.push({fScore[start], start});

        bool found = false;

        while (!openSet.empty()) {
            auto [currF, curr] = openSet.top();
            openSet.pop();

            if (closedSet.find(curr) != closedSet.end()) continue;
            closedSet.insert(curr);
            res.exploredNodeSequence.push_back(curr);
            res.metrics.nodesExpanded++;

            if (curr == target) {
                found = true;
                break;
            }

            for (const auto& edge : graph.getNeighbors(curr)) {
                int next = edge.to;
                if (closedSet.find(next) != closedSet.end()) continue;

                double edgeCost = graph.computeEdgeCost(edge, req);
                if (std::isinf(edgeCost)) continue;

                double tentativeG = gScore[curr] + edgeCost;
                if (gScore.find(next) == gScore.end() || tentativeG < gScore[next]) {
                    parent[next] = curr;
                    gScore[next] = tentativeG;
                    double h = graph.computeHeuristic(next, target);
                    fScore[next] = tentativeG + h;
                    openSet.push({fScore[next], next});
                }
            }
        }

        res.metrics.nodesVisited = static_cast<int>(closedSet.size());

        if (found) {
            res.path = reconstructPath(start, target, parent);
            res.success = !res.path.empty();
            computePathMetrics(graph, res.path, req, res.metrics);
            res.statusMessage = "Optimal heuristic path determined by A* Search.";
        } else {
            res.statusMessage = "A* could not locate a viable path to target.";
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        res.metrics.executionTimeMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
        return res;
    }

    // 5. Greedy Best-First Search (GBFS)
    static PathResult runGBFS(const Graph& graph, const PathfindingRequest& req) {
        auto startTime = std::chrono::high_resolution_clock::now();
        PathResult res;
        res.metrics.algorithmName = "Greedy Best-First Search";
        res.metrics.theoreticalTimeComplexity = "O(b^m) worst, O(d) best";
        res.metrics.theoreticalSpaceComplexity = "O(b^m)";

        int start = req.startNodeId;
        int target = req.targetNodeId;

        if (!graph.hasNode(start) || !graph.hasNode(target)) {
            res.statusMessage = "Start or Target node does not exist in graph.";
            return res;
        }

        // Min-heap: pair<hScore, nodeId>
        using QueueItem = std::pair<double, int>;
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;

        std::unordered_set<int> visited;
        std::unordered_map<int, int> parent;

        parent[start] = -1;
        pq.push({graph.computeHeuristic(start, target), start});

        bool found = false;

        while (!pq.empty()) {
            auto [hVal, curr] = pq.top();
            pq.pop();

            if (visited.find(curr) != visited.end()) continue;
            visited.insert(curr);
            res.exploredNodeSequence.push_back(curr);
            res.metrics.nodesExpanded++;

            if (curr == target) {
                found = true;
                break;
            }

            for (const auto& edge : graph.getNeighbors(curr)) {
                int next = edge.to;
                if (visited.find(next) != visited.end()) continue;
                if (!graph.getNode(next).isWalkable) continue;
                if (req.avoidStairs && edge.isStairs) continue;
                if (req.profile == MobilityProfile::WHEELCHAIR_ACCESSIBLE && edge.isStairs) continue;
                if (graph.getNode(next).hazardLevel > req.maxHazardTolerance) continue;

                parent[next] = curr;
                pq.push({graph.computeHeuristic(next, target), next});
            }
        }

        res.metrics.nodesVisited = static_cast<int>(visited.size());

        if (found) {
            res.path = reconstructPath(start, target, parent);
            res.success = !res.path.empty();
            computePathMetrics(graph, res.path, req, res.metrics);
            res.statusMessage = "Fast approximate path computed via Greedy Best-First Search.";
        } else {
            res.statusMessage = "Greedy search failed to converge to target.";
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        res.metrics.executionTimeMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
        return res;
    }
};

} // namespace pathora
