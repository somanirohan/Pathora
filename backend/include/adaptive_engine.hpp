#pragma once
#include "graph.hpp"
#include "algorithms.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

namespace pathora {

class AdaptiveEngine {
public:
    // Selects the optimal algorithm based on graph state, operational mode, and query constraints
    static PathResult executeAdaptiveSearch(const Graph& graph, PathfindingRequest req) {
        // If in evacuation mode with target == -1 or invalid, find nearest safe emergency exit
        if (req.mode == "evacuation" && (req.targetNodeId == -1 || !graph.hasNode(req.targetNodeId))) {
            int bestExit = findBestEmergencyExit(graph, req.startNodeId, req);
            if (bestExit != -1) {
                req.targetNodeId = bestExit;
            } else {
                PathResult failedRes;
                failedRes.success = false;
                failedRes.statusMessage = "CRITICAL: No accessible emergency exits found from start location!";
                failedRes.metrics.algorithmName = "Adaptive Evacuation Controller";
                return failedRes;
            }
        }

        double nodeCount = static_cast<double>(graph.nodeCount());
        double startTargetDist = graph.computeHeuristic(req.startNodeId, req.targetNodeId);

        // Analyze local hazard & crowd conditions
        double localRisk = graph.getNode(req.startNodeId).hazardLevel;
        double localCrowd = graph.getNode(req.startNodeId).crowdDensity;

        std::string chosenAlg;
        std::string reason;

        if (req.mode == "evacuation") {
            // Under life-threatening hazard or extreme rush:
            // If very high hazard nearby, we prioritize A* with heavy safety weights to dodge fire corridors
            chosenAlg = "astar";
            reason = "Evacuation Mode: A* with risk-weighted heuristic penalty selected to ensure safe egress while minimizing hazard exposure.";
        } else if (req.mode == "crowd_flow") {
            // For crowd balancing, accurate edge weight calculation (congestion & passage capacity) is vital
            chosenAlg = "dijkstra";
            reason = "Crowd Optimization Mode: Multi-cost Dijkstra selected for exact flow-capacity balancing and bottleneck alleviation.";
        } else {
            // Standard Navigation Mode:
            if (startTargetDist > 25.0 && nodeCount > 60) {
                chosenAlg = "astar";
                reason = "Large spatial distance: A* chosen to prune unpromising corridors toward the destination.";
            } else if (localRisk < 0.05 && localCrowd < 0.1 && startTargetDist < 15.0) {
                chosenAlg = "astar";
                reason = "Low congestion query: A* selected for optimal path with minimal compute overhead.";
            } else {
                chosenAlg = "dijkstra";
                reason = "Dense environment or local congestion: Dijkstra chosen to explore alternate unblocked corridors.";
            }
        }

        PathResult result;
        if (chosenAlg == "astar") {
            result = PathfindingAlgorithms::runAStar(graph, req);
        } else if (chosenAlg == "dijkstra") {
            result = PathfindingAlgorithms::runDijkstra(graph, req);
        } else if (chosenAlg == "gbfs") {
            result = PathfindingAlgorithms::runGBFS(graph, req);
        } else if (chosenAlg == "bfs") {
            result = PathfindingAlgorithms::runBFS(graph, req);
        } else {
            result = PathfindingAlgorithms::runDFS(graph, req);
        }

        result.metrics.selectionReason = reason;
        return result;
    }

    // Evaluates multi-exit evacuation by assessing distance, exit capacity, and hazard gradient
    static int findBestEmergencyExit(const Graph& graph, int startNodeId, const PathfindingRequest& req) {
        const auto& exits = graph.getExitNodeIds();
        if (exits.empty()) return -1;

        int bestExit = -1;
        double minScore = std::numeric_limits<double>::infinity();

        for (int exitId : exits) {
            const Node& exitNode = graph.getNode(exitId);
            if (!exitNode.isWalkable || exitNode.hazardLevel > req.maxHazardTolerance) {
                continue; // Exit compromised
            }

            double dist = graph.computeHeuristic(startNodeId, exitId);
            double hazard = exitNode.hazardLevel;
            double crowd = exitNode.crowdDensity;

            // Score: Distance + 20*hazard + 10*crowd
            double score = dist + (hazard * 30.0) + (crowd * 15.0);
            if (score < minScore) {
                minScore = score;
                bestExit = exitId;
            }
        }

        return bestExit;
    }
};

} // namespace pathora
