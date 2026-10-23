#pragma once
#include "types.hpp"
#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <limits>

namespace pathora {

class Graph {
private:
    std::unordered_map<int, Node> nodes_;
    std::unordered_map<int, std::vector<Edge>> adjList_;
    std::vector<int> exitNodeIds_;

public:
    Graph() = default;

    void addNode(const Node& node) {
        nodes_[node.id] = node;
        if (node.type == NodeType::EMERGENCY_EXIT) {
            if (std::find(exitNodeIds_.begin(), exitNodeIds_.end(), node.id) == exitNodeIds_.end()) {
                exitNodeIds_.push_back(node.id);
            }
        }
    }

    void addEdge(int u, int v, double passageWidth = 2.0, bool isStairs = false, bool isElevator = false, bool isRamp = false, bool isDirectional = false) {
        if (nodes_.find(u) == nodes_.end() || nodes_.find(v) == nodes_.end()) {
            return;
        }

        const Node& nu = nodes_[u];
        const Node& nv = nodes_[v];
        double dx = nu.x - nv.x;
        double dy = nu.y - nv.y;
        double dz = (nu.floor - nv.floor) * 4.0; // Assume 4m vertical height between floors
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < 0.1) dist = 1.0;

        Edge e1{u, v, dist, passageWidth, isStairs, isElevator, isRamp, isDirectional};
        adjList_[u].push_back(e1);

        if (!isDirectional) {
            Edge e2{v, u, dist, passageWidth, isStairs, isElevator, isRamp, false};
            adjList_[v].push_back(e2);
        }
    }

    bool hasNode(int id) const {
        return nodes_.find(id) != nodes_.end();
    }

    const Node& getNode(int id) const {
        auto it = nodes_.find(id);
        if (it == nodes_.end()) {
            throw std::runtime_error("Node not found: " + std::to_string(id));
        }
        return it->second;
    }

    Node& getNodeRef(int id) {
        return nodes_.at(id);
    }

    const std::unordered_map<int, Node>& getAllNodes() const {
        return nodes_;
    }

    const std::vector<Edge>& getNeighbors(int id) const {
        static const std::vector<Edge> emptyNeighbors;
        auto it = adjList_.find(id);
        if (it != adjList_.end()) {
            return it->second;
        }
        return emptyNeighbors;
    }

    const std::unordered_map<int, std::vector<Edge>>& getAdjList() const {
        return adjList_;
    }

    const std::vector<int>& getExitNodeIds() const {
        return exitNodeIds_;
    }

    void setNodeHazard(int id, double hazard) {
        if (hasNode(id)) {
            nodes_[id].hazardLevel = std::clamp(hazard, 0.0, 1.0);
            if (nodes_[id].hazardLevel >= 0.95) {
                nodes_[id].isWalkable = false;
            } else {
                nodes_[id].isWalkable = true;
            }
        }
    }

    void setNodeCrowd(int id, double crowd) {
        if (hasNode(id)) {
            nodes_[id].crowdDensity = std::clamp(crowd, 0.0, 1.0);
        }
    }

    void setNodeWalkable(int id, bool walkable) {
        if (hasNode(id)) {
            nodes_[id].isWalkable = walkable;
        }
    }

    void clearGraph() {
        nodes_.clear();
        adjList_.clear();
        exitNodeIds_.clear();
    }

    size_t nodeCount() const { return nodes_.size(); }

    size_t edgeCount() const {
        size_t count = 0;
        for (const auto& kv : adjList_) {
            count += kv.second.size();
        }
        return count;
    }

    // Heuristic: Euclidean distance including floor multiplier
    double computeHeuristic(int u, int v) const {
        if (!hasNode(u) || !hasNode(v)) return 0.0;
        const Node& n1 = nodes_.at(u);
        const Node& n2 = nodes_.at(v);
        double dx = n1.x - n2.x;
        double dy = n1.y - n2.y;
        double dz = (n1.floor - n2.floor) * 8.0; // Floor change heuristic penalty
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Edge traversal cost with Multi-Objective weights & dynamic obstacles/hazards
    double computeEdgeCost(const Edge& edge, const PathfindingRequest& req) const {
        const Node& targetNode = nodes_.at(edge.to);

        // Check hard constraints
        if (!targetNode.isWalkable) {
            return std::numeric_limits<double>::infinity();
        }

        if (targetNode.hazardLevel > req.maxHazardTolerance) {
            return std::numeric_limits<double>::infinity();
        }

        if (req.avoidStairs && edge.isStairs) {
            return std::numeric_limits<double>::infinity();
        }

        if (req.profile == MobilityProfile::WHEELCHAIR_ACCESSIBLE && edge.isStairs) {
            return std::numeric_limits<double>::infinity();
        }

        // Passage width factor: bottlenecks increase traversal penalty
        double widthFactor = 1.0;
        if (edge.passageWidth > 0.1) {
            widthFactor = 1.0 / std::clamp(edge.passageWidth / 2.0, 0.3, 3.0);
        }

        // Base distance contribution
        double distanceCost = edge.baseDistance * widthFactor;

        // Hazard risk contribution (exponential penalty near hazardous nodes)
        double riskCost = 0.0;
        if (targetNode.hazardLevel > 0.01) {
            riskCost = std::exp(targetNode.hazardLevel * 4.0) * 15.0;
        }

        // Crowd congestion contribution
        double crowdCost = 0.0;
        if (targetNode.crowdDensity > 0.05) {
            crowdCost = std::pow(targetNode.crowdDensity, 2.0) * 25.0;
        }

        double totalCost = (req.weightDistance * distanceCost) +
                           (req.weightRisk * riskCost) +
                           (req.weightCongestion * crowdCost);

        return totalCost;
    }
};

} // namespace pathora
