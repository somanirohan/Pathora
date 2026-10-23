#pragma once
#include "graph.hpp"
#include "adaptive_engine.hpp"
#include <vector>
#include <random>
#include <unordered_map>
#include <algorithm>

namespace pathora {

class SimulationEngine {
private:
    Graph& graph_;
    std::vector<Agent> agents_;
    std::vector<int> hazardSources_;
    int tickCount_ = 0;
    int safeEvacuatedCount_ = 0;
    int trappedCount_ = 0;

public:
    SimulationEngine(Graph& graph) : graph_(graph) {}

    void clearSimulation() {
        agents_.clear();
        hazardSources_.clear();
        tickCount_ = 0;
        safeEvacuatedCount_ = 0;
        trappedCount_ = 0;
    }

    void addHazardSource(int nodeId) {
        if (graph_.hasNode(nodeId)) {
            hazardSources_.push_back(nodeId);
            graph_.setNodeHazard(nodeId, 1.0);
        }
    }

    void spawnAgents(int count, MobilityProfile profile = MobilityProfile::STANDARD) {
        const auto& allNodes = graph_.getAllNodes();
        if (allNodes.empty()) return;

        std::vector<int> candidateNodes;
        for (const auto& kv : allNodes) {
            if (kv.second.isWalkable && kv.second.type != NodeType::EMERGENCY_EXIT && kv.second.hazardLevel < 0.2) {
                candidateNodes.push_back(kv.first);
            }
        }

        if (candidateNodes.empty()) return;

        std::mt19937 rng(1337 + static_cast<int>(agents_.size()));
        std::uniform_int_distribution<size_t> dist(0, candidateNodes.size() - 1);

        for (int i = 0; i < count; ++i) {
            Agent agent;
            agent.id = static_cast<int>(agents_.size()) + 1;
            agent.currentNodeId = candidateNodes[dist(rng)];
            agent.targetNodeId = -1;
            agent.profile = profile;
            agent.speed = (profile == MobilityProfile::WHEELCHAIR_ACCESSIBLE) ? 0.75 : 1.0;
            agent.evacuated = false;
            agent.injured = false;

            // Route agent to emergency exit or random destination
            replanAgent(agent);
            agents_.push_back(agent);
        }

        updateCrowdDensities();
    }

    void replanAgent(Agent& agent) {
        PathfindingRequest req;
        req.startNodeId = agent.currentNodeId;
        req.targetNodeId = agent.targetNodeId;
        req.mode = "evacuation";
        req.profile = agent.profile;
        if (agent.profile == MobilityProfile::WHEELCHAIR_ACCESSIBLE) {
            req.avoidStairs = true;
        }

        PathResult res = AdaptiveEngine::executeAdaptiveSearch(graph_, req);
        if (res.success && !res.path.empty()) {
            agent.currentPath = res.path;
            agent.pathIndex = 0;
            if (agent.targetNodeId == -1 && !res.path.empty()) {
                agent.targetNodeId = res.path.back();
            }
        } else {
            agent.currentPath.clear();
            agent.pathIndex = 0;
        }
    }

    void updateCrowdDensities() {
        // Reset crowd density on all nodes
        for (const auto& kv : graph_.getAllNodes()) {
            graph_.getNodeRef(kv.first).currentAgents = 0.0;
        }

        // Count agents per node
        for (const auto& a : agents_) {
            if (!a.evacuated && !a.injured) {
                if (graph_.hasNode(a.currentNodeId)) {
                    graph_.getNodeRef(a.currentNodeId).currentAgents += 1.0;
                }
            }
        }

        // Normalize crowd density [0.0, 1.0] relative to capacity
        for (const auto& kv : graph_.getAllNodes()) {
            Node& n = graph_.getNodeRef(kv.first);
            n.crowdDensity = std::clamp(n.currentAgents / n.capacity, 0.0, 1.0);
        }
    }

    void stepSimulation(bool spreadHazard = true) {
        tickCount_++;

        // 1. Hazard Spread Model (Cellular Automata / Diffusion on Graph)
        if (spreadHazard && tickCount_ % 2 == 0) {
            std::unordered_map<int, double> deltaHazard;
            for (const auto& kv : graph_.getAllNodes()) {
                int u = kv.first;
                double h = kv.second.hazardLevel;
                if (h > 0.15) {
                    for (const auto& edge : graph_.getNeighbors(u)) {
                        int v = edge.to;
                        // Diffusion rate
                        double diffusion = h * 0.25;
                        deltaHazard[v] = std::max(deltaHazard[v], diffusion);
                    }
                }
            }
            for (const auto& kv : deltaHazard) {
                double currentH = graph_.getNode(kv.first).hazardLevel;
                double newH = std::min(1.0, currentH + kv.second);
                graph_.setNodeHazard(kv.first, newH);
            }
        }

        // 2. Advance Agents along their paths
        for (auto& a : agents_) {
            if (a.evacuated || a.injured) continue;

            const Node& currNode = graph_.getNode(a.currentNodeId);

            // Check if agent caught in fire
            if (currNode.hazardLevel >= 0.85) {
                a.injured = true;
                trappedCount_++;
                continue;
            }

            // If target reached
            if (currNode.type == NodeType::EMERGENCY_EXIT || a.currentNodeId == a.targetNodeId) {
                a.evacuated = true;
                safeEvacuatedCount_++;
                continue;
            }

            // Path invalidation check: next step blocked by hazard or closed?
            bool needReplan = false;
            if (a.currentPath.empty() || a.pathIndex >= static_cast<int>(a.currentPath.size()) - 1) {
                needReplan = true;
            } else {
                int nextNodeId = a.currentPath[a.pathIndex + 1];
                if (!graph_.getNode(nextNodeId).isWalkable || graph_.getNode(nextNodeId).hazardLevel > 0.45) {
                    needReplan = true;
                }
            }

            if (needReplan) {
                replanAgent(a);
            }

            // Advance one step if path is valid
            if (!a.currentPath.empty() && a.pathIndex < static_cast<int>(a.currentPath.size()) - 1) {
                a.pathIndex++;
                a.currentNodeId = a.currentPath[a.pathIndex];

                // Check again if newly reached node is exit
                if (graph_.getNode(a.currentNodeId).type == NodeType::EMERGENCY_EXIT) {
                    a.evacuated = true;
                    safeEvacuatedCount_++;
                }
            }
        }

        updateCrowdDensities();
    }

    // Getters for state
    int getTickCount() const { return tickCount_; }
    int getSafeEvacuatedCount() const { return safeEvacuatedCount_; }
    int getTrappedCount() const { return trappedCount_; }
    const std::vector<Agent>& getAgents() const { return agents_; }
};

} // namespace pathora
