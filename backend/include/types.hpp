#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>

namespace pathora {

enum class NodeType {
    NORMAL,
    START,
    TARGET,
    EMERGENCY_EXIT,
    STAIRWAY,
    ELEVATOR,
    RAMP,
    OBSTACLE,
    HAZARD_FIRE,
    HAZARD_SMOKE
};

enum class MobilityProfile {
    STANDARD,
    WHEELCHAIR_ACCESSIBLE,
    EMERGENCY_EVACUEE,
    FIRST_RESPONDER
};

struct Node {
    int id;
    int floor;
    int x;
    int y;
    NodeType type = NodeType::NORMAL;
    std::string label;
    bool isWalkable = true;
    double hazardLevel = 0.0;    // 0.0 (safe) to 1.0 (deadly / blocked)
    double crowdDensity = 0.0;   // 0.0 (empty) to 1.0 (jammed)
    double capacity = 10.0;       // Max agent capacity before severe bottleneck penalty
    double currentAgents = 0.0;
};

struct Edge {
    int from;
    int to;
    double baseDistance;
    double passageWidth;        // Width in meters (affects crowd flow rate)
    bool isStairs = false;
    bool isElevator = false;
    bool isRamp = false;
    bool isDirectional = false; // true if one-way
};

struct PathfindingRequest {
    int startNodeId;
    int targetNodeId;           // In evacuation mode, target can be -1 (nearest safe exit)
    std::string algorithm;      // "adaptive", "dijkstra", "astar", "gbfs", "bfs", "dfs", "multi_objective"
    std::string mode;           // "navigation", "crowd_flow", "evacuation"
    MobilityProfile profile = MobilityProfile::STANDARD;
    
    // Multi-objective weights: w_dist * dist + w_risk * risk + w_crowd * crowd
    double weightDistance = 1.0;
    double weightRisk = 2.5;
    double weightCongestion = 1.5;

    // Constraints
    bool avoidStairs = false;
    double maxHazardTolerance = 0.35;
};

struct AlgorithmMetrics {
    std::string algorithmName;
    long long executionTimeMicroseconds = 0;
    int nodesExpanded = 0;
    int nodesVisited = 0;
    double pathDistance = 0.0;
    double pathRisk = 0.0;
    double pathCongestion = 0.0;
    double compositeCost = 0.0;
    int totalPathNodes = 0;
    std::string theoreticalTimeComplexity;
    std::string theoreticalSpaceComplexity;
    std::string selectionReason; // Used by adaptive selection
};

struct PathResult {
    bool success = false;
    std::vector<int> path;                // Node IDs in sequence
    std::vector<int> exploredNodeSequence; // Nodes in the order they were visited (for visualization animation)
    AlgorithmMetrics metrics;
    std::string statusMessage;
};

struct Agent {
    int id;
    int currentNodeId;
    int targetNodeId;
    std::vector<int> currentPath;
    int pathIndex = 0;
    double speed = 1.0;
    MobilityProfile profile = MobilityProfile::STANDARD;
    bool evacuated = false;
    bool injured = false;
};

struct SimulationState {
    int tick = 0;
    std::vector<Agent> agents;
    std::unordered_map<int, double> hazardSpread;
    int activeEvacuations = 0;
    int totalSafeEvacuated = 0;
    int totalTrapped = 0;
};

} // namespace pathora
