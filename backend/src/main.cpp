#define WIN32_LEAN_AND_MEAN
#include "httplib.h"
#include "json.hpp"
#include "graph.hpp"
#include "algorithms.hpp"
#include "adaptive_engine.hpp"
#include "simulation_engine.hpp"
#include "environment_builder.hpp"

#include <iostream>
#include <mutex>

using json = nlohmann::json;
using namespace pathora;

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  PATHORA: Adaptive Algorithmic Engine Initializing...    \n";
    std::cout << "=========================================================\n";

    Graph graph;
    EnvironmentBuilder::buildDefaultFloorplan(graph);
    SimulationEngine simEngine(graph);
    std::mutex mtx;

    httplib::Server svr;

    // Enable CORS for frontend integration
    svr.set_pre_routing_handler([](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS, PUT, DELETE");
        res.set_header("Access-Control-Allow-Headers", "Content-Type, Authorization");
        if (req.method == "OPTIONS") {
            res.status = 200;
            return httplib::Server::HandlerResponse::Handled;
        }
        return httplib::Server::HandlerResponse::Unhandled;
    });

    // 1. GET /api/graph -> Returns full graph state (nodes, edges, floors)
    svr.Get("/api/graph", [&](const httplib::Request&, httplib::Response& res) {
        std::lock_guard<std::mutex> lock(mtx);
        json jNodes = json::array();
        for (const auto& kv : graph.getAllNodes()) {
            const Node& n = kv.second;
            jNodes.push_back({
                {"id", n.id},
                {"floor", n.floor},
                {"x", n.x},
                {"y", n.y},
                {"type", static_cast<int>(n.type)},
                {"label", n.label},
                {"isWalkable", n.isWalkable},
                {"hazardLevel", n.hazardLevel},
                {"crowdDensity", n.crowdDensity},
                {"capacity", n.capacity},
                {"currentAgents", n.currentAgents}
            });
        }

        json jEdges = json::array();
        for (const auto& kv : graph.getAdjList()) {
            for (const auto& e : kv.second) {
                // Return unique or directional edges
                jEdges.push_back({
                    {"from", e.from},
                    {"to", e.to},
                    {"distance", e.baseDistance},
                    {"passageWidth", e.passageWidth},
                    {"isStairs", e.isStairs},
                    {"isElevator", e.isElevator},
                    {"isRamp", e.isRamp}
                });
            }
        }

        json jExits = graph.getExitNodeIds();

        json resp = {
            {"nodes", jNodes},
            {"edges", jEdges},
            {"exitNodeIds", jExits},
            {"nodeCount", graph.nodeCount()},
            {"edgeCount", graph.edgeCount()}
        };

        res.set_content(resp.dump(), "application/json");
    });

    // 2. POST /api/navigate -> Solves single path finding
    svr.Post("/api/navigate", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            auto body = json::parse(req.body);
            PathfindingRequest pReq;
            pReq.startNodeId = body.value("startNodeId", 1);
            pReq.targetNodeId = body.value("targetNodeId", 5);
            pReq.algorithm = body.value("algorithm", "adaptive");
            pReq.mode = body.value("mode", "navigation");
            
            std::string mobility = body.value("mobility", "standard");
            if (mobility == "wheelchair") {
                pReq.profile = MobilityProfile::WHEELCHAIR_ACCESSIBLE;
                pReq.avoidStairs = true;
            } else if (mobility == "evacuee") {
                pReq.profile = MobilityProfile::EMERGENCY_EVACUEE;
            } else if (mobility == "responder") {
                pReq.profile = MobilityProfile::FIRST_RESPONDER;
                pReq.maxHazardTolerance = 0.8;
            }

            pReq.weightDistance = body.value("weightDistance", 1.0);
            pReq.weightRisk = body.value("weightRisk", 2.5);
            pReq.weightCongestion = body.value("weightCongestion", 1.5);
            pReq.avoidStairs = body.value("avoidStairs", pReq.avoidStairs);

            std::lock_guard<std::mutex> lock(mtx);
            PathResult pResult;

            if (pReq.algorithm == "adaptive") {
                pResult = AdaptiveEngine::executeAdaptiveSearch(graph, pReq);
            } else if (pReq.algorithm == "astar") {
                pResult = PathfindingAlgorithms::runAStar(graph, pReq);
            } else if (pReq.algorithm == "dijkstra") {
                pResult = PathfindingAlgorithms::runDijkstra(graph, pReq);
            } else if (pReq.algorithm == "gbfs") {
                pResult = PathfindingAlgorithms::runGBFS(graph, pReq);
            } else if (pReq.algorithm == "bfs") {
                pResult = PathfindingAlgorithms::runBFS(graph, pReq);
            } else if (pReq.algorithm == "dfs") {
                pResult = PathfindingAlgorithms::runDFS(graph, pReq);
            } else {
                pResult = AdaptiveEngine::executeAdaptiveSearch(graph, pReq);
            }

            json out = {
                {"success", pResult.success},
                {"path", pResult.path},
                {"exploredNodeSequence", pResult.exploredNodeSequence},
                {"statusMessage", pResult.statusMessage},
                {"metrics", {
                    {"algorithmName", pResult.metrics.algorithmName},
                    {"executionTimeMicroseconds", pResult.metrics.executionTimeMicroseconds},
                    {"nodesExpanded", pResult.metrics.nodesExpanded},
                    {"nodesVisited", pResult.metrics.nodesVisited},
                    {"pathDistance", pResult.metrics.pathDistance},
                    {"pathRisk", pResult.metrics.pathRisk},
                    {"pathCongestion", pResult.metrics.pathCongestion},
                    {"compositeCost", pResult.metrics.compositeCost},
                    {"totalPathNodes", pResult.metrics.totalPathNodes},
                    {"theoreticalTimeComplexity", pResult.metrics.theoreticalTimeComplexity},
                    {"theoreticalSpaceComplexity", pResult.metrics.theoreticalSpaceComplexity},
                    {"selectionReason", pResult.metrics.selectionReason}
                }}
            };

            res.set_content(out.dump(), "application/json");
        } catch (const std::exception& e) {
            json err = {{"error", e.what()}};
            res.status = 400;
            res.set_content(err.dump(), "application/json");
        }
    });

    // 3. POST /api/compare -> Runs all algorithms side-by-side on same problem for benchmark analytics
    svr.Post("/api/compare", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            auto body = json::parse(req.body);
            PathfindingRequest pReq;
            pReq.startNodeId = body.value("startNodeId", 1);
            pReq.targetNodeId = body.value("targetNodeId", 10);
            pReq.mode = body.value("mode", "navigation");

            std::lock_guard<std::mutex> lock(mtx);

            PathResult rAStar = PathfindingAlgorithms::runAStar(graph, pReq);
            PathResult rDijkstra = PathfindingAlgorithms::runDijkstra(graph, pReq);
            PathResult rGBFS = PathfindingAlgorithms::runGBFS(graph, pReq);
            PathResult rBFS = PathfindingAlgorithms::runBFS(graph, pReq);
            PathResult rDFS = PathfindingAlgorithms::runDFS(graph, pReq);

            auto formatResult = [](const PathResult& pr) {
                return json{
                    {"algorithm", pr.metrics.algorithmName},
                    {"success", pr.success},
                    {"timeMicroseconds", pr.metrics.executionTimeMicroseconds},
                    {"nodesExpanded", pr.metrics.nodesExpanded},
                    {"pathDistance", pr.metrics.pathDistance},
                    {"compositeCost", pr.metrics.compositeCost},
                    {"pathLength", pr.path.size()},
                    {"timeComplexity", pr.metrics.theoreticalTimeComplexity},
                    {"spaceComplexity", pr.metrics.theoreticalSpaceComplexity}
                };
            };

            json resp = {
                {"benchmark", {
                    formatResult(rAStar),
                    formatResult(rDijkstra),
                    formatResult(rGBFS),
                    formatResult(rBFS),
                    formatResult(rDFS)
                }}
            };

            res.set_content(resp.dump(), "application/json");
        } catch (const std::exception& e) {
            json err = {{"error", e.what()}};
            res.status = 400;
            res.set_content(err.dump(), "application/json");
        }
    });

    // 4. POST /api/node/update -> Set node properties (hazard, crowd, walkable)
    svr.Post("/api/node/update", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            auto body = json::parse(req.body);
            int nodeId = body.at("nodeId");
            std::lock_guard<std::mutex> lock(mtx);

            if (body.contains("hazard")) {
                double h = body["hazard"];
                graph.setNodeHazard(nodeId, h);
                if (h > 0.6) {
                    simEngine.addHazardSource(nodeId);
                }
            }
            if (body.contains("crowd")) {
                graph.setNodeCrowd(nodeId, body["crowd"]);
            }
            if (body.contains("walkable")) {
                graph.setNodeWalkable(nodeId, body["walkable"]);
            }

            res.set_content(json{{"status", "updated"}}.dump(), "application/json");
        } catch (const std::exception& e) {
            json err = {{"error", e.what()}};
            res.status = 400;
            res.set_content(err.dump(), "application/json");
        }
    });

    // 5. POST /api/simulation/spawn -> Spawn crowds of agents
    svr.Post("/api/simulation/spawn", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            auto body = json::parse(req.body);
            int count = body.value("count", 15);
            std::string profileStr = body.value("profile", "standard");
            MobilityProfile prof = (profileStr == "wheelchair") ? MobilityProfile::WHEELCHAIR_ACCESSIBLE : MobilityProfile::STANDARD;

            std::lock_guard<std::mutex> lock(mtx);
            simEngine.spawnAgents(count, prof);

            res.set_content(json{{"status", "spawned"}, {"agentCount", simEngine.getAgents().size()}}.dump(), "application/json");
        } catch (const std::exception& e) {
            json err = {{"error", e.what()}};
            res.status = 400;
            res.set_content(err.dump(), "application/json");
        }
    });

    // 6. POST /api/simulation/step -> Advance simulation tick
    svr.Post("/api/simulation/step", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            auto body = json::parse(req.body.empty() ? "{}" : req.body);
            bool spreadHazard = body.value("spreadHazard", true);

            std::lock_guard<std::mutex> lock(mtx);
            simEngine.stepSimulation(spreadHazard);

            json jAgents = json::array();
            for (const auto& a : simEngine.getAgents()) {
                jAgents.push_back({
                    {"id", a.id},
                    {"currentNodeId", a.currentNodeId},
                    {"targetNodeId", a.targetNodeId},
                    {"evacuated", a.evacuated},
                    {"injured", a.injured},
                    {"profile", static_cast<int>(a.profile)}
                });
            }

            json resp = {
                {"tick", simEngine.getTickCount()},
                {"safeEvacuated", simEngine.getSafeEvacuatedCount()},
                {"trapped", simEngine.getTrappedCount()},
                {"agents", jAgents}
            };

            res.set_content(resp.dump(), "application/json");
        } catch (const std::exception& e) {
            json err = {{"error", e.what()}};
            res.status = 400;
            res.set_content(err.dump(), "application/json");
        }
    });

    // 7. POST /api/reset -> Resets floorplan and simulation state
    svr.Post("/api/reset", [&](const httplib::Request&, httplib::Response& res) {
        std::lock_guard<std::mutex> lock(mtx);
        EnvironmentBuilder::buildDefaultFloorplan(graph);
        simEngine.clearSimulation();
        res.set_content(json{{"status", "reset_successful"}}.dump(), "application/json");
    });

    // Serve frontend static directory directly if requested
    svr.set_mount_point("/", "./frontend");

    int port = 8080;
    std::cout << "[Pathora Server] Listening on http://localhost:" << port << std::endl;
    std::cout << "[Pathora Server] Serving frontend files from ./frontend" << std::endl;
    svr.listen("127.0.0.1", port);

    return 0;
}
