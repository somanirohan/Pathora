#pragma once
#include "graph.hpp"

namespace pathora {

class EnvironmentBuilder {
public:
    // Creates a realistic 2-floor commercial / institutional building complex
    static void buildDefaultFloorplan(Graph& graph) {
        graph.clearGraph();

        // Floor 0 (Ground Floor: Atrium, Main Entrance/Exits, Corridors, Auditoriums, Rooms)
        // Grid 10 x 8
        int idCounter = 1;
        std::unordered_map<int, std::unordered_map<int, int>> floor0Grid;

        for (int r = 0; r < 8; ++r) {
            for (int c = 0; c < 10; ++c) {
                // Avoid placing nodes in non-navigable structural wall blocks
                if ((r == 2 && c >= 2 && c <= 4) || (r == 5 && c >= 6 && c <= 8)) {
                    continue; // Structural pillars / inaccessible utilities
                }

                Node n;
                n.id = idCounter++;
                n.floor = 0;
                n.x = c * 40 + 40;
                n.y = r * 40 + 40;
                n.isWalkable = true;
                n.hazardLevel = 0.0;
                n.crowdDensity = 0.0;
                n.capacity = 12.0;

                // Special nodes assignment
                if (r == 0 && c == 0) {
                    n.type = NodeType::EMERGENCY_EXIT;
                    n.label = "North-West Exit";
                } else if (r == 0 && c == 9) {
                    n.type = NodeType::EMERGENCY_EXIT;
                    n.label = "North-East Exit";
                } else if (r == 7 && c == 4) {
                    n.type = NodeType::EMERGENCY_EXIT;
                    n.label = "South Main Entrance/Exit";
                } else if (r == 3 && c == 1) {
                    n.type = NodeType::STAIRWAY;
                    n.label = "West Stairwell (FL 0)";
                } else if (r == 3 && c == 8) {
                    n.type = NodeType::ELEVATOR;
                    n.label = "East Elevator (FL 0)";
                } else if (r == 4 && c == 1) {
                    n.type = NodeType::RAMP;
                    n.label = "Accessible Ramp (FL 0)";
                } else if (r >= 3 && r <= 4 && c >= 3 && c <= 6) {
                    n.type = NodeType::NORMAL;
                    n.label = "Central Grand Atrium";
                    n.capacity = 30.0;
                } else {
                    n.type = NodeType::NORMAL;
                    n.label = "Corridor / Room " + std::to_string(n.id);
                }

                floor0Grid[r][c] = n.id;
                graph.addNode(n);
            }
        }

        // Add Floor 0 Edges (Orthogonal 4-connectivity)
        for (int r = 0; r < 8; ++r) {
            for (int c = 0; c < 10; ++c) {
                if (floor0Grid.find(r) != floor0Grid.end() && floor0Grid[r].find(c) != floor0Grid[r].end()) {
                    int u = floor0Grid[r][c];
                    // Right
                    if (floor0Grid.find(r) != floor0Grid.end() && floor0Grid[r].find(c + 1) != floor0Grid[r].end()) {
                        int v = floor0Grid[r][c + 1];
                        double width = (r >= 3 && r <= 4) ? 4.5 : 2.2;
                        graph.addEdge(u, v, width);
                    }
                    // Down
                    if (floor0Grid.find(r + 1) != floor0Grid.end() && floor0Grid[r + 1].find(c) != floor0Grid[r + 1].end()) {
                        int v = floor0Grid[r + 1][c];
                        double width = (c >= 3 && c <= 6) ? 4.5 : 2.2;
                        graph.addEdge(u, v, width);
                    }
                }
            }
        }

        // Floor 1 (Upper Floor: Labs, Conference Rooms, Executive Suites, Corridors)
        std::unordered_map<int, std::unordered_map<int, int>> floor1Grid;
        for (int r = 0; r < 8; ++r) {
            for (int c = 0; c < 10; ++c) {
                // Courtyard void opening above central atrium
                if (r >= 3 && r <= 4 && c >= 4 && c <= 5) {
                    continue; // Open-to-sky atrium void
                }
                if ((r == 1 && c >= 7 && c <= 8) || (r == 6 && c >= 1 && c <= 2)) {
                    continue; // Server rooms / restricted zones
                }

                Node n;
                n.id = idCounter++;
                n.floor = 1;
                n.x = c * 40 + 40;
                n.y = r * 40 + 40;
                n.isWalkable = true;
                n.hazardLevel = 0.0;
                n.crowdDensity = 0.0;
                n.capacity = 8.0;

                if (r == 0 && c == 0) {
                    n.type = NodeType::EMERGENCY_EXIT;
                    n.label = "Upper Skybridge Exit (FL 1)";
                } else if (r == 3 && c == 1) {
                    n.type = NodeType::STAIRWAY;
                    n.label = "West Stairwell (FL 1)";
                } else if (r == 3 && c == 8) {
                    n.type = NodeType::ELEVATOR;
                    n.label = "East Elevator (FL 1)";
                } else if (r == 4 && c == 1) {
                    n.type = NodeType::RAMP;
                    n.label = "Accessible Ramp (FL 1)";
                } else {
                    n.type = NodeType::NORMAL;
                    n.label = "FL 1 Suite " + std::to_string(n.id);
                }

                floor1Grid[r][c] = n.id;
                graph.addNode(n);
            }
        }

        // Add Floor 1 Edges
        for (int r = 0; r < 8; ++r) {
            for (int c = 0; c < 10; ++c) {
                if (floor1Grid.find(r) != floor1Grid.end() && floor1Grid[r].find(c) != floor1Grid[r].end()) {
                    int u = floor1Grid[r][c];
                    // Right
                    if (floor1Grid.find(r) != floor1Grid.end() && floor1Grid[r].find(c + 1) != floor1Grid[r].end()) {
                        int v = floor1Grid[r][c + 1];
                        graph.addEdge(u, v, 2.0);
                    }
                    // Down
                    if (floor1Grid.find(r + 1) != floor1Grid.end() && floor1Grid[r + 1].find(c) != floor1Grid[r + 1].end()) {
                        int v = floor1Grid[r + 1][c];
                        graph.addEdge(u, v, 2.0);
                    }
                }
            }
        }

        // Multi-floor interconnections (Stairway, Elevator, Ramp)
        if (floor0Grid[3].count(1) && floor1Grid[3].count(1)) {
            int stair0 = floor0Grid[3][1];
            int stair1 = floor1Grid[3][1];
            graph.addEdge(stair0, stair1, 1.8, true, false, false); // isStairs = true
        }

        if (floor0Grid[3].count(8) && floor1Grid[3].count(8)) {
            int elev0 = floor0Grid[3][8];
            int elev1 = floor1Grid[3][8];
            graph.addEdge(elev0, elev1, 2.5, false, true, false); // isElevator = true
        }

        if (floor0Grid[4].count(1) && floor1Grid[4].count(1)) {
            int ramp0 = floor0Grid[4][1];
            int ramp1 = floor1Grid[4][1];
            graph.addEdge(ramp0, ramp1, 2.2, false, false, true); // isRamp = true
        }
    }
};

} // namespace pathora
