# Pathora: An Adaptive Algorithmic Framework for Indoor Navigation, Crowd Flow Optimization, and Emergency Evacuation

Pathora is a high-performance indoor navigation and crowd evacuation platform. It combines a modern **C++ (C++17)** algorithmic engine with an interactive **HTML5 Canvas, CSS3, and Vanilla JavaScript** dashboard.

---

## 🏗️ Architecture Overview

```
┌────────────────────────────────────────────────────────┐
│     Frontend (HTML5, Modern CSS, Vanilla JS / Canvas)  │
│  - Multi-floor Interactive Canvas (FL 0 & FL 1)        │
│  - Real-Time Hazard, Wall, and Crowd Placement Tools   │
│  - Algorithm Explorer (A*, Dijkstra, GBFS, BFS, DFS)   │
│  - Telemetry Tray: Runtime (μs), Explored Nodes, Cost  │
│  - Side-by-Side Algorithm Benchmark Race               │
└───────────────────────────▲────────────────────────────┘
                            │ HTTP / JSON API (REST)
┌───────────────────────────▼────────────────────────────┐
│              C++ Backend (High Performance)            │
│  - Graph Engine: Multi-floor (Stairs, Elevators, Ramps)│
│  - Search Engine: BFS, DFS, Dijkstra, A*, GBFS         │
│  - Adaptive Engine: Contextual Meta-Heuristic Selector │
│  - Simulation Engine: Diffusion Hazard & Multi-Agent   │
│  - Microsecond Profiler & Telemetry Metrics            │
└────────────────────────────────────────────────────────┘
```

---

## 🚀 Data Structures & Algorithmic Concepts Implemented

1. **Graph Representation:**
   * Multi-layer topological indoor graph lattice with vertical linkages (Stairs, Elevators, Ramps).
   * Adjacency list indexing with spatial coordinates $(x, y, floor)$ and directional edge constraints.

2. **Core Pathfinding Algorithms:**
   * **Breadth-First Search (BFS):** Unweighted minimal-hop search baseline ($O(V + E)$).
   * **Depth-First Search (DFS):** Structural connectivity and exploration ($O(V + E)$).
   * **Dijkstra's Algorithm:** Uniform-cost shortest path optimization with priority queues ($O((V + E) \log V)$).
   * **A\* Search:** Heuristic-driven search with Euclidean spatial heuristics & floor transition penalties ($O(E)$ best).
   * **Greedy Best-First Search (GBFS):** Rapid frontier expansion toward target.

3. **Adaptive Algorithm Selection:**
   * Context-aware algorithm switcher that selects the optimal solver depending on the operational mode (*Navigation*, *Crowd Flow*, or *Evacuation*), graph density, spatial distance, and local risk level.

4. **Multi-Objective Optimization:**
   * Pareto weighted scoring function balancing:
     $$\text{Cost} = w_{\text{dist}} \cdot \text{Distance} + w_{\text{risk}} \cdot \text{Risk} + w_{\text{crowd}} \cdot \text{Congestion}$$

5. **Multi-Agent Simulation & Dynamic Hazards:**
   * Multi-agent crowd flow with capacity-constrained corridors.
   * Cellular automata / diffusion model on graph edges for dynamic hazard spread.
   * Real-time re-routing of agents when current paths are compromised by fire or smoke.

6. **Time & Space Complexity Telemetry:**
   * Microsecond execution measurement ($\mu s$), node expansion count, visited count, and comparative benchmark tables.

---

## 💻 How to Run

1. **C++ Backend:**
   The backend executable `pathora_server.exe` runs an embedded HTTP server on `http://localhost:8080`.
   ```powershell
   ./pathora_server.exe
   ```
   *(Or recompile with: `g++ -std=c++17 -O2 -Ibackend/include backend/src/main.cpp -lws2_32 -o pathora_server.exe`)*

2. **Frontend:**
   The server directly hosts the frontend files from `./frontend`. Open your browser at:
   **[http://localhost:8080](http://localhost:8080)**

---

## 🕹️ Interactive Features
- **Canvas Tools:**
  - 🔍 **Inspect**: Hover over any node to view real-time ID, floor, hazard %, and crowd %.
  - 🔵 **Start / 🔴 Target**: Set customized origin and destination points.
  - 🔥 **Hazard**: Place fire/smoke sources to simulate spreading hazards.
  - 🧱 **Wall**: Toggle non-walkable barriers.
  - 👥 **Crowd**: Inject local congestion hotspots.
- **Floor Switcher**: Toggle between **FL 0 (Ground Atrium)** and **FL 1 (Upper Suites)**.
- **Compare All**: Run a side-by-side race of all 5 algorithms and inspect runtime metrics.
- **Run Dynamic Sim**: Run the continuous multi-agent crowd evacuation loop.
