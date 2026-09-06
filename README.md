C++ Campus Navigation & Pathfinding Benchmark System
# 🧭 CampusNav Engine: High-Performance C++ Spatial Routing & Benchmark Suite

[![C++ Standard](https://img.shields.io/badge/C%2B%2B-17%2F20-blue.svg?style=flat-square&logo=c%2B%2B)](https://en.cppreference.com/w/cpp)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg?style=flat-square)](#compilation--usage)
[![Optimization](https://img.shields.io/badge/GCC-O3%20%7C%20Loops%20Unrolled-orange.svg?style=flat-square)](#performance-engineering)
[![License](https://img.shields.io/badge/license-MIT-lightgrey.svg?style=flat-square)](LICENSE)

A low-latency, modular spatial pathfinding and graph routing engine implemented in modern C++. Engineered to model large-scale spatial networks, evaluate graph exploration heuristics, and dynamically adapt to runtime edge disruptions (e.g., foot-traffic congestion, maintenance barriers) with zero runtime re-allocation overhead.

---

## 📌 Architecture & Design Decisions


           +------------------------------------------------+
           |               NavigationService                |
           |       (Orchestration & Incident Engine)        |
           +-----------------------+------------------------+
                                   |
                 +-----------------+-----------------+
                 |                                   |
                 v                                   v
   +---------------------------+       +---------------------------+
   |        GraphEngine        |       |    IPathfindingRouter     |
   |  - Node spatial registry  |       |       (Strategy API)      |
   |  - Dynamic adjacency list |       +-------------+-------------+
   +---------------------------+                     |
                                       +-------------+-------------+
                                       |                           |
                                       v                           v
                                 [ Dijkstra ]              [ A* Search ]
                                 (Blind h=0)          (Euclidean Heuristic)


* **Strategy Pattern Routing Core**: Decouples graph representations from algorithm implementations via `IPathfindingRouter`, allowing zero-friction extension to Bidirectional A*, Contraction Hierarchies, or custom heuristic evaluators.
* **Unified State Priority Engine**: A single high-throughput `std::priority_queue` core executes both Dijkstra and A* seamlessly, varying only through heuristic injection:
  $$\text{Dijkstra: } h(u) = 0 \quad \longleftrightarrow \quad \text{A* Search: } h(u) = \sqrt{(x_u - x_t)^2 + (y_u - y_t)^2}$$
* **Dynamic Congestion Modifiers**: Edges support decoupled base metric costs alongside runtime multiplier weights ($w_{\text{eff}} = w_{\text{base}} \times c$). This prevents cache-destroying topology rebuilds when simulating traffic or roadblocks.
* **Admissible & Consistent Heuristics**: The Euclidean distance heuristic satisfies the triangle inequality ($h(u) \le c(u, v) + h(v)$), ensuring monotonic $f$-score evaluations, optimal path termination, and preventing multiple node expansions.

---

## 📊 Algorithmic Characteristics

| Attribute | Dijkstra's Algorithm | A* Search (Euclidean) |
| :--- | :--- | :--- |
| **Heuristic Function** | Admissible ($h(n) = 0$) | Admissible & Consistent |
| **Search Space Pattern** | Radial / Uniform-Cost Frontier | Directed Elliptic Cone toward Goal |
| **Time Complexity** | $\mathcal{O}((V + E) \log V)$ | $\mathcal{O}((V + E) \log V)$ worst-case |
| **Space Complexity** | $\mathcal{O}(V)$ | $\mathcal{O}(V)$ |
| **Pruning Efficiency** | Explores baseline breadth | Prunes high-cost non-promising subtrees |
| **Path Optimality** | Guaranteed optimal | Guaranteed optimal |

---

## ⚡ Performance Engineering

* **Vectorized Distance Calculations**: `Point2D::distanceTo` is marked `[[nodiscard]] noexcept` for optimal inlining and SIMD autovectorization under compiler flags `-O3 -mavx2`.
* **Fast I/O & Branch Optimization**: Fast C++ stream decoupling (`sync_with_stdio(false)`) and compiler loop unrolling pragmas (`#pragma GCC optimize("O3,unroll-loops")`).
* **High-Precision Telemetry**: Integrated micro-profiler measuring execution latency in nanoseconds (`std::chrono::high_resolution_clock`) without third-party instrumentation overhead.

---

## 🚀 Compilation & Usage

### Prerequisites
* A C++17 compliant compiler (`g++` 9.0+, `clang++` 10.0+, or Apple Clang).
* CMake 3.16+ (Optional) or direct terminal compilation.

### Quick Run (Terminal)

# Clone the repository
git clone [https://github.com/](https://github.com/)<your-username>/campus-nav-pathfinding.git
cd campus-nav-pathfinding

# Compile with high optimization and standard warnings
g++ -std=c++17 -O3 -Wall -Wextra main.cpp -o campus_nav

# Execute benchmark
./campus_nav

---

## 🔬 Sample Benchmark Output

[PHASE 1: Standard Campus Routing]

===============================================================
 ROUTING QUERY: Gate_A -> Hostel_Tower
===============================================================
Dijkstra     | Cost: 13.50  | Explored: 6   | Latency: 1420 ns
A* (Euclid)  | Cost: 13.50  | Explored: 4   | Latency: 910 ns
---------------------------------------------------------------
Optimal Path: Gate_A -> Admin_Block -> Central_Library -> Hostel_Tower
===============================================================

[PHASE 2: Dynamic Traffic Incident Simulation]
[SYSTEM MONITOR] Incident reported between Central_Library -> Hostel_Tower (Congestion factor updated to 10.00x)

===============================================================
 ROUTING QUERY: Gate_A -> Hostel_Tower
===============================================================
Dijkstra     | Cost: 16.90  | Explored: 7   | Latency: 1850 ns
A* (Euclid)  | Cost: 16.90  | Explored: 5   | Latency: 1120 ns
---------------------------------------------------------------
Optimal Path: Gate_A -> Turing_Lab -> Cafeteria -> Hostel_Tower
===============================================================

---

## 🗺️ Roadmap & Scalability Improvements

* [ ] **Contraction Hierarchies (CH)**: Pre-processing graph shortcuts for sub-microsecond queries on $10^6+$ node networks.
* [ ] **Dynamic Road Closure API**: Support directional one-way locks and bidirectional graph weight mutations via JSON ingestion.
* [ ] **Custom Linear Allocators**: Replace default priority queue dynamic memory spikes with an indexed arena/flat buffer allocator.
* [ ] **GeoJSON Importer**: Direct parser for OpenStreetMap (OSM) highway network exports.

---

## 📄 License

Distributed under the MIT License. See `LICENSE` for more information.
---
