#pragma GCC optimize("O3,unroll-loops")

#include <iostream>
#include <vector>
#include "httplib.h"
#include <queue>
#include <cmath>
#include <string>
#include <sstream>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include <chrono>
#include <limits>
#include <iomanip>

// ============================================================================
// Core Domain Models & Definitions
// ============================================================================

constexpr double INF = std::numeric_limits<double>::infinity();

enum class AlgorithmType {
    DIJKSTRA,
    ASTAR
};

struct Point2D {
    double x{0.0};
    double y{0.0};

    [[nodiscard]] double distanceTo(const Point2D& other) const noexcept {
        const double dx = x - other.x;
        const double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

struct LandmarkNode {
    int id{-1};
    std::string label;
    Point2D coordinates;
};

struct DirectedEdge {
    int target_id{-1};
    double base_weight{0.0};
    double congestion_factor{1.0}; // Dynamic edge scaling multiplier

    [[nodiscard]] double getEffectiveWeight() const noexcept {
        return base_weight * congestion_factor;
    }
};

struct PathResult {
    bool found{false};
    double total_cost{INF};
    int nodes_explored{0};
    long long execution_time_ns{0};
    std::vector<int> node_path;
    std::vector<std::string> label_path;
};

// ============================================================================
// Graph Storage & Management Engine
// ============================================================================

class GraphEngine {
private:
    std::unordered_map<std::string, int> label_to_id;
    std::vector<LandmarkNode> nodes;
    std::vector<std::vector<DirectedEdge>> adj;

public:
    GraphEngine() = default;

    int addNode(const std::string& label, double x, double y) {
        if (label_to_id.find(label) != label_to_id.end()) {
            return label_to_id[label];
        }
        int new_id = static_cast<int>(nodes.size());
        nodes.push_back({new_id, label, {x, y}});
        adj.emplace_back();
        label_to_id[label] = new_id;
        return new_id;
    }

    void addEdge(int from, int to, double weight, bool bidirectional = false) {
        if (from >= 0 && from < static_cast<int>(nodes.size()) && 
            to >= 0 && to < static_cast<int>(nodes.size())) {
            adj[from].push_back({to, weight, 1.0});
            if (bidirectional) {
                adj[to].push_back({from, weight, 1.0});
            }
        }
    }

    bool updateEdgeCongestion(int from, int to, double factor) {
        if (from >= static_cast<int>(adj.size())) return false;
        for (auto& edge : adj[from]) {
            if (edge.target_id == to) {
                edge.congestion_factor = std::max(0.0, factor);
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] int getNodeCount() const noexcept { return static_cast<int>(nodes.size()); }
    [[nodiscard]] const LandmarkNode& getNode(int id) const { return nodes.at(id); }
    [[nodiscard]] const std::vector<DirectedEdge>& getOutgoingEdges(int id) const { return adj.at(id); }
    
    [[nodiscard]] int getIdByLabel(const std::string& label) const {
        auto it = label_to_id.find(label);
        return (it != label_to_id.end()) ? it->second : -1;
    }
};

// ============================================================================
// Pathfinding Strategy Interface & Implementations
// ============================================================================

class IPathfindingRouter {
public:
    virtual ~IPathfindingRouter() = default;
    [[nodiscard]] virtual PathResult calculateRoute(const GraphEngine& graph, int source, int target) = 0;
};

class UnifiedRoutingEngine : public IPathfindingRouter {
private:
    AlgorithmType algorithm;

    [[nodiscard]] double computeHeuristic(const GraphEngine& graph, int u, int target) const noexcept {
        if (algorithm == AlgorithmType::DIJKSTRA) {
            return 0.0;
        }
        return graph.getNode(u).coordinates.distanceTo(graph.getNode(target).coordinates);
    }

public:
    explicit UnifiedRoutingEngine(AlgorithmType algo) : algorithm(algo) {}

    PathResult calculateRoute(const GraphEngine& graph, int source, int target) override {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        int n = graph.getNodeCount();
        if (source < 0 || source >= n || target < 0 || target >= n) {
            return {};
        }

        std::vector<double> g_score(n, INF);
        std::vector<int> parent(n, -1);
        std::vector<bool> closed_set(n, false);

        // Min-heap storing: {f_score, node_id}
        using State = std::pair<double, int>;
        std::priority_queue<State, std::vector<State>, std::greater<State>> open_set;

        g_score[source] = 0.0;
        open_set.push({computeHeuristic(graph, source, target), source});

        int nodes_explored = 0;
        bool target_reached = false;

        while (!open_set.empty()) {
            auto [current_f, u] = open_set.top();
            open_set.pop();

            if (closed_set[u]) continue;
            closed_set[u] = true;
            nodes_explored++;

            if (u == target) {
                target_reached = true;
                break;
            }

            for (const auto& edge : graph.getOutgoingEdges(u)) {
                int v = edge.target_id;
                if (closed_set[v]) continue;

                double tentative_g = g_score[u] + edge.getEffectiveWeight();
                if (tentative_g < g_score[v]) {
                    g_score[v] = tentative_g;
                    parent[v] = u;
                    double f_score = tentative_g + computeHeuristic(graph, v, target);
                    open_set.push({f_score, v});
                }
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();

        PathResult result;
        result.found = target_reached;
        result.total_cost = target_reached ? g_score[target] : INF;
        result.nodes_explored = nodes_explored;
        result.execution_time_ns = duration;

        if (target_reached) {
            for (int curr = target; curr != -1; curr = parent[curr]) {
                result.node_path.push_back(curr);
            }
            std::reverse(result.node_path.begin(), result.node_path.end());

            for (int id : result.node_path) {
                result.label_path.push_back(graph.getNode(id).label);
            }
        }

        return result;
    }
};

// ============================================================================
// Service Layer & CLI Orchestrator
// ============================================================================

class NavigationService {
private:
    GraphEngine graph;

public:
    NavigationService() = default;

    void bootstrapCampusNetwork() {
        // Node Registration (X, Y in hundred-meter coordinate units)
        int gate_a = graph.addNode("Gate_A", 0.0, 0.0);
        int admin  = graph.addNode("Admin_Block", 2.0, 3.0);
        int library = graph.addNode("Central_Library", 5.0, 8.0);
        int lab    = graph.addNode("Turing_Lab", 4.0, 2.0);
        int cafet  = graph.addNode("Cafeteria", 7.0, 4.0);
        int hostel = graph.addNode("Hostel_Tower", 9.0, 9.0);
        int sports = graph.addNode("Sports_Complex", 8.0, 1.0);

        // Edge Registration (Distance / Nominal Transit Costs)
        graph.addEdge(gate_a, admin, 3.6, true);
        graph.addEdge(gate_a, lab, 4.4, true);
        graph.addEdge(admin, library, 5.8, true);
        graph.addEdge(admin, lab, 2.2, true);
        graph.addEdge(lab, cafet, 3.6, true);
        graph.addEdge(lab, sports, 4.1, true);
        graph.addEdge(library, cafet, 4.4, true);
        graph.addEdge(library, hostel, 4.1, true);
        graph.addEdge(cafet, hostel, 5.3, true);
        graph.addEdge(sports, cafet, 3.1, true);
    }

    void simulateIncident(const std::string& from_label, const std::string& to_label, double congestion_multiplier) {
        int u = graph.getIdByLabel(from_label);
        int v = graph.getIdByLabel(to_label);
        if (graph.updateEdgeCongestion(u, v, congestion_multiplier)) {
            std::cout << "[SYSTEM MONITOR] Incident reported between " 
                      << from_label << " -> " << to_label 
                      << " (Congestion factor updated to " << congestion_multiplier << "x)\n";
        }
    }

    void runBenchmark(const std::string& start_label, const std::string& end_label) {
        int src = graph.getIdByLabel(start_label);
        int dst = graph.getIdByLabel(end_label);

        if (src == -1 || dst == -1) {
            std::cout << "[ERROR] Invalid landmark queried.\n";
            return;
        }

        UnifiedRoutingEngine dijkstra_engine(AlgorithmType::DIJKSTRA);
        UnifiedRoutingEngine astar_engine(AlgorithmType::ASTAR);

        PathResult dijkstra_res = dijkstra_engine.calculateRoute(graph, src, dst);
        PathResult astar_res = astar_engine.calculateRoute(graph, src, dst);

        std::cout << "\n===============================================================\n";
        std::cout << " ROUTING QUERY: " << start_label << " -> " << end_label << "\n";
        std::cout << "===============================================================\n";

        auto printMetricRow = [](const std::string& algo, const PathResult& res) {
            std::cout << std::left << std::setw(12) << algo 
                      << " | Cost: " << std::setw(6) << std::fixed << std::setprecision(2) << res.total_cost
                      << " | Explored: " << std::setw(3) << res.nodes_explored
                      << " | Latency: " << std::setw(6) << res.execution_time_ns << " ns\n";
        };

        printMetricRow("Dijkstra", dijkstra_res);
        printMetricRow("A* (Euclid)", astar_res);

        std::cout << "---------------------------------------------------------------\n";
        std::cout << "Optimal Path: ";
        for (size_t i = 0; i < astar_res.label_path.size(); ++i) {
            std::cout << astar_res.label_path[i] << (i + 1 < astar_res.label_path.size() ? " -> " : "");
        }
        std::cout << "\n===============================================================\n\n";
    }
};

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);

    NavigationService nav_service;
    nav_service.bootstrapCampusNetwork();

    // Baseline queries
    std::cout << "[PHASE 1: Standard Campus Routing]\n";
    nav_service.runBenchmark("Gate_A", "Hostel_Tower");

    // Dynamic condition change (Simulate heavy maintenance / roadblock between Library and Hostel)
    std::cout << "[PHASE 2: Dynamic Traffic Incident Simulation]\n";
    nav_service.simulateIncident("Central_Library", "Hostel_Tower", 10.0);
    nav_service.runBenchmark("Gate_A", "Hostel_Tower");


    //to deploy

    NavigationService service;
    service.bootstrapCampusNetwork();

    httplib::Server svr;

    // Health check endpoint
    svr.Get("/health", [](const httplib::Request&, httplib::Response& res) {
        res.set_content("{\"status\": \"healthy\"}", "application/json");
    });

    // Routing endpoint: e.g., /route?src=Gate_A&dst=Hostel_Tower
    svr.Get("/route", [&](const httplib::Request& req, httplib::Response& res) {
        std::string src = req.get_param_value("src");
        std::string dst = req.get_param_value("dst");

        if (src.empty() || dst.empty()) {
            res.status = 400;
            res.set_content("{\"error\": \"Query parameters 'src' and 'dst' are required.\"}", "application/json");
            return;
        }

        // Use your service to compute routes and return serialized results
        std::ostringstream out;
        out << "{\n"
            << "  \"source\": \"" << src << "\",\n"
            << "  \"target\": \"" << dst << "\",\n"
            << "  \"engine\": \"UnifiedRoutingEngine\"\n"
            << "}";

        res.set_content(out.str(), "application/json");
    });

    // Cloud services inject PORT environment variable dynamically
    const char* port_env = std::getenv("PORT");
    int port = port_env ? std::stoi(port_env) : 8080;

    std::cout << "Server active on port " << port << "...\n";
    svr.listen("0.0.0.0", port);

    return 0;
}