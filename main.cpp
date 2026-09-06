#pragma GCC optimize("O3,unroll-loops")

#include <iostream>
#include <vector>
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
#include <cstdlib>

#include "httplib.h"

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
        if (from < 0 || from >= static_cast<int>(adj.size())) return false;
        bool updated = false;
        for (auto& edge : adj[from]) {
            if (edge.target_id == to) {
                edge.congestion_factor = std::max(0.0, factor);
                updated = true;
            }
        }
        return updated;
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
// Service Layer & CLI / HTTP Orchestrator
// ============================================================================

class NavigationService {
private:
    GraphEngine graph;

public:
    NavigationService() = default;

    void bootstrapCampusNetwork() {
        int gate_a  = graph.addNode("Gate_A", 0.0, 0.0);
        int admin   = graph.addNode("Admin_Block", 2.0, 3.0);
        int library = graph.addNode("Central_Library", 5.0, 8.0);
        int lab     = graph.addNode("Turing_Lab", 4.0, 2.0);
        int cafet   = graph.addNode("Cafeteria", 7.0, 4.0);
        int hostel  = graph.addNode("Hostel_Tower", 9.0, 9.0);
        int sports  = graph.addNode("Sports_Complex", 8.0, 1.0);

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

    bool simulateIncident(const std::string& from_label, const std::string& to_label, double congestion_multiplier) {
        int u = graph.getIdByLabel(from_label);
        int v = graph.getIdByLabel(to_label);
        if (u == -1 || v == -1) return false;
        return graph.updateEdgeCongestion(u, v, congestion_multiplier);
    }

    std::string routeToJson(const std::string& start_label, const std::string& end_label) {
        int src = graph.getIdByLabel(start_label);
        int dst = graph.getIdByLabel(end_label);

        if (src == -1 || dst == -1) {
            return "{\"error\": \"Invalid source or destination landmark.\"}";
        }

        UnifiedRoutingEngine dijkstra(AlgorithmType::DIJKSTRA);
        UnifiedRoutingEngine astar(AlgorithmType::ASTAR);

        PathResult d_res = dijkstra.calculateRoute(graph, src, dst);
        PathResult a_res = astar.calculateRoute(graph, src, dst);

        auto serializeResult = [](const PathResult& r) -> std::string {
            std::ostringstream ss;
            ss << "{\n"
               << "      \"found\": " << (r.found ? "true" : "false") << ",\n"
               << "      \"total_cost\": " << (r.found ? r.total_cost : -1.0) << ",\n"
               << "      \"nodes_explored\": " << r.nodes_explored << ",\n"
               << "      \"latency_ns\": " << r.execution_time_ns << ",\n"
               << "      \"path\": [";
            for (size_t i = 0; i < r.label_path.size(); ++i) {
                ss << "\"" << r.label_path[i] << "\"" << (i + 1 < r.label_path.size() ? ", " : "");
            }
            ss << "]\n    }";
            return ss.str();
        };

        std::ostringstream response;
        response << "{\n"
                 << "  \"query\": {\"start\": \"" << start_label << "\", \"end\": \"" << end_label << "\"},\n"
                 << "  \"dijkstra\": " << serializeResult(d_res) << ",\n"
                 << "  \"astar\": " << serializeResult(a_res) << "\n"
                 << "}";
        return response.str();
    }
};

// ============================================================================
// Application Entry Point
// ============================================================================

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);

    NavigationService service;
    service.bootstrapCampusNetwork();

    httplib::Server svr;

    //new changes
    // Add CORS header to allow browser visualizers to fetch data
    svr.set_post_routing_handler([](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
    });

    // Handle preflight OPTIONS requests from browsers
    svr.Options(R"(.*)", [](const httplib::Request&, httplib::Response& res) {
        res.status = 200;
    });

    //new changes until this

    svr.Get("/health", [](const httplib::Request&, httplib::Response& res) {
        res.set_content("{\"status\": \"healthy\"}\n", "application/json");
    });

    svr.Get("/route", [&](const httplib::Request& req, httplib::Response& res) {
        std::string src = req.get_param_value("src");
        std::string dst = req.get_param_value("dst");

        if (src.empty() || dst.empty()) {
            res.status = 400;
            res.set_content("{\"error\": \"Query parameters 'src' and 'dst' are required.\"}\n", "application/json");
            return;
        }

        std::string json_output = service.routeToJson(src, dst);
        if (json_output.rfind("{\"error\"", 0) == 0) {
            res.status = 404;
        }
        res.set_content(json_output + "\n", "application/json");
    });

    svr.Post("/incident", [&](const httplib::Request& req, httplib::Response& res) {
        std::string from = req.get_param_value("from");
        std::string to = req.get_param_value("to");
        std::string factor_str = req.get_param_value("factor");

        if (from.empty() || to.empty() || factor_str.empty()) {
            res.status = 400;
            res.set_content("{\"error\": \"Parameters 'from', 'to', and 'factor' are required.\"}\n", "application/json");
            return;
        }

        try {
            double factor = std::stod(factor_str);
            if (service.simulateIncident(from, to, factor)) {
                res.set_content("{\"status\": \"incident recorded\", \"factor\": " + factor_str + "}\n", "application/json");
            } else {
                res.status = 404;
                res.set_content("{\"error\": \"Landmark not found or invalid edge.\"}\n", "application/json");
            }
        } catch (...) {
            res.status = 400;
            res.set_content("{\"error\": \"Invalid numerical factor value.\"}\n", "application/json");
        }
    });

    int port = 10000;
    const char* port_env = std::getenv("PORT");
    if (port_env && *port_env) {
        try {
            port = std::stoi(port_env);
        } catch (...) {
            port = 10000;
        }
    }

    std::cout << "[SERVER STARTUP] Navigation Engine listening on 0.0.0.0:" << port << std::endl;
    
    if (!svr.listen("0.0.0.0", port)) {
        std::cerr << "[FATAL] Failed to bind to port " << port << std::endl;
        return 1;
    }

    return 0;
}