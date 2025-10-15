//
// Created by astonek on 9/5/25.
//
#include <iostream>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include "project_utils.h"

// Type aliases for graph-related structures
using Graph = std::unordered_map<int, std::vector<int> >;
using Edge = std::pair<int, int>;
using Edgelist = std::vector<Edge>;
using Vec = std::vector<double>;

// ============================================================================
// Graph Construction Utilities
// ============================================================================

/**
 * @brief Constructs an undirected graph from an edge list.
 * @param edges A list of edges.
 * @return An undirected graph as a dictionary.
 */
Graph build_edges_dict_undirected(const Edgelist& edges) {
    Graph d;
    for (const auto& edge : edges) {
        int u = edge.first;
        int v = edge.second;
        d[u].push_back(v);
        d[v].push_back(u); // Add reverse edge
    }
    return d;
}

/**
 * @brief Constructs a directed graph from an edge list.
 * @param edges A list of edges.
 * @return A directed graph as a dictionary.
 */
Graph build_edges_dict_directed(const Edgelist& edges) {
    Graph d;
    for (const auto& edge : edges) {
        d[edge.first].push_back(edge.second);
    }
    return d;
}

/**
 * @brief Builds a list of all edges that are missing in the undirected graph.
 * @param d The graph.
 * @return List of missing (complementary) edges.
 */
Edgelist build_complementary_edgelist_undirected(const Graph& d) {
    Edgelist complementary;
    int n = d.size(); // assumes all nodes are indexed from 0 to n-1
    for (int u = 0; u < n; ++u) {
        for (int v = u + 1; v < n; ++v) {
            // Check if u and v are connected
            if (d.find(u) == d.end() || d.find(v) == d.end()) continue;
            if (std::find(d.at(u).begin(), d.at(u).end(), v) == d.at(u).end()) {
                // If not connected, add edge in both directions
                complementary.emplace_back(u, v);
                complementary.emplace_back(v, u);
            }
        }
    }
    return complementary;
}

/**
 * @brief Builds a list of all directed edges missing from the current graph.
 * @param edges Existing edges.
 * @param s Vector of node opinions (used to determine graph size).
 * @return List of missing (complementary) directed edges.
 */
Edgelist build_complementary_edgelist_directed(const Edgelist& edges, const Vec& s) {
    Edgelist complementary;
    int n = s.size();
    for (int u = 0; u < n; ++u) {
        for (int v = 0; v < n; ++v) {
            if (u != v && std::find(edges.begin(), edges.end(), Edge(u, v)) == edges.end()) {
                complementary.emplace_back(u, v);
            }
        }
    }
    return complementary;
}

/**
 * @brief Adds an edge (and optionally its reverse) to a graph.
 * @param d The graph to update.
 * @param edge The edge to add.
 * @param undirected Whether the graph is undirected.
 * @return The updated graph.
 */
Graph add_edge_to_dict(Graph& d, const Edge& edge, bool undirected) {
    int u = edge.first, v = edge.second;

    d[u].push_back(v);
    if (undirected) {
        d[v].push_back(u);
    }
    return d;
}

/**
 * @brief Removes an edge (and optionally its reverse) from a graph.
 * @param d The graph to update.
 * @param edge The edge to remove.
 * @param undirected Whether the graph is undirected.
 */
void remove_edge_from_dict(Graph& d, const Edge& edge, bool undirected) {
    int u = edge.first, v = edge.second;

    auto& from_u = d[u];
    from_u.erase(std::remove(from_u.begin(), from_u.end(), v), from_u.end());

    if (undirected) {
        auto& from_v = d[v];
        from_v.erase(std::remove(from_v.begin(), from_v.end(), u), from_v.end());
    }
}

// ============================================================================
// Metrics
// ============================================================================

/**
 * @brief Calculates polarization (variance) of a vector of opinions.
 * @param s Vector of opinions.
 * @return The polarization score.
 */
double calculate_polarization(const Vec& s) {
    double mean = std::accumulate(s.begin(), s.end(), 0.0) / s.size();
    double sum_sq = 0.0;
    for (double val : s) {
        sum_sq += (val - mean) * (val - mean);
    }
    return sum_sq / s.size();
}

/**
 * @brief Calculates disagreement between neighboring nodes.
 * @param d Graph structure.
 * @param z Final opinions of nodes.
 * @param undirected Whether the graph is undirected.
 * @return Normalized disagreement score.
 */
double calculate_disagreement(const Graph& d, const Vec& z, bool undirected) {
    int m = 0;
    for (const auto& [u, neighbors] : d) {
        m += neighbors.size();
    }
    if (undirected) m /= 2;

    double dis = 0.0;
    for (const auto& [u, neighbors] : d) {
        if (u >= z.size()) continue;
        for (int v : neighbors) {
            if (v < z.size()) {
                double diff = z[u] - z[v];
                dis += diff * diff;
            }
        }
    }

    if (undirected) dis /= 2.0;
    return dis / m;
}

// ============================================================================
// File I/O
// ============================================================================

/**
 * @brief Reads a vector of opinions from a CSV file.
 * @param filename File path.
 * @return Vector of opinions.
 */
std::vector<double> read_opinions_csv(const std::string& filename) {
    std::vector<double> opinions;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error when opening file: " << filename << std::endl;
        return opinions;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Whitespace entfernen
        line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());

        // Komma zu Punkt ersetzen (optional)
        std::replace(line.begin(), line.end(), ',', '.');

        // Leere Zeilen überspringen
        if (line.empty()) continue;

        try {
            double value = std::stod(line);
            opinions.push_back(value);
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid number in file: \"" << line << "\"\n";
        }
    }

    return opinions;
}

/**
 * @brief Reads an edge list from a text file.
 * @param filename File path.
 * @return List of edges.
 */
std::vector<std::pair<int, int> > read_edgelist_txt(const std::string& filename) {
    std::vector<std::pair<int, int> > edges;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error when opening file: " << filename << std::endl;
        return edges;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        int u, v;
        if (ss >> u >> v) {
            edges.emplace_back(u, v);
        } else {
            std::cerr << "Ignored invalid row: \"" << line << "\"\n";
        }
    }

    return edges;
}

/**
 * @brief Writes an edge list to a CSV file.
 * @param edges Edge list to write.
 * @param filename Output file path.
 */
void write_edgelist_to_csv(const std::vector<std::pair<int, int> >& edges, const std::string& filename) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error when opening file to write: " << filename << std::endl;
        return;
    }

    // Optional: Header schreiben
    file << "source,target\n";

    for (const auto& edge : edges) {
        file << edge.first << "," << edge.second << "\n";
    }

    file.close();
    std::cout << "Edgelist successfully stored in: " << filename << std::endl;
}

// ============================================================================
// Utilities
// ============================================================================

/**
 * @brief Computes the average degree of a graph.
 * @param graph The graph.
 * @param undirected Whether the graph is undirected.
 * @return Average node degree.
 */
double average_degree(const Graph& graph, bool undirected) {
    if (graph.empty()) return 0.0;

    size_t num_nodes = graph.size();
    size_t total_degree = 0;

    for (const auto& [_, neighbors] : graph) {
        total_degree += neighbors.size();
    }

    if (undirected) {
        return (2.0 * total_degree) / num_nodes;
    } else {
        return static_cast<double>(total_degree) / num_nodes;
    }
}

/**
 * @brief Prints the graph to the console.
 *
 * Displays each node and its neighbors.
 * Useful for debugging purposes.
 *
 * @param g The graph to print.
 */
void print_graph(const Graph& g) {
    std::cout << "graph:" << "\n";
    for (const auto& [node, neighbors] : g) {
        std::cout << node << " : ";
        for (int neighbor : neighbors) {
            std::cout << neighbor << " ";
        }
        std::cout << "\n";
    }
}

/**
 * @brief Removes a specific edge from a complementary edge list.
 *
 * This is useful when maintaining a list of non-existing edges (i.e., potential edges to be added).
 * It ensures the representation is consistent for undirected edges by sorting node IDs.
 *
 * @param comp_edges The list of complementary (non-existing) edges.
 * @param edge The edge to remove.
 * @param undirected Indicates whether the graph is undirected.
 */
void remove_edge_from_complementary_edgelist(Edgelist& comp_edges, const Edge& edge, bool undirected) {
    int u = edge.first;
    int v = edge.second;

    if (undirected && u > v) {
        std::swap(u, v); // Ensure consistent representation for undirected edges
    }

    auto it = std::remove_if(comp_edges.begin(), comp_edges.end(),
        [u, v, undirected](const Edge& e) {
            int a = e.first;
            int b = e.second;

            if (undirected && a > b) std::swap(a, b);

            return a == u && b == v;
        });

    // Erase the matching edge(s) from the list
    if (it != comp_edges.end()) {
        comp_edges.erase(it, comp_edges.end());
    }
}





