//
// Created by astonek on 9/5/25.
//
#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <limits>
#include "project_utils.h"
#include "models.h"
#include "interventions.h"

// Type aliases for readability
using Graph = std::unordered_map<int, std::vector<int>>;
using Edge = std::pair<int,int>;
using Edgelist = std::vector<Edge>;
using Vec = std::vector<double>;

/**
 * @brief Full search over all possible non-existing edges to find the k best ones
 *        that reduce polarization the most.
 *
 * @param edgelist         Original edge list of the graph.
 * @param s                Initial opinions.
 * @param k                Number of edges to add.
 * @param undirected       Whether the graph is undirected.
 * @param model            Opinion dynamics model: "FJ" or "BC".
 * @param epsilon          Threshold for BC model.
 * @param c                Confidence parameter for FJ model.
 * @return                 List of k selected edges.
 */
Edgelist find_k_edges_full_search_space(const Edgelist& edgelist, const Vec& s, int k, bool undirected, const std::string& model, double epsilon, double c) {

    // Build dictionary and complementary edge list
    Graph d = undirected ? build_edges_dict_undirected(edgelist) : build_edges_dict_directed(edgelist);
    Edgelist complementary_edgelist = undirected ? build_complementary_edgelist_undirected(d) : build_complementary_edgelist_directed(edgelist, s);

    Edgelist added_edges;

    // Iteratively find best edges
    for (int iter = 0; iter < k; ++iter) {

        Vec z_star;
        if (model == "FJ") z_star = friedkin_johnsen(d, s, c);
        else if (model == "BC") z_star = bounded_confidence(d, s, epsilon);

        double polarization = calculate_polarization(z_star);

        Edge best_edge = {-1,-1};
        for (const Edge& edge : complementary_edgelist) {
             //std::cout << edge.first << " -> " << edge.second << std::endl;
            d = add_edge_to_dict(d, edge, undirected);

            Vec z;
            if (model == "FJ") z = friedkin_johnsen(d, s, c);
            else if (model == "BC") z = bounded_confidence(d, s, epsilon);

            double pol = calculate_polarization(z);

            // Save the edge if it improves polarization and hasn't already been added
            if (pol < polarization && std::find(added_edges.begin(), added_edges.end(), edge) == added_edges.end()) {
                polarization = pol;
                best_edge = edge;
            }
            remove_edge_from_dict(d, edge, undirected);
        }
        if (best_edge.first != -1) {
            d = add_edge_to_dict(d, best_edge, undirected);
            added_edges.push_back(best_edge);
            remove_edge_from_complementary_edgelist(complementary_edgelist, best_edge, undirected);
        }
    }
    return added_edges;
}

/**
 * @brief Uses a reduced search space to efficiently find k beneficial edges.
 *
 * For BC, edges are preselected based on confidence intervals.
 * For FJ, edges are selected between extreme-opinion nodes.
 */
Edgelist find_k_edges_reduced_search_space(const Edgelist& edgelist, const Vec& s, int k, bool undirected, const std::string& model, double epsilon, double c) {
    Graph d = undirected ? build_edges_dict_undirected(edgelist) : build_edges_dict_directed(edgelist);
    Edgelist candidate_edges;
    Edgelist final_edges;

    if (model == "BC") {
        // Select candidate edges using bounded confidence logic
        candidate_edges = find_candidate_edges_bounded_confidence(d, s, k, undirected, epsilon);
    } else if (model == "FJ") {
        // Generate candidate edges between low and high opinion nodes
        std::vector<std::pair<int, double>> ascending, descending;
        for (size_t i = 0; i < s.size(); ++i) ascending.emplace_back(i, s[i]);
        descending = ascending;
        std::sort(ascending.begin(), ascending.end(), [](auto& a, auto& b) { return a.second < b.second; });
        std::sort(descending.begin(), descending.end(), [](auto& a, auto& b) { return a.second > b.second; });

        for (size_t i = 0; i < ascending.size() && candidate_edges.size() < 10 * k; ++i) {
            int u = ascending[i].first;
            int v = descending[i].first;

            bool exists = std::find(d[u].begin(), d[u].end(), v) != d[u].end();
            if (!exists) {
                candidate_edges.emplace_back(u, v);
                if (undirected) candidate_edges.emplace_back(v, u);
            }
        }
    }

    // From the candidates, select k that reduce polarization most
    for (int iter = 0; iter < k; ++iter) {

        Vec z_star;
        if (model == "FJ") z_star = friedkin_johnsen(d, s, c);
        else if (model == "BC") z_star = bounded_confidence(d, s, epsilon);

        double polarization = calculate_polarization(z_star);

        Edge best_edge = {-1,-1};
        for (const Edge& edge : candidate_edges) {
            //std::cout << edge.first << " -> " << edge.second << std::endl;
            d = add_edge_to_dict(d, edge, undirected);

            Vec z;
            if (model == "FJ") z = friedkin_johnsen(d, s, c);
            else if (model == "BC") z = bounded_confidence(d, s, epsilon);

            double pol = calculate_polarization(z);

            if (pol < polarization && std::find(final_edges.begin(), final_edges.end(), edge) == final_edges.end()) {
                polarization = pol;
                best_edge = edge;
            }
            remove_edge_from_dict(d, edge, undirected);
        }
        if (best_edge.first != -1) {
            d = add_edge_to_dict(d, best_edge, undirected);
            final_edges.push_back(best_edge);
            remove_edge_from_complementary_edgelist(candidate_edges, best_edge, undirected);
        }
    }

    return final_edges;
}

/**
 * @brief Generates candidate edges for the BC model by identifying
 *        opinion intervals and connecting agents within those intervals.
 *
 * @param d          Dictionary of the graph.
 * @param s          Initial opinions.
 * @param k          Desired number of edges.
 * @param undirected Whether the graph is undirected.
 * @param epsilon    Confidence interval for bounded influence.
 * @return           List of candidate edges to evaluate.
 */
Edgelist find_candidate_edges_bounded_confidence(
        const Graph& d,
        const Vec& s,
        int k,
        bool undirected,
        double epsilon
) {
    Edgelist candidate_edges;

    // Pair each node index with its opinion value
    std::vector<std::pair<int, double>> indexed_opinions;
    for (size_t i = 0; i < s.size(); ++i) {
        indexed_opinions.emplace_back(i, s[i]);
    }

    // Sort nodes by opinion value in ascending order
    std::sort(indexed_opinions.begin(), indexed_opinions.end(), [](auto& a, auto& b) {
        return a.second < b.second;
    });

    // Partition nodes into opinion intervals of size epsilon
    std::vector<std::vector<std::pair<int, double>>> intervals;
    double starting_point = indexed_opinions[0].second;
    std::vector<std::pair<int, double>> interval;

    for (const auto& opinion : indexed_opinions) {
        if (opinion.second < (starting_point + epsilon)) {
            interval.push_back(opinion);
        } else {
            if (!interval.empty()) intervals.push_back(interval);
            interval.clear();
            interval.push_back(opinion);
            starting_point = opinion.second;
        }
    }
    // Add last interval if not empty
    if (!interval.empty()) intervals.push_back(interval);

    // Generate candidate edges by connecting low-opinion to high-opinion nodes within each interval
    // Limit the total candidate edges to 10*k for efficiency
    for (const auto& group : intervals) {
        for (size_t i = 0; i < group.size() && candidate_edges.size() < 10 * k; ++i) {
            int u = group[i].first;
            int v = group[group.size() - 1 - i].first;
            if (u == v) continue;  // Skip self-loops

            // Check if edge (u,v) already exists in the graph
            bool exists = false;
            auto it = d.find(u);
            if (it != d.end()) {
                exists = std::find(it->second.begin(), it->second.end(), v) != it->second.end();
            }

            // If edge does not exist, add it
            if (!exists) {
                candidate_edges.emplace_back(u, v);

                // If undirected graph, add reverse edge as well
                if (undirected && candidate_edges.size() < 10 * k) {
                    candidate_edges.emplace_back(v, u);
                }
            }
        }
    }

    // If no suitable intervals found (e.g., all opinions similar),
    // fall back to connecting low-opinion nodes with high-opinion nodes directly
    if (candidate_edges.empty()) {

        auto descending = indexed_opinions;
        std::reverse(descending.begin(), descending.end());

        for (const auto& u_pair : indexed_opinions) {
            int u = u_pair.first;

            for (const auto& v_pair : descending) {
                int v = v_pair.first;
                if (u == v) continue;

                if (candidate_edges.size() >= 10 * k) break;

                // Check if edge (u,v) already exists
                bool exists = false;
                auto it = d.find(u);
                if (it != d.end()) {
                    exists = std::find(it->second.begin(), it->second.end(), v) != it->second.end();
                }

                if (!exists) {
                    candidate_edges.emplace_back(u, v);

                    if (undirected && candidate_edges.size() < 10 * k) {
                        // Check if reverse edge (v,u) exists before adding
                        bool reverse_exists = false;
                        auto it_rev = d.find(v);
                        if (it_rev != d.end()) {
                            reverse_exists = std::find(it_rev->second.begin(), it_rev->second.end(), u) != it_rev->second.end();
                        }
                        if (!reverse_exists) {
                            candidate_edges.emplace_back(v, u);
                        }
                    }
                }
            }

            if (candidate_edges.size() >= 10 * k) break;
        }
    }

    return candidate_edges;
}

