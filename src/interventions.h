//
// Created by astonek on 9/5/25.
//

#ifndef MASTER_THESIS_INTERVENTIONS_H
#define MASTER_THESIS_INTERVENTIONS_H

#include <iostream>
#include <unordered_map>
#include <vector>

// Type aliases for better readability
using Graph = std::unordered_map<int, std::vector<int> >;
using Edge = std::pair<int,int>;
using Edgelist = std::vector<Edge>;
using Vec = std::vector<double>;

/**
 * Finds the best k edges to add by exhaustively searching the full edge space.
 *
 * @param edgelist Current graph edges.
 * @param s Initial opinions vector.
 * @param k Number of edges to add.
 * @param undirected Whether the graph is undirected.
 * @param model Opinion dynamics model ("FJ" for Friedkin-Johnsen, "BC" for Bounded Confidence).
 * @param epsilon Confidence threshold for BC model (default 0.05).
 * @param c Resistance parameter for FJ model (default 0.0).
 * @return List of edges selected to add to the graph.
 */
Edgelist find_k_edges_full_search_space(const Edgelist& edgelist, const Vec& s, int k, bool undirected, const std::string& model, double epsilon = 0.05, double c = 0.0);

/**
 * Finds the best k edges to add by searching a reduced candidate edge space.
 * This is more efficient than the full search.
 *
 * @param edgelist Current graph edges.
 * @param s Initial opinions vector.
 * @param k Number of edges to add.
 * @param undirected Whether the graph is undirected.
 * @param model Opinion dynamics model ("FJ" or "BC").
 * @param epsilon Confidence threshold for BC model (default 0.05).
 * @param c Resistance parameter for FJ model (default 0.0).
 * @return List of edges selected to add to the graph.
 */
Edgelist find_k_edges_reduced_search_space(const Edgelist& edgelist, const Vec& s, int k, bool undirected, const std::string& model, double epsilon = 0.05, double c = 0.0);

/**
 * Generates candidate edges to add for the Bounded Confidence model by
 * grouping nodes with similar opinions (within epsilon intervals).
 *
 * @param d Current graph dictionary.
 * @param s Initial opinions vector.
 * @param k Target number of edges to find candidates for.
 * @param undirected Whether the graph is undirected.
 * @param epsilon Confidence threshold.
 * @return Candidate edges that can be added to the graph.
 */
Edgelist find_candidate_edges_bounded_confidence(const Graph& d, const Vec& s, int k, bool undirected, double epsilon);

#endif //MASTER_THESIS_INTERVENTIONS_H