//
// Created by astonek on 9/5/25.
//

#ifndef MASTER_THESIS_UTILS_H
#define MASTER_THESIS_UTILS_H

#include <iostream>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

// Type aliases for improved readability
using Graph = std::unordered_map<int, std::vector<int>>;
using Edge = std::pair<int, int>;
using Edgelist = std::vector<Edge>;
using Vec = std::vector<double>;

/**
 * @brief Constructs an undirected dictionary from a list of edges.
 * @param edges The list of edges.
 * @return The constructed undirected graph.
 */
Graph build_edges_dict_undirected(const Edgelist& edges);

/**
 * @brief Constructs a directed dictionary from a list of edges.
 * @param edges The list of edges.
 * @return The constructed directed graph.
 */
Graph build_edges_dict_directed(const Edgelist& edges);

/**
 * @brief Builds a list of all missing edges in an undirected graph.
 * @param d The current graph.
 * @return List of complementary (missing) edges.
 */
Edgelist build_complementary_edgelist_undirected(const Graph& d);

/**
 * @brief Builds a list of all missing directed edges.
 * @param edges The existing edge list.
 * @param s Vector of node opinions (used to determine number of nodes).
 * @return List of complementary (missing) directed edges.
 */
Edgelist build_complementary_edgelist_directed(const Edgelist& edges, const Vec& s);

/**
 * @brief Adds an edge to the graph (and its reverse if undirected).
 * @param d Graph to update (in-place).
 * @param edge The edge to add.
 * @param undirected If true, adds both (u, v) and (v, u).
 * @return The updated graph.
 */
Graph add_edge_to_dict(Graph& d, const Edge& edge, bool undirected);

/**
 * @brief Removes an edge from the graph (and its reverse if undirected).
 * @param d Graph to update (in-place).
 * @param edge The edge to remove.
 * @param undirected If true, removes both (u, v) and (v, u).
 */
void remove_edge_from_dict(Graph& d, const Edge& edge, bool undirected);

/**
 * @brief Removes an edge from a complementary edge list.
 * @param comp_edges The complementary edge list to modify.
 * @param edge The edge to remove.
 * @param undirected Whether the edge list represents an undirected graph.
 */
void remove_edge_from_complementary_edgelist(Edgelist& comp_edges, const Edge& edge, bool undirected);

/**
 * @brief Calculates the polarization of a vector (i.e., its variance).
 * @param s The input vector (e.g. opinions).
 * @return The variance (polarization).
 */
double calculate_polarization(const Vec& s);

/**
 * @brief Calculates disagreement based on neighboring opinion differences.
 * @param d The graph structure.
 * @param z Vector of final opinions.
 * @param undirected Whether the graph is undirected.
 * @return The normalized disagreement score.
 */
double calculate_disagreement(const Graph& d, const Vec& z, bool undirected);

/**
 * @brief Reads a CSV file containing opinions (one per line).
 * @param filename Path to the input file.
 * @return Vector of opinions.
 */
std::vector<double> read_opinions_csv(const std::string& filename);

/**
 * @brief Reads an edge list from a text file (each line: u v).
 * @param filename Path to the edge list file.
 * @return Vector of edges.
 */
std::vector<std::pair<int, int>> read_edgelist_txt(const std::string& filename);

/**
 * @brief Writes an edge list to a CSV file.
 * @param edges The edge list to write.
 * @param filename Output CSV file path.
 */
void write_edgelist_to_csv(const std::vector<std::pair<int, int>>& edges, const std::string& filename);

/**
 * @brief Computes the average degree of the graph.
 * @param graph The graph.
 * @param undirected Whether the graph is undirected.
 * @return Average degree.
 */
double average_degree(const Graph& graph, bool undirected);

/**
 * @brief Prints the graph's dictionary to the console.
 * @param g The graph.
 */
void print_graph(const Graph& g);

#endif //MASTER_THESIS_UTILS_H