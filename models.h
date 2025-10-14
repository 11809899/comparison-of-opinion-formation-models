//
// Created by astonek on 9/4/25.
//

#ifndef MASTER_THESIS_MODELS_H
#define MASTER_THESIS_MODELS_H

#include <vector>
#include <unordered_map>

/// @brief Type alias for a graph represented as a dictionary
using Graph = std::unordered_map<int, std::vector<int>>;

/// @brief Type alias for a vector of opinions
using Vec = std::vector<double>;

/**
 * @brief Simulates the Friedkin-Johnsen opinion dynamics model.
 *
 * Each node updates its opinion by averaging between its initial opinion
 * (weighted by parameter c) and the opinions of its neighbors.
 *
 * Iteration continues until convergence.
 *
 * @param d         Graph represented as a dictionary.
 * @param s         Initial opinion vector.
 * @param c         Weight on the initial opinions (confidence parameter).
 * @param z_star    Optional starting vector (defaults to s if nullptr).
 * @return          Final opinion vector after convergence.
 */
Vec friedkin_johnsen(const Graph& d, const Vec& s, double c, const Vec* z_star = nullptr);

/**
 * @brief Simulates the Bounded Confidence (BC) opinion dynamics model.
 *
 * Each node averages the opinions of neighbors whose opinions differ by
 * no more than epsilon. The process repeats until convergence.
 *
 * @param d         Graph represented as a dictionary.
 * @param s         Initial opinion vector.
 * @param epsilon   Confidence threshold (maximum opinion difference).
 * @param z_star    Optional starting vector (defaults to s if nullptr).
 * @return          Final opinion vector after convergence.
 */
Vec bounded_confidence(const Graph& d, const Vec& s, double epsilon, const Vec* z_star = nullptr);

#endif //MASTER_THESIS_MODELS_H