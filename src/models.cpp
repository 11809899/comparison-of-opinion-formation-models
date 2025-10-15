//
// Created by astonek on 9/4/25.
//
#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include "models.h"

// Type aliases for readability
using Graph = std::unordered_map<int, std::vector<int> >;
using Vec = std::vector<double>;

/**
 * @brief Simulates the Friedkin-Johnsen opinion dynamics model.
 *
 * Each node updates its opinion as a weighted average between its initial opinion (with weight c)
 * and the opinions of its neighbors.
 *
 * The process iterates until convergence below a fixed threshold.
 *
 * @param d        The network graph (dictionary).
 * @param s        Initial opinions of each node.
 * @param c        Confidence parameter (weight on initial opinion).
 * @param z_star   Optional initial state to start from (otherwise starts from s).
 * @return         Final opinion vector after convergence.
 */
Vec friedkin_johnsen(const Graph& d, const Vec& s, double c, const Vec* z_star) {
    const double threshold = 0.01;

    // z_old is the previous state; z_new is the next state
    Vec z_old = (z_star != nullptr) ? *z_star : s;
    Vec z_new = s;


    while (true) {

        for (size_t k = 0; k < s.size(); ++k) {
            const auto it = d.find(k);
            const std::vector<int>& neighbors = (it != d.end()) ? it->second : std::vector<int>();

            double sum = 0.0;
            for (int j : neighbors) {
                sum += z_old[j];
            }
            // Update opinion: weighted average between initial and neighbors' opinions
            z_new[k] = (c * s[k] + sum) / (c + neighbors.size());
        }

        // Compute squared difference between current and previous state
        double diff = 0.0;
        for (size_t i = 0; i < z_old.size(); ++i) {
            double delta = z_old[i] - z_new[i];
            diff += delta * delta;
        }

        // Stop if change is below threshold (convergence)
        if (diff < threshold * threshold){
            break;
        }

        z_old = z_new; // Prepare for next iteration

    }

    return z_new;
}

/**
 * @brief Simulates the Bounded Confidence (BC) opinion dynamics model.
 *
 * Each node only considers neighbors whose opinions differ by at most epsilon.
 * The new opinion is the average of all accepted neighbors (including self).
 *
 * The process iterates until convergence below a fixed threshold.
 *
 * @param d        The network graph (dictionary).
 * @param s        Initial opinions of each node.
 * @param epsilon  Maximum allowed opinion difference to influence another node.
 * @param z_star   Optional initial state to start from (otherwise starts from s).
 * @return         Final opinion vector after convergence.
 */
Vec bounded_confidence(const Graph& d, const Vec& s, double epsilon, const Vec* z_star) {
    const double threshold = 0.01;

    // Initialize opinion vectors
    Vec z_old = (z_star != nullptr) ? *z_star : s;
    Vec z_new = z_old;

    while (true) {
        for (size_t k = 0; k < z_old.size(); ++k) {
            std::vector<int> candidates = {static_cast<int>(k)}; // Always include self

            auto it = d.find(k);
            if (it != d.end()) {
                for (int j : it->second) {
                    // Only include neighbors within epsilon range
                    if (std::abs(z_old[j] - z_old[k]) <= epsilon) {
                        candidates.push_back(j);
                    }
                }
            }

            // Update opinion as average of all acceptable neighbors
            if (candidates.size() == 1) {
                z_new[k] = z_old[k]; // No change if only self
            } else {
                double sum = 0.0;
                for (int j : candidates) {
                    sum += z_old[j];
                }
                z_new[k] = sum / candidates.size();
            }
        }

        // Compute squared difference between current and previous state
        double diff = 0.0;
        for (size_t i = 0; i < z_old.size(); ++i) {
            double delta = z_old[i] - z_new[i];
            diff += delta * delta;
        }

        // Check for convergence
        if (diff < threshold * threshold) break;

        z_old = z_new; // Prepare for next iteration
    }

    return z_new;
}
