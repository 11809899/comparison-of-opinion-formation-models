//
// Created by astonek on 9/4/25.
//

#include <iostream>
#include <unordered_map>
#include <string>
#include <utility>
#include "src/project_utils.h"
#include "src/interventions.h"
#include "src/models.h"

/**
 * @brief Executes the edge intervention procedure on a given dataset.
 *
 * Depending on the model ("FJ" or "BC"), this function identifies
 * a set of k optimal edges to add to the graph to influence opinions,
 * using either a reduced or full search space.
 *
 * @param dataset_path   Path to the dataset (without file extension).
 * @param output_path    Path prefix for output CSV files.
 * @param model          The opinion dynamics model: "FJ" or "BC".
 * @param epsilon        Optional parameter for the "BC" model (ignored for "FJ").
 */

void run_on_dataset(const std::string& dataset_path, const std::string& output_path, const std::string& model, double epsilon = 0.0) {
    std::string directory = "";
    if (dataset_path == "BAG_10" or dataset_path == "BAG_20" or dataset_path == "BAG_30"){
        directory = "Barabasi-Albert Graphs";
    } else if (dataset_path == "SBM_low_cohesion" or dataset_path == "SBM_high_cohesion"){
        directory = "Stochastic Block Model";
    } else if (dataset_path == "twitter_large"){
        directory = "Twitter Large";
    } else if (dataset_path == "reddit"){
        directory = "Reddit";
    } else if (dataset_path == "flixster"){
        directory = "Flixster Graph";
    } else {
        std::cerr << "Error: Dataset not supported.\n";
    }


    // 1. Load edge list and opinion data
    auto edgelist = read_edgelist_txt("data/graphs/"+ directory + "/" + dataset_path + ".edgelist");
    auto opinions = read_opinions_csv("data/graphs/"+ directory + "/" + "opinion_" + dataset_path + ".csv");

    // Special handling for pre-defined datasets using reduced search space
    if (dataset_path == "twitter_large") {
        int k = 20;
        if (model == "FJ") {
            // Run reduced search space algorithm for the FJ model
            auto edges_undirected = find_k_edges_reduced_search_space(edgelist, opinions, k, true, model);
            write_edgelist_to_csv(edges_undirected, output_path + "_FJ_undirected.csv");

            auto edges_directed = find_k_edges_reduced_search_space(edgelist, opinions, k, false, model);
            write_edgelist_to_csv(edges_directed, output_path + "_FJ_directed.csv");
        } else if (model == "BC") {
            // BC model requires epsilon as an additional parameter
            auto edges_undirected = find_k_edges_reduced_search_space(edgelist, opinions, k, true, model, epsilon);
            write_edgelist_to_csv(edges_undirected, output_path + "_" + std::to_string(epsilon) +  "_undirected.csv");

            auto edges_directed = find_k_edges_reduced_search_space(edgelist, opinions, k, false, model, epsilon);
            write_edgelist_to_csv(edges_directed, output_path + "_" + std::to_string(epsilon) + "_directed.csv");

        }
    } else {
        // Default case: Use full search space for edge selection
            int k = 20;
            if (model == "FJ") {
                // Build graphs and compute average degree to set c parameter
                Graph d_undirected = build_edges_dict_undirected(edgelist);
                Graph d_directed = build_edges_dict_directed(edgelist);
                double avg_deg_undirected = average_degree(d_undirected, true);
                double c_undirected = 2 * avg_deg_undirected;


                double avg_deg_directed = average_degree(d_directed, false);
                double c_directed = 2 * avg_deg_directed;

                // Run full search space for undirected graph
                auto edges_undirected = find_k_edges_full_search_space(edgelist, opinions, k, true, model, 0.0, c_undirected);
                write_edgelist_to_csv(edges_undirected, output_path + "_FJ_undirected.csv");

                // ... and for directed graph
                auto edges_directed = find_k_edges_full_search_space(edgelist, opinions, k, false, model, 0.0, c_directed);
                write_edgelist_to_csv(edges_directed, output_path + "_FJ_directed.csv");
            } else if (model == "BC") {
                // BC model does not require c parameter
                auto edges_undirected = find_k_edges_full_search_space(edgelist, opinions, k, true, model, epsilon);
                write_edgelist_to_csv(edges_undirected, output_path + "_" + std::to_string(epsilon) +  "_undirected.csv");

                auto edges_directed = find_k_edges_full_search_space(edgelist, opinions, k, false, model, epsilon);
                write_edgelist_to_csv(edges_directed, output_path + "_" + std::to_string(epsilon) + "_directed.csv");

            }
    }



}




int main(int argc, char *argv[]) {
    // Expected command-line arguments:
    // argv[1] = dataset path (without extension)
    // argv[2] = output path prefix
    // argv[3] = model name ("FJ" or "BC")
    // argv[4] = epsilon (only used for "BC" model)

    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <dataset_path> <output_path> <model> [epsilon]\n";
        return 1;
    }

    if (std::string(argv[3]) == "BC" && argc < 5) {
        std::cerr << "Error: BC model requires an epsilon value.\n";
        return 1;
    }

    std::string dataset_path = argv[1];

    std::string output_path = argv[2];

    std::string model = argv[3];

    double epsilon = 0.0;

    // Call main processing function based on model
    if (model == "FJ") {
        run_on_dataset(dataset_path, output_path, model);
    } else if (model == "BC") {
        epsilon = std::stod(argv[4]);
        run_on_dataset(dataset_path, output_path, model, epsilon);
    }


    return 0;
}