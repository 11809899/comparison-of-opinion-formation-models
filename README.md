# Comparison of Opinion Formation Models

This repository contains the code for my Master's Thesis:  
**"Assessing the Impact of Interventions in Different Opinion Formation Models"**

The project investigates how different opinion dynamics models affect the outcome of edge-based interventions in networks. It provides a simulation framework, model implementations, and tools for measuring polarization and disagreement across networks.

---

## Contents

- `src/` – Main source code including opinion models and intervention logic
- `data/` – Input data such as edgelists and opinion vectors and output data (interventions)
- `main.cpp` – Entry point for simulations

---

## Requirements

- C++17 or higher
- Standard C++ compiler (e.g., G++, Clang)

---

## Building and Running

### 1. Clone the repository

```bash
git clone https://github.com/11809899/comparison-of-opinion-formation-models.git
cd comparison-of-opinion-formation-models
```

### 2. Compile the project
`g++ -std=c++17 -o out.exe main.cpp src/interventions.cpp src/models.cpp src/project_utils.cpp`

### 3. Run simulations
`./out.exe <dataset_path> <output_path> <model> [epsilon]`

---

## File Formats
`opinion_<dataset_path>.csv`

A single-column CSV file with one opinion value per line (floating-point).

`<dataset_path>.edgelist`

Each line defines a directed edge in the format: `source_node target_node`

---

## Configuration
The BC-Model accepts the confidence threshold ε as a parameter and the FJ-Model accepts the stubbornness constant c.
You can modify the stubbornness constant c directly in the code (e.g., main.cpp) and pass ε via command line.

---
## Citation

If you use this code or reproduce results in an academic context, please cite:


Stonek, Anna: Assessing the Impact of Interventions in Different Opinion Formation Models. - 
Master's Thesis, TU Wien, 2025.

---
## Contact

For questions or collaborations, please reach out:

📧 [anna.stonek@gmail.com]

