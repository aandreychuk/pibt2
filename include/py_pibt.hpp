#pragma once
#include "pibt.hpp"
#include <vector>
#include <string>
#include <memory>
#include <tuple>

class PyPIBT {
public:
    // Internal Position struct (not exposed to Python)
    struct Position {
        int x, y;
        Position(int x = 0, int y = 0) : x(x), y(y) {}
    };

private:
    std::unique_ptr<Grid> graph;
    std::unique_ptr<Problem> problem_instance;
    std::unique_ptr<PIBT> solver;
    std::mt19937 mt_generator;
    
    // Internal state
    bool initialized;
    bool is_lmapf;
    int timestep;
    

    
    // Options
    bool disable_dist_init;
    
    // Helper methods for conversion
    std::vector<Position> tuplesToPositions(const std::vector<std::tuple<int, int>>& tuples);
    std::vector<std::tuple<int, int>> positionsToTuples(const std::vector<Position>& positions) const;
    
    // Helper methods (agent management now handled by PIBT internally)
    
    // Map creation from 2D matrix
    bool createMapFromMatrix(const std::vector<std::vector<int>>& map_matrix);
    
    // Edge weights setup from list of lists
    void setupEdgeWeights(const std::vector<std::vector<double>>& weights_matrix);
    
public:
    PyPIBT();
    ~PyPIBT();
    
    // Unified initialization method with explicit LMAPF flag
    bool initialize(const std::vector<std::vector<int>>& map_matrix,
                   const std::vector<std::tuple<int, int>>& starts,
                   const std::vector<std::tuple<int, int>>& goals,
                   bool is_lmapf = false,
                   const std::vector<std::vector<double>>& edge_weights = {},
                   bool disable_distance_init = false,
                   int seed = 0);
    

    
    // Step method - executes one step and returns current positions
    std::vector<std::tuple<int, int>> step();
    
    // Update goals for specific agents (provide new goal positions for agents that need updates)
    // agent_goal_updates: vector of (agent_id, x, y) tuples for agents that need new goals  
    void updateGoals(const std::vector<std::tuple<int, int, int>>& agent_goal_updates);
    
    // Utility methods
    int getNumAgents() const;
    int getTimestep() const;
    
    // Map information
    int getMapWidth() const;
    int getMapHeight() const;
    bool isValidPosition(int x, int y) const;
    std::vector<std::vector<int>> getMapMatrix() const;
}; 