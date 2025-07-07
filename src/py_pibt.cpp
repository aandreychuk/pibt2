#include "py_pibt.hpp"
#include "util.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

// Custom Grid class that can be initialized from 2D matrix
class DataGrid : public Grid {
public:
    DataGrid(const std::vector<std::vector<int>>& map_matrix) {
        if (map_matrix.empty() || map_matrix[0].empty()) {
            std::cerr << "Error: Empty map matrix" << std::endl;
            return;
        }
        
        height = map_matrix.size();
        width = map_matrix[0].size();
        
        // Validate map matrix
        for (const auto& row : map_matrix) {
            if (static_cast<int>(row.size()) != width) {
                std::cerr << "Error: Inconsistent row width in map matrix" << std::endl;
                return;
            }
        }
        
        // Create nodes
        V = Nodes(width * height, nullptr);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int cell_value = map_matrix[y][x];
                if (cell_value == 0) {  // 0 = free space, non-zero = obstacle
                    int id = width * y + x;
                    Node* v = new Node(id, x, y);
                    V[id] = v;
                }
            }
        }
        
        // Create edges
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (!existNode(x, y)) continue;
                Node* v = getNode(x, y);
                // left
                if (existNode(x - 1, y)) v->neighbor.push_back(getNode(x - 1, y));
                // right
                if (existNode(x + 1, y)) v->neighbor.push_back(getNode(x + 1, y));
                // up
                if (existNode(x, y - 1)) v->neighbor.push_back(getNode(x, y - 1));
                // down
                if (existNode(x, y + 1)) v->neighbor.push_back(getNode(x, y + 1));
            }
        }
    }
};

PyPIBT::PyPIBT() : initialized(false), is_lmapf(false), timestep(0), disable_dist_init(false) {
    mt_generator.seed(0);
}

PyPIBT::~PyPIBT() {
    // Agent cleanup is now handled by PIBT internally
}

std::vector<PyPIBT::Position> PyPIBT::tuplesToPositions(const std::vector<std::tuple<int, int>>& tuples) {
    std::vector<Position> positions;
    for (const auto& t : tuples) {
        positions.emplace_back(std::get<0>(t), std::get<1>(t));
    }
    return positions;
}

std::vector<std::tuple<int, int>> PyPIBT::positionsToTuples(const std::vector<Position>& positions) const {
    std::vector<std::tuple<int, int>> tuples;
    for (const auto& pos : positions) {
        tuples.emplace_back(pos.x, pos.y);
    }
    return tuples;
}

bool PyPIBT::createMapFromMatrix(const std::vector<std::vector<int>>& map_matrix) {
    try {
        graph = std::make_unique<DataGrid>(map_matrix);
        return graph != nullptr;
    } catch (const std::exception& e) {
        std::cerr << "Error creating map from matrix: " << e.what() << std::endl;
        return false;
    }
}

void PyPIBT::setupEdgeWeights(const std::vector<std::vector<double>>& weights_matrix) {
    if (weights_matrix.empty() || !solver) return;
    
    // Validate weights matrix format
    int total_nodes = getMapWidth() * getMapHeight();
    if (static_cast<int>(weights_matrix.size()) != total_nodes) {
        std::cerr << "Warning: Weights matrix size (" << weights_matrix.size() 
                  << ") does not match number of nodes (" << total_nodes << ")" << std::endl;
        return;
    }
    
    // Validate that each row has exactly 5 values
    for (size_t i = 0; i < weights_matrix.size(); ++i) {
        if (weights_matrix[i].size() != 5) {
            std::cerr << "Warning: Weights row " << i << " has " << weights_matrix[i].size() 
                      << " values, expected 5 [right, up, left, down, wait]" << std::endl;
            return;
        }
    }
    
    // Create a temporary weights file from the matrix
    std::string temp_weights_file = "/tmp/pypibt_weights.csv";
    std::ofstream weights_file(temp_weights_file);
    
    if (!weights_file) {
        std::cerr << "Warning: Could not create temporary weights file" << std::endl;
        return;
    }
    
    // Write header
    weights_file << "node_id,right,up,left,down,wait\n";
    
    // Write weights data
    // weights_matrix[node_id] = [right, up, left, down, wait]
    for (int node_id = 0; node_id < static_cast<int>(weights_matrix.size()); ++node_id) {
        const auto& weights = weights_matrix[node_id];
        weights_file << node_id << "," 
                     << weights[0] << "," // right
                     << weights[1] << "," // up  
                     << weights[2] << "," // left
                     << weights[3] << "," // down
                     << weights[4] << "\n"; // wait
    }
    
    weights_file.close();
    
    // Load weights into solver
    solver->setWeightsFile(temp_weights_file);
    
    // Clean up temporary file
    std::remove(temp_weights_file.c_str());
}

bool PyPIBT::initialize(const std::vector<std::vector<int>>& map_matrix,
                       const std::vector<std::tuple<int, int>>& starts,
                       const std::vector<std::tuple<int, int>>& goals,
                       bool is_lmapf_flag,
                       const std::vector<std::vector<double>>& edge_weights,
                       bool disable_distance_init,
                       int seed) {
    try {
        // Set options
        is_lmapf = is_lmapf_flag;
        disable_dist_init = disable_distance_init;
        mt_generator.seed(seed);
        
        // Create grid from matrix
        if (!createMapFromMatrix(map_matrix)) {
            std::cerr << "Error: Failed to create map from matrix" << std::endl;
            return false;
        }
        
        // Validate positions
        if (starts.size() != goals.size()) {
            std::cerr << "Error: Number of starts and goals must match" << std::endl;
            return false;
        }
        
        // Convert tuples to positions for validation
        auto start_positions = tuplesToPositions(starts);
        auto goal_positions = tuplesToPositions(goals);
        
        // Check if all positions are valid
        for (const auto& pos : start_positions) {
            if (!graph->existNode(pos.x, pos.y)) {
                std::cerr << "Error: Start position (" << pos.x << ", " << pos.y << ") does not exist" << std::endl;
                return false;
            }
        }
        
        for (const auto& pos : goal_positions) {
            if (!graph->existNode(pos.x, pos.y)) {
                std::cerr << "Error: Goal position (" << pos.x << ", " << pos.y << ") does not exist" << std::endl;
                return false;
            }
        }
        
        // Create temporary problem instance for solver initialization
        Config config_s, config_g;
        for (const auto& pos : start_positions) {
            config_s.push_back(graph->getNode(pos.x, pos.y));
        }
        for (const auto& pos : goal_positions) {
            config_g.push_back(graph->getNode(pos.x, pos.y));
        }
        
        // Create a simple Problem instance with is_lmapf flag
        // The flag controls whether we do dynamic goal updates or not
        problem_instance = std::unique_ptr<Problem>(new Problem(
            "PyPIBT_Instance", graph.get(), &mt_generator, config_s, config_g, 
            starts.size(), 1000, 60000, is_lmapf
        ));
        
        // Create solver
        solver = std::make_unique<PIBT>(problem_instance.get());
        
        // Setup edge weights if provided
        if (!edge_weights.empty()) {
            setupEdgeWeights(edge_weights);
        }
        
        // Initialize step execution
        if (!solver->initializeStep(problem_instance->getConfigStart(), problem_instance->getConfigGoal())) {
            std::cerr << "Error: Failed to initialize step execution" << std::endl;
            return false;
        }
        

        
        initialized = true;
        timestep = 0;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error initializing PyPIBT: " << e.what() << std::endl;
        return false;
    }
}



std::vector<std::tuple<int, int>> PyPIBT::step() {
    if (!initialized) {
        return {};
    }
    
    // Execute one step of PIBT
    if (!solver->stepOnce()) {
        return {}; // Failed to compute next step
    }
    
    // Get current positions from solver
    Config current_positions = solver->getCurrentPositions();
    
    // Convert result back to position tuples
    std::vector<Position> next_positions;
    for (const auto& node : current_positions) {
        next_positions.emplace_back(node->pos.x, node->pos.y);
    }
    
    timestep++;
    
    return positionsToTuples(next_positions);
}

void PyPIBT::updateGoals(const std::vector<std::tuple<int, int, int>>& agent_goal_updates) {
    if (!initialized) {
        return;
    }
    
    // Update goals for the specified agents
    for (const auto& update : agent_goal_updates) {
        int agent_id = std::get<0>(update);
        int goal_x = std::get<1>(update);  
        int goal_y = std::get<2>(update);
        
        // Validate agent ID and goal position
        if (agent_id < 0 || agent_id >= getNumAgents()) {
            continue; // Skip invalid agent ID
        }
        
        if (!graph->existNode(goal_x, goal_y)) {
            continue; // Skip invalid goal position
        }
        
        // Get the goal node
        Node* new_goal = graph->getNode(goal_x, goal_y);
        
        // Update the goal in the problem instance
        problem_instance->setAgentGoal(agent_id, new_goal);
        
        // Update distance table and agent goal/priority in the solver
        solver->updateAgentGoal(agent_id, new_goal);
    }
}






int PyPIBT::getNumAgents() const {
    return problem_instance ? problem_instance->getNum() : 0;
}

int PyPIBT::getTimestep() const {
    return timestep;
}


int PyPIBT::getMapWidth() const {
    if (!graph) return 0;
    Grid* grid = dynamic_cast<Grid*>(graph.get());
    return grid ? grid->getWidth() : 0;
}

int PyPIBT::getMapHeight() const {
    if (!graph) return 0;
    Grid* grid = dynamic_cast<Grid*>(graph.get());
    return grid ? grid->getHeight() : 0;
}

bool PyPIBT::isValidPosition(int x, int y) const {
    return graph && graph->existNode(x, y);
}

std::vector<std::vector<int>> PyPIBT::getMapMatrix() const {
    std::vector<std::vector<int>> matrix;
    
    if (!graph) return matrix;
    
    int width = getMapWidth();
    int height = getMapHeight();
    
    matrix.resize(height, std::vector<int>(width, 1)); // Initialize with obstacles
    
    // Mark free spaces as 0
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (graph->existNode(x, y)) {
                matrix[y][x] = 0; // Free space
            }
        }
    }
    
    return matrix;
} 