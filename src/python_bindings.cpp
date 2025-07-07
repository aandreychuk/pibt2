#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "py_pibt.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pypibt, m) {
    m.doc() = "Python bindings for PIBT (Priority Inheritance with Backtracking) multi-agent pathfinding solver";
    
    // Expose PyPIBT class with matrix-based API
    py::class_<PyPIBT>(m, "PIBT")
        .def(py::init<>(), "Create a new PIBT solver instance")
        
        .def("initialize", &PyPIBT::initialize,
             "Initialize the PIBT solver with map matrix, starts, and goals",
             py::arg("map_matrix"), 
             py::arg("starts"), 
             py::arg("goals"),
             py::arg("is_lmapf") = false,
             py::arg("edge_weights") = std::vector<std::vector<double>>(),
             py::arg("disable_distance_init") = false,
             py::arg("seed") = 0,
             R"pbdoc(
                Initialize the PIBT solver.
                
                Parameters:
                    map_matrix (list[list[int]] or numpy.ndarray): 2D grid map where 0=free, 1=obstacle
                    starts (list[tuple]): Initial positions as list of (x, y) tuples
                    goals (list[tuple]): Initial goal positions as list of (x, y) tuples
                    is_lmapf (bool, optional): True for LMAPF mode, False for regular MAPF
                    edge_weights (list[list[float]], optional): Edge weights matrix where each row has 
                        [right_weight, up_weight, left_weight, down_weight, wait_weight] for corresponding node_id
                    disable_distance_init (bool, optional): Disable distance-based priority initialization
                    seed (int, optional): Random seed for reproducible results
                    
                Returns:
                    bool: True if initialization successful, False otherwise
                    
                Example:
                    # Map as 2D matrix (0=free, 1=obstacle)
                    map_matrix = [
                        [1, 1, 1, 1, 1, 1],
                        [1, 0, 0, 0, 0, 1], 
                        [1, 0, 0, 0, 0, 1],
                        [1, 1, 1, 1, 1, 1]
                    ]
                    starts = [(1, 1), (2, 1)]
                    goals = [(4, 2), (3, 2)]
                    
                    # For regular MAPF
                    solver.initialize(map_matrix, starts, goals, is_lmapf=False)
                    
                    # For LMAPF (goals updated dynamically via update_goals())
                    solver.initialize(map_matrix, starts, goals, is_lmapf=True)
             )pbdoc")
        

        
        .def("step", &PyPIBT::step,
             "Perform one planning step and return current positions",
             R"pbdoc(
                Perform one planning step of the PIBT algorithm.
                
                This method executes one step of the PIBT pathfinding algorithm using 
                the agents' current positions and goals (set during initialization). 
                The solver maintains agent state internally and returns current positions 
                after the step in agent ID order.
                
                Returns:
                    list[tuple]: Current positions as (x, y) tuples in agent ID order
                    
                Example:
                    # After initialization
                    next_pos = solver.step()
                    # next_pos might be [(2, 1), (2, 2)]
                    
                Note: 
                    Agent ordering is preserved - the i-th position in the output 
                    corresponds to the i-th agent from initialization. Goals are 
                    maintained from initialization, but can be updated using 
                    updateGoals() for LMAPF instances.
             )pbdoc")
        
        .def("update_goals", &PyPIBT::updateGoals,
             "Update goals for specific agents with new goal positions",
             py::arg("agent_goal_updates"),
             R"pbdoc(
                Update goals for specific agents that have reached their current goals.
                
                This method updates the goals for agents that need new goals. Python code
                should track goal sequences and agent progress, then call this method
                with the new goal positions.
                
                Parameters:
                    agent_goal_updates (list[tuple]): List of (agent_id, x, y) tuples
                        for agents that need new goals
                    
                Example:
                    # Agent 0 needs new goal at (5, 3), Agent 2 needs new goal at (2, 7)
                    solver.update_goals([(0, 5, 3), (2, 2, 7)])
             )pbdoc")
        
        .def("get_num_agents", &PyPIBT::getNumAgents,
             "Get the number of agents")
             
        .def("get_timestep", &PyPIBT::getTimestep,
             "Get the current timestep")
             
        .def("get_map_width", &PyPIBT::getMapWidth,
             "Get the width of the map")
             
        .def("get_map_height", &PyPIBT::getMapHeight,
             "Get the height of the map")
             
        .def("is_valid_position", &PyPIBT::isValidPosition,
             "Check if a position is valid (not an obstacle)",
             py::arg("x"), py::arg("y"))
             
        .def("get_map_matrix", &PyPIBT::getMapMatrix,
             "Get the current map as a 2D matrix (0=free, 1=obstacle)");
    
    // Module-level utility functions
    m.def("create_empty_map", [](int width, int height) {
        std::vector<std::vector<int>> map_matrix(height, std::vector<int>(width, 1));
        
        // Create empty interior with border walls
        for (int y = 1; y < height - 1; ++y) {
            for (int x = 1; x < width - 1; ++x) {
                map_matrix[y][x] = 0; // Free space
            }
        }
        
        return map_matrix;
    }, "Create an empty rectangular map matrix with border walls (0=free, 1=obstacle)", 
       py::arg("width"), py::arg("height"));
    
    m.def("matrix_from_array", [](const py::array_t<int>& array) {
        auto buf = array.request();
        if (buf.ndim != 2) {
            throw std::runtime_error("Array must be 2-dimensional");
        }
        
        int height = buf.shape[0];
        int width = buf.shape[1];
        int* ptr = static_cast<int*>(buf.ptr);
        
        std::vector<std::vector<int>> map_matrix(height, std::vector<int>(width));
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                map_matrix[y][x] = ptr[y * width + x];
            }
        }
        return map_matrix;
    }, "Convert 2D numpy array to map matrix",
       py::arg("array"));
    
    m.def("array_from_matrix", [](const std::vector<std::vector<int>>& matrix) {
        if (matrix.empty() || matrix[0].empty()) {
            throw std::runtime_error("Matrix cannot be empty");
        }
        
        int height = matrix.size();
        int width = matrix[0].size();
        
        auto result = py::array_t<int>({height, width});
        auto buf = result.request();
        int* ptr = static_cast<int*>(buf.ptr);
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                ptr[y * width + x] = matrix[y][x];
            }
        }
        return result;
    }, "Convert map matrix to 2D numpy array",
       py::arg("matrix"));
    
    m.def("create_uniform_weights", [](int total_nodes, double weight = 1.0) {
        return std::vector<std::vector<double>>(total_nodes, std::vector<double>(5, weight));
    }, "Create uniform edge weights matrix with same weight for all directions",
       py::arg("total_nodes"), py::arg("weight") = 1.0);
    
    m.def("create_directional_weights", [](int total_nodes, double right, double up, double left, double down, double wait) {
        std::vector<std::vector<double>> weights(total_nodes);
        for (int i = 0; i < total_nodes; ++i) {
            weights[i] = {right, up, left, down, wait};
        }
        return weights;
    }, "Create edge weights matrix with specified directional weights",
       py::arg("total_nodes"), py::arg("right"), py::arg("up"), py::arg("left"), py::arg("down"), py::arg("wait"));
    
    m.def("print_matrix", [](const std::vector<std::vector<int>>& matrix) {
        for (const auto& row : matrix) {
            std::string line;
            for (int val : row) {
                line += (val == 0) ? '.' : '@';
            }
            py::print(line);
        }
    }, "Print map matrix to console (0='.', 1='@')", py::arg("matrix"));
    
    m.def("node_id_to_coords", [](int node_id, int width) {
        return std::make_tuple(node_id % width, node_id / width);
    }, "Convert node ID to (x, y) coordinates",
       py::arg("node_id"), py::arg("width"));
    
    m.def("coords_to_node_id", [](int x, int y, int width) {
        return y * width + x;
    }, "Convert (x, y) coordinates to node ID",
       py::arg("x"), py::arg("y"), py::arg("width"));
    
    // Version info
    m.attr("__version__") = "2.0.0";
} 