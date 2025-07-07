/*
 * Implementation of Priority Inheritance with Backtracking (PIBT)
 *
 * - ref
 * Okumura, K., Machida, M., Défago, X., & Tamura, Y. (2019).
 * Priority Inheritance with Backtracking for Iterative Multi-agent Path
 * Finding. In Proceedings of the Twenty-Eighth International Joint Conference
 * on Artificial Intelligence (pp. 535–542).
 */

#pragma once
#include "solver.hpp"
#include <fstream>

class PIBT : public MAPF_Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  // PIBT agent
  struct Agent {
    int id;
    Node* v_now;        // current location
    Node* v_next;       // next location
    Node* g;            // goal
    int elapsed;        // eta
    int init_d;         // initial distance
    float tie_breaker;  // epsilon, tie-breaker
  };
  using Agents = std::vector<Agent*>;

  // <node-id, agent>, whether the node is occupied or not
  // work as reservation table
  Agents occupied_now;
  Agents occupied_next;

  // options
  bool disable_dist_init = false;
  std::string weights_file = "";

  // LMAPF-specific metrics
  int reached_goals = 0;
  int timesteps_run = 0;
  double throughput = 0.0;
  bool is_lmapf_instance = false;

  // Step-by-step execution state
  Agents step_agents;
  bool step_initialized = false;

  // result of priority inheritance: true -> valid, false -> invalid
  bool funcPIBT(Agent* ai, Agent* aj = nullptr);

  // Override logging methods for LMAPF support
  void makeLogBasicInfo(std::ofstream& log) override;

public:
  PIBT(Problem* _P);
  ~PIBT();

  // Step-by-step execution methods for PyPIBT
  bool initializeStep(const Config& start_positions, const Config& goal_positions);
  bool stepOnce();
  Config getCurrentPositions() const;
  void updateGoals();  // For LMAPF instances
  
  // Manual goal update for individual agents (for PyPIBT LMAPF support)
  bool updateAgentGoal(int agent_id, Node* new_goal);

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
