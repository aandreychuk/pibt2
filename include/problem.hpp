#pragma once
#include "graph.hpp"
#include <random>

#include "default_params.hpp"
#include "util.hpp"

using Config = std::vector<Node*>;  // < loc_0[t], loc_1[t], ... >
using Configs = std::vector<Config>;

// check two configurations are same or not
[[maybe_unused]] static bool sameConfig(const Config& config_i,
                                        const Config& config_j)
{
  if (config_i.size() != config_j.size()) return false;
  const int size_i = config_i.size();
  for (int k = 0; k < size_i; ++k) {
    if (config_i[k] != config_j[k]) return false;
  }
  return true;
}

[[maybe_unused]] static int getPathCost(const Path& path)
{
  int cost = path.size() - 1;
  auto itr = path.end() - 1;
  while (itr != path.begin() && *itr == *(itr - 1)) {
    --cost;
    --itr;
  }
  return cost;
}

class Problem
{
protected:
  std::string instance;  // instance name
  Graph* G;              // graph
  std::mt19937* MT;      // seed
  Config config_s;       // initial configuration
  Config config_g;       // goal configuration
  int num_agents;        // number of agents
  int max_timestep;      // timestep limit
  int max_comp_time;     // comp_time limit, ms
  
  // LMAPF support
  bool is_lmapf_mode;    // flag to control MAPF vs LMAPF behavior

  // utilities
  void halt(const std::string& msg) const;
  void warn(const std::string& msg) const;

public:
  Problem(){};
  Problem(const std::string& _instance) : instance(_instance), is_lmapf_mode(false) {}
  Problem(std::string _instance, Graph* _G, std::mt19937* _MT, Config _config_s,
          Config _config_g, int _num_agents, int _max_timestep,
          int _max_comp_time, bool _is_lmapf = false);
  virtual ~Problem(){};

  Graph* getG() { return G; }
  int getNum() { return num_agents; }
  std::mt19937* getMT() { return MT; }
  Node* getStart(int i) const;  // return start of a_i
  Node* getGoal(int i) const;   // return  goal of a_i
  Config getConfigStart() const { return config_s; };
  Config getConfigGoal() const { return config_g; };
  int getMaxTimestep() const { return max_timestep; };
  int getMaxCompTime() const { return max_comp_time; };
  std::string getInstanceFileName() { return instance; };
  bool isLMAPF() const { return is_lmapf_mode; }

  void setMaxCompTime(const int t) { max_comp_time = t; }
  void setLMAPFMode(bool lmapf) { is_lmapf_mode = lmapf; }
  
  // Method to set an individual agent's goal (unified for both MAPF and LMAPF)
  bool setAgentGoal(int agent_id, Node* new_goal) {
    if (agent_id >= 0 && agent_id < num_agents && new_goal) {
      config_g[agent_id] = new_goal;
      return true;
    }
    return false;
  }
};


