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

  // utilities
  void halt(const std::string& msg) const;
  void warn(const std::string& msg) const;

public:
  Problem(){};
  Problem(const std::string& _instance) : instance(_instance) {}
  Problem(std::string _instance, Graph* _G, std::mt19937* _MT, Config _config_s,
          Config _config_g, int _num_agents, int _max_timestep,
          int _max_comp_time);
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

  void setMaxCompTime(const int t) { max_comp_time = t; }
};

class LMAPF_Instance : public Problem
{
  const bool instance_initialized;  // for memory manage
  Configs all_goals;
  std::vector<int> cur_goals;

public:
  int seed;
  LMAPF_Instance(const std::string& _instance, int max_agents = -1);
  LMAPF_Instance():instance_initialized(false) {}
  void update_goals(Config cur_positions);
  ~LMAPF_Instance();
  
  // Getters for validation
  const Configs& getAllGoals() const { return all_goals; }
  const std::vector<int>& getCurrentGoals() const { return cur_goals; }
  Config getGoalConfigForAgent(int agent_id, int goal_index) const {
    if (agent_id >= 0 && agent_id < (int)all_goals.size() && 
        goal_index >= 0 && goal_index < (int)all_goals[agent_id].size()) {
      return {all_goals[agent_id][goal_index]};
    }
    return {};
  }
};

class MAPF_Instance : public Problem
{
private:
  const bool instance_initialized;  // for memory manage

  // set starts and goals randomly
  void setRandomStartsGoals();

  // set well-formed instance
  void setWellFormedInstance();

public:
  MAPF_Instance(const std::string& _instance);
  MAPF_Instance(MAPF_Instance* P, Config _config_s, Config _config_g,
                int _max_comp_time, int _max_timestep);
  MAPF_Instance(MAPF_Instance* P, int _max_comp_time);
  ~MAPF_Instance();

  bool isInitializedInstance() const { return instance_initialized; }

  // used when making new instance file
  void makeScenFile(const std::string& output_file);
};
