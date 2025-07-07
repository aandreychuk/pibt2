#include "problem.hpp"

#include <fstream>
#include <regex>

#include "util.hpp"

Problem::Problem(std::string _instance, Graph* _G, std::mt19937* _MT,
                 Config _config_s, Config _config_g, int _num_agents,
                 int _max_timestep, int _max_comp_time, bool _is_lmapf)
    : instance(_instance),
      G(_G),
      MT(_MT),
      config_s(_config_s),
      config_g(_config_g),
      num_agents(_num_agents),
      max_timestep(_max_timestep),
      max_comp_time(_max_comp_time),
      is_lmapf_mode(_is_lmapf){};

Node* Problem::getStart(int i) const
{
  if (!(0 <= i && i < (int)config_s.size())) halt("invalid index");
  return config_s[i];
}

Node* Problem::getGoal(int i) const
{
  if (!(0 <= i && i < (int)config_g.size())) halt("invalid index");
  return config_g[i];
}

void Problem::halt(const std::string& msg) const
{
  std::cout << "error@Problem: " << msg << std::endl;
  this->~Problem();
  std::exit(1);
}

void Problem::warn(const std::string& msg) const
{
  std::cout << "warn@Problem: " << msg << std::endl;
}