#include "pibt.hpp"

const std::string PIBT::SOLVER_NAME = "PIBT";

PIBT::PIBT(Problem* _P) : MAPF_Solver(_P), step_initialized(false)
{
  occupied_now.resize(P->getG()->getNodesSize());
  occupied_next.resize(P->getG()->getNodesSize());
  solver_name = SOLVER_NAME;
  
  // Detect instance type
  is_lmapf_instance = P->isLMAPF();
}

PIBT::~PIBT() {
  // Clean up step execution agents if they exist
  for (auto a : step_agents) {
    delete a;
  }
  step_agents.clear();
}

void PIBT::run()
{
  std::cout<<"RUN PIBT\n";
  // compare priority of agents
  auto compare = [](Agent* a, const Agent* b) {
    if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;
    // use initial distance
    if (a->init_d != b->init_d) return a->init_d > b->init_d;
    return a->tie_breaker > b->tie_breaker;
  };
  Agents A;
  reached_goals = 0;

  // initialize
  for (int i = 0; i < P->getNum(); ++i) {
    Node* s = P->getStart(i);
    Node* g = P->getGoal(i);
    int d = disable_dist_init ? 0 : pathDist(i);
    Agent* a = new Agent{i,                          // id
                         s,                          // current location
                         nullptr,                    // next location
                         g,                          // goal
                         0,                          // elapsed
                         d,                          // dist from s -> g
                         getRandomFloat(0, 1, MT)};  // tie-breaker
    A.push_back(a);
    occupied_now[s->id] = a;
  }
  solution.add(P->getConfigStart());

  // Determine instance type once
  bool is_lmapf = P->isLMAPF();

  // main loop
  int timestep = 0;
  while (true) {
    //info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);

    // planning
    std::sort(A.begin(), A.end(), compare);
    for (auto a : A) {
      // if the agent has next location, then skip
      if (a->v_next == nullptr) {
        // determine its next location
        funcPIBT(a);
      }
    }

    // acting
    bool check_goal_cond = true;  // Initialize as true for MAPF
    Config config(P->getNum(), nullptr);
    for (auto a : A) {
      // clear
      if (occupied_now[a->v_now->id] == a) occupied_now[a->v_now->id] = nullptr;
      occupied_next[a->v_next->id] = nullptr;
      // set next location
      config[a->id] = a->v_next;
      occupied_now[a->v_next->id] = a;
      // check goal condition (only relevant for MAPF)
      check_goal_cond &= (a->v_next == a->g);
      // update priority
      a->elapsed = (a->v_next == a->g) ? 0 : a->elapsed + 1;
      // reset params
      a->v_now = a->v_next;
      a->v_next = nullptr;
    }

    // update plan
    solution.add(config);
    
    // Handle goal updates differently for LMAPF vs MAPF
    if (is_lmapf) {
      // LMAPF: Goal updates are handled externally (by PyPIBT)
      // Just count reached goals for throughput calculation
      for (auto a : A) {
        if (a->v_next == a->g) {
          reached_goals++;
        }
      }
    } else {
      // MAPF: Agents stay at their goal once reached
      // No goal updates needed - agents keep their original goals
      for (auto a : A) {
        if (a->v_next == a->g) {
          reached_goals++;
        }
      }
    }

    ++timestep;

    // success (only for MAPF - when all agents reach their goals)
    if (!is_lmapf && check_goal_cond) {
      solved = true;
      break;
    }

    // failed
    if (timestep >= max_timestep || overCompTime()) {
      break;
    }
  }
  
  // Store metrics
  timesteps_run = timestep;
  
  // Output different metrics based on instance type
  std::ofstream out("log.json", std::ios::app);
  int seed = 0; // Default seed - can be enhanced later if needed
  
  if (is_lmapf) {
    // LMAPF metrics: throughput
    throughput = double(reached_goals) / max_timestep;
    std::cout << "Throughput = " << throughput << "\n";
    out << R"({"metrics": {"throughput": )" << throughput 
        << R"(, "runtime": )" << getSolverElapsedTime()/1000.0 
        << R"(}, "env_grid_search": {"map_name": "wfi_warehouse", "num_agents": )" << P->getNum() 
        << R"(, "seed": )" << seed 
        << R"(}, "algorithm": "PIBT"}, )";
  } else {
    // MAPF metrics: Sum of Costs (SoC) and makespan
    int soc = solution.getSOC();
    int makespan = solution.getMakespan();
    std::cout << "Sum of Costs (SoC) = " << soc << "\n";
    std::cout << "Makespan = " << makespan << "\n";
    out << R"({"metrics": {"SoC": )" << soc 
        << R"(, "makespan": )" << makespan
        << R"(, "runtime": )" << getSolverElapsedTime()/1000.0 
        << R"(}, "env_grid_search": {"map_name": "wfi_warehouse", "num_agents": )" << P->getNum() 
        << R"(, "seed": )" << seed 
        << R"(}, "algorithm": "PIBT"}, )";
  }
  out.close();

  // memory clear
  for (auto a : A) delete a;
}

void PIBT::makeLogBasicInfo(std::ofstream& log)
{
  Grid* grid = reinterpret_cast<Grid*>(P->getG());
  log << "instance=" << P->getInstanceFileName() << "\n";
  log << "agents=" << P->getNum() << "\n";
  log << "map_file=" << grid->getMapFileName() << "\n";
  log << "solver=" << solver_name << "\n";
  
  if (is_lmapf_instance) {
    // LMAPF-specific metrics
    log << "solved=1\n";  // LMAPF doesn't have a "solved" state, always consider it as running
    log << "reached_goals=" << reached_goals << "\n";
    log << "timesteps_run=" << timesteps_run << "\n";
    log << "throughput=" << throughput << "\n";
    log << "goals_per_timestep=" << (timesteps_run > 0 ? (double)reached_goals / timesteps_run : 0.0) << "\n";
    log << "max_timestep=" << max_timestep << "\n";
    
    // Still include SOC and makespan for compatibility but note they're less meaningful for LMAPF
    log << "soc=" << solution.getSOC() << "\n";
    log << "lb_soc=" << getLowerBoundSOC() << "\n";
    log << "makespan=" << solution.getMakespan() << "\n";
    log << "lb_makespan=" << getLowerBoundMakespan() << "\n";
  } else {
    // MAPF-specific metrics (original behavior)
    log << "solved=" << solved << "\n";
    log << "soc=" << solution.getSOC() << "\n";
    log << "lb_soc=" << getLowerBoundSOC() << "\n";
    log << "makespan=" << solution.getMakespan() << "\n";
    log << "lb_makespan=" << getLowerBoundMakespan() << "\n";
  }
  
  log << "comp_time=" << getCompTime() << "\n";
  log << "preprocessing_comp_time=" << preprocessing_comp_time << "\n";
}

bool PIBT::funcPIBT(Agent* ai, Agent* aj)
{
  // compare two nodes
  auto compare = [&](Node* const v, Node* const u) {
    int d_v = pathDist(v, ai->g);
    int d_u = pathDist(u, ai->g);
    if (d_v != d_u) return d_v < d_u;
    // tie-break
    if (occupied_now[v->id] != nullptr && occupied_now[u->id] == nullptr)
      return false;
    if (occupied_now[v->id] == nullptr && occupied_now[u->id] != nullptr)
      return true;
    return false;
  };

  // get candidates
  Nodes C = ai->v_now->neighbor;
  C.push_back(ai->v_now);
  // randomize
  std::shuffle(C.begin(), C.end(), *MT);
  // sort
  std::sort(C.begin(), C.end(), compare);

  for (auto u : C) {
    // avoid conflicts
    if (occupied_next[u->id] != nullptr) continue;
    if (aj != nullptr && u == aj->v_now) continue;

    // reserve
    occupied_next[u->id] = ai;
    ai->v_next = u;

    auto ak = occupied_now[u->id];
    if (ak != nullptr && ak->v_next == nullptr) {
      if (!funcPIBT(ak, ai)) continue;  // replanning
    }
    // success to plan next one step
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

void PIBT::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"disable-dist-init", no_argument, 0, 'd'},
      {"weights-file", required_argument, 0, 'w'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "dw:", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'd':
        disable_dist_init = true;
        break;
      case 'w':
        weights_file = std::string(optarg);
        setWeightsFile(weights_file);
        break;
      default:
        break;
    }
  }
}

bool PIBT::initializeStep(const Config& start_positions, const Config& goal_positions) {
  // Clean up any existing step agents
  for (auto a : step_agents) {
    delete a;
  }
  step_agents.clear();
  
  // Clear occupied tables
  std::fill(occupied_now.begin(), occupied_now.end(), nullptr);
  std::fill(occupied_next.begin(), occupied_next.end(), nullptr);
  
  // Initialize agents
  for (int i = 0; i < P->getNum(); ++i) {
    Node* s = start_positions[i];
    Node* g = goal_positions[i];
    int d = disable_dist_init ? 0 : pathDist(s, g);
    Agent* a = new Agent{i,                          // id
                         s,                          // current location
                         nullptr,                    // next location
                         g,                          // goal
                         0,                          // elapsed
                         d,                          // dist from s -> g
                         getRandomFloat(0, 1, MT)};  // tie-breaker
    step_agents.push_back(a);
    occupied_now[s->id] = a;
  }
  
  step_initialized = true;
  return true;
}

bool PIBT::stepOnce() {
  if (!step_initialized || step_agents.empty()) {
    return false;
  }
  
  // Clear occupied_next for this step
  std::fill(occupied_next.begin(), occupied_next.end(), nullptr);
  
  // Compare priority of agents (same as in run())
  auto compare = [](Agent* a, const Agent* b) {
    if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;
    if (a->init_d != b->init_d) return a->init_d > b->init_d;
    return a->tie_breaker > b->tie_breaker;
  };
  
  // Planning phase
  std::sort(step_agents.begin(), step_agents.end(), compare);
  for (auto a : step_agents) {
    if (a->v_next == nullptr) {
      funcPIBT(a);
    }
  }
  
  // Acting phase - update agent positions
  for (auto a : step_agents) {
    // Clear old position from occupied table
    if (occupied_now[a->v_now->id] == a) {
      occupied_now[a->v_now->id] = nullptr;
    }
    
    // Update agent position
    a->v_now = a->v_next;
    a->v_next = nullptr;
    
    // Update occupied table
    occupied_now[a->v_now->id] = a;
    
    // Update priority
    a->elapsed = (a->v_now == a->g) ? 0 : a->elapsed + 1;
  }
  
  return true;
}

Config PIBT::getCurrentPositions() const {
  Config positions;
  if (!step_initialized) {
    return positions;
  }
  
  positions.resize(step_agents.size());
  for (const auto& agent : step_agents) {
    positions[agent->id] = agent->v_now;
  }
  
  return positions;
}

void PIBT::updateGoals() {
  if (!step_initialized || step_agents.empty()) {
    return;
  }
  
  // Check if this is an LMAPF mode problem
  if (!P->isLMAPF()) {
    return; // Not in LMAPF mode, no goal updates needed
  }
  
  // This method is now primarily used by PyPIBT which handles the goal updates manually
  // We keep this for compatibility but the actual goal updates are done in PyPIBT::updateGoals()
}

bool PIBT::updateAgentGoal(int agent_id, Node* new_goal) {
  if (!step_initialized || step_agents.empty() || agent_id < 0 || agent_id >= static_cast<int>(step_agents.size())) {
    return false;
  }
  
  // Find the agent by ID
  Agent* agent = nullptr;
  for (auto a : step_agents) {
    if (a->id == agent_id) {
      agent = a;
      break;
    }
  }
  
  if (!agent || !new_goal) {
    return false;
  }
  
  // Update the agent's goal
  Node* old_goal = agent->g;
  agent->g = new_goal;
  
  // Update the problem instance's goal configuration
  P->setAgentGoal(agent_id, new_goal);
  
  // Update distance table and priority for this agent
  if (old_goal->id != new_goal->id) {
    createDistanceTable(agent_id);
    reached_goals++;
    // Update initial distance for priority calculation
    agent->init_d = disable_dist_init ? 0 : pathDist(agent->v_now, agent->g);
  }
  
  return true;
}

void PIBT::printHelp()
{
  std::cout << PIBT::SOLVER_NAME << "\n"
            << "  -d --disable-dist-init"
            << "        "
            << "disable initialization of priorities "
            << "using distance from starts to goals\n"
            << "  -w --weights-file [FILE]"
            << "      "
            << "use edge weights from CSV file for distance calculation" << std::endl;
}
