#ifndef SEARCH_CONTROLLER_HPP
#define SEARCH_CONTROLLER_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "defines.hpp"
#include "include/adachi.hpp"
#include "include/logging_task.hpp"
#include "include/motion_planning.hpp"
#include "include/planning_task.hpp"

class SearchController {
public:
  SearchController();
  ~SearchController();
  void reset();
  void exec(param_set_t &p_set, SearchMode sm);

  void set_lgc(std::shared_ptr<MazeSolverBaseLgc> &_lgc);
  void set_motion_plannning(std::shared_ptr<MotionPlanning> &_mp);
  void set_planning_task(std::shared_ptr<PlanningTask> &_pt);
  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
  void set_logging_task(std::shared_ptr<LoggingTask> &_lt);
  void set_userinterface(std::shared_ptr<UserInterface> &_ui);
  
  void print_maze();

private:
  std::shared_ptr<MotionPlanning> mp;
  std::shared_ptr<MazeSolverBaseLgc> lgc;
  std::shared_ptr<PlanningTask> pt;
  std::shared_ptr<Adachi> adachi;
  std::shared_ptr<LoggingTask> lt;
  std::shared_ptr<UserInterface> ui;

  unordered_map<unsigned int, unsigned char> tmp_goal_list;

  std::shared_ptr<ego_t> ego;
  std::shared_ptr<sensing_result_entity_t> sensing_result;

  MotionResult go_straight_wrapper(param_set_t &p_set);
  MotionResult slalom(param_set_t &p_set, const TurnDirection td);
  MotionResult pivot(param_set_t &p_set);
  MotionResult finish(param_set_t &p_set);

  void judge_wall();
  bool is_goaled();
};

#endif