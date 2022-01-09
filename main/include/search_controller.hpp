#ifndef SEARCH_CONTROLLER_HPP
#define SEARCH_CONTROLLER_HPP

#include "defines.hpp"
#include "include/adachi.hpp"
#include "include/motion_planning.hpp"

class SearchController {
public:
  SearchController();
  ~SearchController();
  void reset();
  void exec(param_set_t &p_set);

  void set_lgc(std::shared_ptr<MazeSolverBaseLgc> &_lgc);
  void set_motion_plannning(std::shared_ptr<MotionPlanning> &_mp);

  void set_param();

private:
  std::shared_ptr<MotionPlanning> mp;
  std::shared_ptr<MazeSolverBaseLgc> lgc;

  std::shared_ptr<Adachi> adachi;
  std::shared_ptr<ego_t> ego;

  MotionResult go_straight_wrapper(param_set_t &p_set);
  MotionResult slalom(param_set_t &p_set, const TurnDirection td);
  MotionResult pivot(param_set_t &p_set);
  MotionResult finish(param_set_t &p_set);
};

#endif