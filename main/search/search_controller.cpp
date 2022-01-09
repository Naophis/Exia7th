#include "include/search_controller.hpp"

SearchController::SearchController() {
  adachi = std::make_shared<Adachi>();
  ego = std::make_shared<ego_t>();
  adachi->set_ego(ego);
}
SearchController::~SearchController() {}

void SearchController::set_lgc(std::shared_ptr<MazeSolverBaseLgc> &_lgc) {
  lgc = _lgc;
  adachi->set_logic(_lgc);
}
void SearchController::set_motion_plannning(
    std::shared_ptr<MotionPlanning> &_mp) {
  mp = _mp;
}
void SearchController::reset() {
  ego->dir = Direction::North;
  ego->x = 0;
  ego->y = 0;
  ego->prev_motion = 0; // not use
  lgc->set_default_wall_data();
}
MotionResult SearchController::go_straight_wrapper(param_set_t &p_set) {
  param_straight_t p;
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = p.v_max;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 90;
  return mp->go_straight(p);
}
MotionResult SearchController::slalom(param_set_t &p_set,
                                      const TurnDirection td) {
  slalom_param2_t sp;
  next_motionr_t nm;

  return mp->slalom(sp, td, nm);
}
MotionResult SearchController::pivot(param_set_t &p_set) {
  param_roll_t pr;
  return mp->pivot_turn(pr);
}
MotionResult SearchController::finish(param_set_t &p_set) {
  param_straight_t p;
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = 10;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 45;
  return mp->go_straight(p);
}
void SearchController::exec(param_set_t &p_set) {

  while (1) {
    auto next_motion = adachi->exec();
    // int motion_result = 0;
    if (next_motion == Motion::Straight) {
      go_straight_wrapper(p_set);
    } else if (next_motion == Motion::TurnRight) {
      slalom(p_set, TurnDirection::Right);
    } else if (next_motion == Motion::TurnLeft) {
      slalom(p_set, TurnDirection::Left);
    } else if (next_motion == Motion::Back) {
      pivot(p_set);
    } else if (next_motion == Motion::NONE) {
      finish(p_set);
      break;
    }
  }
}