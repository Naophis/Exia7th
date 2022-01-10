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

void SearchController::set_planning_task(std::shared_ptr<PlanningTask> &_pt) {
  pt = _pt;
}

void SearchController::reset() {
  ego->dir = Direction::North;
  ego->x = 0;
  ego->y = 1;
  ego->prev_motion = 0; // not use
  lgc->set_default_wall_data();
}
MotionResult SearchController::go_straight_wrapper(param_set_t &p_set) {
  param_straight_t p;
  p.v_max = p.v_end = p_set.str_map[StraightType::Search].v_max;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 90;
  p.motion_type = MotionType::NONE;
  return mp->go_straight(p);
}
MotionResult SearchController::slalom(param_set_t &p_set,
                                      const TurnDirection td) {
  slalom_param2_t sp = p_set.map[TurnType::Normal];

  next_motionr_t nm;
  nm.v_max = p_set.map[TurnType::Normal].v;
  nm.v_end = p_set.map[TurnType::Normal].v;
  nm.accl = p_set.str_map[StraightType::Search].accl;
  nm.decel = p_set.str_map[StraightType::Search].decel;
  nm.is_turn = false;

  return mp->slalom(sp, td, nm);
}
MotionResult SearchController::pivot(param_set_t &p_set) {
  param_straight_t p;
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = 0;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 45;
  p.motion_type = MotionType::NONE;
  mp->go_straight(p);

  mp->reset_tgt_data();
  mp->reset_ego_data();
  vTaskDelay(100 / portTICK_RATE_MS);
  pt->motor_disable();

  param_roll_t pr;
  pr.w_max = p_set.str_map[StraightType::Search].w_max;
  pr.alpha = p_set.str_map[StraightType::Search].alpha;
  pr.w_end = 0;
  pr.ang = PI;
  pr.RorL = TurnDirection::Right;
  pt->motor_enable();
  mp->pivot_turn(pr);
  mp->reset_tgt_data();
  mp->reset_ego_data();
  pt->motor_disable();
  vTaskDelay(100 / portTICK_RATE_MS);

  mp->reset_gyro_ref();

  mp->reset_tgt_data();
  mp->reset_ego_data();
  pt->motor_enable();
  p.v_max = p.v_end = p_set.str_map[StraightType::Search].v_max;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 45;
  p.motion_type = MotionType::NONE;
  return mp->go_straight(p);
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

  mp->reset_gyro_ref_with_check();
  reset();

  param_straight_t p;
  p.v_max = p.v_end = p_set.str_map[StraightType::Search].v_max;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 45;
  p.motion_type = MotionType::NONE;

  pt->motor_enable();
  mp->reset_tgt_data();
  mp->reset_ego_data();
  mp->go_straight(p);

  while (1) {
    auto next_motion = adachi->exec();
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
    break;
  }

  p.v_max = p.v_end = p_set.str_map[StraightType::Search].v_max;
  p.accl = 0;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 45;
  p.motion_type = MotionType::NONE;
  mp->go_straight(p);

  pt->motor_disable();
  mp->coin();
}