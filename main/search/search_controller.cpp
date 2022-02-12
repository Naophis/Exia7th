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

void SearchController::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_entity) {
  sensing_result = _entity;
}

void SearchController::set_logging_task(std::shared_ptr<LoggingTask> &_lt) {
  lt = _lt;
}

void SearchController::set_userinterface(std::shared_ptr<UserInterface> &_ui) {
  ui = _ui;
}
void SearchController::reset() {
  ego->dir = Direction::North;
  ego->x = 0;
  ego->y = 1;
  ego->prev_motion = 0; // not use
  lgc->set_default_wall_data();
  adachi->reset_goal();
}
MotionResult SearchController::go_straight_wrapper(param_set_t &p_set) {
  param_straight_t p;
  p.v_max = p.v_end = p_set.str_map[StraightType::Search].v_max;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 90;
  p.motion_type = MotionType::STRAIGHT;
  p.wall_off_req = WallOffReq::SEARCH;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_dist_r = param->sen_ref_p.search_exist.offset_r;
  p.wall_off_dist_l = param->sen_ref_p.search_exist.offset_l;
  p.dia_mode = false;
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
  MotionResult res;
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = 20;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 40;
  p.motion_type = MotionType::STRAIGHT;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  res = mp->go_straight(p);

  if (res == MotionResult::ERROR)
    return MotionResult::ERROR;

  p.v_max = 20;
  p.v_end = 5;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 5;
  p.motion_type = MotionType::STRAIGHT;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  res = mp->go_straight(p);
  if (res == MotionResult::ERROR)
    return MotionResult::ERROR;

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

  res = mp->pivot_turn(pr);
  // if (res == MotionResult::ERROR)
  //   return MotionResult::ERROR;

  mp->reset_tgt_data();
  mp->reset_ego_data();
  vTaskDelay(100 / portTICK_RATE_MS);
  pt->motor_disable();

  mp->reset_gyro_ref();

  ui->coin(100);

  vTaskDelay(100 / portTICK_RATE_MS);
  pt->motor_enable();
  mp->reset_tgt_data();
  mp->reset_ego_data();
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = p_set.str_map[StraightType::Search].v_max;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 45;
  p.motion_type = MotionType::STRAIGHT;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  res = mp->go_straight(p);
  // ui->coin(100);

  return res;
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
bool SearchController::is_goaled() { return tmp_goal_list.size() == 0; }

void SearchController::exec(param_set_t &p_set, SearchMode sm) {

  mp->reset_gyro_ref_with_check();
  lt->start_slalom_log();
  reset();

  for (const auto p : lgc->goal_list) {
    lgc->map[p.x + p.y * lgc->maze_size] =
        (lgc->map[p.x + p.y * lgc->maze_size] & 0x0f);
  }

  param_straight_t p;
  p.v_max = p.v_end = p_set.str_map[StraightType::Search].v_max;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 45 + param->offset_start_dist;
  p.sct = SensorCtrlType::Straight;
  p.motion_type = MotionType::STRAIGHT;
  // p.wall_off_req = WallOffReq::SEARCH;
  p.wall_off_req = WallOffReq::NONE;
  p.wall_off_dist_r = param->sen_ref_p.search_exist.offset_r;
  p.wall_off_dist_l = param->sen_ref_p.search_exist.offset_l;

  pt->motor_enable();
  mp->reset_tgt_data();
  mp->reset_ego_data();
  mp->go_straight(p);
  int back_cnt = 0;
  // pivot(p_set);
  while (1) {
    // sensing(ego);
    judge_wall();
    //この探索中ゴールしたか
    tmp_goal_list.erase(ego->x + ego->y * lgc->maze_size);
    // 片道モードでゴールしたらbreak
    if (sm == SearchMode::Kata)
      if (adachi->is_goaled())
        break;
    //往復モードでゴールしたらbreak
    if (sm == SearchMode::Return)
      if (adachi->is_goaled() && (ego->x == 0 && ego->y == 0))
        break;

    // 足立法で行き先決定
    auto next_motion = adachi->exec();
    // go_straight_wrapper(p_set);
    if (next_motion == Motion::Straight) {
      go_straight_wrapper(p_set);
    } else if (next_motion == Motion::TurnRight) {
      slalom(p_set, TurnDirection::Right);
    } else if (next_motion == Motion::TurnLeft) {
      slalom(p_set, TurnDirection::Left);
    } else if (next_motion == Motion::Back) {
      pivot(p_set);
      back_cnt++;
      // break;
    } else if (next_motion == Motion::NONE) {
      break;
    }

    if (next_motion == Motion::Back) {
      back_cnt++;
    } else {
      back_cnt = 0;
    }

    if (back_cnt == 3) {
      break;
    }
  }
  finish(p_set);
  pt->motor_disable();
  lt->stop_slalom_log();
  lt->save(slalom_log_file);
  mp->coin();
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
}
void SearchController::judge_wall() {
  bool wall_n = false;
  bool wall_e = false;
  bool wall_w = false;
  bool wall_s = false;

  if (ego->dir == Direction::North) {
    wall_n = sensing_result->ego.front_lp > param->sen_ref_p.search_exist.front;
    wall_e =
        sensing_result->ego.right45_lp > param->sen_ref_p.search_exist.right45;
    wall_w =
        sensing_result->ego.left45_lp > param->sen_ref_p.search_exist.left45;
  } else if (ego->dir == Direction::East) {
    wall_e = sensing_result->ego.front_lp > param->sen_ref_p.search_exist.front;
    wall_s =
        sensing_result->ego.right45_lp > param->sen_ref_p.search_exist.right45;
    wall_n =
        sensing_result->ego.left45_lp > param->sen_ref_p.search_exist.left45;

  } else if (ego->dir == Direction::West) {
    wall_w = sensing_result->ego.front_lp > param->sen_ref_p.search_exist.front;
    wall_n =
        sensing_result->ego.right45_lp > param->sen_ref_p.search_exist.right45;
    wall_s =
        sensing_result->ego.left45_lp > param->sen_ref_p.search_exist.left45;

  } else if (ego->dir == Direction::South) {
    wall_s = sensing_result->ego.front_lp > param->sen_ref_p.search_exist.front;
    wall_w =
        sensing_result->ego.right45_lp > param->sen_ref_p.search_exist.right45;
    wall_e =
        sensing_result->ego.left45_lp > param->sen_ref_p.search_exist.left45;
  }
  lgc->set_wall_data(ego->x, ego->y, Direction::North, wall_n);
  lgc->set_wall_data(ego->x, ego->y + 1, Direction::South, wall_n);

  lgc->set_wall_data(ego->x, ego->y, Direction::East, wall_e);
  lgc->set_wall_data(ego->x + 1, ego->y, Direction::West, wall_e);

  lgc->set_wall_data(ego->x, ego->y, Direction::West, wall_w);
  lgc->set_wall_data(ego->x - 1, ego->y, Direction::East, wall_w);

  lgc->set_wall_data(ego->x, ego->y, Direction::South, wall_s);
  lgc->set_wall_data(ego->x, ego->y - 1, Direction::North, wall_s);

  // lgc->set_wall_data(0, 0, Direction::East, true);
  // lgc->set_wall_data(1, 0, Direction::West, true);
}
void SearchController::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param) {
  param = _param;
}

void SearchController::print_maze() {
  printf("\r\n");
  for (int j = lgc->maze_size - 1; j >= 0; j--) {
    printf("+");
    for (int i = 0; i < lgc->maze_size; i++) {
      if (lgc->existWall(i, j, Direction::North)) {
        printf("----+");
      } else {
        printf("    +");
      }
    }
    printf("\r\n");
    for (int i = 0; i < lgc->maze_size; i++) {
      if (lgc->existWall(i, j, Direction::West)) {
        printf("| ");
      } else {
        printf("  ");
      }
      printf("%2x ", lgc->dist[i + j * lgc->maze_size]);
    }
    printf("|\r\n");
  }
  printf("+");

  for (int i = 0; i < lgc->maze_size; i++) {
    if (lgc->existWall(i, 0, Direction::South)) {
      printf("----+");
    } else {
      printf("    +");
    }
  }
  printf("\r\n");
}