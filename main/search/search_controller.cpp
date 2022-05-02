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
  lgc->set_ego(ego);
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
  // param_straight_t p;
  p.v_max = p.v_end = p_set.str_map[StraightType::Search].v_max;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 90;
  p.motion_type = MotionType::STRAIGHT;
  // p.wall_off_req = WallOffReq::SEARCH;
  p.wall_off_req = WallOffReq::NONE;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_dist_r = param->sen_ref_p.search_exist.offset_r;
  p.wall_off_dist_l = param->sen_ref_p.search_exist.offset_l;
  p.dia_mode = false;
  return mp->go_straight(p, adachi);
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
void SearchController::front_wall_ctrl() {
  mp->reset_tgt_data();
  mp->reset_ego_data();
  vTaskDelay(25 / portTICK_RATE_MS);
  pt->motor_enable();
  mp->front_ctrl(false);
  mp->reset_tgt_data();
  mp->reset_ego_data();
  vTaskDelay(25 / portTICK_RATE_MS);
  pt->motor_disable(false);
}
MotionResult SearchController::pivot(param_set_t &p_set) {
  // param_straight_t p;
  MotionResult res;
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = 20;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 45;
  bool back_enable = false;
  if (40 < sensing_result->ego.front_dist &&
      (sensing_result->ego.front_dist < 100)) {
    p.dist += (sensing_result->ego.front_dist - 85);
    back_enable = true;
  }
  p.motion_type = MotionType::PIVOT_PRE;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  res = mp->go_straight(p);

  // if (res == MotionResult::ERROR)
  //   return MotionResult::ERROR;

  p.v_max = 20;
  p.v_end = 5;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 2;
  p.motion_type = MotionType::PIVOT_PRE2;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  res = mp->go_straight(p);
  if (res == MotionResult::ERROR)
    return MotionResult::ERROR;

  bool flag = false;
  float tmp_dist = 0;
  pr.w_max = p_set.str_map[StraightType::Search].w_max;
  pr.alpha = p_set.str_map[StraightType::Search].alpha;
  pr.w_end = p_set.str_map[StraightType::Search].w_end;

  if (sensing_result->ego.left45_dist < 45) {
    pr.RorL = TurnDirection::Left;
    flag = true;
  } else if (sensing_result->ego.right45_dist < 45) {
    pr.RorL = TurnDirection::Right;
    flag = true;
  } else {
    pr.ang = PI;
    pr.RorL = TurnDirection::Right;
    flag = false;
  }
  if (back_enable) {
    tmp_dist = 20;
  }
  bool front_ctrl = (sensing_result->ego.front_dist < 60);
  mp->reset_tgt_data();
  mp->reset_ego_data();
  vTaskDelay(25 / portTICK_RATE_MS);
  pt->motor_disable(false);

  if (adachi->goal_step && !saved) {
    vTaskDelay(200 / portTICK_RATE_MS);
    save_maze_data();
    vTaskDelay(200 / portTICK_RATE_MS);
    saved = true;
  }

  mp->reset_tgt_data();
  mp->reset_ego_data();

  if (front_ctrl) {
    front_wall_ctrl();
  }

  mp->reset_tgt_data();
  mp->reset_ego_data();
  pt->motor_enable();

  vTaskDelay(5 / portTICK_RATE_MS);

  if (flag) {
    p.motion_type = MotionType::PIVOT_OFFSET;
    p.sct = SensorCtrlType::NONE;
    pr.ang = PI / 2;

    res = mp->pivot_turn(pr);
    vTaskDelay(10 / portTICK_RATE_MS);
    front_ctrl = (sensing_result->ego.front_dist < 60);
    pt->motor_disable(false);
    vTaskDelay(10 / portTICK_RATE_MS);

    if (front_ctrl) {
      front_wall_ctrl();
    }

    mp->reset_tgt_data();
    mp->reset_ego_data();
    pt->motor_enable();
    res = mp->go_straight(p);
    mp->reset_tgt_data();
    mp->reset_ego_data();
    vTaskDelay(10 / portTICK_RATE_MS);
    pt->motor_disable(false);
    vTaskDelay(10 / portTICK_RATE_MS);
    mp->reset_tgt_data();
    mp->reset_ego_data();
    pt->motor_enable();
    res = mp->pivot_turn(pr);
    mp->reset_tgt_data();
    mp->reset_ego_data();
    vTaskDelay(10 / portTICK_RATE_MS);
    pt->motor_disable(false);
  } else {
    res = mp->pivot_turn(pr);
    mp->reset_tgt_data();
    mp->reset_ego_data();
    vTaskDelay(10 / portTICK_RATE_MS);
    pt->motor_disable(false);
  }

  // if (res == MotionResult::ERROR)
  //   return MotionResult::ERROR;

  if (back_enable) {
    p.v_max = -p_set.str_map[StraightType::Search].v_max;
    p.v_end = -10; // p_set.str_map[StraightType::Search].v_max;
    p.accl = -p_set.str_map[StraightType::Search].accl;
    p.decel = -p_set.str_map[StraightType::Search].decel;
    p.dist = -tmp_dist;
    p.motion_type = MotionType::BACK_STRAIGHT;
    p.sct = SensorCtrlType::NONE;
    p.wall_off_req = WallOffReq::NONE;
    mp->reset_tgt_data();
    mp->reset_ego_data();
    pt->motor_enable();
    res = mp->go_straight(p);
    mp->reset_tgt_data();
    mp->reset_ego_data();
    vTaskDelay(10 / portTICK_RATE_MS);
    pt->motor_disable(false);
  }

  // mp->reset_gyro_ref();
  // ui->coin(100);

  adachi->update();

  vTaskDelay(10 / portTICK_RATE_MS);
  mp->reset_tgt_data();
  mp->reset_ego_data();
  pt->motor_enable();
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = p_set.str_map[StraightType::Search].v_max;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  if (tmp_dist > 0) {
    p.dist = 45 + tmp_dist;
  } else {
    p.dist = 50;
  }
  p.motion_type = MotionType::PIVOT_AFTER;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  res = mp->go_straight(p);
  // ui->coin(100);

  return res;
}

MotionResult SearchController::pivot90(param_set_t &p_set,
                                       const TurnDirection td) {
  // param_straight_t p;
  MotionResult res;
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = 20;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 45;
  if (40 < sensing_result->ego.front_dist &&
      (sensing_result->ego.front_dist < 100)) {
    p.dist += (sensing_result->ego.front_dist - param->front_dist_offset_pivot);
  }
  p.motion_type = MotionType::PIVOT_PRE;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;

  float offset_dist = 45;
  bool front_ctrl = (sensing_result->ego.front_dist < 60);

  if (td == TurnDirection::Right) {
    if (sensing_result->ego.left45_dist < 58) {
      offset_dist += (45 - sensing_result->ego.left45_dist);
    }
  } else {
    if (sensing_result->ego.right45_dist < 58) {
      offset_dist += (45 - sensing_result->ego.right45_dist);
    }
  }

  res = mp->go_straight(p);

  // if (res == MotionResult::ERROR)
  //   return MotionResult::ERROR;

  p.v_max = 20;
  p.v_end = 5;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 2;
  p.motion_type = MotionType::PIVOT_PRE2;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  res = mp->go_straight(p);
  if (res == MotionResult::ERROR)
    return MotionResult::ERROR;

  if (front_ctrl) {
    front_wall_ctrl();
  }

  float tmp_dist = 0;
  pr.w_max = p_set.str_map[StraightType::Search].w_max;
  pr.alpha = p_set.str_map[StraightType::Search].alpha;
  pr.w_end = p_set.str_map[StraightType::Search].w_end;
  pr.ang = PI / 2;
  pr.RorL = td;

  mp->reset_tgt_data();
  mp->reset_ego_data();
  vTaskDelay(25 / portTICK_RATE_MS);
  pt->motor_disable(false);

  adachi->update();

  mp->reset_tgt_data();
  mp->reset_ego_data();
  pt->motor_enable();

  vTaskDelay(5 / portTICK_RATE_MS);

  res = mp->pivot_turn(pr);
  mp->reset_tgt_data();
  mp->reset_ego_data();
  vTaskDelay(10 / portTICK_RATE_MS);
  pt->motor_disable(false);

  vTaskDelay(10 / portTICK_RATE_MS);
  mp->reset_tgt_data();
  mp->reset_ego_data();
  pt->motor_enable();

  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = p_set.str_map[StraightType::Search].v_max;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 45;
  if (offset_dist != 45) {
    p.dist = offset_dist;
  }
  p.motion_type = MotionType::PIVOT_AFTER;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  res = mp->go_straight(p);
  // ui->coin(100);

  return res;
}
MotionResult SearchController::finish(param_set_t &p_set) {
  // param_straight_t p;
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = 10;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 45;
  return mp->go_straight(p);
}
bool SearchController::is_goaled() { return tmp_goal_list.size() == 0; }

SearchResult SearchController::exec(param_set_t &p_set, SearchMode sm) {

  mp->reset_gyro_ref_with_check();
  if (param->search_log_enable > 0)
    lt->start_slalom_log();
  reset();

  for (const auto p : lgc->goal_list) {
    lgc->map[p.x + p.y * lgc->maze_size] =
        (lgc->map[p.x + p.y * lgc->maze_size] & 0x0f);
  }

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
  saved = false;
  while (1) {
    mr = MotionResult::NONE;
    // sensing(ego);
    judge_wall();
    //この探索中ゴールしたか
    tmp_goal_list.erase(ego->x + ego->y * lgc->maze_size);
    // 片道モードでゴールしたらbreak
    if (sm == SearchMode::Kata) {
      if (adachi->goal_step) {
        break;
      }
    }
    //往復モードでゴールしたらbreak
    if (sm == SearchMode::Return) {
      if (adachi->goal_step) {
        if (ego->x == 0 && ego->y == 0) {
          break;
        } else {
          adachi->subgoal_list.clear();
        }
      }
    }
    // 足立法で行き先決定
    auto next_motion = adachi->exec();
    // go_straight_wrapper(p_set);
    if (next_motion == Motion::Straight) {
      mr = go_straight_wrapper(p_set);
    } else if (next_motion == Motion::TurnRight) {
      if (sensing_result->ego.front_dist < param->front_dist_offset_pivot_th) {
        mr = pivot90(p_set, TurnDirection::Right);
      } else {
        mr = slalom(p_set, TurnDirection::Right);
      }
    } else if (next_motion == Motion::TurnLeft) {
      if (sensing_result->ego.front_dist < param->front_dist_offset_pivot_th) {
        mr = pivot90(p_set, TurnDirection::Left);
      } else {
        mr = slalom(p_set, TurnDirection::Left);
      }
    } else if (next_motion == Motion::Back) {
      mr = pivot(p_set);
    } else if (next_motion == Motion::NONE) {
      break;
    }
    if (sm == SearchMode::ALL) {
      if (adachi->goal_step)
        if (adachi->subgoal_list.size() == 0)
          break;
    }

    if (mr == MotionResult::ERROR) {
      break;
    }

    if (next_motion == Motion::Back) {
      back_cnt++;
    } else {
      back_cnt = 0;
    }

    if (back_cnt >= 3) {
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
  if (mr == MotionResult::ERROR) {
    return SearchResult::FAIL;
  }
  return SearchResult::SUCCESS;
}
void SearchController::judge_wall() {
  wall_n = false;
  wall_e = false;
  wall_w = false;
  wall_s = false;

  if (ego->dir == Direction::North) {
    wall_n =
        sensing_result->ego.left90_dist < param->sen_ref_p.search_exist.front &&
        sensing_result->ego.right90_dist < param->sen_ref_p.search_exist.front;
    wall_e = sensing_result->ego.right45_dist <
             param->sen_ref_p.search_exist.right45;
    wall_w =
        sensing_result->ego.left45_dist < param->sen_ref_p.search_exist.left45;
  } else if (ego->dir == Direction::East) {
    wall_e =
        sensing_result->ego.left90_dist < param->sen_ref_p.search_exist.front &&
        sensing_result->ego.right90_dist < param->sen_ref_p.search_exist.front;
    wall_s = sensing_result->ego.right45_dist <
             param->sen_ref_p.search_exist.right45;
    wall_n =
        sensing_result->ego.left45_dist < param->sen_ref_p.search_exist.left45;
  } else if (ego->dir == Direction::West) {
    wall_w =
        sensing_result->ego.left90_dist < param->sen_ref_p.search_exist.front &&
        sensing_result->ego.right90_dist < param->sen_ref_p.search_exist.front;
    wall_n = sensing_result->ego.right45_dist <
             param->sen_ref_p.search_exist.right45;
    wall_s =
        sensing_result->ego.left45_dist < param->sen_ref_p.search_exist.left45;
  } else if (ego->dir == Direction::South) {
    wall_s =
        sensing_result->ego.left90_dist < param->sen_ref_p.search_exist.front &&
        sensing_result->ego.right90_dist < param->sen_ref_p.search_exist.front;
    wall_w = sensing_result->ego.right45_dist <
             param->sen_ref_p.search_exist.right45;
    wall_e =
        sensing_result->ego.left45_dist < param->sen_ref_p.search_exist.left45;
  }
  lgc->set_wall_data(ego->x, ego->y, Direction::North, wall_n);
  lgc->set_wall_data(ego->x, ego->y + 1, Direction::South, wall_n);

  lgc->set_wall_data(ego->x, ego->y, Direction::East, wall_e);
  lgc->set_wall_data(ego->x + 1, ego->y, Direction::West, wall_e);

  lgc->set_wall_data(ego->x, ego->y, Direction::West, wall_w);
  lgc->set_wall_data(ego->x - 1, ego->y, Direction::East, wall_w);

  lgc->set_wall_data(ego->x, ego->y, Direction::South, wall_s);
  lgc->set_wall_data(ego->x, ego->y - 1, Direction::North, wall_s);

  lgc->set_wall_data(0, 0, Direction::East, true);
  lgc->set_wall_data(1, 0, Direction::West, true);
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
        if (lgc->maze_size == 16)
          printf("---+");
        else
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
      if (lgc->maze_size == 16)
        printf("%2x ", lgc->dist[i + j * lgc->maze_size]);
      else
        printf("%3x", lgc->dist[i + j * lgc->maze_size]);
    }
    printf("|\r\n");
  }
  printf("+");

  for (int i = 0; i < lgc->maze_size; i++) {
    if (lgc->existWall(i, 0, Direction::South)) {
      if (lgc->maze_size == 16)
        printf("---+");
      else
        printf("----+");
    } else {
      if (lgc->maze_size == 16)
        printf("    +");
      else
        printf("     +");
    }
  }
  printf("\r\n");
}
void SearchController::save_maze_data() {
  auto *f = fopen(maze_log_kata_file.c_str(), "wb");
  if (f == NULL)
    return;
  for (const auto d : lgc->map) {
    fprintf(f, "%d,", d);
  }
  fclose(f);
}