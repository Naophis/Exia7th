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
MotionResult SearchController::go_straight_wrapper(param_set_t &p_set,
                                                   float diff) {
  // param_straight_t p;
  p.v_max = p.v_end = p_set.str_map[StraightType::Search].v_max;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  if (adachi->goal_step) {
    p.dist = param->cell2 - diff;
  } else {
    p.dist = param->cell - diff;
  }
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
                                      const TurnDirection td, float diff) {
  slalom_param2_t sp = p_set.map[TurnType::Normal];

  next_motion_t nm;
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
  vTaskDelay(25 / portTICK_RATE_MS);
  pt->motor_disable();
  mp->reset_tgt_data();
  mp->reset_ego_data();
}
MotionResult SearchController::pivot(param_set_t &p_set, float diff) {
  // param_straight_t p;
  MotionResult res;
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = 20;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 43 - diff;
  bool back_enable = false;
  if (10 < sensing_result->ego.left90_dist &&
      sensing_result->ego.left90_dist < 130 &&
      10 < sensing_result->ego.right90_dist &&
      sensing_result->ego.right90_dist < 130) {
    p.dist += (sensing_result->ego.front_dist - param->front_dist_offset);
    back_enable = true;
  }
  p.motion_type = MotionType::PIVOT_PRE;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;
  if (p.dist < 10) {
    p.dist = 10;
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

  bool flag = false;
  float tmp_dist = 0;
  pr.w_max = p_set.str_map[StraightType::Search].w_max;
  pr.alpha = p_set.str_map[StraightType::Search].alpha;
  pr.w_end = p_set.str_map[StraightType::Search].w_end;

  bool left_exist =
      sensing_result->ego.left45_dist < param->sen_ref_p.search_exist.left90;
  bool right_exist =
      sensing_result->ego.right45_dist < param->sen_ref_p.search_exist.right90;

  flag = (left_exist || right_exist);
  pr.ang = (flag) ? PI / 2 : PI;
  bool front_ctrl2 = false;
  if (!left_exist && !right_exist) {
    pr.RorL = TurnDirection::Right;
  } else if (left_exist) {
    pr.RorL = TurnDirection::Left;
    front_ctrl2 = true;
  } else if (right_exist) {
    pr.RorL = TurnDirection::Right;
    front_ctrl2 = true;
  }

  if (back_enable) {
    tmp_dist = 20;
  }
  bool front_ctrl = (sensing_result->ego.front_dist < 60);
  pt->motor_disable();
  mp->reset_tgt_data();
  mp->reset_ego_data();
  vTaskDelay(25 / portTICK_RATE_MS);

  if (adachi->goal_step && !saved) {
    vTaskDelay(25 / portTICK_RATE_MS);
    save_maze_data();
    vTaskDelay(25 / portTICK_RATE_MS);
    saved = true;
  }

  mp->reset_tgt_data();
  mp->reset_ego_data();

  if (front_ctrl) {
    front_wall_ctrl();
  }

  vTaskDelay(25 / portTICK_RATE_MS);
  mp->reset_tgt_data();
  mp->reset_ego_data();
  // mp->req_error_reset();
  pt->motor_enable();

  if (flag) {
    p.motion_type = MotionType::PIVOT_OFFSET;
    p.sct = SensorCtrlType::NONE;
    pr.ang = PI / 2;

    res = mp->pivot_turn(pr);
    vTaskDelay(25 / portTICK_RATE_MS);
    front_ctrl = (sensing_result->ego.front_dist < 60);
    pt->motor_disable();
    vTaskDelay(25 / portTICK_RATE_MS);

    if (front_ctrl2) {
      front_wall_ctrl();
    }
    vTaskDelay(25 / portTICK_RATE_MS);

    // mp->reset_tgt_data();
    pt->motor_enable();
    mp->reset_tgt_data();
    mp->reset_ego_data();
    p.dist = 0.1;
    p.motion_type = MotionType::PIVOT_PRE2;
    p.sct = SensorCtrlType::NONE;
    res = mp->go_straight(p);
    pt->motor_disable();
    mp->reset_tgt_data();
    mp->reset_ego_data();
    vTaskDelay(25 / portTICK_RATE_MS);
    mp->reset_tgt_data();
    mp->reset_ego_data();
    pt->motor_enable();
    mp->reset_tgt_data();
    mp->reset_ego_data();
    res = mp->pivot_turn(pr);
    pt->motor_disable();
    mp->reset_tgt_data();
    mp->reset_ego_data();
    vTaskDelay(25 / portTICK_RATE_MS);
  } else {
    pr.ang = PI;
    res = mp->pivot_turn(pr);
    pt->motor_disable();
    mp->reset_tgt_data();
    mp->reset_ego_data();
    vTaskDelay(25 / portTICK_RATE_MS);
  }

  // if (res == MotionResult::ERROR)
  //   return MotionResult::ERROR;

  if (back_enable) {
    vTaskDelay(25 / portTICK_RATE_MS);
    pt->motor_enable();
    mp->reset_tgt_data();
    mp->reset_ego_data();
    p.v_max = -p_set.str_map[StraightType::Search].v_max;
    p.v_end = -10; // p_set.str_map[StraightType::Search].v_max;
    p.accl = -p_set.str_map[StraightType::Search].accl;
    p.decel = -p_set.str_map[StraightType::Search].decel;
    p.dist = -tmp_dist;
    p.motion_type = MotionType::BACK_STRAIGHT;
    p.sct = SensorCtrlType::NONE;
    p.wall_off_req = WallOffReq::NONE;
    res = mp->go_straight(p);
    mp->reset_tgt_data();
    mp->reset_ego_data();
    vTaskDelay(25 / portTICK_RATE_MS);
    pt->motor_disable();
  }

  // mp->reset_gyro_ref();
  // ui->coin(100);

  adachi->update();

  vTaskDelay(25 / portTICK_RATE_MS);
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
  pt->motor_enable();
  res = mp->go_straight(p);
  // ui->coin(100);

  return res;
}

MotionResult SearchController::pivot90(param_set_t &p_set,
                                       const TurnDirection td, float diff) {
  // param_straight_t p;
  MotionResult res;
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = 20;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 40 - diff;
  if (10 < sensing_result->ego.left90_dist &&
      sensing_result->ego.left90_dist < 130 &&
      10 < sensing_result->ego.right90_dist &&
      sensing_result->ego.right90_dist < 130) {
    p.dist += (sensing_result->ego.front_dist - param->front_dist_offset_pivot);
  }
  p.motion_type = MotionType::PIVOT_PRE;
  p.sct = SensorCtrlType::Straight;
  p.wall_off_req = WallOffReq::NONE;

  float offset_dist = 45;

  // if (td == TurnDirection::Right) {
  //   if (sensing_result->ego.left45_dist < 58) {
  //     offset_dist += (45 - sensing_result->ego.left45_dist);
  //   }
  // } else {
  //   if (sensing_result->ego.right45_dist < 58) {
  //     offset_dist += (45 - sensing_result->ego.right45_dist);
  //   }
  // }
  if (p.dist < 10) {
    p.dist = 10;
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

  bool front_ctrl = (sensing_result->ego.front_dist < 60);
  if (front_ctrl) {
    front_wall_ctrl();
    vTaskDelay(25 / portTICK_RATE_MS);
  }

  float tmp_dist = 0;
  pr.w_max = p_set.str_map[StraightType::Search].w_max;
  pr.alpha = p_set.str_map[StraightType::Search].alpha;
  pr.w_end = p_set.str_map[StraightType::Search].w_end;
  pr.ang = PI / 2;
  pr.RorL = td;

  pt->motor_disable();
  mp->reset_tgt_data();
  mp->reset_ego_data();
  vTaskDelay(25 / portTICK_RATE_MS);

  adachi->update();

  mp->reset_tgt_data();
  mp->reset_ego_data();
  pt->motor_enable();
  res = mp->pivot_turn(pr);
  pt->motor_disable();
  mp->reset_tgt_data();
  mp->reset_ego_data();
  vTaskDelay(25 / portTICK_RATE_MS);

  mp->reset_tgt_data();
  mp->reset_ego_data();

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
  pt->motor_enable();
  res = mp->go_straight(p);

  return res;
}
MotionResult SearchController::finish(param_set_t &p_set) {
  // param_straight_t p;
  p.v_max = p_set.str_map[StraightType::Search].v_max;
  p.v_end = 10;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 40;
  mp->go_straight(p);

  p.v_max = 20;
  p.v_end = 5;
  p.accl = p_set.str_map[StraightType::Search].accl;
  p.decel = p_set.str_map[StraightType::Search].decel;
  p.dist = 2;
  return mp->go_straight(p);
}
bool SearchController::is_goaled() { return tmp_goal_list.size() == 0; }

SearchResult SearchController::exec(param_set_t &p_set, SearchMode sm) {
  unsigned long long start_time = pt->global_msec_timer;
  unsigned long long end_time = pt->global_msec_timer;
  adachi->lgc->reset_dist_map();
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
  bool first = false;
  // pivot(p_set);
  saved = false;
  int go_home_fix_cnt = 0;
  adachi->sm = sm;
  while (1) {
    mr = MotionResult::NONE;
    // sensing(ego);
    bool is_stepped = lgc->is_stepped(ego->x, ego->y);
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
    float before = mp->tgt_val->global_pos.dist;
    before = mp->tgt_val->ego_in.dist;
    auto next_motion = adachi->exec(is_stepped);
    float after = mp->tgt_val->global_pos.dist;
    after = mp->tgt_val->ego_in.dist;
    // float diff = after - before;
    adachi->diff = ABS(after - before);

    // go_straight_wrapper(p_set);
    if (next_motion == Motion::Straight) {
      mr = go_straight_wrapper(p_set, adachi->diff);
    } else if (next_motion == Motion::TurnRight) {
      if (10 < sensing_result->ego.left90_dist &&
          sensing_result->ego.left90_dist < param->front_dist_offset_pivot_th &&
          10 < sensing_result->ego.right90_dist &&
          sensing_result->ego.right90_dist <
              param->front_dist_offset_pivot_th) {
        // (90 - p_set.map[TurnType::Normal].front.right)
        mr = pivot90(p_set, TurnDirection::Right, adachi->diff);
      } else {
        mr = slalom(p_set, TurnDirection::Right, adachi->diff);
      }
    } else if (next_motion == Motion::TurnLeft) {
      if (10 < sensing_result->ego.left90_dist &&
          sensing_result->ego.left90_dist < param->front_dist_offset_pivot_th &&
          10 < sensing_result->ego.right90_dist &&
          sensing_result->ego.right90_dist <
              param->front_dist_offset_pivot_th) {
        // (90 - p_set.map[TurnType::Normal].front.left)
        mr = pivot90(p_set, TurnDirection::Left, adachi->diff);
      } else {
        mr = slalom(p_set, TurnDirection::Left, adachi->diff);
      }
    } else if (next_motion == Motion::Back) {
      mr = pivot(p_set, adachi->diff);
      // break;
    } else if (next_motion == Motion::NONE) {
      break;
    } else {
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

    if (back_cnt >= 4) {
      break;
    }
    if (sm == SearchMode::ALL) {
      if (adachi->goal_step) {
        if (ego->x == 0 && ego->y == 0) {
          break;
        }
        if (adachi->subgoal_list.size() == 0 && adachi->pt_list.size() == 1 &&
            adachi->pt_list[0].x == 0 && adachi->pt_list[0].y == 0) {
          // break;
          go_home_fix_cnt++;
        } else {
          go_home_fix_cnt = 0;
        }
        if (go_home_fix_cnt == 10) {
          // break;
        }
      }
    }
    end_time = pt->global_msec_timer;
    if (ABS(end_time - start_time) / 1000 > 60 * 3) {
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
  // auto *f = fopen(maze_log_kata_file.c_str(), "wb");
  auto *f = fopen(maze_log_file.c_str(), "wb");
  if (f == NULL)
    return;
  for (const auto d : lgc->map) {
    fprintf(f, "%d,", d);
  }
  fclose(f);
}