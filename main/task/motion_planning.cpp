#include "include/motion_planning.hpp"

void MotionPlanning::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}

void MotionPlanning::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param) {
  param = _param;
}

void MotionPlanning::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_entity) {
  sensing_result = _entity;
}

void MotionPlanning::set_userinterface(std::shared_ptr<UserInterface> &_ui) {
  ui = _ui;
}

void MotionPlanning::set_path_creator(std::shared_ptr<PathCreator> &_pc) {
  pc = _pc;
}
void MotionPlanning::set_planning_task(std::shared_ptr<PlanningTask> &_pt) {
  pt = _pt;
}
void MotionPlanning::set_logging_task(std::shared_ptr<LoggingTask> &_lt) {
  lt = _lt; //
}
MotionResult MotionPlanning::go_straight(param_straight_t &p) {
  tgt_val->nmr.v_max = p.v_max;
  tgt_val->nmr.v_end = p.v_end;
  tgt_val->nmr.accl = p.accl;
  tgt_val->nmr.decel = p.decel;
  tgt_val->nmr.dist = p.dist;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.motion_mode = RUN_MODE2::ST_RUN;
  tgt_val->nmr.motion_type = MotionType::STRAIGHT;
  tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  tgt_val->nmr.dia_mode = p.dia_mode;
  tgt_val->nmr.sct = p.sct;
  tgt_val->ego_in.dist = 0;

  if (p.motion_type != MotionType::NONE) {
    tgt_val->nmr.motion_type = p.motion_type;
  }
  tgt_val->nmr.timstamp++;
  bool wall_off_req = p.wall_off_req == WallOffReq::SEARCH;
  bool exist_r = false;
  bool exist_l = false;
  if (wall_off_req) {
    exist_r =
        sensing_result->ego.right90_lp > param->sen_ref_p.search_exist.right90;
    exist_l =
        sensing_result->ego.left90_lp > param->sen_ref_p.search_exist.left90;
  }

  vTaskDelay(1 / portTICK_RATE_MS);
  while (1) {
    if (exist_r) {
      if (sensing_result->ego.right90_lp <
          param->sen_ref_p.search_exist.kireme_r) {
        tgt_val->nmr.dist = p.wall_off_dist_r;
        p.dist = p.wall_off_dist_r;
        tgt_val->nmr.timstamp++;
        exist_l = exist_r = false;
      }
    }
    if (exist_l) {
      if (sensing_result->ego.left90_lp <
          param->sen_ref_p.search_exist.kireme_l) {
        tgt_val->nmr.dist = p.wall_off_dist_l;
        p.dist = p.wall_off_dist_l;
        tgt_val->nmr.timstamp++;
        exist_l = exist_r = false;
      }
    }
    vTaskDelay(1 / portTICK_RATE_MS);

    if (tgt_val->ego_in.dist > p.dist) {
      break;
    }
    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return MotionResult::ERROR;
    }
  }
  return MotionResult::NONE;
}

MotionResult MotionPlanning::pivot_turn(param_roll_t &p) {
  // 一度初期化
  tgt_val->motion_type = MotionType::NONE;
  tgt_val->nmr.timstamp++;
  vTaskDelay(1 / portTICK_RATE_MS);

  tgt_val->nmr.v_max = 0;
  tgt_val->nmr.v_end = 0;
  tgt_val->nmr.accl = 0;
  tgt_val->nmr.decel = 0;
  tgt_val->nmr.dist = 0;
  if (p.RorL == TurnDirection::Left) {
    tgt_val->nmr.w_max = p.w_max;
    tgt_val->nmr.w_end = p.w_end;
    tgt_val->nmr.alpha = p.alpha;
    tgt_val->nmr.ang = p.ang;
    tgt_val->nmr.motion_dir = MotionDirection::LEFT;
  } else {
    tgt_val->nmr.w_max = -p.w_max;
    tgt_val->nmr.w_end = -p.w_end;
    tgt_val->nmr.alpha = -p.alpha;
    tgt_val->nmr.ang = p.ang;
    tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  }
  tgt_val->nmr.motion_mode = RUN_MODE2::PIVOT_TURN;
  tgt_val->nmr.motion_type = MotionType::PIVOT;
  tgt_val->nmr.sct = SensorCtrlType::NONE;
  tgt_val->nmr.timstamp++;

  while (1) {
    vTaskDelay(1 / portTICK_RATE_MS);
    if (ABS(tgt_val->ego_in.ang) > ABS(p.ang)) {
      break;
    }
    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return MotionResult::ERROR;
    }
  }
  return MotionResult::NONE;
}

MotionResult MotionPlanning::slalom(slalom_param2_t &sp, TurnDirection td,
                                    next_motionr_t &next_motion) {
  return slalom(sp, td, next_motion, false);
}

MotionResult MotionPlanning::slalom(slalom_param2_t &sp, TurnDirection td,
                                    next_motionr_t &next_motion, bool dia) {

  ps_front.v_max = sp.v;
  ps_front.v_end = sp.v;
  ps_front.accl = next_motion.accl;
  ps_front.decel = next_motion.decel;
  ps_front.dist = (td == TurnDirection::Right) ? sp.front.right : sp.front.left;
  ps_front.motion_type = MotionType::SLA_FRONT_STR;
  ps_front.sct = !dia ? SensorCtrlType::Straight : SensorCtrlType::Dia;
  ps_front.wall_off_req = WallOffReq::NONE;

  ps_back.dist = (td == TurnDirection::Right) ? sp.back.right : sp.back.left;

  if (sp.type == TurnType::Normal) {
    search_front_ctrl(ps_front); // 前壁制御
    if (sensing_result->ego.front_dist < 87) {
      ps_front.dist += (87 - sensing_result->ego.front_dist);
    }
    auto res_f = go_straight(ps_front);
    if (res_f != MotionResult::NONE) {
      return MotionResult::ERROR;
    }

    if (td == TurnDirection::Right) {
      if (sensing_result->ego.left45_dist < th_offset_dist) {
        ps_back.dist += (45 - sensing_result->ego.left45_dist);
      }
    } else {
      if (sensing_result->ego.right45_dist < th_offset_dist) {
        ps_back.dist += (45 - sensing_result->ego.right45_dist);
      }
    }

  } else {
    auto res_f = go_straight(ps_front);
    if (res_f != MotionResult::NONE) {
      return MotionResult::ERROR;
    }
  }

  float alphaTemp = ((td == TurnDirection::Right) ? -1 : 1) * (sp.v / sp.rad);

  tgt_val->nmr.motion_dir = (td == TurnDirection::Left)
                                ? MotionDirection::LEFT
                                : MotionDirection::RIGHT;

  tgt_val->nmr.v_max = sp.v;
  tgt_val->nmr.v_end = sp.v;
  tgt_val->nmr.accl = next_motion.accl;
  tgt_val->nmr.decel = next_motion.decel;
  tgt_val->nmr.dist = 180 * 1000;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.sla_alpha = alphaTemp;
  tgt_val->nmr.sla_time = sp.time;
  tgt_val->nmr.sla_pow_n = sp.pow_n;
  tgt_val->nmr.motion_mode = RUN_MODE2::SLAROM_RUN;
  tgt_val->nmr.motion_type = MotionType::SLALOM;
  tgt_val->nmr.sct = SensorCtrlType::NONE;
  tgt_val->nmr.timstamp++;
  tgt_val->ego_in.sla_param.counter = 1;

  while (1) {
    vTaskDelay(1 / portTICK_RATE_MS);
    if (tgt_val->ego_in.sla_param.counter >=
        tgt_val->ego_in.sla_param.limit_time_count) {
      tgt_val->ego_in.w = 0;
      break;
    }
    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return MotionResult::ERROR;
    }
    // if (type != Dia90) {
    //   if (!fail) {
    //     alphaMode = 0;
    //     alphaTemp = 0;
    //     slaTerm = 0;
    //     omegaTemp = 0;
    //     return 0;
    //   }
    // }
  }
  ps_back.v_max = next_motion.v_max;
  ps_back.v_end = next_motion.v_end;
  ps_back.accl = next_motion.accl;
  ps_back.decel = next_motion.decel;
  ps_back.motion_type = MotionType::SLA_BACK_STR;
  ps_back.sct = !dia ? SensorCtrlType::Straight : SensorCtrlType::Dia;
  ps_back.wall_off_req = WallOffReq::NONE;
  //  : SensorCtrlType::Dia;
  auto res_b = go_straight(ps_back);
  if (res_b != MotionResult::NONE) {
    return MotionResult::ERROR;
  }
  return MotionResult::NONE;
}
void MotionPlanning::normal_slalom(param_normal_slalom_t &p,
                                   param_straight_t &p_str) {
  float alpha = (2 * p.v_max * p.v_max / (p.radius * p.radius * p.ang / 2));

  if (p.RorL == TurnDirection::Left) {
    tgt_val->tgt_in.w_max = 200000;
    tgt_val->tgt_in.end_w = 0;
    tgt_val->tgt_in.alpha = alpha;
    tgt_val->tgt_in.tgt_angle = p.ang;
  } else {
    tgt_val->tgt_in.w_max = -200000;
    tgt_val->tgt_in.end_w = -0;
    tgt_val->tgt_in.alpha = -alpha;
    tgt_val->tgt_in.tgt_angle = p.ang;
  }

  tgt_val->motion_mode = (int32_t)RUN_MODE2::SLALOM_RUN2;

  tgt_val->tgt_in.v_max = p.v_max;
  tgt_val->tgt_in.end_v = p.v_max;
  tgt_val->tgt_in.accl = 0;
  tgt_val->tgt_in.decel = 0;
  tgt_val->tgt_in.tgt_dist = 0;

  tgt_val->ego_in.state = 0;
  tgt_val->ego_in.pivot_state = 0;

  tgt_val->ego_in.img_ang = 0;
  tgt_val->ego_in.img_dist = 0;
  tgt_val->ego_in.ang = 0;
  tgt_val->ego_in.dist = 0;

  while (1) {
    vTaskDelay(1 / portTICK_RATE_MS);
    if (tgt_val->ego_in.pivot_state == 3) {
      tgt_val->ego_in.w = 0;
      break;
    }

    if (tgt_val->ego_in.img_dist >= p.ang * p.radius) {
      tgt_val->ego_in.w = 0;
      break;
    }
    if (ABS(tgt_val->ego_in.img_ang) + 0.001 >= ABS(p.ang)) {
      tgt_val->ego_in.w = 0;
      break;
    }
    // if (type != Dia90) {
    //   if (!fail) {
    //     alphaMode = 0;
    //     alphaTemp = 0;
    //     slaTerm = 0;
    //     omegaTemp = 0;
    //     return 0;
    //   }
    // }
  }
}

void MotionPlanning::reset_tgt_data() {
  tgt_val->tgt_in.v_max = 0;
  tgt_val->tgt_in.end_v = 0;
  tgt_val->tgt_in.accl = 0;
  tgt_val->tgt_in.decel = 0;
  tgt_val->tgt_in.w_max = 0;
  tgt_val->tgt_in.end_w = 0;
  tgt_val->tgt_in.alpha = 0;
  tgt_val->tgt_in.tgt_dist = 0;
  tgt_val->tgt_in.tgt_angle = 0;

  tgt_val->motion_mode = 0;

  tgt_val->tgt_in.accl_param.limit = 2500;
  tgt_val->tgt_in.accl_param.n = 4;
  tgt_val->global_pos.ang = 0;
  tgt_val->global_pos.img_ang = 0;
  tgt_val->global_pos.dist = 0;
  tgt_val->global_pos.img_dist = 0;
}

void MotionPlanning::reset_ego_data() {
  tgt_val->ego_in.accl = 0;
  tgt_val->ego_in.alpha = 0;
  tgt_val->ego_in.ang = 0;
  tgt_val->ego_in.dist = 0;
  tgt_val->ego_in.pivot_state = 0;
  tgt_val->ego_in.sla_param.base_alpha = 0;
  tgt_val->ego_in.sla_param.base_time = 0;
  tgt_val->ego_in.sla_param.counter = 0;
  tgt_val->ego_in.sla_param.limit_time_count = 0;
  tgt_val->ego_in.sla_param.pow_n = 0;
  tgt_val->ego_in.sla_param.state = 0;
  tgt_val->ego_in.state = 0;
  tgt_val->ego_in.img_ang = 0;
  tgt_val->ego_in.img_dist = 0;

  tgt_val->ego_in.v = 0;
  tgt_val->ego_in.w = 0;

  tgt_val->motion_mode = 0;

  tgt_val->nmr.motion_mode = RUN_MODE2::NONE_MODE;
  tgt_val->nmr.motion_type = MotionType::NONE;
  tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  // 一度初期化
  tgt_val->motion_type = MotionType::NONE;
  tgt_val->nmr.timstamp++;
  vTaskDelay(1 / portTICK_RATE_MS);
}

void MotionPlanning::reset_gyro_ref() {
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  float gyro_raw_data_sum = 0;

  for (int i = 0; i < RESET_GYRO_LOOP_CNT; i++) {
    gyro_raw_data_sum += sensing_result->gyro.raw;
    vTaskDelay(xDelay); //他モジュールの起動待ち
  }
  tgt_val->gyro_zero_p_offset = gyro_raw_data_sum / RESET_GYRO_LOOP_CNT;
}
void MotionPlanning::reset_gyro_ref_with_check() {
  ui->motion_check();
  reset_gyro_ref();
}

void MotionPlanning::coin() { ui->coin(120); }

void MotionPlanning::keep() {
  tgt_val->nmr.v_max = 0;
  tgt_val->nmr.v_end = 0;
  tgt_val->nmr.accl = 0;
  tgt_val->nmr.decel = 0;
  tgt_val->nmr.dist = 0;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.sla_alpha = 0;
  tgt_val->nmr.sla_time = 0;
  tgt_val->nmr.sla_pow_n = 0;
  tgt_val->nmr.motion_mode = RUN_MODE2::KEEP;
  tgt_val->nmr.motion_type = MotionType::NONE;
  tgt_val->nmr.timstamp++;
  tgt_val->nmr.dia_mode = false;
  tgt_val->ego_in.sla_param.counter = 1;

  while (1) {
    vTaskDelay(1 / portTICK_RATE_MS);
    if (ui->button_state_hold()) {
      break;
    }
  }
}
void MotionPlanning::exec_path_running(param_set_t &p_set) {
  ego_odom_t ego;
  ego.x = ego.y = ego.ang = 0;
  ego.dir = Direction::North;
  bool dia = false;

  // default straight parma
  param_straight_t ps;
  ps.v_max = p_set.str_map[StraightType::FastRun].v_max;
  ps.v_end = p_set.str_map[StraightType::FastRun].v_max;
  ps.accl = p_set.str_map[StraightType::FastRun].accl;
  ps.decel = p_set.str_map[StraightType::FastRun].decel;
  ps.motion_type = MotionType::STRAIGHT;
  next_motionr_t nm;

  reset_gyro_ref_with_check();
  // lt->start_slalom_log();
  // reset();
  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  for (int i = 0; i < pc->path_size; i++) {
    float dist = 0.5 * pc->path_s[i] - 1;
    auto turn_dir = tc.get_turn_dir(pc->path_t[i]);
    auto turn_type = tc.get_turn_type(pc->path_t[i]);

    if (dist > 0 || i == 0) {
      auto st = !dia ? StraightType::FastRun : StraightType::FastRunDia;
      ps.v_max = p_set.str_map[st].v_max;
      ps.v_end = p_set.map[turn_type].v;
      ps.accl = p_set.str_map[st].accl;
      ps.decel = p_set.str_map[st].decel;
      ps.dia_mode = dia;

      ps.dist = !dia ? (dist * cell_size) : (dist * cell_size * ROOT2);
      if (i == 0) {
        if (dist == 0) {
          // 初手ターンの場合は距離合成して加速区間を増やす
          dist = (turn_dir == TurnDirection::Left)
                     ? p_set.map[turn_type].front.left
                     : p_set.map[turn_type].front.right;
        }
        ps.dist += param->offset_start_dist; // 初期加速距離を加算
      }
      if (turn_type == TurnType::Finish) {
        ps.dist += 40;
        ps.v_end = 20;
      }
      ps.motion_type = MotionType::STRAIGHT;
      go_straight(ps);
      if (turn_type == TurnType::Finish) {
        break;
      }
    }

    if (!((turn_type == TurnType::None) || (turn_type == TurnType::Finish))) {
      // TODO wall_off
      auto st = !dia ? StraightType::FastRun : StraightType::FastRunDia;
      bool exist_next_idx = (i + 1) < pc->path_size; //絶対true
      float dist3 = 0;
      float dist4 = 0;
      if (exist_next_idx) {
        dist3 = 0.5 * pc->path_s[i + 1] * cell_size;
        dist4 = 0.5 * pc->path_s[i + 1] - 1;
      }
      // スラロームの後距離の目標速度を指定
      nm.v_max = p_set.map[turn_type].v;
      nm.v_end = p_set.map[turn_type].v;
      nm.accl = p_set.str_map[st].accl;
      nm.decel = p_set.str_map[st].decel;
      nm.is_turn = false;

      if (exist_next_idx && !(dist3 > 0 && dist4 > 0)) {
        //連続スラロームのとき、次のスラロームの速度になるように加速
        auto next_turn_type = tc.get_turn_type(pc->path_t[i + 1]);
        nm.v_max = p_set.map[next_turn_type].v;
        nm.v_end = p_set.map[next_turn_type].v;
        nm.is_turn = true;
      }

      slalom(p_set.map[turn_type], turn_dir, nm);
      ego.dir = tc.get_next_dir(ego.dir, turn_type, turn_dir);
      // ego.ang = trj_ele.ang;
      dia =
          (ego.dir == Direction::NorthEast || ego.dir == Direction::NorthWest ||
           ego.dir == Direction::SouthEast || ego.dir == Direction::SouthWest);
    }
  }

  ps.v_max = 20;
  ps.v_end = 1;
  ps.dist = 5;
  ps.accl = p_set.str_map[StraightType::FastRun].accl;
  ps.decel = p_set.str_map[StraightType::FastRun].decel;
  go_straight(ps);

  pt->motor_disable();
  // lt->stop_slalom_log();
  coin();
}

MotionResult MotionPlanning::search_front_ctrl(param_straight_t &p) {
  tgt_val->nmr.v_max = p.v_max;
  tgt_val->nmr.v_end = p.v_max;
  tgt_val->nmr.accl = p.accl;
  tgt_val->nmr.decel = p.decel;
  tgt_val->nmr.dist = 200; // max
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.motion_mode = RUN_MODE2::ST_RUN;
  tgt_val->nmr.motion_type = MotionType::FRONT_CTRL;
  tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  tgt_val->nmr.dia_mode = p.dia_mode;
  tgt_val->nmr.sct = p.sct;

  if (p.motion_type != MotionType::NONE) {
    tgt_val->nmr.motion_type = p.motion_type;
  }
  tgt_val->nmr.timstamp++;
  bool wall_off_req = sensing_result->ego.front_lp >
                          param->sen_ref_p.search_exist.front_ctrl_th ||
                      (70 < sensing_result->ego.front_dist &&
                       sensing_result->ego.front_dist < 110);

  if (!wall_off_req) {
    return MotionResult::NONE;
  }

  while (1) {
    if (sensing_result->ego.front_lp >
        param->sen_ref_p.search_exist.front_ctrl) {
      break;
    }
    if (sensing_result->ego.front_dist < 87) {
      break;
    }
    vTaskDelay(1 / portTICK_RATE_MS);

    if (tgt_val->ego_in.dist > p.dist) {
      break;
    }
  }
  return MotionResult::NONE;
}