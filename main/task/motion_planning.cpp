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
MotionResult MotionPlanning::go_straight(param_straight_t &p,
                                         std::shared_ptr<Adachi> &adachi) {
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
  tgt_val->ego_in.dist -= tgt_val->ego_in.img_dist;
  tgt_val->ego_in.img_dist -= tgt_val->ego_in.img_dist;
  if (p.motion_type != MotionType::NONE) {
    tgt_val->nmr.motion_type = p.motion_type;
  }
  tgt_val->nmr.timstamp++;

  if (adachi != nullptr) {
    adachi->update();
  }

  vTaskDelay(1 / portTICK_RATE_MS);
  while (1) {
    vTaskDelay(1 / portTICK_RATE_MS);
    if (ABS(tgt_val->ego_in.dist) >= ABS(p.dist)) {
      break;
    }
    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return MotionResult::ERROR;
    }
  }
  return MotionResult::NONE;
}

MotionResult MotionPlanning::go_straight(param_straight_t &p) {
  return go_straight(p, fake_adachi);
}

MotionResult MotionPlanning::pivot_turn(param_roll_t &p) {
  // 一度初期化
  pt->motor_enable();
  reset_tgt_data();
  reset_ego_data();
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
  tgt_val->ego_in.dist -= tgt_val->ego_in.img_dist;
  tgt_val->ego_in.img_dist = 0;
  vTaskDelay(10 / portTICK_RATE_MS);

  while (1) {
    vTaskDelay(1 / portTICK_RATE_MS);
    if (ABS(tgt_val->ego_in.ang) >= ABS(p.ang) &&
        ABS(tgt_val->ego_in.ang * 180 / PI) > 10) {
      break;
    }
    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return MotionResult::ERROR;
    }
  }
  return MotionResult::NONE;
}
void MotionPlanning::req_error_reset() {
  tgt_val->pl_req.error_vel_reset = 1;
  tgt_val->pl_req.error_gyro_reset = 1;
  tgt_val->pl_req.error_ang_reset = 1;
  tgt_val->pl_req.error_dist_reset = 1;
  tgt_val->pl_req.time_stamp++;
}
MotionResult MotionPlanning::slalom(slalom_param2_t &sp, TurnDirection td,
                                    next_motionr_t &next_motion) {
  return slalom(sp, td, next_motion, false);
}

MotionResult MotionPlanning::slalom(slalom_param2_t &sp, TurnDirection td,
                                    next_motionr_t &next_motion, bool dia) {
  return slalom(sp, td, next_motion, dia, fake_adachi);
}
MotionResult MotionPlanning::slalom(slalom_param2_t &sp, TurnDirection td,
                                    next_motionr_t &next_motion, bool dia,
                                    std::shared_ptr<Adachi> &adachi) {
  bool find = false;
  bool find_r = false;
  bool find_l = false;
  ps_front.v_max = sp.v;
  ps_front.v_end = sp.v;
  ps_front.accl = next_motion.accl;
  ps_front.decel = next_motion.decel;
  ps_front.dist = (td == TurnDirection::Right) ? sp.front.right : sp.front.left;
  ps_front.motion_type = MotionType::SLA_FRONT_STR;
  ps_front.sct = SensorCtrlType::Straight;
  if (sp.type == TurnType::Dia45_2 || sp.type == TurnType::Dia135_2 ||
      sp.type == TurnType::Dia90) {
    ps_front.sct = SensorCtrlType::NONE;
  }
  // ps_front.sct = SensorCtrlType::NONE;
  ps_front.wall_off_req = WallOffReq::NONE;

  ps_back.dist = (td == TurnDirection::Right) ? sp.back.right : sp.back.left;

  if (sp.type == TurnType::Normal) {
    // search_front_ctrl(ps_front); // 前壁制御
    if (sensing_result->ego.left90_dist < 130 &&
        sensing_result->ego.right90_dist < 130) {
      ps_front.dist +=
          (sensing_result->ego.front_dist - param->front_dist_offset);
    }
    if (td == TurnDirection::Right) {
      if (sensing_result->ego.left45_dist < th_offset_dist) {
        ps_back.dist +=
            (param->sla_wall_ref_l - sensing_result->ego.left45_dist);
      }
    } else {
      if (sensing_result->ego.right45_dist < th_offset_dist) {
        ps_back.dist +=
            (param->sla_wall_ref_r - sensing_result->ego.right45_dist);
      }
    }
    res_f = go_straight(ps_front);
    if (res_f != MotionResult::NONE) {
      return MotionResult::ERROR;
    }
  } else if (sp.type == TurnType::Large) {
    bool b = true;
    if (sensing_result->ego.left90_dist < 150 &&
        sensing_result->ego.right90_dist < 150) {
      ps_front.dist -=
          (param->front_dist_offset2 - sensing_result->ego.front_dist);
      b = false;
    }
    float dist_r = ps_back.dist;
    float dist_l = ps_back.dist;
    if (td == TurnDirection::Right) {
      if (sensing_result->ego.right45_dist < th_offset_dist) {
        dist_r -= param->sla_wall_ref_r - sensing_result->ego.right45_dist;
        find_r = true;
      }
      if (sensing_result->ego.left45_dist < th_offset_dist) {
        dist_l += param->sla_wall_ref_l - sensing_result->ego.left45_dist;
        find_l = true;
      }
    } else {
      if (sensing_result->ego.left45_dist < th_offset_dist) {
        dist_l -= param->sla_wall_ref_l - sensing_result->ego.left45_dist;
        find_l = true;
      }
      if (sensing_result->ego.right45_dist < th_offset_dist) {
        dist_r += param->sla_wall_ref_r - sensing_result->ego.right45_dist;
        find_r = true;
      }
    }
    if (find_r && find_l) {
      ps_back.dist = (ABS(ps_back.dist - dist_r) < ABS(ps_back.dist - dist_l))
                         ? dist_r
                         : dist_l;
    } else if (find_r) {
      ps_back.dist = dist_r;
    } else if (find_l) {
      ps_back.dist = dist_l;
    }
    if (b) {
      wall_off(td, ps_front);
    }
    res_f = go_straight(ps_front);
    if (res_f != MotionResult::NONE) {
      return MotionResult::ERROR;
    }
  } else if (sp.type == TurnType::Orval) {
    bool b = true;
    if (sensing_result->ego.left90_dist < 150 &&
        sensing_result->ego.right90_dist < 150) {
      ps_front.dist +=
          (sensing_result->ego.front_dist - param->front_dist_offset2);
      b = false;
    }
    float rad_r = sp.rad;
    float rad_l = sp.rad;
    if (td == TurnDirection::Right) {
      if (sensing_result->ego.right45_dist < th_offset_dist) {
        float rad2 =
            (sensing_result->ego.right45_dist - param->sla_wall_ref_r) / 2;
        rad_r += rad2;
        find_r = true;
      }
      if (sensing_result->ego.left45_dist < th_offset_dist) {
        float rad2 =
            (param->sla_wall_ref_l - sensing_result->ego.left45_dist) / 2;
        rad_l += rad2;
        find_l = true;
      }
    } else {
      if (sensing_result->ego.left45_dist < th_offset_dist) {
        float rad2 =
            (sensing_result->ego.left45_dist - param->sla_wall_ref_l) / 2;
        rad_l += rad2;
        find_l = true;
      }
      if (!find) {
        if (sensing_result->ego.right45_dist < th_offset_dist) {
          float rad2 =
              (param->sla_wall_ref_r - sensing_result->ego.right45_dist) / 2;
          rad_r += rad2;
          find_r = true;
        }
      }
    }
    if (find_r && find_l) {
      sp.rad = (ABS(sp.rad - rad_r) < ABS(sp.rad - rad_l)) ? rad_r : rad_l;
    } else if (find_r) {
      sp.rad = rad_r;
    } else if (find_l) {
      sp.rad = rad_l;
    }
    if (b) {
      wall_off(td, ps_front);
    }
    if (ps_front.dist > 0) {
      res_f = go_straight(ps_front);
      if (res_f != MotionResult::NONE) {
        return MotionResult::ERROR;
      }
    }
  } else if (sp.type == TurnType::Dia45 || sp.type == TurnType::Dia135) {
    bool b = true;
    if (sensing_result->ego.left90_dist < 150 &&
        sensing_result->ego.right90_dist < 150) {
      ps_front.dist -=
          (param->front_dist_offset2 - sensing_result->ego.front_dist);
      b = false;
    }
    if (b) {
      wall_off(td, ps_front);
    }
    res_f = go_straight(ps_front);
    ps_back.dist -= (td == TurnDirection::Right) ? param->offset_after_turn_r
                                                 : param->offset_after_turn_l;
    if (res_f != MotionResult::NONE) {
      return MotionResult::ERROR;
    }
  } else if (sp.type == TurnType::Dia45_2 || sp.type == TurnType::Dia135_2) {
    // TODO front wall offset
    wall_off_dia(td, ps_front);
    res_f = go_straight(ps_front);
    ps_back.dist -= (td == TurnDirection::Right)
                        ? param->offset_after_turn_dia_r
                        : param->offset_after_turn_dia_l;
    if (res_f != MotionResult::NONE) {
      return MotionResult::ERROR;
    }
  } else if (sp.type == TurnType::Dia90) {
    // TODO front wall offset
    wall_off_dia(td, ps_front);
    res_f = go_straight(ps_front);
    ps_back.dist -= (td == TurnDirection::Right)
                        ? param->offset_after_turn_dia_r
                        : param->offset_after_turn_dia_l;
    if (res_f != MotionResult::NONE) {
      return MotionResult::ERROR;
    }
  } else {
    res_f = go_straight(ps_front);
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

  tgt_val->nmr.sla_alpha = alphaTemp;
  tgt_val->nmr.sla_time = sp.time;
  tgt_val->nmr.sla_pow_n = sp.pow_n;

  tgt_val->nmr.motion_mode = RUN_MODE2::SLAROM_RUN;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;

  tgt_val->nmr.motion_type = MotionType::SLALOM;
  tgt_val->nmr.sct = SensorCtrlType::NONE;

  if (sp.type == TurnType::Orval) {
    tgt_val->nmr.motion_mode = RUN_MODE2::SLALOM_RUN2;
    tgt_val->nmr.ang = sp.ang;
    tgt_val->nmr.sla_rad = sp.rad;
    if (td == TurnDirection::Right) {
      tgt_val->nmr.w_max = -200000;
      tgt_val->nmr.w_end = 0;
      tgt_val->nmr.alpha = -(2 * sp.v * sp.v / (sp.rad * sp.rad * sp.ang / 2));
    } else {
      tgt_val->nmr.w_max = 200000;
      tgt_val->nmr.w_end = 0;
      tgt_val->nmr.alpha = (2 * sp.v * sp.v / (sp.rad * sp.rad * sp.ang / 2));
    }
  }
  tgt_val->nmr.timstamp++;
  tgt_val->ego_in.sla_param.counter = 1;
  tgt_val->ego_in.sla_param.limit_time_count = (int)(sp.time * 2 / dt);
  tgt_val->ego_in.dist -= tgt_val->ego_in.img_dist;
  tgt_val->ego_in.img_dist = 0;
  if (adachi != nullptr) {
    adachi->update();
  }
  while (1) {
    vTaskDelay(1 / portTICK_RATE_MS);

    if (sp.type == TurnType::Orval) {
      if (tgt_val->ego_in.pivot_state == 3) {
        tgt_val->ego_in.w = 0;
        break;
      }
      if (ABS(tgt_val->ego_in.img_ang) + 0.001 >= ABS(sp.ang)) {
        tgt_val->ego_in.w = 0;
        break;
      }
    } else {

      if (tgt_val->ego_in.sla_param.counter >= (sp.time * 2 / dt)) {
        tgt_val->ego_in.w = 0;
        break;
      }
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
  // ps_back.dist = (td == TurnDirection::Right) ? sp.back.right :
  // sp.back.left;

  // ps_back.v_max = next_motion.v_max;
  // ps_back.v_end = next_motion.v_end;
  ps_back.v_max = sp.v;
  ps_back.v_end = sp.v;
  ps_back.accl = next_motion.accl;
  ps_back.decel = next_motion.decel;
  ps_back.motion_type = MotionType::SLA_BACK_STR;
  ps_back.sct = SensorCtrlType::Straight;

  if (sp.type == TurnType::Dia45 || sp.type == TurnType::Dia135 ||
      sp.type == TurnType::Dia90) {
    ps_back.sct = SensorCtrlType::NONE;
  }
  // ps_back.sct = SensorCtrlType::NONE;
  ps_back.wall_off_req = WallOffReq::NONE;
  //  : SensorCtrlType::Dia;
  MotionResult res_b = MotionResult::NONE;
  if (ps_back.dist > 0) {
    res_b = go_straight(ps_back);
    if (res_b != MotionResult::NONE) {
      return MotionResult::ERROR;
    }
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
  tgt_val->ego_in.ang = 0;
  tgt_val->ego_in.dist -= tgt_val->ego_in.img_dist;
  tgt_val->ego_in.img_dist = 0;

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

MotionResult MotionPlanning::front_ctrl(bool limit) {
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
  tgt_val->nmr.motion_type = MotionType::FRONT_CTRL;
  tgt_val->nmr.timstamp++;
  tgt_val->nmr.dia_mode = false;
  tgt_val->ego_in.sla_param.counter = 1;

  unsigned int cnt = 0;
  unsigned int max_cnt = 0;
  while (1) {
    vTaskDelay(1 / portTICK_RATE_MS);
    if (ui->button_state_hold()) {
      break;
    }
    if (ABS(sensing_result->ego.front_dist -
            param->sen_ref_p.search_exist.front_ctrl) <
            param->sen_ref_p.search_exist.kireme_l &&
        ABS((sensing_result->ego.right90_dist -
             sensing_result->ego.left90_dist) /
                2 -
            param->sen_ref_p.search_exist.kireme_r) <
            param->sen_ref_p.search_exist.offset_r) {
      cnt++;
    } else {
      cnt = 0;
    }
    max_cnt++;
    if (!limit) {
      if (cnt > param->sen_ref_p.search_exist.front_ctrl_th)
        break;
      if (max_cnt > 500)
        break;
    }
  }
  return MotionResult::NONE;
}

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
  tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  tgt_val->nmr.dia_mode = false;
  tgt_val->nmr.sct = SensorCtrlType::NONE;
  tgt_val->nmr.timstamp++;

  while (1) {
    vTaskDelay(1 / portTICK_RATE_MS);
    if (ui->button_state_hold()) {
      break;
    }
  }
}
void MotionPlanning::exec_path_running(param_set_t &p_set) {
  ego.x = ego.y = ego.ang = 0;
  ego.dir = Direction::North;
  dia = false;

  // default straight parma
  ps.v_max = p_set.str_map[StraightType::FastRun].v_max;
  ps.v_end = p_set.str_map[StraightType::FastRun].v_max;
  ps.accl = p_set.str_map[StraightType::FastRun].accl;
  ps.decel = p_set.str_map[StraightType::FastRun].decel;
  ps.motion_type = MotionType::STRAIGHT;
  ps.sct = SensorCtrlType::Straight;

  reset_gyro_ref_with_check();
  if (param->fast_log_enable > 0)
    lt->start_slalom_log();
  // reset();
  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();
  auto size = pc->path_s.size();
  for (int i = 0; i < size; i++) {
    float dist = 0.5 * pc->path_s[i] - 1;
    auto turn_dir = tc.get_turn_dir(pc->path_t[i]);
    auto turn_type = tc.get_turn_type(pc->path_t[i], dia);

    if (dist > 0 || i == 0) {
      auto st = !dia ? StraightType::FastRun : StraightType::FastRunDia;
      ps.v_max = p_set.str_map[st].v_max;
      ps.v_end = p_set.map[turn_type].v;
      ps.accl = p_set.str_map[st].accl;
      ps.decel = p_set.str_map[st].decel;
      ps.dia_mode = dia;

      ps.dist = !dia ? (dist * cell_size) : (dist * cell_size * ROOT2);
      if (i == 0) {
        if (dist == 0) { // 初手ターンの場合は距離合成して加速区間を増やす
          dist = (turn_dir == TurnDirection::Left)
                     ? p_set.map[turn_type].front.left
                     : p_set.map[turn_type].front.right;
        }
        ps.dist += param->offset_start_dist; // 初期加速距離を加算
        auto tmp_v2 = 2 * ps.accl * ps.dist;
        if (ps.v_end * ps.v_end > tmp_v2) {
          ps.accl = (ps.v_end * ps.v_end) / (2 * ps.dist) + 100;
          ps.decel = -ps.accl;
        }
      }
      if (turn_type == TurnType::Finish) {
        ps.dist += 40;
        ps.v_end = 500;
      }
      ps.motion_type = MotionType::STRAIGHT;
      ps.sct = !dia ? SensorCtrlType::Straight : SensorCtrlType::Dia;
      go_straight(ps);
      if (turn_type == TurnType::Finish) {
        break;
      }
    }

    if (!((turn_type == TurnType::None) || (turn_type == TurnType::Finish))) {
      auto st = !dia ? StraightType::FastRun : StraightType::FastRunDia;
      bool exist_next_idx = (i + 1) < pc->path_size; //絶対true
      float dist3 = 0;
      float dist4 = 0;
      if (exist_next_idx) {
        dist3 = 0.5 * pc->path_s[i + 1] * cell_size;
        dist4 = 0.5 * pc->path_s[i + 1] - 1;
      }
      // スラロームの後距離の目標速度を指定

      //　TODO
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

      slalom(p_set.map[turn_type], turn_dir, nm, dia);
      ego.dir = tc.get_next_dir(ego.dir, turn_type, turn_dir);
      // ego.ang = trj_ele.ang;
      dia =
          (ego.dir == Direction::NorthEast || ego.dir == Direction::NorthWest ||
           ego.dir == Direction::SouthEast || ego.dir == Direction::SouthWest);
    }
  }
  float dist = sensing_result->ego.front_dist - 45;
  ps.v_max = 500;
  ps.v_end = 20;
  ps.dist = dist;
  ps.accl = p_set.str_map[StraightType::FastRun].accl;
  ps.decel = p_set.str_map[StraightType::FastRun].decel;
  ps.sct = !dia ? SensorCtrlType::Straight : SensorCtrlType::Dia;
  if (dist > 0)
    go_straight(ps);
  reset_tgt_data();
  reset_ego_data();
  vTaskDelay(250 / portTICK_RATE_MS);
  pt->motor_disable(false);

  reset_tgt_data();
  reset_ego_data();
  vTaskDelay(25 / portTICK_RATE_MS);

  // pt->motor_enable();
  // front_ctrl(false);
  // reset_tgt_data();
  // reset_ego_data();
  // vTaskDelay(25 / portTICK_RATE_MS);
  // pt->motor_disable(false);

  lt->stop_slalom_log();
  lt->save(slalom_log_file);
  coin();
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
}

MotionResult MotionPlanning::search_front_ctrl(param_straight_t &p) {
  return MotionResult::NONE;
}

MotionResult MotionPlanning::wall_off(param_straight_t &p, bool dia) {
  return MotionResult::NONE;
}

void MotionPlanning::wall_off(TurnDirection td, param_straight_t &ps_front) {
  tgt_val->nmr.v_max = ps_front.v_max;
  tgt_val->nmr.v_end = ps_front.v_end;
  tgt_val->nmr.accl = ps_front.accl;
  tgt_val->nmr.decel = ps_front.decel;
  tgt_val->nmr.dist = 180;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.motion_mode = RUN_MODE2::ST_RUN;
  tgt_val->motion_type = MotionType::WALL_OFF;
  tgt_val->nmr.motion_type = MotionType::WALL_OFF;
  tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  tgt_val->nmr.dia_mode = ps_front.dia_mode;
  tgt_val->nmr.sct = SensorCtrlType::Straight;
  tgt_val->ego_in.dist -= tgt_val->ego_in.img_dist;
  tgt_val->ego_in.img_dist = 0;
  tgt_val->nmr.timstamp++;
  if (td == TurnDirection::Right) {
    while (true) {
      if (sensing_result->ego.right45_dist <
          param->wall_off_dist.exist_dist_r) {
        break;
      }
      vTaskDelay(1 / portTICK_RATE_MS);
    }
    while (true) {
      if (sensing_result->ego.right45_dist >
          param->wall_off_dist.noexist_th_r) {
        ps_front.dist += param->wall_off_dist.right_str;
        return;
      }
      vTaskDelay(1 / portTICK_RATE_MS);
    }
  } else {
    while (true) {
      if (sensing_result->ego.left45_dist < param->wall_off_dist.exist_dist_l) {
        break;
      }
      vTaskDelay(1 / portTICK_RATE_MS);
    }
    while (true) {
      if (sensing_result->ego.left45_dist > param->wall_off_dist.noexist_th_l) {
        ps_front.dist += param->wall_off_dist.left_str;
        return;
      }
      vTaskDelay(1 / portTICK_RATE_MS);
    }
  }
}

void MotionPlanning::wall_off_dia(TurnDirection td,
                                  param_straight_t &ps_front) {
  tgt_val->nmr.v_max = ps_front.v_max;
  tgt_val->nmr.v_end = ps_front.v_end;
  tgt_val->nmr.accl = ps_front.accl;
  tgt_val->nmr.decel = ps_front.decel;
  tgt_val->nmr.dist = 180;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.motion_mode = RUN_MODE2::ST_RUN;
  tgt_val->motion_type = MotionType::WALL_OFF_DIA;
  tgt_val->nmr.motion_type = MotionType::WALL_OFF_DIA;
  tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  tgt_val->nmr.dia_mode = ps_front.dia_mode;
  tgt_val->nmr.sct = SensorCtrlType::Dia;
  tgt_val->ego_in.dist -= tgt_val->ego_in.img_dist;
  tgt_val->ego_in.img_dist = 0;
  tgt_val->nmr.timstamp++;
  if (td == TurnDirection::Right) {
    while (true) {
      if (sensing_result->ego.right45_dist <
          param->wall_off_dist.exist_dia_th_r) {
        break;
      }
      vTaskDelay(1 / portTICK_RATE_MS);
    }
    while (true) {
      if (sensing_result->ego.right45_dist >
          param->wall_off_dist.noexist_dia_th_r) {
        ps_front.dist += param->wall_off_dist.right_dia;
        return;
      }
      vTaskDelay(1 / portTICK_RATE_MS);
    }
  } else {
    while (true) {
      if (sensing_result->ego.left45_dist <
          param->wall_off_dist.exist_dia_th_l) {
        break;
      }
      vTaskDelay(1 / portTICK_RATE_MS);
    }
    while (true) {
      if (sensing_result->ego.left45_dist >
          param->wall_off_dist.noexist_dia_th_l) {
        ps_front.dist += param->wall_off_dist.left_dia;
        return;
      }
      vTaskDelay(1 / portTICK_RATE_MS);
    }
  }
}