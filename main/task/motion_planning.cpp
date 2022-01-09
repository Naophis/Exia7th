#include "include/motion_planning.hpp"

void MotionPlanning::set_ego_entity(std::shared_ptr<ego_entity_t> &_ego) {
  ego = _ego;
}
void MotionPlanning::set_tgt_entity(std::shared_ptr<tgt_entity_t> &_tgt) {
  tgt = _tgt;
}
void MotionPlanning::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}

MotionResult MotionPlanning::go_straight(param_straight_t &p) {
  tgt_val->tgt_in.v_max = p.v_max;
  tgt_val->tgt_in.end_v = p.v_end;
  tgt_val->tgt_in.accl = p.accl;
  tgt_val->tgt_in.decel = p.decel;
  tgt_val->tgt_in.tgt_dist = p.dist;

  tgt_val->tgt_in.w_max = 0;
  tgt_val->tgt_in.end_w = 0;
  tgt_val->tgt_in.alpha = 0;

  tgt_val->ego_in.state = 0;
  tgt_val->ego_in.img_ang = 0;
  tgt_val->ego_in.img_dist = 0;
  tgt_val->ego_in.ang = 0;
  tgt_val->ego_in.dist = 0;
  tgt_val->ego_in.state = 0;
  
  tgt_val->motion_mode = (int32_t)RUN_MODE2::ST_RUN;
  tgt_val->motion_type = MotionType::STRAIGHT;

  if (p.motion_type != MotionType::NONE) {
    tgt_val->motion_type = p.motion_type;
  }

  while (1) {
    if (tgt_val->ego_in.dist > p.dist) {
      break;
    }
    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return MotionResult::ERROR;
    }
    vTaskDelay(1 / portTICK_RATE_MS);
  }
  return MotionResult::NONE;
}
MotionResult MotionPlanning::pivot_turn(param_roll_t &p) {
  if (p.RorL == TurnDirection::Left) {
    tgt_val->tgt_in.w_max = p.w_max;
    tgt_val->tgt_in.end_w = p.w_end;
    tgt_val->tgt_in.alpha = p.alpha;
    tgt_val->tgt_in.tgt_angle = p.ang;
  } else {
    tgt_val->tgt_in.w_max = -p.w_max;
    tgt_val->tgt_in.end_w = -p.w_end;
    tgt_val->tgt_in.alpha = -p.alpha;
    tgt_val->tgt_in.tgt_angle = p.ang;
  }

  tgt_val->motion_mode = (int32_t)RUN_MODE2::PIVOT_TURN;
  tgt_val->motion_type = MotionType::PIVOT;

  tgt_val->tgt_in.v_max = 0;
  tgt_val->tgt_in.end_v = 0;
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
    if (ABS(tgt_val->ego_in.ang) > ABS(p.ang)) {
      break;
    }
    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return MotionResult::ERROR;
    }
    vTaskDelay(1 / portTICK_RATE_MS);
  }
  return MotionResult::NONE;
}

MotionResult MotionPlanning::slalom(slalom_param2_t &sp, TurnDirection td,
                                    next_motionr_t &next_motion) {

  param_straight_t ps_front;
  ps_front.v_max = sp.v;
  ps_front.v_end = sp.v;
  ps_front.accl = next_motion.accl;
  ps_front.decel = next_motion.decel;
  ps_front.dist = (td == TurnDirection::Right) ? sp.front.right : sp.front.left;
  ps_front.motion_type = MotionType::SLA_FRONT_STR;
  auto res_f = go_straight(ps_front);
  if (res_f != MotionResult::NONE) {
    return MotionResult::ERROR;
  }

  tgt_val->tgt_in.accl = next_motion.accl;
  tgt_val->tgt_in.alpha = 0;
  tgt_val->tgt_in.decel = next_motion.decel;
  tgt_val->tgt_in.end_v = sp.v;
  tgt_val->tgt_in.end_w = 0;
  tgt_val->tgt_in.tgt_angle = 0;
  tgt_val->tgt_in.tgt_dist = 180 * 1000;
  tgt_val->tgt_in.v_max = sp.v;
  tgt_val->tgt_in.w_max = 0;

  float alphaTemp = ((td == TurnDirection::Right) ? -1 : 1) * (sp.v / sp.rad);

  tgt_val->ego_in.sla_param.base_alpha = alphaTemp;
  tgt_val->ego_in.sla_param.base_time = sp.time;
  tgt_val->ego_in.sla_param.counter = 1;
  tgt_val->ego_in.sla_param.limit_time_count = sp.time * 2 / dt;
  tgt_val->ego_in.sla_param.pow_n = sp.pow_n;
  tgt_val->ego_in.sla_param.state = 0;

  tgt_val->ego_in.state = 0;
  tgt_val->ego_in.img_ang = 0;
  tgt_val->ego_in.img_dist = 0;

  tgt_val->motion_mode = (int32_t)RUN_MODE2::SLAROM_RUN;
  ps_front.motion_type = MotionType::SLALOM;

  while (1) {
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

  param_straight_t ps_back;
  ps_back.v_max = next_motion.v_max;
  ps_back.v_end = next_motion.v_end;
  ps_back.accl = next_motion.accl;
  ps_back.decel = next_motion.decel;
  ps_back.dist = (td == TurnDirection::Right) ? sp.back.right : sp.back.left;
  ps_back.motion_type = MotionType::SLA_BACK_STR;
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