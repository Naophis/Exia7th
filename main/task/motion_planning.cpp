#include "include/motion_planning.hpp"

void MotionPlanning::set_tgt_entity(tgt_entity_t *_tgt) { //
  tgt = _tgt;
}
void MotionPlanning::set_tgt_val(motion_tgt_val_t *_tgt) { //
  tgt_val = _tgt;
}
void MotionPlanning::set_ego_entity(ego_entity_t *_ego) { //
  ego = _ego;
}
int MotionPlanning::go_straight(param_straight_t &p) {
  tgt_val->tgt_in.v_max = p.v_max;
  tgt_val->tgt_in.end_v = p.v_end;
  tgt_val->tgt_in.accl = p.accl;
  tgt_val->tgt_in.decel = p.decel;
  tgt_val->tgt_in.tgt_dist = p.dist;

  tgt_val->motion_mode = (int32_t)RUN_MODE2::ST_RUN;

  tgt_val->tgt_in.w_max = 0;
  tgt_val->tgt_in.end_w = 0;
  tgt_val->tgt_in.alpha = 0;

  tgt_val->ego_in.state = 0;
  tgt_val->ego_in.img_ang = 0;
  tgt_val->ego_in.img_dist = 0;
  tgt_val->ego_in.ang = 0;
  tgt_val->ego_in.dist = 0;
  tgt_val->ego_in.state = 0;

  ego->dist = tgt_val->ego_in.dist;
  ego->angle = tgt_val->ego_in.ang;

  while (1) {
    if (tgt_val->ego_in.dist > p.dist) {
      break;
    }
    vTaskDelay(1 / portTICK_RATE_MS);
  }
  return 0;
}
int MotionPlanning::pivot_turn(param_roll_t &p) {
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

  ego->dist = tgt_val->ego_in.dist;
  ego->angle = tgt_val->ego_in.ang;

  while (1) {
    if (ABS(tgt_val->ego_in.ang) > ABS(p.ang)) {
      break;
    }
    vTaskDelay(1 / portTICK_RATE_MS);
  }
  return 0;
}

void MotionPlanning::normal_slalom(param_normal_slalom_t &p,
                                   param_straight_t &p_str) {
  double alpha = (2 * p.v_max * p.v_max / (p.radius * p.radius * p.ang / 2));

  if (p.RorL == TurnDirection::Left) {
    tgt_val->tgt_in.w_max = 10000;
    tgt_val->tgt_in.end_w = 0;
    tgt_val->tgt_in.alpha = alpha;
    tgt_val->tgt_in.tgt_angle = p.ang;
  } else {
    tgt_val->tgt_in.w_max = -10000;
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

  ego->dist = tgt_val->ego_in.dist;
  ego->angle = tgt_val->ego_in.ang;
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