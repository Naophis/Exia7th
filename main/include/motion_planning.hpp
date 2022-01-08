#ifndef MotionPlanning_HPP
#define MotionPlanning_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/defines.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class MotionPlanning {
public:
  MotionPlanning() {}
  virtual ~MotionPlanning() {}

  void set_ego_entity(std::shared_ptr<ego_entity_t> &_ego);
  void set_tgt_entity(std::shared_ptr<tgt_entity_t> &_tgt);
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);

  int go_straight(param_straight_t &p);
  int pivot_turn(param_roll_t &p);
  void normal_slalom(param_normal_slalom_t &p, param_straight_t &p_str);
  void n_slalom(param_normal_slalom_t &p, param_straight_t &p_str);

  int slalom(slalom_param2_t &sp, TurnDirection dir,
             next_motionr_t &next_motion);

private:
  std::shared_ptr<ego_entity_t> ego;
  std::shared_ptr<tgt_entity_t> tgt;
  std::shared_ptr<motion_tgt_val_t> tgt_val;
};
#endif