#ifndef MotionPlanning_HPP
#define MotionPlanning_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/defines.hpp"
#include "include/ui.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class MotionPlanning {
public:
  MotionPlanning() {}
  virtual ~MotionPlanning() {}

  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);

  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
  void set_input_param_entity(std::shared_ptr<input_param_t> &_param);

  void set_userinterface(std::shared_ptr<UserInterface> &_ui);

  MotionResult go_straight(param_straight_t &p);
  MotionResult pivot_turn(param_roll_t &p);
  void normal_slalom(param_normal_slalom_t &p, param_straight_t &p_str);
  void n_slalom(param_normal_slalom_t &p, param_straight_t &p_str);

  MotionResult slalom(slalom_param2_t &sp, TurnDirection dir,
                      next_motionr_t &next_motion);
  void reset_tgt_data();
  void reset_ego_data();
  void reset_gyro_ref();
  void reset_gyro_ref_with_check();
  void coin();
  void keep();

private:
  std::shared_ptr<UserInterface> ui;

  std::shared_ptr<motion_tgt_val_t> tgt_val;

  std::shared_ptr<sensing_result_entity_t> sensing_result;
  std::shared_ptr<input_param_t> param;
};
#endif