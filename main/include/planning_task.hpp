#ifndef PLANNING_TASK_HPP
#define PLANNING_TASK_HPP

#include "defines.hpp"
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gen_code/mpc_tgt_calc.h"
#include "include/maze_solver.hpp"

#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

class PlanningTask {
public:
  PlanningTask();
  virtual ~PlanningTask();
  void create_task(const BaseType_t xCoreID);
  void motor_enable();
  void suction_enable(float duty);
  void motor_disable();
  void suction_disable();

  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity_ro);
  void set_ego_param_entity(std::shared_ptr<ego_param_t> &_param_ro);
  void set_ego_entity(std::shared_ptr<ego_entity_t> &_ego);
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);

  void buzzer(ledc_channel_config_t &buzzer_ch,
              ledc_timer_config_t &buzzer_timer);
  static void task_entry_point(void *task_instance);
  virtual void task();

  void active_logging(FILE *_f);
  void inactive_logging();
  void dump_log();

private:
  xTaskHandle handle = 0;
  bool motor_en;
  bool suction_en;

  void check_fail_safe();
  void update_ego_motion();
  void set_next_duty(float duty_l, float duty_r, float duty_suction);
  void init_gpio();
  void calc_tgt_duty();

  void cp_request();

  duty_t tgt_duty;
  pid_error_entity_t error_entity;
  int buzzer_time_cnt = 0;
  int buzzer_timestamp = 0;
  int motion_req_timestamp = 0;
  int pid_req_timestamp = 0;
  void pl_req_activate();
  void cp_tgt_val();
  mpc_tgt_calcModelClass mpc_tgt_calc;

  bool log_active = false;
  // log_t log_list2[10];
  int log_list2_size = 0;
  char line[1024];
  FILE *f;

  std::shared_ptr<sensing_result_entity_t> entity_ro;
  std::shared_ptr<ego_param_t> param_ro;
  std::shared_ptr<ego_entity_t> ego;
  std::shared_ptr<motion_tgt_val_t> tgt_val;

  t_tgt *mpc_tgt;
  t_ego *mpc_now_ego;
  int32_t mpc_mode;
  int32_t mpc_step;
  t_ego mpc_next_ego;

  fail_safe_t fail_safe;
};

#endif