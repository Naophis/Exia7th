#ifndef PLANNING_TASK_HPP
#define PLANNING_TASK_HPP

#include "defines.hpp"
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gen_code/mpc_tgt_calc.h"
#include "include/maze_solver.hpp"

#define LEDC_HIGH_SPEED_MODE 0
#define LEDC_TIMER_10_BIT 10
#define LEDC_TIMER_0 0
#define LEDC_CHANNEL_0 0
#define BATTERY_BUZZER_MAX_CNT 25
#define LOW_BATTERY_TH 3.95
class PlanningTask {
public:
  PlanningTask();
  virtual ~PlanningTask();
  void create_task(const BaseType_t xCoreID);
  void motor_enable();
  void suction_enable(double duty);
  void motor_disable();
  void suction_disable();

  void set_sensing_entity(sensing_result_entity_t *_entity);
  void set_ego_entity(ego_entity_t *_ego);
  void set_ego_param_entity(ego_param_t *_param);
  void set_tgt_entity(tgt_entity_t *_tgt);
  void set_tgt_val(motion_tgt_val_t *_tgt) { tgt_val = _tgt; }
  void buzzer(ledc_channel_config_t &buzzer_ch,
              ledc_timer_config_t &buzzer_timer);
  static void task_entry_point(void *task_instance);
  virtual void task();

  // read only
  sensing_result_entity_t *entity_ro;
  ego_param_t *param_ro;
  tgt_entity_t *tgt;

  // read_write
  ego_entity_t *ego;

  t_tgt *mpc_tgt;
  t_ego *mpc_now_ego;
  int32_t mpc_mode;
  int32_t mpc_step;
  t_ego mpc_next_ego;

private:
  xTaskHandle handle = 0;
  bool motor_en;
  bool suction_en;

  void update_ego_motion();
  void set_next_duty(double duty_l, double duty_r, double duty_suction);
  void init_gpio();
  void calc_tgt_duty();
  void calc_next_tgt_val();
  duty_t tgt_duty;
  pid_error_entity_t error_entity;
  motion_tgt_val_t *tgt_val;
  int buzzer_time_cnt = 0;
  int buzzer_timestamp = 0;
  int pid_req_timestamp = 0;
  void reset_error();
  void cp_tgt_val();
  mpc_tgt_calcModelClass mpc_tgt_calc;
};

#endif