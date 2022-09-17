#ifndef PLANNING_TASK_HPP
#define PLANNING_TASK_HPP

#include "defines.hpp"
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gen_code_mpc/mpc_tgt_calc.h"
#include "include/logging_task.hpp"
#include "include/maze_solver.hpp"

#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
// #include "gen_code_pid/pid_controller.h"
// #include "gen_code_pid_2dof/pid_controller_2dof.h"
#include "gen_code_simple_pid/simple_pid_controller.h"

#include <cmath>

class PlanningTask {
public:
  PlanningTask();
  virtual ~PlanningTask();
  void create_task(const BaseType_t xCoreID);
  void motor_enable();
  void suction_enable(float duty);
  void motor_disable();
  void motor_disable(bool reset_req);
  void suction_disable();

  void
  set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_sensing_result);
  void set_input_param_entity(std::shared_ptr<input_param_t> &_param_ro);
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);
  void set_logging_task(std::shared_ptr<LoggingTask> &_lt);

  void buzzer(ledc_channel_config_t &buzzer_ch,
              ledc_timer_config_t &buzzer_timer);
  static void task_entry_point(void *task_instance);
  virtual void task();

  void active_logging(FILE *_f);
  void inactive_logging();
  void dump_log();

  unsigned long long global_msec_timer = 0;

  bool motor_en = false;
  bool suction_en = false;

private:
  xTaskHandle handle = 0;

  void check_fail_safe();
  void update_ego_motion();
  void set_next_duty(float duty_l, float duty_r, float duty_suction);
  void init_gpio();
  void calc_tgt_duty();

  void cp_request();

  void calc_sensor_dist_diff();

  void calc_sensor_dist_all();

  float satuate_sen_duty(float duty_sen);

  float calc_sensor(float date, float a, float b);

  void calc_filter();

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
  // char line[1024];
  // FILE *f;

  std::shared_ptr<sensing_result_entity_t> sensing_result;
  std::shared_ptr<input_param_t> param_ro;
  std::shared_ptr<motion_tgt_val_t> tgt_val;
  std::shared_ptr<LoggingTask> lt;

  t_tgt *mpc_tgt;
  t_ego *mpc_now_ego;
  int32_t mpc_mode;
  int32_t mpc_step;
  t_ego mpc_next_ego;
  t_ego mpc_next_ego2;

  fail_safe_t fail_safe;
  float get_feadforward_front();
  float get_feadforward_front(TurnDirection td);
  float get_feadforward_roll();
  float get_rpm_ff_val(TurnDirection td);
  float calc_sensor_pid();
  float calc_sensor_pid_dia();
  float check_sen_error();
  float check_sen_error_dia();
  float error_right = 0;
  float error_left = 0;
  bool check_right = false;
  bool check_left = false;

  ledc_channel_config_t buzzer_ch;
  ledc_timer_config_t buzzer_timer;
  mcpwm_config_t motor_pwm_conf;
  mcpwm_config_t suction_pwm_conf;
  float duty_c = 0;
  float duty_c2 = 0;
  float duty_roll = 0;
  float duty_roll2 = 0;
  sen_log2_t sen_log;
  slip_t slip_param;
  std::deque<float> enc_v_q;
  std::deque<float> accl_x_q;
  float sum_v = 0;

  // PID_Controller vel_pid;
  // PID_Controller dist_pid;
  // PID_Controller sen_pid;
  // PID_Controller sen_dia_pid;
  // PID_Controller gyro_pid;
  // PID_Controller angle_pid;

  Simple_PID_Controller vel_pid;
  Simple_PID_Controller gyro_pid;

  // Simple_PID_Controller dist_pid;
  // Simple_PID_Controller sen_pid;
  // Simple_PID_Controller sen_dia_pid;
  // Simple_PID_Controller gyro_pid;
  // Simple_PID_Controller angle_pid;

  // PID_Controller_2dof vel_pid_2dof;
  // PID_Controller_2dof gyro_pid_2dof;
  unsigned char w_reset = 0;
};

#endif