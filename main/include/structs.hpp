#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include "gen_code/bus.h"
#include "include/defines.hpp"
#include "include/enums.hpp"
#include "include/maze_solver.hpp"

#include <initializer_list>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

union LED_bit {
  struct {
    unsigned int b0 : 1;
    unsigned int b1 : 1;
    unsigned int b2 : 1;
    unsigned int b3 : 1;
    unsigned int b4 : 1;
    unsigned int b5 : 3;
  };
  uint8_t byte;
};

typedef struct {
  int16_t right = 0;
  int16_t left = 0;
} encoder_data_t;

typedef struct {
  int raw = 0;
  float data = 0;
} sensing_data_t;

typedef struct {
  sensing_data_t right90;
  sensing_data_t right45;
  sensing_data_t front;
  sensing_data_t left45;
  sensing_data_t left90;
} led_sensor_t;

typedef struct {
  float right = 0;
  float left = 0;
} rpm_t;

typedef struct {
  float duty_l = 0;
  float duty_r = 0;
  float duty_suction = 0;
} duty_t;

typedef struct {
  float front;
  float roll;
} ff_duty_t;

typedef struct {
  float v_r = 0;
  float v_l = 0;
  float v_c = 0;
  float w_raw = 0;
  float w_lp = 0;
  float w_kalman = 0;
  float ang_kalman = 0;
  float battery_raw = 0;
  float battery_lp = 0;

  float right90_raw = 0;
  float right90_lp = 0;
  float right45_raw = 0;
  float right45_lp = 0;
  float front_raw = 0;
  float front_lp = 0;
  float left45_raw = 0;
  float left45_lp = 0;
  float left90_raw = 0;
  float left90_lp = 0;

  rpm_t rpm;
  duty_t duty;
  ff_duty_t ff_duty;
  char motion_type = 0;
} ego_entity_t;

typedef struct {
  led_sensor_t led_sen;
  led_sensor_t led_sen_after;
  led_sensor_t led_sen_before;
  sensing_data_t gyro;
  int gyro_list[4];
  sensing_data_t battery;
  encoder_data_t encoder_raw;
  encoder_data_t encoder;
  ego_entity_t ego;
} sensing_result_entity_t;

typedef struct {
  float vel = 0;
  float speed = 0;
  float accl = 0;
} xva_t;

typedef struct {
  float p = 0;
  float i = 0;
  float d = 0;
  char mode = 0;
} pid_param_t;

typedef struct {
  float gyro_w_gain_right = 0;
  float gyro_w_gain_left = 0;
  float lp_delay = 0;
} gyro_param_t;

typedef struct {
  float lp_delay = 0;
} sen_param_t;

typedef struct {
  float dt = 0.001;
  float tire = 12;
  float gear_a = 37;
  float gear_b = 8;
  float max_duty = 99;
  float Ke = 0;
  float Km = 0;
  float Resist = 0;
  float Mass = 0;
  float Lm = 0;
  float Kalman_ang = 0.0;
  float Kalman_bias = 0.003f;
  float Kalman_measure = 0.03f;
  float tread = 38;
  int FF_front = 0;
  int FF_roll = 0;
  int FF_keV = 0;
  pid_param_t motor_pid;
  pid_param_t dist_pid;
  pid_param_t gyro_pid;
  pid_param_t angle_pid;
  pid_param_t sensor_pid;
  pid_param_t sensor_pid_dia;
  gyro_param_t gyro_param;
  sen_param_t battery_param;
  sen_param_t led_param;
  MotionDirection motion_dir;
} input_param_t;

typedef struct {
  float error_p;
  float error_i;
  float error_d;
} pid_error_t;

typedef struct {
  float gain_z;
  float gain_zz;
} gain_log_t;

typedef struct {
  pid_error_t v;
  pid_error_t dist;
  pid_error_t w;
  pid_error_t ang;
  gain_log_t v_log;
  gain_log_t dist_log;
  gain_log_t w_log;
  gain_log_t ang_log;
} pid_error_entity_t;

// 指示速度
typedef struct {
  float v_max = 0;
  float accl = 0;
  float w_max = 0;
  float alpha = 0;
} motion_tgt_t;

typedef struct {
  int hz = 0;
  int time = 0;
  int timstamp = 0;
} buzzer_t;

typedef struct {
  int time_stamp = 0;
  int error_gyro_reset = 0;
  int error_vel_reset = 0;
  int error_led_reset = 0;
  int error_ang_reset = 0;
  int error_dist_reset = 0;
  // int log_start = 0;
  // int log_end = 0;
} planning_req_t;

typedef struct {
  int error;
} fail_safe_state_t;

typedef struct {
  float v_max;
  float v_end;
  float accl;
  float decel;
  float dist;
  float w_max;
  float w_end;
  float alpha;
  float ang;
  float sla_alpha;
  float sla_time;
  float sla_pow_n;
  RUN_MODE2 motion_mode;
  MotionType motion_type;

  int timstamp = 0;
  MotionDirection motion_dir;
  SensorCtrlType sct;
} new_motion_req_t;

typedef struct {
  float img_dist;
  float img_ang;
  float dist;
  float ang;
} global_ego_pos_t;

typedef struct {
  t_tgt tgt_in;
  t_ego ego_in;
  global_ego_pos_t global_pos;
  int32_t motion_mode;
  MotionType motion_type;
  MotionDirection motion_dir;
  planning_req_t pl_req;
  fail_safe_state_t fss;
  float gyro_zero_p_offset = 0;
  buzzer_t buzzer;
  new_motion_req_t nmr;
} motion_tgt_val_t;

typedef struct {
  float v_max;
  float v_end;
  float accl;
  float decel;
  float dist;
  MotionType motion_type;
  SensorCtrlType sct;
} param_straight_t;

typedef struct {
  float w_max;
  float w_end;
  float alpha;
  float ang;
  TurnDirection RorL;
} param_roll_t;

typedef struct {
  float radius;
  float v_max;
  float v_end;
  float ang;
  TurnDirection RorL;
} param_normal_slalom_t;

typedef struct {
  float v_max;
  float end_v;
  float accl;
  float decel;
  float dist;
  float w_max;
  float alpha;
  float ang;
  int suction_active;
  float suction_duty;
  float sla_dist;
  int file_idx;
  int sla_type;
  int sla_return;
  int sla_type2;
  int turn_times;
} test_mode_t;

typedef struct {
  std::vector<point_t> goals;
  int maze_size;
  int user_mode;
  test_mode_t test;
} system_t;

typedef struct {
  int normal;
  int large;
  int orval;
  int dia45;
  int dia45_2;
  int dia135;
  int dia135_2;
  int dia90;
} profile_idx_t;

typedef struct {
  std::vector<std::string> file_list;
  int file_list_size;
  int profile_idx_size;
  std::vector<profile_idx_t> profile_list;
} turn_param_profile_t;

typedef struct {
  float right;
  float left;
} slalom_offset_t;

typedef struct {
  float v;
  float ang;
  float rad;
  slalom_offset_t front;
  slalom_offset_t back;
  int pow_n;
  float time;
  TurnType type;
} slalom_param2_t;

typedef struct {
  float v_max;
  float accl;
  float decel;
  float w_max;
  float alpha;
} straight_param_t;

typedef struct {
  std::unordered_map<TurnType, slalom_param2_t> map;
  std::unordered_map<StraightType, straight_param_t> str_map;
} param_set_t;

typedef struct {
  bool is_turn;
  TurnType next_turn_type;
  float v_max;
  float v_end;
  float accl;
  float decel;
} next_motionr_t;

typedef struct {
  // int idx;
  float img_v;
  float v_l;
  float v_c;
  float v_r;
  float accl;
  float img_w;
  float w_lp;
  float alpha;

  float img_dist;
  float dist;
  float img_ang;
  float ang;

  float duty_l;
  float duty_r;

  float left90_lp;
  float left45_lp;
  float front_lp;
  float right45_lp;
  float right90_lp;
  float battery_lp;

  char motion_type;

  float duty_ff_front;
  float duty_ff_roll;

} log_data_t;

typedef struct {
  int invalid_duty_r_cnt;
  int invalid_duty_l_cnt;
} fail_safe_t;

#endif