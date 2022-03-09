#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include "gen_code_mpc/bus.h"
#include "include/defines.hpp"
#include "include/enums.hpp"
#include "include/maze_solver.hpp"

#include "gen_code_conv_single2half/half_type.h"
#include "gen_code_conv_single2half/rtwtypes.h"

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
  float sen = 0;
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

  float front_lp_old = 0;
  float left45_lp_old = 0;
  float left90_lp_old = 0;
  float right45_lp_old = 0;
  float right90_lp_old = 0;

  float front_dist = 0;
  float left45_dist = 0;
  float left90_dist = 0;
  float right45_dist = 0;
  float right90_dist = 0;

  float front_dist_old = 0;
  float left45_dist_old = 0;
  float left90_dist_old = 0;
  float right45_dist_old = 0;
  float right90_dist_old = 0;

  rpm_t rpm;
  duty_t duty;
  ff_duty_t ff_duty;
  char motion_type = 0;
} ego_entity_t;

typedef struct {
  float sensor_dist = 0;
  float global_run_dist = 0;
} sen_log_t;

typedef struct {
  // sen_log_t l90;
  sen_log_t l45;
  // sen_log_t front;
  sen_log_t r45;
  // sen_log_t r90;
} sen_logs_t;

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
  sen_logs_t sen;
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
  float right45;
  float left45;
  float right90;
  float left90;
  float front;
  float kireme_r;
  float kireme_l;
} sen_ref_param3_t;

typedef struct {
  float front;
  float right45;
  float left45;
  float right90;
  float left90;
  float kireme_r;
  float kireme_l;
  float offset_r;
  float offset_l;
  float front_ctrl;
  float front_ctrl_th;
} sen_search_param_t;

typedef struct {
  sen_ref_param3_t ref;
  sen_ref_param3_t exist;
} sen_ref_param2_t;

typedef struct {
  sen_ref_param2_t normal;
  sen_ref_param2_t dia;
  sen_search_param_t search_exist;
} sen_ref_param_t;

typedef struct {
  float a;
  float b;
} sensor_gain_param_t;

typedef struct {
  sensor_gain_param_t l90;
  sensor_gain_param_t l45;
  sensor_gain_param_t front;
  sensor_gain_param_t r45;
  sensor_gain_param_t r90;
} sensor_gain_t;

// typedef struct{

// } wall_off_p

typedef struct {
  float left_str;
  float right_str;
  float left_dia;
  float right_dia;
  float exist_dist_l;
  float exist_dist_r;
  float noexist_th_l;
  float noexist_th_r;
  float noexist_dia_th_l;
  float noexist_dia_th_r;
} wall_off_hold_dist_t;

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
  float offset_start_dist = 0;
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
  sen_ref_param_t sen_ref_p;
  sensor_gain_t sensor_gain;
  int sakiyomi_time = 1;
  float clear_angle = 0;
  float clear_dist_order = 0;
  float front_dist_offset = 0;
  float front_dist_offset2 = 0;
  float front_dist_offset_dia_front = 0;
  float front_dist_offset_dia_45_th = 0;
  float front_dist_offset_dia_right45 = 0;
  float front_dist_offset_dia_left45 = 0;

  float clear_dist_ragne_from = 0;
  float clear_dist_ragne_to = 0;
  float wall_off_hold_dist;
  wall_off_hold_dist_t wall_off_dist;

  int search_log_enable = 0;
  int test_log_enable = 0;
  int fast_log_enable = 0;
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
  gain_log_t sen_log;
  gain_log_t sen_log_dia;
  pid_error_t sen;
  pid_error_t sen_dia;
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
  float sla_rad;
  RUN_MODE2 motion_mode;
  MotionType motion_type;

  int timstamp = 0;
  MotionDirection motion_dir;
  bool dia_mode = false;
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
  bool dia_mode = false;
  planning_req_t pl_req;
  fail_safe_state_t fss;
  float gyro_zero_p_offset = 0;
  buzzer_t buzzer;
  new_motion_req_t nmr;
} motion_tgt_val_t;

typedef struct {
  float v_max = 0;
  float v_end = 0;
  float accl = 0;
  float decel = 0;
  float dist = 0;
  MotionType motion_type = MotionType::NONE;
  SensorCtrlType sct = SensorCtrlType::NONE;
  WallOffReq wall_off_req = WallOffReq::NONE;
  float wall_off_dist_r = 0;
  float wall_off_dist_l = 0;
  bool dia_mode = false;
} param_straight_t;

typedef struct {
  float w_max = 0;
  float w_end = 0;
  float alpha = 0;
  float ang = 0;
  TurnDirection RorL = TurnDirection::None;

} param_roll_t;

typedef struct {
  float radius = 0;
  float v_max = 0;
  float v_end = 0;
  float ang = 0;
  TurnDirection RorL = TurnDirection::None;
} param_normal_slalom_t;

typedef struct {
  float v_max = 0;
  float end_v = 0;
  float accl = 0;
  float decel = 0;
  float dist = 0;
  float w_max = 0;
  float w_end = 0;
  float alpha = 0;
  float ang = 0;
  int suction_active = 0;
  float suction_duty = 0;
  float sla_dist = 0;
  int file_idx = 0;
  int sla_type = 0;
  int sla_return = 0;
  int sla_type2 = 0;
  int turn_times = 0;
  int ignore_opp_sen = 0;
} test_mode_t;

typedef struct {
  std::vector<point_t> goals;
  int maze_size = 0;
  int user_mode = 0;
  test_mode_t test;
} system_t;

typedef struct {
  int normal = 0;
  int large = 0;
  int orval = 0;
  int dia45 = 0;
  int dia45_2 = 0;
  int dia135 = 0;
  int dia135_2 = 0;
  int dia90 = 0;
} profile_idx_t;

typedef struct {
  std::vector<std::string> file_list;
  int file_list_size = 0;
  int profile_idx_size = 0;
  std::vector<profile_idx_t> profile_list;
} turn_param_profile_t;

typedef struct {
  float right = 0;
  float left = 0;
} slalom_offset_t;

typedef struct {
  float v = 0;
  float ang = 0;
  float rad = 0;
  slalom_offset_t front;
  slalom_offset_t back;
  int pow_n = 0;
  float time = 0;
  TurnType type = TurnType::None;
} slalom_param2_t;

typedef struct {
  float v_max = 0;
  float accl = 0;
  float decel = 0;
  float w_max = 0;
  float w_end = 0;
  float alpha = 0;
} straight_param_t;

typedef struct {
  std::unordered_map<TurnType, slalom_param2_t> map;
  std::unordered_map<StraightType, straight_param_t> str_map;
} param_set_t;

typedef struct {
  bool is_turn = false;
  TurnType next_turn_type;
  float v_max = 0;
  float v_end = 0;
  float accl = 0;
  float decel = 0;
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
  float duty_sensor_ctrl;
} log_data_t;

union float16_bitmap {
  struct {
    unsigned int s : 1;
    unsigned int e : 8;
    unsigned int m : 23;
  };
  float data;
};

union uint16_bitmap {
  struct {
    unsigned int s : 1;
    unsigned int e : 5;
    unsigned int m : 10;
  };
  int16_t data;
};

typedef struct {
  // int idx;
  real16_T img_v;
  real16_T v_l;
  real16_T v_c;
  real16_T v_r;
  real16_T accl;
  real16_T img_w;
  real16_T w_lp;
  real16_T alpha;

  real16_T img_dist;
  real16_T dist;
  real16_T img_ang;
  real16_T ang;

  real16_T duty_l;
  real16_T duty_r;

  real16_T left90_lp;
  real16_T left45_lp;
  real16_T front_lp;
  real16_T right45_lp;
  real16_T right90_lp;
  real16_T battery_lp;

  char motion_type;
  int16_t motion_timestamp;

  real16_T duty_ff_front;
  real16_T duty_ff_roll;
  real16_T duty_sensor_ctrl;
  real16_T sen_log_l45;
  real16_T sen_log_r45;
} log_data_t2;

typedef struct {
  int invalid_duty_r_cnt;
  int invalid_duty_l_cnt;
} fail_safe_t;

#endif