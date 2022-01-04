
#ifndef DEFINES_HPP
#define DEFINES_HPP

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "gen_code/bus.h"
#include "include/maze_solver.hpp"
#include "sdkconfig.h"
#include "soc/adc_channel.h"
#include "soc/ledc_periph.h"

#include <initializer_list>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#define ABS(IN) ((IN) < 0 ? -(IN) : (IN))

#define LED_R90 GPIO_NUM_9
#define LED_R45 GPIO_NUM_10
#define LED_F GPIO_NUM_13
#define LED_L45 GPIO_NUM_16
#define LED_L90 GPIO_NUM_18

#define TXD GPIO_NUM_43
#define RXD GPIO_NUM_44
#define RTS GPIO_NUM_15
#define CTS GPIO_NUM_16

#define A_CW_CCW GPIO_NUM_39
#define B_CW_CCW GPIO_NUM_41
#define A_PWM GPIO_NUM_40
#define B_PWM GPIO_NUM_42

#define BUZZER GPIO_NUM_34

#define LED1 GPIO_NUM_1
#define LED2 GPIO_NUM_46
#define LED3 GPIO_NUM_45
#define LED4 GPIO_NUM_8
#define LED5 GPIO_NUM_21
#define LED_R GPIO_NUM_8
#define LED_L GPIO_NUM_21

#define SW1 GPIO_NUM_38

#define EN_MOSI GPIO_NUM_2
#define EN_CLK GPIO_NUM_3
#define EN_GN_SSL GPIO_NUM_4
#define EN_MISO GPIO_NUM_5

#define ENC_R_A GPIO_NUM_6
#define ENC_R_B GPIO_NUM_7
#define ENC_L_A GPIO_NUM_35
#define ENC_L_B GPIO_NUM_36

#define SEN_R90 ADC2_CHANNEL_0
#define SEN_R45 ADC2_CHANNEL_1
#define SEN_F ADC2_CHANNEL_4
#define SEN_L45 ADC2_CHANNEL_6
#define SEN_L90 ADC2_CHANNEL_8
#define BATTERY ADC2_CHANNEL_9

#define MOTOR_HZ 100000
#define SUCTION_MOTOR_HZ 10000

#define SUCTION_PWM GPIO_NUM_37
#define LEDC_AUTO_CLK 0
#define LEDC_TIMER_10_BIT 10
#define LEDC_TIMER_14_BIT 14
// #define LEDC_TIMER_0 0
#define LEDC_CHANNEL_0 0
#define LEDC_INTR_DISABLE 0
// #define LEDC_HIGH_SPEED_MODE 0

#define ENCODER_H_LIM_VAL 32767
#define ENCODER_L_LIM_VAL -32767
#define READ_FLAG 0x80
#define ESC 0x1B
#define BUF_SIZE (4096)

// constexpr double MOTION_CHECK_TH = 1000;
#define MOTION_CHECK_TH 1000
#define ENC_OPE_V_R_TH 175

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
  int16_t right;
  int16_t left;
} encoder_data_t;

typedef struct {
  int raw;
  double data;
} sensing_data_t;

typedef struct {
  sensing_data_t right90;
  sensing_data_t right45;
  sensing_data_t front;
  sensing_data_t left45;
  sensing_data_t left90;
} led_sensor_t;

typedef struct {
  led_sensor_t led_sen;
  led_sensor_t led_sen_after;
  led_sensor_t led_sen_before;
  sensing_data_t gyro;
  sensing_data_t battery;
  encoder_data_t encoder_raw;
  encoder_data_t encoder;
} sensing_result_entity_t;

typedef struct {
  double vel;
  double speed;
  double accl;
} xva_t;

typedef struct {
  double right;
  double left;
} rpm_t;

typedef struct {
  double duty_l;
  double duty_r;
  double duty_suction;
} duty_t;

typedef struct {
  double v_r;
  double v_l;
  double v_c;
  double w;
  // double dist;
  // double angle;
  rpm_t rpm;
  duty_t duty;
} ego_entity_t;

typedef struct {
  double p;
  double i;
  double d;
} pid_param_t;

typedef struct {
  double gyro_w_gain_right;
  double gyro_w_gain_left;
} gyro_param_t;

typedef struct {
  double dt;
  double tire;
  double gear_a;
  double gear_b;
  double max_duty;
  double Ke;
  double Km;
  double Resist;
  double Mass;
  double Lm;
  pid_param_t motor_pid;
  pid_param_t gyro_pid;
  pid_param_t sensor_pid;
  pid_param_t sensor_pid_dia;
  gyro_param_t gyro_param;
} ego_param_t;

typedef struct {
  double error_p;
  double error_i;
  double error_d;
} pid_error_t;

typedef struct {
  pid_error_t v;
  pid_error_t w;
} pid_error_entity_t;

// 指示速度
typedef struct {
  double v_max = 0;
  double accl = 0;
  double w_max = 0;
  double alpha = 0;
} motion_tgt_t;

typedef struct {
  int hz = 0;
  int time = 0;
  int timstamp = 0;
} buzzer_t;

typedef struct {
  double gyro_zero_p_offset;
  // motion_tgt_t motion_tgt;
  buzzer_t buzzer;
} tgt_entity_t;

typedef struct {
  int time_stamp = 0;
  int error_gyro_reset = 0;
  int error_vel_reset = 0;
} planning_req_t;

typedef struct {
  t_tgt tgt_in;
  t_ego ego_in;
  int32_t motion_mode;
  planning_req_t pl_req;
} motion_tgt_val_t;

enum class RUN_MODE2 : int {
  NONE_MODE = 0,
  KEEP = 0,
  SLAROM_RUN = 1,
  PIVOT_TURN = 2,
  ST_RUN = 3,
  SLALOM_RUN2 = 4,
};

typedef struct {
  double v_max;
  double v_end;
  double accl;
  double decel;
  double dist;
} param_straight_t;

typedef struct {
  double w_max;
  double w_end;
  double alpha;
  double ang;
  TurnDirection RorL;
} param_roll_t;

typedef struct {
  double radius;
  double v_max;
  double v_end;
  double ang;
  TurnDirection RorL;
} param_normal_slalom_t;

typedef struct {
  double v_max;
  double accl;
  double decel;
  double dist;
  double w_max;
  double alpha;
  double ang;
  int suction_active;
  double suction_duty;
  int file_idx;
  int sla_type;
  int sla_return;
  int sla_type2;
} test_mode_t;

typedef struct {
  std::vector<point_t> goals;
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
  double right;
  double left;
} slalom_offset_t;

typedef struct {
  double v;
  double ang;
  double rad;
  slalom_offset_t front;
  slalom_offset_t back;
  int pow_n;
  double time;
} slalom_param2_t;

typedef struct {
  std::unordered_map<TurnType, slalom_param2_t> map;
} slalom_parameter_t;

typedef struct {
  bool is_turn;
  TurnType next_turn_type;
  double v_max;
  double v_end;
  double accl;
  double decel;
} next_motionr_t;

static std::initializer_list<std::pair<TurnType, std::string>> turn_name_list =
    {
        {TurnType::Normal, "normal"},    //
        {TurnType::Large, "large"},      //
        {TurnType::Orval, "orval"},      //
        {TurnType::Dia45, "dia45"},      //
        {TurnType::Dia135, "dia135"},    //
        {TurnType::Dia90, "dia90"},      //
        {TurnType::Dia45_2, "dia45_2"},  //
        {TurnType::Dia135_2, "dia135_2"} //
};
#endif