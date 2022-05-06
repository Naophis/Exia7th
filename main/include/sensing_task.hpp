#ifndef SENSING_TASK_HPP
#define SENSING_TASK_HPP

#include "defines.hpp"
#include "driver/pcnt.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icm20689.hpp"
#include <deque>
#include <driver/adc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class SensingTask {
public:
  SensingTask();
  virtual ~SensingTask();

  void create_task(const BaseType_t xCoreID);

  std::shared_ptr<sensing_result_entity_t> sensing_result;
  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);

  static void task_entry_point(void *task_instance);
  virtual void task();
  static void task_entry_point0(void *task_instance);
  static void task_entry_point1(void *task_instance);
  static void task_entry_point2(void *task_instance);
  static void task_entry_point3(void *task_instance);
  static void task_entry_point4(void *task_instance);
  virtual void task0();
  virtual void task1();
  virtual void task2();
  virtual void task3();
  virtual void task4();
  void set_input_param_entity(std::shared_ptr<input_param_t> &_param);

  ICM20689 gyro_if;
  bool is_ready() { return ready; }
  std::deque<int> gyro_q;
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);

private:
  std::shared_ptr<input_param_t> param;
  int led_light_delay_cnt = 10000;
  xTaskHandle handle = 0;
  bool ready;
  timer_isr_handle_t handle_isr;
  void encoder_init(const pcnt_unit_t unit, const gpio_num_t pinA,
                    const gpio_num_t pinB);
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

  // void timer_isr(void *parameters);
  std::shared_ptr<motion_tgt_val_t> tgt_val;

  bool itr_state = true;
  static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
  static const adc_atten_t atten = ADC_ATTEN_DB_11;
  pcnt_config_t pcnt_config_0 = {
      // .pulse_gpio_num = pinA,
      // .ctrl_gpio_num = pinB,
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_REVERSE,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DEC,
      .counter_h_lim = ENCODER_H_LIM_VAL,
      .counter_l_lim = ENCODER_L_LIM_VAL,
      // .unit = unit,
      .channel = PCNT_CHANNEL_0,
  };
  pcnt_config_t pcnt_config_1 = {
      // .pulse_gpio_num = pinB,
      // .ctrl_gpio_num = pinA,
      .lctrl_mode = PCNT_MODE_REVERSE,
      .hctrl_mode = PCNT_MODE_KEEP,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DEC,
      .counter_h_lim = ENCODER_H_LIM_VAL,
      .counter_l_lim = ENCODER_L_LIM_VAL,
      // .unit = unit,
      .channel = PCNT_CHANNEL_1,
  };
};

#endif