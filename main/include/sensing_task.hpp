#ifndef SENSING_TASK_HPP
#define SENSING_TASK_HPP

#include "defines.hpp"
#include "driver/pcnt.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icm20689.hpp"
#include <driver/adc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BATTERY_GAIN 2.97324
class SensingTask {
public:
  SensingTask();
  virtual ~SensingTask();

  void create_task(const BaseType_t xCoreID);
  static void task_entry_point(void *task_instance);
  static void IRAM_ATTR isr_entry_point(void *task_instance);
  static void IRAM_ATTR timerCallback(void *arg);

  std::shared_ptr<sensing_result_entity_t> entity;
  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
  virtual void task();

private:
  ICM20689 gyro_if;
  xTaskHandle handle = 0;
  timer_isr_handle_t handle_isr;
  void encoder_init(const pcnt_unit_t unit, const gpio_num_t pinA,
                    const gpio_num_t pinB);
  void timer_init_grp0_timer0();
  void IRAM_ATTR timer_isr();
  // void timer_isr(void *parameters);

  bool itr_state = true;
  int c = 0;
  int d = 0;
};

#endif