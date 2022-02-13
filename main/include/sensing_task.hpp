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
  static void task_entry_point(void *task_instance);

  std::shared_ptr<sensing_result_entity_t> sensing_result;
  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
  virtual void task();

  ICM20689 gyro_if;
  bool is_ready() { return ready; }
  std::deque<int> gyro_q;
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);

private:
  xTaskHandle handle = 0;
  bool ready;
  timer_isr_handle_t handle_isr;
  void encoder_init(const pcnt_unit_t unit, const gpio_num_t pinA,
                    const gpio_num_t pinB);

  // void timer_isr(void *parameters);
  std::shared_ptr<motion_tgt_val_t> tgt_val;

  bool itr_state = true;
};

#endif