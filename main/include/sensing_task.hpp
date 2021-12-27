#ifndef SENSING_TASK_HPP
#define SENSING_TASK_HPP

#include "defines.hpp"
#include "driver/pcnt.h"
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

  sensing_result_entity_t *entity;
  void set_sensing_entity(sensing_result_entity_t *_entity);
  virtual void task();

private:
  ICM20689 gyro_if;
  xTaskHandle handle = 0;
  void encoder_init(const pcnt_unit_t unit, const gpio_num_t pinA,
                    const gpio_num_t pinB);
};

#endif