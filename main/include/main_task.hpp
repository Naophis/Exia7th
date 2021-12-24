#ifndef MAIN_TASK_HPP
#define MAIN_TASK_HPP

#include "defines.hpp"
#include "driver/pcnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icm20689.hpp"
#include "include/maze_solver.hpp"
#include "include/planning_task.hpp"
#include <driver/adc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "include/integrated_entity.hpp"
class MainTask {
public:
  MainTask();
  virtual ~MainTask();

  void create_task(const BaseType_t xCoreID);
  static void task_entry_point(void *task_instance);

  sensing_entity_t *entity;
  ego_param_t *param;
  ego_entity_t *ego;
  PlanningTask *pt;
  void set_sensing_entity(sensing_entity_t *_entity);
  void set_ego_entity(ego_entity_t *_ego);
  void set_planning_task(PlanningTask *_pt);
  virtual void task();

private:
  xTaskHandle handle = 0;
};

#endif