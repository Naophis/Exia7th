#ifndef PLANNING_TASK_HPP
#define PLANNING_TASK_HPP

#include "defines.hpp"
#include "driver/mcpwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/maze_solver.hpp"

class PlanningTask {
public:
  PlanningTask();
  virtual ~PlanningTask();
  void create_task(const BaseType_t xCoreID);
  void motor_enable();
  void suction_enable();
  void motor_disable();
  void suction_disable();

  void set_sensing_entity(sensing_entity_t *_entity);
  void set_ego_entity(ego_entity_t *_ego);
  void set_ego_param_entity(ego_param_t *_param);

  static void task_entry_point(void *task_instance);
  virtual void task();

  sensing_entity_t *entity;
  ego_param_t *param;
  ego_entity_t *ego;

private:
  xTaskHandle handle = 0;
  bool motor_en;
  bool suction_en;

  void update_ego_motion();
  void set_next_duty(const double duty_l, const double duty_r,
                     const double duty_suction);
};

#endif