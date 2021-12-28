#ifndef MAIN_TASK_HPP
#define MAIN_TASK_HPP

#include "defines.hpp"
#include "driver/pcnt.h"
#include "driver/uart.h"
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
#include "include/motion_planning.hpp"
#include "include/ui.hpp"

#include "libs/nlohmnn-json/json.hpp"
constexpr int RESET_GYRO_LOOP_CNT = 100;
class MainTask {
public:
  MainTask();

  // MainTask(const exia_io_t &_exia) : exia_io(_exia);
  virtual ~MainTask();

  void create_task(const BaseType_t xCoreID);
  static void task_entry_point(void *task_instance);

  // read
  sensing_result_entity_t *entity_ro;
  ego_param_t *param;
  ego_entity_t *ego;
  PlanningTask *pt;

  // write
  tgt_entity_t *tgt;
  void set_sensing_entity(sensing_result_entity_t *_entity);
  void set_ego_entity(ego_entity_t *_ego);
  void set_planning_task(PlanningTask *_pt);
  void set_tgt_entity(tgt_entity_t *_tgt);
  virtual void task();
  void check_battery();
  void reset_gyro_ref();
  void reset_motion_tgt();
  void set_tgt_val(motion_tgt_val_t *_tgt) { tgt_val = _tgt; }

private:
  xTaskHandle handle = 0;
  motion_tgt_val_t *tgt_val;
  UserInterface ui;
  MotionPlanning mp;
  void dump1();
  void operation();
  void keep_pivot();
  void echo_sensing_result_with_json();
  nlohmann::json json_instance;
  void entity_to_json(nlohmann::json &j);
  void recieve_data();
};

#endif