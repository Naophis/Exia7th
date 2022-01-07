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

#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

#include "gen_code/mpc_tgt_calc.h"
#include "include/integrated_entity.hpp"
#include "include/motion_planning.hpp"
#include "include/ui.hpp"

// #include "libs/nlohmnn-json/json.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "include/logging_task.hpp"

#include "cJSON.h"
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
  tgt_entity_t *tgt;
  motion_tgt_val_t *tgt_val;
  PlanningTask *pt;
  LoggingTask *lt;

  // write
  void set_sensing_entity(sensing_result_entity_t *_entity);
  void set_ego_param_entity(ego_param_t *_param);
  void set_ego_entity(ego_entity_t *_ego);
  void set_tgt_entity(tgt_entity_t *_tgt);
  void set_tgt_val(motion_tgt_val_t *_tgt);

  void set_planning_task(PlanningTask *_pt);
  void set_logging_task(LoggingTask *_lt);

  virtual void task();
  void check_battery();
  void reset_gyro_ref();
  void reset_motion_tgt();

private:
  xTaskHandle handle = 0;
  UserInterface ui;
  MotionPlanning mp;
  // nlohmann::json json_instance;

  param_straight_t ps;
  param_roll_t pr;
  param_normal_slalom_t pns;
  system_t sys;
  turn_param_profile_t tpp;

  const char *base_path = "/spiflash";
  esp_vfs_fat_mount_config_t mount_config;
  wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

  void dump1();
  int select_mode();
  void keep_pivot();
  void echo_sensing_result_with_json();
  // void entity_to_json(nlohmann::json &j);
  void recieve_data();
  void test_run();

  void reset_ego_data();
  void reset_tgt_data();
  void req_error_reset();
  void test_turn();
  void test_sla();
  void rx_uart_json();

  void save_json_data(std::string &str);
  vector<string> split(const string &s, char delim);
  void load_param();
  void load_hw_param();
  void load_sys_param();
  void load_turn_param_profiles();
  void load_slalom_param();
  void dump_log();
  std::vector<slalom_parameter_t> turn_param_list;
};

#endif