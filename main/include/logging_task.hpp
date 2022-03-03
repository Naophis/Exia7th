#ifndef LOGGING_TASK_HPP
#define LOGGING_TASK_HPP

#include "defines.hpp"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gen_code_conv_single2half/half_type.h"

class LoggingTask {
public:
  LoggingTask(){};
  ~LoggingTask(){};

  void create_task(const BaseType_t xCoreID);
  static void task_entry_point(void *task_instance);
  virtual void task();
  void dump();

  void
  set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_sensing_result);
  void set_input_param_entity(std::shared_ptr<input_param_t> &_param);
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);

  void start_slalom_log();
  void stop_slalom_log();

  void dump_log(const std::string file_name);
  void save(const std::string file_name);

private:
  bool logging_active = false;
  xTaskHandle handle = 0;
  float16_bitmap fbm;
  uint16_bitmap uibm;

  std::shared_ptr<sensing_result_entity_t> sensing_result;
  std::shared_ptr<input_param_t> param;
  std::shared_ptr<motion_tgt_val_t> tgt_val;

  bool active_slalom_log = false;
  int idx_slalom_log = 0;
  FILE *f_slalom_log;
  // std::vector<std::shared_ptr<log_data_t>> log_vec;
  std::vector<std::shared_ptr<log_data_t2>> log_vec;
  float calc_sensor(float data, float a, float b, char motion_type);
};

#endif