#include "include/logging_task.hpp"

void LoggingTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "logging_task", 8192, this, 1,
                          &handle, xCoreID);
}

void LoggingTask::task_entry_point(void *task_instance) {
  static_cast<LoggingTask *>(task_instance)->task();
}

void LoggingTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_entity) {
  sensing_result = _entity;
}
void LoggingTask::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param) {
  param = _param;
}

void LoggingTask::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}

void LoggingTask::start_slalom_log() {
  active_slalom_log = true; //
  idx_slalom_log = 0;
  log_vec.clear();
}

void LoggingTask::stop_slalom_log() {
  active_slalom_log = false; //
}

void LoggingTask::task() {
  const TickType_t xDelay = 4 / portTICK_PERIOD_MS;
  while (1) {
    logging_active = active_slalom_log;
    if (logging_active) {
      if (active_slalom_log && idx_slalom_log <= LOG_SIZE) {
        auto ld = std::make_shared<log_data_t2>();

        ld->img_v = floatToHalf(tgt_val->ego_in.v);
        ld->v_l = floatToHalf(sensing_result->ego.v_l);
        ld->v_c = floatToHalf(sensing_result->ego.v_c);
        ld->v_r = floatToHalf(sensing_result->ego.v_r);
        ld->accl = floatToHalf(tgt_val->ego_in.accl);

        ld->img_w = floatToHalf(tgt_val->ego_in.w);
        ld->w_lp = floatToHalf(sensing_result->ego.w_lp);
        ld->alpha = floatToHalf(tgt_val->ego_in.alpha);

        ld->img_dist = floatToHalf(tgt_val->ego_in.img_dist);
        ld->dist = floatToHalf(tgt_val->ego_in.dist);

        ld->img_ang = floatToHalf(tgt_val->ego_in.img_ang * 180 / PI);
        ld->ang = floatToHalf(tgt_val->ego_in.ang * 180 / PI);

        ld->left90_lp = floatToHalf(sensing_result->ego.left90_lp);
        ld->left45_lp = floatToHalf(sensing_result->ego.left45_lp);
        ld->front_lp = floatToHalf(sensing_result->ego.front_lp);
        ld->right45_lp = floatToHalf(sensing_result->ego.right45_lp);
        ld->right90_lp = floatToHalf(sensing_result->ego.right90_lp);

        ld->battery_lp = floatToHalf(sensing_result->ego.battery_lp);
        ld->duty_l = floatToHalf(sensing_result->ego.duty.duty_l);
        ld->duty_r = floatToHalf(sensing_result->ego.duty.duty_r);

        ld->motion_type = static_cast<char>(tgt_val->motion_type);

        ld->duty_sensor_ctrl = floatToHalf(sensing_result->ego.duty.sen);
        ld->duty_ff_front = floatToHalf(sensing_result->ego.ff_duty.front);
        ld->duty_ff_roll = floatToHalf(sensing_result->ego.ff_duty.roll);

        log_vec.push_back(ld);
        idx_slalom_log++;
      }
      vTaskDelay(xDelay);
    } else {
      vTaskDelay(xDelay);
    }
  }
}

void LoggingTask::save(std::string file_name) {
  printf("usefile: %s\n", slalom_log_file.c_str());
  f_slalom_log = fopen(slalom_log_file.c_str(), "wb");
  if (f_slalom_log == NULL)
    printf("slalom_file_load_failed\n");

  const char *f1 = format1.c_str();
  const char *f2 = format2.c_str();
  const char *f3 = format3.c_str();

  int i = 0;

  for (const auto ld : log_vec) {

    fprintf(f_slalom_log, f1,                   //
            i++,                                //
            halfToFloat(ld->img_v),             //
            halfToFloat(ld->v_c),               //
            halfToFloat(ld->v_l),               //
            halfToFloat(ld->v_r),               //
            halfToFloat(ld->accl));             //
    fprintf(f_slalom_log, f2,                   //
            halfToFloat(ld->img_w),             //
            halfToFloat(ld->w_lp),              //
            halfToFloat(ld->alpha),             //
            halfToFloat(ld->img_dist),          //
            halfToFloat(ld->dist),              //
            halfToFloat(ld->img_ang),           //
            halfToFloat(ld->ang));              //
    fprintf(f_slalom_log, f3,                   //
            halfToFloat(ld->left90_lp),         //
            halfToFloat(ld->left45_lp),         //
            halfToFloat(ld->front_lp),          //
            halfToFloat(ld->right45_lp),        //
            halfToFloat(ld->right90_lp),        //
            halfToFloat(ld->battery_lp),        //
            halfToFloat(ld->duty_l),            //
            halfToFloat(ld->duty_r),            //
            (ld->motion_type),                  //
            halfToFloat(ld->duty_sensor_ctrl),  //
            halfToFloat(ld->duty_ff_roll),      //
            halfToFloat(ld->duty_sensor_ctrl)); //
  }

  if (f_slalom_log != NULL) {
    fclose(f_slalom_log);
    printf("close\n");
  }
}

void LoggingTask::dump_log(std::string file_name) {

  const TickType_t xDelay2 = 100 / portTICK_PERIOD_MS;
  FILE *f = fopen(file_name.c_str(), "rb");
  if (f == NULL)
    return;
  char line_buf[LINE_BUF_SIZE];
  printf("start___\n"); // csvファイル作成トリガー
  vTaskDelay(xDelay2);
  printf("index,ideal_v,v_c,v_l,v_r,accl,ideal_w,w_lp,alpha,ideal_dist,dist,"
         "ideal_ang,ang,left90,left45,front,right45,right90,battery,duty_l,"
         "duty_r,motion_state,ff_duty_front,ff_duty_roll,duty_sen\n");
  while (fgets(line_buf, sizeof(line_buf), f) != NULL)
    printf("%s\n", line_buf);
  printf("end___\n"); // csvファイル追記終了トリガー

  fclose(f);
}