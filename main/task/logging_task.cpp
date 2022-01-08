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
void LoggingTask::set_ego_param_entity(std::shared_ptr<ego_param_t> &_param) {
  param = _param;
}
void LoggingTask::set_ego_entity(std::shared_ptr<ego_entity_t> &_ego) {
  ego = _ego;
}
void LoggingTask::set_tgt_entity(std::shared_ptr<tgt_entity_t> &_tgt) {
  tgt = _tgt;
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
  const TickType_t xDelay_fast = 1 / portTICK_PERIOD_MS;
  const TickType_t xDelay2 = 100 / portTICK_PERIOD_MS;
  while (1) {
    logging_active = active_slalom_log;
    if (logging_active) {
      if (active_slalom_log && idx_slalom_log <= LOG_SIZE) {
        auto ld = std::make_shared<log_data_t>();
        ld->img_v = tgt_val->ego_in.v;
        ld->v_l = ego->v_l;
        ld->v_c = ego->v_c;
        ld->v_r = ego->v_r;
        ld->accl = tgt_val->ego_in.accl;

        ld->img_w = tgt_val->ego_in.w;
        ld->w_lp = ego->w_lp;
        ld->alpha = tgt_val->ego_in.alpha;

        ld->img_dist = tgt_val->ego_in.img_dist;
        ld->dist = tgt_val->ego_in.dist;

        ld->img_ang = tgt_val->ego_in.img_ang * 180 / PI;
        ld->ang = tgt_val->ego_in.ang * 180 / PI;

        ld->duty_l = ego->duty.duty_l;
        ld->duty_r = ego->duty.duty_r;

        ld->left90_lp = ego->left90_lp;
        ld->left45_lp = ego->left45_lp;
        ld->front_lp = ego->front_lp;
        ld->right45_lp = ego->right45_lp;
        ld->right90_lp = ego->right90_lp;

        ld->battery_lp = ego->battery_lp;
        ld->duty_l = ego->duty.duty_l;
        ld->duty_r = ego->duty.duty_r;
        log_vec.push_back(ld);
        idx_slalom_log++;
      }
      vTaskDelay(xDelay_fast);
    } else {
      vTaskDelay(xDelay2);
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
    fprintf(f_slalom_log, f1, // "%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,", i,
            i++, ld->img_v, ld->v_c, ld->v_l, ld->v_r, ld->accl);
    fprintf(f_slalom_log, f2, //"%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,", //
            ld->img_w, ld->w_lp, ld->alpha, ld->img_dist, ld->dist, ld->img_ang,
            ld->ang);
    fprintf(f_slalom_log, f3, //"%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n", //
            ld->left90_lp, ld->left45_lp, ld->front_lp, ld->right45_lp,
            ld->right90_lp, ld->battery_lp, ld->duty_l, ld->duty_r);
  }

  if (f_slalom_log != NULL) {
    fclose(f_slalom_log);
    printf("close\n");
  }
}

void LoggingTask::dump_log(std::string file_name) {

  FILE *f = fopen(file_name.c_str(), "rb");
  if (f == NULL)
    return;
  char line_buf[LOG_BUF_SIZE];
  printf("start___\n"); // csvファイル作成トリガー
  printf("index,ideal_v,v_c,v_l,v_r,accl,ideal_w,w_lp,alpha,ideal_dist,dist,"
         "ideal_ang,ang,left90,left45,front,right45,right90,battery,duty_l,duty_r\n");
  while (fgets(line_buf, sizeof(line_buf), f) != NULL)
    printf("%s\n", line_buf);
  printf("end___\n"); // csvファイル追記終了トリガー

  fclose(f);
}