#include "include/logging_task.hpp"

void LoggingTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "logging_task", 8192, this, 1,
                          &handle, xCoreID);
}
void LoggingTask::task_entry_point(void *task_instance) {
  static_cast<LoggingTask *>(task_instance)->task();
}

void LoggingTask::set_sensing_entity(sensing_result_entity_t *_entity) {
  sensing_result = _entity;
}
void LoggingTask::set_ego_param_entity(ego_param_t *_param) {
  param = _param; //
}
void LoggingTask::set_ego_entity(ego_entity_t *_ego) {
  ego = _ego; //
}
void LoggingTask::set_tgt_entity(tgt_entity_t *_tgt) {
  tgt = _tgt; //
}
void LoggingTask::set_tgt_val(motion_tgt_val_t *_tgt) {
  tgt_val = _tgt; ///
}

void LoggingTask::start_slalom_log() {
  active_slalom_log = true; //
  idx_slalom_log = 0;
}
void LoggingTask::stop_slalom_log() {
  active_slalom_log = false; //
}

void LoggingTask::task() {
  const TickType_t xDelay_fast = 1 / portTICK_PERIOD_MS;
  const TickType_t xDelay2 = 100 / portTICK_PERIOD_MS;
  // const char *f1 = format1.c_str();
  // const char *f2 = format2.c_str();
  // const char *f3 = format3.c_str();
  while (1) {
    logging_active = active_slalom_log;
    if (logging_active) {
      if (active_slalom_log && idx_slalom_log <= LOG_SIZE) {
        log_list2[idx_slalom_log].img_v = tgt_val->ego_in.v;
        log_list2[idx_slalom_log].v_l = ego->v_l;
        log_list2[idx_slalom_log].v_c = ego->v_c;
        log_list2[idx_slalom_log].v_r = ego->v_r;
        log_list2[idx_slalom_log].accl = tgt_val->ego_in.accl;

        log_list2[idx_slalom_log].img_w = tgt_val->ego_in.w;
        log_list2[idx_slalom_log].w_lp = ego->w_lp;
        log_list2[idx_slalom_log].alpha = tgt_val->ego_in.alpha;

        log_list2[idx_slalom_log].img_dist = tgt_val->ego_in.img_dist;
        log_list2[idx_slalom_log].dist = tgt_val->ego_in.dist;

        log_list2[idx_slalom_log].img_ang = tgt_val->ego_in.img_ang * 180 / PI;
        log_list2[idx_slalom_log].ang = tgt_val->ego_in.ang * 180 / PI;

        log_list2[idx_slalom_log].duty_l = ego->duty.duty_l;
        log_list2[idx_slalom_log].duty_r = ego->duty.duty_r;

        log_list2[idx_slalom_log].left90_lp = ego->left90_lp;
        log_list2[idx_slalom_log].left45_lp = ego->left45_lp;
        log_list2[idx_slalom_log].front_lp = ego->front_lp;
        log_list2[idx_slalom_log].right45_lp = ego->right45_lp;
        log_list2[idx_slalom_log].right90_lp = ego->right90_lp;

        log_list2[idx_slalom_log].battery_lp = ego->battery_lp;

        idx_slalom_log++;
      }
      vTaskDelay(xDelay_fast);
    } else {
      vTaskDelay(xDelay2);
    }
  }
}

void LoggingTask::save(std::string file_name) {
  const TickType_t xDelay_fast = 1 / portTICK_PERIOD_MS;
  printf("usefile: %s\n", slalom_log_file.c_str());
  f_slalom_log = fopen(slalom_log_file.c_str(), "wb");
  if (f_slalom_log == NULL)
    printf("slalom_file_load_failed\n");

  const char *f1 = format1.c_str();
  const char *f2 = format2.c_str();
  const char *f3 = format3.c_str();
  for (int i = 0; i < idx_slalom_log; i++) {
    fprintf(f_slalom_log, f1,      // "%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,", i, //
            i, log_list2[i].img_v, //
            log_list2[i].v_c, log_list2[i].v_l, log_list2[i].v_r, // v
            log_list2[i].accl);
    vTaskDelay(xDelay_fast);

    fprintf(f_slalom_log, f2, //"%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,", //
            log_list2[i].img_w, log_list2[i].w_lp, log_list2[i].alpha,
            log_list2[i].img_dist, log_list2[i].dist, log_list2[i].img_ang,
            log_list2[i].ang);

    vTaskDelay(xDelay_fast);

    fprintf(f_slalom_log, f3, //"%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n", //
            log_list2[i].left90_lp, log_list2[i].left45_lp,
            log_list2[i].front_lp, log_list2[i].right45_lp,
            log_list2[i].right90_lp, log_list2[i].battery_lp);
    vTaskDelay(xDelay_fast);
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

  while (fgets(line_buf, sizeof(line_buf), f) != NULL)
    printf("%s\n", line_buf);

  fclose(f);
}