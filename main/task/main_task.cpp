#include "include/main_task.hpp"

MainTask::MainTask() {}

MainTask::~MainTask() {}

void MainTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "main_task", 8192, this, 2, &handle,
                          xCoreID);
}
void MainTask::task_entry_point(void *task_instance) {
  static_cast<MainTask *>(task_instance)->task();
}
void MainTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_entity_ro) {
  entity_ro = _entity_ro;
  ui.set_sensing_entity(_entity_ro);
}
void MainTask::set_ego_param_entity(std::shared_ptr<ego_param_t> &_param) {
  param = _param;
}
void MainTask::set_ego_entity(std::shared_ptr<ego_entity_t> &_ego) {
  ego = _ego;
  ui.set_ego_entity(_ego);
  mp.set_ego_entity(_ego);
}
void MainTask::set_tgt_entity(std::shared_ptr<tgt_entity_t> &_tgt) {
  tgt = _tgt;
  ui.set_tgt_entity(_tgt);
  mp.set_tgt_entity(_tgt);
}
void MainTask::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
  mp.set_tgt_val(_tgt_val);
}
void MainTask::set_planning_task(std::shared_ptr<PlanningTask> &_pt) {
  pt = _pt;
}
void MainTask::set_logging_task(std::shared_ptr<LoggingTask> &_lt) {
  lt = _lt; //
}
void MainTask::check_battery() {
  vTaskDelay(1000 / portTICK_PERIOD_MS); //他モジュールの起動待ち

  printf("battery= %f\n", ego->battery_raw);
  if (ego->battery_raw > LOW_BATTERY_TH)
    return;
  while (1) {
    ui.music_sync(MUSIC::G5_, 250);
    vTaskDelay(tgt->buzzer.time / portTICK_PERIOD_MS);
  }
}
void MainTask::reset_gyro_ref() {
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  float gyro_raw_data_sum = 0;

  ui.motion_check();
  for (int i = 0; i < RESET_GYRO_LOOP_CNT; i++) {
    gyro_raw_data_sum += entity_ro->gyro.raw;
    vTaskDelay(xDelay); //他モジュールの起動待ち
  }
  tgt->gyro_zero_p_offset = gyro_raw_data_sum / RESET_GYRO_LOOP_CNT;
}

void MainTask::dump1() {
  const TickType_t xDelay = 100 / portTICK_PERIOD_MS;
  while (1) {
    printf("%c[2J", ESC);   /* 画面消去 */
    printf("%c[0;0H", ESC); /* 戦闘戻す*/
    printf("SW1 %d \n", gpio_get_level(SW1));

    printf("gyro: %d\t(%0.3f)\n", entity_ro->gyro.raw, tgt->gyro_zero_p_offset);
    printf("battery: %0.3f\n", ego->battery_lp);
    printf("encoder: %d, %d\n", entity_ro->encoder.left,
           entity_ro->encoder.right);
    printf("sensor: %d, %d, %d, %d, %d\n", entity_ro->led_sen.left90.raw,
           entity_ro->led_sen.left45.raw, entity_ro->led_sen.front.raw,
           entity_ro->led_sen.right45.raw, entity_ro->led_sen.right90.raw);

    printf("ego_v: %0.3f, %0.3f, %0.3f, %0.3f\n", ego->v_l, ego->v_c, ego->v_r,
           tgt_val->ego_in.dist);
    printf("calc_v: %0.3f, %0.3f\n", tgt_val->ego_in.v, tgt_val->ego_in.w);

    printf("ego_w: %0.3f, %0.3f, %0.3f, %0.3f deg\n", ego->w_raw, ego->w_lp,
           tgt_val->ego_in.ang, tgt_val->ego_in.ang * 180 / PI);
    printf("duty: %0.3f, %0.3f\n", ego->duty.duty_l, ego->duty.duty_r);

    if (ui.button_state()) {
      tgt_val->ego_in.ang = tgt_val->ego_in.dist = 0;
    }

    vTaskDelay(xDelay);
  }
}

int MainTask::select_mode() {
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  int mode_num = 0;
  LED_bit lbit;
  lbit.byte = 0;
  while (1) {
    int res = ui.encoder_operation();
    mode_num += res;
    if (mode_num == -1) {
      mode_num = 14;
    } else if (mode_num == 15) {
      mode_num = (int)(MODE::SEARCH);
    }
    lbit.byte = mode_num + 1;
    ui.LED_bit(lbit.b0, lbit.b1, lbit.b2, lbit.b3, lbit.b4);
    if (ui.button_state_hold()) {
      ui.coin(100);
      break;
    }
    vTaskDelay(xDelay);
  }
  return mode_num;
}

void MainTask::reset_tgt_data() {
  tgt_val->tgt_in.v_max = 0;
  tgt_val->tgt_in.end_v = 0;
  tgt_val->tgt_in.accl = 0;
  tgt_val->tgt_in.decel = 0;
  tgt_val->tgt_in.w_max = 0;
  tgt_val->tgt_in.end_w = 0;
  tgt_val->tgt_in.alpha = 0;
  tgt_val->tgt_in.tgt_dist = 0;
  tgt_val->tgt_in.tgt_angle = 0;

  tgt_val->motion_mode = 0;

  tgt_val->tgt_in.accl_param.limit = 2500;
  tgt_val->tgt_in.accl_param.n = 4;
}

void MainTask::reset_ego_data() {
  tgt_val->ego_in.accl = 0;
  tgt_val->ego_in.alpha = 0;
  tgt_val->ego_in.ang = 0;
  tgt_val->ego_in.dist = 0;
  tgt_val->ego_in.pivot_state = 0;
  tgt_val->ego_in.sla_param.base_alpha = 0;
  tgt_val->ego_in.sla_param.base_time = 0;
  tgt_val->ego_in.sla_param.counter = 0;
  tgt_val->ego_in.sla_param.limit_time_count = 0;
  tgt_val->ego_in.sla_param.pow_n = 0;
  tgt_val->ego_in.sla_param.state = 0;
  tgt_val->ego_in.state = 0;
  tgt_val->ego_in.img_ang = 0;
  tgt_val->ego_in.img_dist = 0;

  tgt_val->ego_in.v = 0;
  tgt_val->ego_in.w = 0;

  tgt_val->motion_mode = 0;
}

void MainTask::keep_pivot() {
  const TickType_t xDelay = 100 / portTICK_PERIOD_MS;
  reset_tgt_data();
  reset_ego_data();

  ui.motion_check();
  pt->motor_enable();

  req_error_reset();

  tgt_val->motion_mode = (int32_t)RUN_MODE2::KEEP;

  while (1) {
    if (ui.button_state_hold())
      break;
    vTaskDelay(xDelay);
  }
}
// void MainTask::entity_to_json(nlohmann::json &j) {
//   // j = nlohmann::json{
//   //     {"led_sen",
//   //      {{"right90", {{"raw", ego->right90_raw}, {"lp", ego->right90_lp}}},
//   //       {"right45", {{"raw", ego->right45_raw}, {"lp", ego->right45_lp}}},
//   //       {"front", {{"raw", ego->front_raw}, {"lp", ego->front_lp}}},
//   //       {"left45", {{"raw", ego->left45_raw}, {"lp", ego->left45_lp}}},
//   //       {"left90", {{"raw", ego->left90_raw}, {"lp", ego->left90_lp}}}}},
//   //     {"gyro", {{"raw", ego->w_raw}, {"lp", ego->w_lp}}},
//   //     {"battery", {{"raw", ego->battery_raw}, {"lp", ego->battery_lp}}},
//   //     {"ego",
//   //      {{"angle", tgt_val->ego_in.ang}, {"dist", tgt_val->ego_in.dist}}}};
// }
void MainTask::echo_sensing_result_with_json() {
  const TickType_t xDelay = 50 / portTICK_PERIOD_MS;
  reset_gyro_ref();
  while (1) {
    // entity_to_json(json_instance);
    // printf("%s\n", json_instance.dump().c_str());
    vTaskDelay(xDelay);
    bool break_btn = ui.button_state_hold();
    if (break_btn) {
      ui.coin(100);
      break;
    }
  }
}
vector<string> MainTask::split(const string &s, char delim) {
  vector<string> elems;
  stringstream ss(s);
  string item;
  while (getline(ss, item, delim)) {
    if (!item.empty()) {
      elems.push_back(item);
    }
  }
  return elems;
}

void MainTask::save_json_data(std::string &str) {
  //「ファイル名@JSON文字列」の体裁でクエリーが来ること前提
  auto res = split(str, '@'); // セパレータ「＠」で分割
  if (res.size() != 2)        // invalid check
    return;

  // mount領域+ファイル名を指定しファイルを開く
  auto tgt_file = "/spiflash/" + res[0];
  auto *f = fopen(tgt_file.c_str(), "wb");
  if (f == NULL)
    return;
  // 書き込み&ファイルclose
  fprintf(f, res[1].c_str());
  fclose(f);
  // printf("%s\n", str.c_str());
  ui.coin(25);
}

void MainTask::load_hw_param() {
  FILE *f = fopen("/spiflash/hardware.txt", "rb");
  if (f == NULL) {
    return;
  }
  char line_buf[LOG_BUF_SIZE];
  fgets(line_buf, sizeof(line_buf), f);
  fclose(f);

  printf("%s\n", line_buf);

  cJSON *root = cJSON_CreateObject(), *motor_pid, *gyro_pid, *gyro_param,
        *battery_param, *led_param;
  root = cJSON_Parse(line_buf);

  param->dt = cJSON_GetObjectItem(root, "dt")->valuedouble;
  param->tire = cJSON_GetObjectItem(root, "tire")->valuedouble;
  param->gear_a = cJSON_GetObjectItem(root, "gear_a")->valuedouble;
  param->gear_b = cJSON_GetObjectItem(root, "gear_b")->valuedouble;
  param->max_duty = cJSON_GetObjectItem(root, "max_duty")->valuedouble;
  param->Ke = cJSON_GetObjectItem(root, "Ke")->valuedouble;
  param->Km = cJSON_GetObjectItem(root, "Km")->valuedouble;
  param->Resist = cJSON_GetObjectItem(root, "Resist")->valuedouble;
  param->Mass = cJSON_GetObjectItem(root, "Mass")->valuedouble;
  param->Lm = cJSON_GetObjectItem(root, "Lm")->valuedouble;

  motor_pid = cJSON_GetObjectItem(root, "motor_pid");
  param->motor_pid.p = cJSON_GetObjectItem(motor_pid, "p")->valuedouble;
  param->motor_pid.i = cJSON_GetObjectItem(motor_pid, "i")->valuedouble;
  param->motor_pid.d = cJSON_GetObjectItem(motor_pid, "d")->valuedouble;

  gyro_pid = cJSON_GetObjectItem(root, "gyro_pid");
  param->gyro_pid.p = cJSON_GetObjectItem(gyro_pid, "p")->valuedouble;
  param->gyro_pid.i = cJSON_GetObjectItem(gyro_pid, "i")->valuedouble;
  param->gyro_pid.d = cJSON_GetObjectItem(gyro_pid, "d")->valuedouble;

  gyro_param = cJSON_GetObjectItem(root, "gyro_param");
  param->gyro_param.gyro_w_gain_right =
      cJSON_GetObjectItem(gyro_param, "gyro_w_gain_right")->valuedouble;
  param->gyro_param.gyro_w_gain_left =
      cJSON_GetObjectItem(gyro_param, "gyro_w_gain_left")->valuedouble;
  param->gyro_param.lp_delay =
      cJSON_GetObjectItem(gyro_param, "lp_delay")->valuedouble;

  battery_param = cJSON_GetObjectItem(root, "battery_param");
  param->battery_param.lp_delay =
      cJSON_GetObjectItem(battery_param, "lp_delay")->valuedouble;

  led_param = cJSON_GetObjectItem(root, "led_param");
  param->led_param.lp_delay =
      cJSON_GetObjectItem(led_param, "lp_delay")->valuedouble;

  cJSON_free(root);
  cJSON_free(motor_pid);
  cJSON_free(gyro_pid);
  cJSON_free(gyro_param);
  cJSON_free(battery_param);
  cJSON_free(led_param);
}

void MainTask::load_sys_param() {
  FILE *f = fopen("/spiflash/system.txt", "rb");
  if (f == NULL) {
    return;
  }
  char line_buf[LOG_BUF_SIZE];
  fgets(line_buf, sizeof(line_buf), f);
  fclose(f);

  printf("%s\n", line_buf);

  cJSON *root = cJSON_CreateObject(), *test, *goals;
  root = cJSON_Parse(line_buf);

  sys.goals.clear();
  goals = cJSON_GetObjectItem(root, "goals");
  int goal_size = cJSON_GetArraySize(goals);
  for (int i = 0; i < goal_size; i++) {
    point_t pt;
    pt.x = cJSON_GetArrayItem(cJSON_GetArrayItem(goals, i), 0)->valueint;
    pt.y = cJSON_GetArrayItem(cJSON_GetArrayItem(goals, i), 1)->valueint;
    sys.goals.emplace_back(pt);
    printf("%u %u\n", pt.x, pt.y);
  }

  sys.user_mode = cJSON_GetObjectItem(root, "mode")->valueint;
  test = cJSON_GetObjectItem(root, "test");

  sys.test.v_max = cJSON_GetObjectItem(test, "v_max")->valuedouble;
  sys.test.end_v = cJSON_GetObjectItem(test, "end_v")->valuedouble;
  sys.test.accl = cJSON_GetObjectItem(test, "accl")->valuedouble;
  sys.test.decel = cJSON_GetObjectItem(test, "decel")->valuedouble;
  sys.test.dist = cJSON_GetObjectItem(test, "dist")->valuedouble;
  sys.test.w_max = cJSON_GetObjectItem(test, "w_max")->valuedouble;
  sys.test.alpha = cJSON_GetObjectItem(test, "alpha")->valuedouble;
  sys.test.ang = cJSON_GetObjectItem(test, "ang")->valuedouble;
  sys.test.suction_active =
      cJSON_GetObjectItem(test, "suction_active")->valueint;
  sys.test.suction_duty =
      cJSON_GetObjectItem(test, "suction_duty")->valuedouble;
  sys.test.file_idx = cJSON_GetObjectItem(test, "file_idx")->valueint;
  sys.test.sla_type = cJSON_GetObjectItem(test, "sla_type")->valueint;
  sys.test.sla_return = cJSON_GetObjectItem(test, "sla_return")->valueint;
  sys.test.sla_type2 = cJSON_GetObjectItem(test, "sla_type2")->valueint;
  sys.test.sla_dist = cJSON_GetObjectItem(test, "sla_dist")->valuedouble;

  cJSON_free(root);
  cJSON_free(goals);
  cJSON_free(test);
}

void MainTask::load_turn_param_profiles() {
  FILE *f = fopen("/spiflash/profiles.txt", "rb");
  if (f == NULL)
    return;
  char line_buf[LOG_BUF_SIZE];
  fgets(line_buf, sizeof(line_buf), f);
  fclose(f);

  cJSON *root = cJSON_CreateObject(), *profile_list, *profile_idx;
  root = cJSON_Parse(line_buf);

  tpp.file_list.clear();
  profile_list = cJSON_GetObjectItem(root, "list");
  int profile_list_size = cJSON_GetArraySize(profile_list);
  printf("profile_list\n");
  for (int i = 0; i < profile_list_size; i++) {
    tpp.file_list.push_back(cJSON_GetArrayItem(profile_list, i)->valuestring);
    tpp.file_list_size++;
  }
  printf("tpp.file_list.size() = %d\n", tpp.file_list.size());

  tpp.profile_idx_size =
      cJSON_GetObjectItem(root, "profile_idx_size")->valueint;
  printf("tpp.profile_idx_size= %d\n", tpp.profile_idx_size);

  tpp.profile_list.clear();
  profile_idx = cJSON_GetObjectItem(root, "profile_idx");
  int profile_idx_size = cJSON_GetArraySize(profile_idx);

  profile_idx_t p_idx;
  for (int i = 0; i < profile_idx_size; i++) {
    p_idx.normal =
        cJSON_GetObjectItem(cJSON_GetArrayItem(profile_idx, i), "normal")
            ->valueint;
    p_idx.large =
        cJSON_GetObjectItem(cJSON_GetArrayItem(profile_idx, i), "large")
            ->valueint;
    p_idx.orval =
        cJSON_GetObjectItem(cJSON_GetArrayItem(profile_idx, i), "orval")
            ->valueint;
    p_idx.dia45 =
        cJSON_GetObjectItem(cJSON_GetArrayItem(profile_idx, i), "dia45")
            ->valueint;
    p_idx.dia45_2 =
        cJSON_GetObjectItem(cJSON_GetArrayItem(profile_idx, i), "dia45_2")
            ->valueint;
    p_idx.dia135 =
        cJSON_GetObjectItem(cJSON_GetArrayItem(profile_idx, i), "dia135")
            ->valueint;
    p_idx.dia135_2 =
        cJSON_GetObjectItem(cJSON_GetArrayItem(profile_idx, i), "dia135_2")
            ->valueint;
    p_idx.dia90 =
        cJSON_GetObjectItem(cJSON_GetArrayItem(profile_idx, i), "dia90")
            ->valueint;
    tpp.profile_list.emplace_back(p_idx);
  }
  cJSON_free(root);
  cJSON_free(profile_list);
  cJSON_free(profile_idx);
}
void MainTask::load_slalom_param() {
  turn_param_list.clear();
  for (const auto file_name : tpp.file_list) {
    const auto path = std::string("/spiflash/" + file_name);

    FILE *f = fopen(path.c_str(), "rb");
    if (f == NULL) {
      return;
    }
    printf("%s\n", path.c_str());
    char line_buf[LOG_BUF_SIZE];
    fgets(line_buf, sizeof(line_buf), f);
    fclose(f);

    cJSON *root = cJSON_CreateObject();
    root = cJSON_Parse(line_buf);

    slalom_parameter_t sp;
    sp.map.clear();

    slalom_param2_t sp2;
    for (const auto p : turn_name_list) {
      sp2.v =
          cJSON_GetObjectItem(cJSON_GetObjectItem(root, p.second.c_str()), "v")
              ->valuedouble;
      sp2.ang = cJSON_GetObjectItem(cJSON_GetObjectItem(root, p.second.c_str()),
                                    "ang")
                    ->valuedouble;
      sp2.ang = PI * sp2.ang / 180;

      sp2.rad = cJSON_GetObjectItem(cJSON_GetObjectItem(root, p.second.c_str()),
                                    "rad")
                    ->valuedouble;
      sp2.pow_n = cJSON_GetObjectItem(
                      cJSON_GetObjectItem(root, p.second.c_str()), "pow_n")
                      ->valueint;
      sp2.time = cJSON_GetObjectItem(
                     cJSON_GetObjectItem(root, p.second.c_str()), "time")
                     ->valuedouble;
      sp2.front.right =
          cJSON_GetObjectItem(
              cJSON_GetObjectItem(cJSON_GetObjectItem(root, p.second.c_str()),
                                  "front"),
              "right")
              ->valuedouble;
      sp2.front.left =
          cJSON_GetObjectItem(
              cJSON_GetObjectItem(cJSON_GetObjectItem(root, p.second.c_str()),
                                  "front"),
              "left")
              ->valuedouble;
      sp2.back.right =
          cJSON_GetObjectItem(
              cJSON_GetObjectItem(cJSON_GetObjectItem(root, p.second.c_str()),
                                  "back"),
              "right")
              ->valuedouble;
      sp2.back.left =
          cJSON_GetObjectItem(
              cJSON_GetObjectItem(cJSON_GetObjectItem(root, p.second.c_str()),
                                  "back"),
              "left")
              ->valuedouble;

      sp.map[p.first] = sp2;
    }

    turn_param_list.emplace_back(sp);
    cJSON_free(root);
  }
}

void MainTask::load_param() {
  load_hw_param();
  load_sys_param();
  load_turn_param_profiles();
  load_slalom_param();
}
void MainTask::rx_uart_json() {

  load_param();

  uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
  ui.coin(40);
  while (1) {
    int len = uart_read_bytes(UART_NUM_0, data, (BUF_SIZE - 1),
                              250 / portTICK_RATE_MS);
    uart_write_bytes(UART_NUM_0, (const char *)data, len);
    if (len) {
      data[len] = '\0';
      std::string str = std::string((const char *)data);
      save_json_data(str);
    }
    if (ui.button_state_hold()) {
      break;
    }
  }
  free(data);

  ui.coin(100);
  vTaskDelay(100 / portTICK_PERIOD_MS);
}
void MainTask::task() {
  const TickType_t xDelay = 100 / portTICK_PERIOD_MS;
  pt->motor_disable();
  check_battery();
  // ui.init();

  ui.coin(80);
  rx_uart_json(); // uartでファイルを受け取る
  reset_tgt_data();
  reset_ego_data();

  if (sys.user_mode != 0) {
    if (sys.user_mode == 1) {
      test_sla();
    } else if (sys.user_mode == 2) {
      test_run();
    } else if (sys.user_mode == 3) {
      test_turn();
    } else if (sys.user_mode == 15) {
      echo_sensing_result_with_json();
    }
  } else {
    ui.hello_exia();
    // int mode = select_mode();
  }

  // echo_sensing_result_with_json();

  dump1(); // taskの最終行に配置すること
  while (1) {
    vTaskDelay(xDelay);
  }
}

void MainTask::recieve_data() {
  // Read data from UART.
  const uart_port_t uart_num = UART_NUM_2;
  uint8_t data[128];
  int length = 0;
  ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *)&length));
  length = uart_read_bytes(uart_num, data, length, 100);
}

void MainTask::req_error_reset() {
  tgt_val->pl_req.error_vel_reset = 1;
  tgt_val->pl_req.error_gyro_reset = 1;
  tgt_val->pl_req.time_stamp++;
}

void MainTask::test_run() {
  reset_gyro_ref();
  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();
  lt->start_slalom_log();
  // pt->active_logging(_f);

  ps.v_max = sys.test.v_max;
  ps.v_end = sys.test.end_v;
  ps.dist = sys.test.dist;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;

  mp.go_straight(ps);
  reset_tgt_data();
  reset_ego_data();
  vTaskDelay(250 / portTICK_RATE_MS);
  pt->motor_disable();

  lt->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  lt->save(slalom_log_file);
  ui.coin(120);

  while (1) {
    if (ui.button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui.button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
}

void MainTask::test_turn() {
  TurnDirection rorl = ui.select_direction();

  reset_gyro_ref();
  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();

  // pt->active_logging();
  pr.w_max = sys.test.w_max;
  pr.alpha = sys.test.alpha;
  pr.w_end = 0;
  pr.ang = sys.test.ang * PI / 180;
  pr.RorL = rorl;

  for (int i = 0; i < 4; i++) {
    mp.pivot_turn(pr);
    vTaskDelay(250 / portTICK_RATE_MS);
  }
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();

  pt->inactive_logging();
  ui.coin(120);
  // dump_log();
  lt->dump_log(slalom_log_file);
}

void MainTask::test_sla() {

  int file_idx = sys.test.file_idx;

  if (file_idx >= tpp.file_list_size) {
    ui.error();
    return;
  }

  auto sla_p =
      turn_param_list[file_idx].map[static_cast<TurnType>(sys.test.sla_type)];

  printf("%d %f\n", sla_p.pow_n, sla_p.time);

  TurnDirection rorl = ui.select_direction();

  reset_gyro_ref();

  if (sys.test.suction_active) {
    pt->suction_enable(sys.test.suction_duty);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();

  // pt->active_logging();

  ps.v_max = sla_p.v;
  ps.v_end = sla_p.v;
  ps.dist = sys.test.sla_dist;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  mp.go_straight(ps);

  // pns.ang = sla_p.ang * PI / 180;
  // pns.radius = sla_p.rad;
  // pns.RorL = rorl;
  // pns.v_end = sla_p.v;
  // pns.v_max = sla_p.v;
  // mp.normal_slalom(pns, ps);

  next_motionr_t nm;
  nm.v_max = sla_p.v;
  nm.v_end = sla_p.v;
  nm.accl = sys.test.accl;
  nm.decel = sys.test.decel;
  nm.is_turn = false;

  mp.slalom(sla_p, rorl, nm);

  ps.v_max = sla_p.v;
  ps.v_end =  sys.test.end_v;
  ps.dist = 90;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  mp.go_straight(ps);

  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();

  pt->suction_disable();
  pt->inactive_logging();
  vTaskDelay(500 / portTICK_PERIOD_MS);

  ui.coin(120);

  dump_log();
}

void MainTask::dump_log() {

  while (1) {
    if (ui.button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
  FILE *f = fopen(slalom_log_file.c_str(), "rb");
  if (f == NULL) {
    printf("log_file_error\n");
    return;
  }
  char line_buf[LOG_BUF_SIZE];
  while (fgets(line_buf, sizeof(line_buf), f) != NULL)
    printf("%s\n", line_buf);
  fclose(f);

  while (1) {
    if (ui.button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
}