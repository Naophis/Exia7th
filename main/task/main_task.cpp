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

void MainTask::set_sensing_entity(sensing_result_entity_t *_entity) {
  entity_ro = _entity;
  ui.set_sensing_entity(_entity);
}
void MainTask::set_ego_entity(ego_entity_t *_ego) {
  ego = _ego;
  ui.set_ego_entity(_ego);
  mp.set_ego_entity(_ego);
}
void MainTask::set_planning_task(PlanningTask *_pt) { //
  pt = _pt;
}
void MainTask::set_tgt_entity(tgt_entity_t *_tgt) {
  tgt = _tgt;
  ui.set_tgt_entity(_tgt);
  mp.set_tgt_entity(_tgt);
}
void MainTask::set_tgt_val(motion_tgt_val_t *_tgt) {
  tgt_val = _tgt;
  mp.set_tgt_val(_tgt);
}

void MainTask::check_battery() {
  vTaskDelay(1000 / portTICK_PERIOD_MS); //他モジュールの起動待ち

  if (entity_ro->battery.data > LOW_BATTERY_TH)
    return;
  while (1) {
    ui.music_sync(MUSIC::G5_, 250);
    vTaskDelay(tgt->buzzer.time / portTICK_PERIOD_MS);
  }
}
void MainTask::reset_gyro_ref() {
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  double gyro_raw_data_sum = 0;

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
    printf("battery: %0.3f\n", entity_ro->battery.data);
    printf("encoder: %d, %d\n", entity_ro->encoder.left,
           entity_ro->encoder.right);
    printf("sensor: %d, %d, %d, %d, %d\n", entity_ro->led_sen.left90.raw,
           entity_ro->led_sen.left45.raw, entity_ro->led_sen.front.raw,
           entity_ro->led_sen.right45.raw, entity_ro->led_sen.right90.raw);

    printf("ego_v: %0.3f, %0.3f, %0.3f, %0.3f\n", ego->v_l, ego->v_c, ego->v_r,
           tgt_val->ego_in.dist);
    printf("calc_v: %0.3f, %0.3f\n", tgt_val->ego_in.v, tgt_val->ego_in.w);

    printf("ego_w: %0.3f, %0.3f, %0.3f deg\n", ego->w, tgt_val->ego_in.ang,
           tgt_val->ego_in.ang * 180 / PI);
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
void MainTask::entity_to_json(nlohmann::json &j) {
  j = nlohmann::json{
      {"led_sen",
       {{"right90", entity_ro->led_sen.right90.raw},
        {"right45", entity_ro->led_sen.right45.raw},
        {"front", entity_ro->led_sen.front.raw},
        {"left45", entity_ro->led_sen.left45.raw},
        {"left90", entity_ro->led_sen.left90.raw}}},
      {"gyro", {{"raw", entity_ro->gyro.raw}, {"data", entity_ro->gyro.data}}},
      {"battery",
       {{"raw", entity_ro->battery.raw}, {"data", entity_ro->battery.data}}},
      {"ego",
       {{"angle", tgt_val->ego_in.ang}, {"dist", tgt_val->ego_in.dist}}}};
}
void MainTask::echo_sensing_result_with_json() {
  const TickType_t xDelay = 50 / portTICK_PERIOD_MS;
  while (1) {
    entity_to_json(json_instance);
    printf("%s\n", json_instance.dump().c_str());
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
void MainTask::set_ego_param_entity(ego_param_t *_param) {
  param = _param; //
}

void MainTask::load_hw_param() {
  FILE *f = fopen("/spiflash/hardware.txt", "rb");
  if (f == NULL) {
    return;
  }
  char line[1024];
  fgets(line, sizeof(line), f);
  fclose(f);

  std::string json_str = std::string(line);
  printf("%s\n", json_str.c_str());
  nlohmann::json j = nlohmann::json::parse(json_str);
  param->dt = j["dt"];
  param->tire = j["tire"];
  param->gear_a = j["gear_a"];
  param->gear_b = j["gear_b"];
  param->max_duty = j["max_duty"];
  param->Ke = j["Ke"];
  param->Km = j["Km"];
  param->Resist = j["Resist"];
  param->Mass = j["Mass"];
  param->Lm = j["Lm"];
  param->motor_pid.p = j["motor_pid"]["p"];
  param->motor_pid.i = j["motor_pid"]["i"];
  param->motor_pid.d = j["motor_pid"]["d"];
  param->gyro_pid.p = j["gyro_pid"]["p"];
  param->gyro_pid.i = j["gyro_pid"]["i"];
  param->gyro_pid.d = j["gyro_pid"]["d"];
  param->gyro_param.gyro_w_gain_right = j["gyro_param"]["gyro_w_gain_right"];
  param->gyro_param.gyro_w_gain_left = j["gyro_param"]["gyro_w_gain_left"];
}

void MainTask::load_sys_param() {
  FILE *f = fopen("/spiflash/system.txt", "rb");
  if (f == NULL) {
    return;
  }
  char line[1024];
  fgets(line, sizeof(line), f);
  fclose(f);

  std::string json_str = std::string(line);
  printf("%s\n", json_str.c_str());
  nlohmann::json j = nlohmann::json::parse(json_str);
  sys.goals.clear();
  for (auto ele : j["goals"]) {
    point_t pt;
    pt.x = ele[0];
    pt.y = ele[1];
    sys.goals.emplace_back(pt);
    printf("%u %u\n", pt.x, pt.y);
  }

  sys.user_mode = j["mode"];
  sys.test.v_max = j["test"]["v_max"];
  sys.test.accl = j["test"]["accl"];
  sys.test.decel = j["test"]["decel"];
  sys.test.dist = j["test"]["dist"];
  sys.test.w_max = j["test"]["w_max"];
  sys.test.alpha = j["test"]["alpha"];
  sys.test.ang = j["test"]["ang"];
  sys.test.suction_active = j["test"]["suction_active"];
  sys.test.suction_duty = j["test"]["suction_duty"];
  sys.test.file_idx = j["test"]["file_idx"];
  sys.test.sla_type = j["test"]["sla_type"];
  sys.test.sla_return = j["test"]["sla_return"];
  sys.test.sla_type2 = j["test"]["sla_type2"];
}

void MainTask::load_turn_param_profiles() {
  FILE *f = fopen("/spiflash/profiles.txt", "rb");
  if (f == NULL)
    return;

  char line[1024];
  fgets(line, sizeof(line), f);
  fclose(f);

  std::string json_str = std::string(line);
  printf("%s\n", json_str.c_str());
  nlohmann::json j = nlohmann::json::parse(json_str);
  tpp.file_list.clear();
  printf("profile_list\n");
  for (std::string ele : j["list"]) {
    tpp.file_list.push_back(ele);
    printf("- %s\n", ele.c_str());
    tpp.file_list_size++;
  }
  printf("tpp.file_list.size() = %d\n", tpp.file_list.size());

  tpp.profile_idx_size = j["profile_idx_size"];
  printf("tpp.profile_idx_size= %d\n", tpp.profile_idx_size);

  tpp.profile_list.clear();
  for (auto ele : j["profile_idx"]) {
    profile_idx_t p_idx;
    for (auto it = ele.begin(); it != ele.end(); ++it) {
      std::string key = std::string(it.key());
      int idx = static_cast<int>(it.value());

      // プロファイルの数以上を指定した場合、頭打ちさせる
      if (idx >= tpp.file_list_size)
        idx = tpp.file_list_size - 1;

      if (key == "normal")
        p_idx.normal = idx;
      else if (key == "large")
        p_idx.large = idx;
      else if (key == "orval")
        p_idx.orval = idx;
      else if (key == "dia45")
        p_idx.dia45 = idx;
      else if (key == "dia45_2")
        p_idx.dia45_2 = idx;
      else if (key == "dia135")
        p_idx.dia135 = idx;
      else if (key == "dia135_2")
        p_idx.dia135_2 = idx;
      else if (key == "dia90")
        p_idx.dia90 = idx;
    }
    tpp.profile_list.emplace_back(p_idx);
  }
}
void MainTask::load_slalom_param() {
  char line[2048];
  turn_param_list.clear();
  for (const auto file_name : tpp.file_list) {
    const auto path = std::string("/spiflash/" + file_name);

    FILE *f = fopen(path.c_str(), "rb");
    if (f == NULL) {
      return;
    }
    printf("%s\n", path.c_str());
    fgets(line, sizeof(line), f);
    fclose(f);

    std::string json_str = std::string(line);
    // printf("%s\n", json_str.c_str());
    nlohmann::json j = nlohmann::json::parse(json_str);
    slalom_parameter_t sp;
    sp.map.clear();

    for (const auto p : turn_name_list) {
      slalom_param2_t sp2;
      sp2.v = j[p.second]["v"];
      sp2.ang = j[p.second]["ang"];
      sp2.rad = j[p.second]["rad"];
      sp2.front.right = j[p.second]["front"]["right"];
      sp2.front.left = j[p.second]["front"]["left"];
      sp2.back.right = j[p.second]["back"]["right"];
      sp2.back.left = j[p.second]["back"]["left"];
      sp.map[p.first] = sp2;
    }

    turn_param_list.emplace_back(sp);
  }
}

void MainTask::load_param() {
  load_hw_param(); //
  load_sys_param();
  load_turn_param_profiles();
  load_slalom_param();
}
void MainTask::rx_uart_json() {

  mount_config.max_files = 8;
  mount_config.format_if_mount_failed = true;
  mount_config.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;

  esp_err_t err = esp_vfs_fat_spiflash_mount(base_path, "storage",
                                             &mount_config, &s_wl_handle);
  if (err != ESP_OK) {
    printf("Failed to mount FATFS (%s)\n", esp_err_to_name(err));
    return;
  } else {
    printf("mount OK\n");
  }
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
    }
  } else {
    ui.hello_exia();
    int mode = select_mode();
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

  ps.v_max = sys.test.v_max;
  ps.v_end = 30;
  ps.dist = sys.test.dist;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;

  int res = mp.go_straight(ps);
  reset_tgt_data();
  reset_ego_data();
  vTaskDelay(250 / portTICK_RATE_MS);
  pt->motor_disable();

  ui.coin(120);
}

void MainTask::test_turn() {
  TurnDirection rorl = ui.select_direction();

  reset_gyro_ref();
  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();

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

  ui.coin(120);
}

void MainTask::test_sla() {

  int file_idx = sys.test.file_idx;

  if (file_idx >= tpp.file_list_size) {
    ui.error();
    return;
  }

  const auto sla_p =
      turn_param_list[file_idx].map[static_cast<TurnType>(sys.test.sla_type)];

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

  ps.v_max = sla_p.v;
  ps.v_end = sla_p.v;
  ps.dist = 90;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  mp.go_straight(ps);

  pns.ang = sla_p.ang * PI / 180;
  pns.radius = sla_p.rad;
  pns.RorL = rorl;
  pns.v_end = sla_p.v;
  pns.v_max = sla_p.v;
  mp.normal_slalom(pns, ps);

  ps.v_max = sla_p.v;
  ps.v_end = 30;
  ps.dist = 90;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  mp.go_straight(ps);

  pt->motor_disable();

  pt->suction_disable();
  vTaskDelay(500 / portTICK_PERIOD_MS);

  ui.coin(120);
}