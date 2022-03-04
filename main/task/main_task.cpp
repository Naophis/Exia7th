#include "include/main_task.hpp"

MainTask::MainTask() {
  ui = std::make_shared<UserInterface>();
  mp = std::make_shared<MotionPlanning>();
  lgc = std::make_shared<MazeSolverBaseLgc>();
  search_ctrl = std::make_shared<SearchController>();
  pc = std::make_shared<PathCreator>();
  mp->set_path_creator(pc);
}

MainTask::~MainTask() {}

void MainTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "main_task", 8192, this, 2, &handle,
                          xCoreID);
}
void MainTask::task_entry_point(void *task_instance) {
  static_cast<MainTask *>(task_instance)->task();
}
void MainTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
  ui->set_sensing_entity(_sensing_result);
  mp->set_sensing_entity(_sensing_result);
  search_ctrl->set_sensing_entity(_sensing_result);
}
void MainTask::set_input_param_entity(std::shared_ptr<input_param_t> &_param) {
  param = _param;
  mp->set_input_param_entity(_param);
  search_ctrl->set_input_param_entity(_param);
}
void MainTask::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
  ui->set_tgt_val(_tgt_val);
  mp->set_tgt_val(_tgt_val);
}
void MainTask::set_planning_task(std::shared_ptr<PlanningTask> &_pt) {
  pt = _pt;
  search_ctrl->set_planning_task(_pt);
  mp->set_planning_task(_pt);
}
void MainTask::set_logging_task(std::shared_ptr<LoggingTask> &_lt) {
  lt = _lt; //
  search_ctrl->set_logging_task(lt);
  mp->set_logging_task(lt);
}
void MainTask::check_battery() {
  vTaskDelay(1500 / portTICK_PERIOD_MS); //他モジュールの起動待ち

  printf("battery= %f\n", sensing_result->ego.battery_raw);
  if (sensing_result->ego.battery_raw > LOW_BATTERY_TH)
    return;
  while (1) {
    ui->music_sync(MUSIC::G5_, 250);
    vTaskDelay(tgt_val->buzzer.time / portTICK_PERIOD_MS);
  }
}

TurnType MainTask::cast_turn_type(std::string str) {
  if (str == "normal")
    return TurnType::Normal;
  if (str == "large")
    return TurnType::Large;
  if (str == "orval")
    return TurnType::Orval;
  if (str == "dia45")
    return TurnType::Dia45;
  if (str == "dia45_2")
    return TurnType::Dia45_2;
  if (str == "dia135")
    return TurnType::Dia135;
  if (str == "dia135_2")
    return TurnType::Dia135_2;
  if (str == "dia90")
    return TurnType::Dia90;
  return TurnType::None;
}
void MainTask::dump1() {

  tgt_val->nmr.motion_type = MotionType::READY;
  tgt_val->nmr.timstamp++;
  while (1) {
    printf("%c[2J", ESC);   /* 画面消去 */
    printf("%c[0;0H", ESC); /* 戦闘戻す*/
    printf("SW1 %d \n", gpio_get_level(SW1));

    printf("gyro: %d\t(%0.3f)\n", sensing_result->gyro.raw,
           tgt_val->gyro_zero_p_offset);
    printf("battery: %0.3f\n", sensing_result->ego.battery_lp);
    printf("encoder: %d, %d\n", sensing_result->encoder.left,
           sensing_result->encoder.right);
    printf("sensor: %d, %d, %d, %d, %d\n", sensing_result->led_sen.left90.raw,
           sensing_result->led_sen.left45.raw,
           sensing_result->led_sen.front.raw,
           sensing_result->led_sen.right45.raw,
           sensing_result->led_sen.right90.raw);
    printf("sensor_dist: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\n",
           sensing_result->ego.left90_dist,  //
           sensing_result->ego.left45_dist,  //
           sensing_result->ego.front_dist,   //
           sensing_result->ego.right45_dist, //
           sensing_result->ego.right90_dist);

    printf("sensor: %0.1f, %0.1f, %0.1f, %0.1f, %0.1f\n",
           param->sen_ref_p.search_exist.left90,
           param->sen_ref_p.search_exist.left45,
           param->sen_ref_p.search_exist.front,
           param->sen_ref_p.search_exist.right45,
           param->sen_ref_p.search_exist.right90);

    printf("ego_v: %0.3f, %0.3f, %0.3f, %0.3f\n", sensing_result->ego.v_l,
           sensing_result->ego.v_c, sensing_result->ego.v_r,
           tgt_val->ego_in.dist);
    printf("calc_v: %0.3f, %0.3f\n", tgt_val->ego_in.v, tgt_val->ego_in.w);

    printf("ego_w: %0.3f, %0.3f, %0.3f, %0.3f deg\n", sensing_result->ego.w_raw,
           sensing_result->ego.w_lp, tgt_val->ego_in.ang,
           tgt_val->ego_in.ang * 180 / PI);
    printf("duty: %0.3f, %0.3f\n", sensing_result->ego.duty.duty_l,
           sensing_result->ego.duty.duty_r);

    if (ui->button_state()) {
      tgt_val->ego_in.ang = tgt_val->ego_in.dist = 0;
    }

    vTaskDelay(xDelay100);
  }
}

void MainTask::dump2() {

  tgt_val->nmr.motion_type = MotionType::READY;
  tgt_val->nmr.timstamp++;
  while (1) {
    printf("%d, %d, %d, %d, %d\n", sensing_result->led_sen.left90.raw,
           sensing_result->led_sen.left45.raw,
           sensing_result->led_sen.front.raw,
           sensing_result->led_sen.right45.raw,
           sensing_result->led_sen.right90.raw);

    if (ui->button_state()) {
      tgt_val->ego_in.ang = tgt_val->ego_in.dist = 0;
    }

    vTaskDelay(xDelay100);
  }
}

int MainTask::select_mode() {
  int mode_num = 0;
  lbit.byte = 0;
  while (1) {
    int res = ui->encoder_operation();
    mode_num += res;
    if (mode_num == -1) {
      mode_num = 15;
    } else if (mode_num == 16) {
      mode_num = (int)(MODE::SEARCH);
    }
    lbit.byte = mode_num + 1;
    ui->LED_bit(lbit.b0, lbit.b1, lbit.b2, lbit.b3, lbit.b4);
    if (ui->button_state_hold()) {
      ui->coin(100);
      break;
    }
    vTaskDelay(xDelay1);
  }
  return mode_num;
}

void MainTask::reset_tgt_data() { mp->reset_tgt_data(); }

void MainTask::reset_ego_data() { mp->reset_ego_data(); }

void MainTask::keep_pivot() {
  reset_tgt_data();
  reset_ego_data();

  mp->reset_gyro_ref_with_check();
  pt->motor_enable();

  req_error_reset();

  mp->keep();
  pt->motor_disable();
}
void MainTask::echo_sensing_result_with_json() {
  mp->reset_gyro_ref_with_check();
  while (1) {
    // entity_to_json(json_instance);
    // printf("%s\n", json_instance.dump().c_str());
    vTaskDelay(xDelay50);
    bool break_btn = ui->button_state_hold();
    if (break_btn) {
      ui->coin(100);
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
  ui->coin(25);
}

void MainTask::load_hw_param() {
  FILE *f = fopen("/spiflash/hardware.txt", "rb");
  if (f == NULL) {
    return;
  }
  char line_buf[LINE_BUF_SIZE];
  fgets(line_buf, sizeof(line_buf), f);
  fclose(f);

  printf("%s\n", line_buf);

  cJSON *root = cJSON_CreateObject(), *motor_pid, *gyro_pid, *gyro_param,
        *kalman_config, *battery_param, *led_param, *angle_pid, *dist_pid,
        *sen_pid, *sen_pid_dia;
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
  param->offset_start_dist =
      cJSON_GetObjectItem(root, "offset_start_dist")->valuedouble;
  param->sakiyomi_time =
      cJSON_GetObjectItem(root, "sakiyomi_time")->valuedouble;
  param->clear_angle = cJSON_GetObjectItem(root, "clear_angle")->valuedouble;

  param->FF_front = cJSON_GetObjectItem(root, "FF_front")->valueint;
  param->FF_roll = cJSON_GetObjectItem(root, "FF_roll")->valueint;
  param->FF_keV = cJSON_GetObjectItem(root, "FF_keV")->valueint;

  motor_pid = cJSON_GetObjectItem(root, "motor_pid");
  param->motor_pid.p = cJSON_GetObjectItem(motor_pid, "p")->valuedouble;
  param->motor_pid.i = cJSON_GetObjectItem(motor_pid, "i")->valuedouble;
  param->motor_pid.d = cJSON_GetObjectItem(motor_pid, "d")->valuedouble;
  param->motor_pid.mode = cJSON_GetObjectItem(motor_pid, "mode")->valueint;

  sen_pid = cJSON_GetObjectItem(root, "sensor_pid");
  param->sensor_pid.p = cJSON_GetObjectItem(sen_pid, "p")->valuedouble;
  param->sensor_pid.i = cJSON_GetObjectItem(sen_pid, "i")->valuedouble;
  param->sensor_pid.d = cJSON_GetObjectItem(sen_pid, "d")->valuedouble;
  param->sensor_pid.mode = cJSON_GetObjectItem(sen_pid, "mode")->valueint;

  sen_pid_dia = cJSON_GetObjectItem(root, "sensor_pid_dia");
  param->sensor_pid_dia.p = cJSON_GetObjectItem(sen_pid_dia, "p")->valuedouble;
  param->sensor_pid_dia.i = cJSON_GetObjectItem(sen_pid_dia, "i")->valuedouble;
  param->sensor_pid_dia.d = cJSON_GetObjectItem(sen_pid_dia, "d")->valuedouble;
  param->sensor_pid_dia.mode =
      cJSON_GetObjectItem(sen_pid_dia, "mode")->valueint;

  dist_pid = cJSON_GetObjectItem(root, "dist_pid");
  param->dist_pid.p = cJSON_GetObjectItem(dist_pid, "p")->valuedouble;
  param->dist_pid.i = cJSON_GetObjectItem(dist_pid, "i")->valuedouble;
  param->dist_pid.d = cJSON_GetObjectItem(dist_pid, "d")->valuedouble;
  param->dist_pid.mode = cJSON_GetObjectItem(dist_pid, "mode")->valueint;

  gyro_pid = cJSON_GetObjectItem(root, "gyro_pid");
  param->gyro_pid.p = cJSON_GetObjectItem(gyro_pid, "p")->valuedouble;
  param->gyro_pid.i = cJSON_GetObjectItem(gyro_pid, "i")->valuedouble;
  param->gyro_pid.d = cJSON_GetObjectItem(gyro_pid, "d")->valuedouble;
  param->gyro_pid.mode = cJSON_GetObjectItem(gyro_pid, "mode")->valueint;

  angle_pid = cJSON_GetObjectItem(root, "angle_pid");
  param->angle_pid.p = cJSON_GetObjectItem(angle_pid, "p")->valuedouble;
  param->angle_pid.i = cJSON_GetObjectItem(angle_pid, "i")->valuedouble;
  param->angle_pid.d = cJSON_GetObjectItem(angle_pid, "d")->valuedouble;
  param->angle_pid.mode = cJSON_GetObjectItem(angle_pid, "mode")->valueint;

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

  kalman_config = cJSON_GetObjectItem(root, "kalman_config");
  param->Kalman_ang = cJSON_GetObjectItem(kalman_config, "q_ang")->valuedouble;
  param->Kalman_bias =
      cJSON_GetObjectItem(kalman_config, "q_bias")->valuedouble;
  param->Kalman_measure =
      cJSON_GetObjectItem(kalman_config, "r_measure")->valuedouble;

  cJSON_free(root);
  cJSON_free(motor_pid);
  cJSON_free(gyro_pid);
  cJSON_free(sen_pid);
  cJSON_free(sen_pid_dia);
  cJSON_free(gyro_param);
  cJSON_free(battery_param);
  cJSON_free(led_param);
  cJSON_free(dist_pid);
  cJSON_free(angle_pid);
  cJSON_free(kalman_config);
}

void MainTask::load_sensor_param() {
  FILE *f = fopen("/spiflash/sensor.txt", "rb");
  if (f == NULL) {
    return;
  }
  char line_buf[LINE_BUF_SIZE];
  fgets(line_buf, sizeof(line_buf), f);
  fclose(f);

  printf("%s\n", line_buf);

  cJSON *root = cJSON_CreateObject(), *normal, *normal_ref, *normal_exist, *dia,
        *dia_ref, *dia_exist, *search, *search_exist, *gain;
  root = cJSON_Parse(line_buf);

  normal = cJSON_GetObjectItem(root, "normal");
  normal_ref = cJSON_GetObjectItem(normal, "ref");
  normal_exist = cJSON_GetObjectItem(normal, "exist");
  param->sen_ref_p.normal.ref.right45 =
      cJSON_GetObjectItem(normal_ref, "right45")->valuedouble;
  param->sen_ref_p.normal.ref.left45 =
      cJSON_GetObjectItem(normal_ref, "left45")->valuedouble;
  param->sen_ref_p.normal.ref.kireme_r =
      cJSON_GetObjectItem(normal_ref, "kireme_r")->valuedouble;
  param->sen_ref_p.normal.ref.kireme_l =
      cJSON_GetObjectItem(normal_ref, "kireme_l")->valuedouble;

  printf("%f,%f\n", param->sen_ref_p.normal.ref.kireme_l,
         param->sen_ref_p.normal.ref.kireme_r);

  param->sen_ref_p.normal.exist.right45 =
      cJSON_GetObjectItem(normal_exist, "right45")->valuedouble;
  param->sen_ref_p.normal.exist.left45 =
      cJSON_GetObjectItem(normal_exist, "left45")->valuedouble;
  param->sen_ref_p.normal.exist.front =
      cJSON_GetObjectItem(normal_exist, "front")->valuedouble;
  param->sen_ref_p.normal.exist.right90 =
      cJSON_GetObjectItem(normal_exist, "right90")->valuedouble;
  param->sen_ref_p.normal.exist.left90 =
      cJSON_GetObjectItem(normal_exist, "left90")->valuedouble;

  dia = cJSON_GetObjectItem(root, "dia");
  dia_ref = cJSON_GetObjectItem(dia, "ref");
  dia_exist = cJSON_GetObjectItem(dia, "exist");
  param->sen_ref_p.dia.ref.right45 =
      cJSON_GetObjectItem(dia_ref, "right45")->valuedouble;
  param->sen_ref_p.dia.ref.left45 =
      cJSON_GetObjectItem(dia_ref, "left45")->valuedouble;

  param->sen_ref_p.dia.exist.right45 =
      cJSON_GetObjectItem(dia_exist, "right45")->valuedouble;
  param->sen_ref_p.dia.exist.left45 =
      cJSON_GetObjectItem(dia_exist, "left45")->valuedouble;

  search = cJSON_GetObjectItem(root, "search");
  search_exist = cJSON_GetObjectItem(search, "exist");
  param->sen_ref_p.search_exist.front =
      cJSON_GetObjectItem(search_exist, "front")->valuedouble;
  param->sen_ref_p.search_exist.right45 =
      cJSON_GetObjectItem(search_exist, "right45")->valuedouble;
  param->sen_ref_p.search_exist.right90 =
      cJSON_GetObjectItem(search_exist, "right90")->valuedouble;
  param->sen_ref_p.search_exist.left45 =
      cJSON_GetObjectItem(search_exist, "left45")->valuedouble;
  param->sen_ref_p.search_exist.left90 =
      cJSON_GetObjectItem(search_exist, "left90")->valuedouble;
  param->sen_ref_p.search_exist.kireme_r =
      cJSON_GetObjectItem(search_exist, "kireme_r")->valuedouble;
  param->sen_ref_p.search_exist.kireme_l =
      cJSON_GetObjectItem(search_exist, "kireme_l")->valuedouble;
  param->sen_ref_p.search_exist.offset_r =
      cJSON_GetObjectItem(search_exist, "offset_r")->valuedouble;
  param->sen_ref_p.search_exist.offset_l =
      cJSON_GetObjectItem(search_exist, "offset_l")->valuedouble;
  param->sen_ref_p.search_exist.front_ctrl_th =
      cJSON_GetObjectItem(search_exist, "front_ctrl_th")->valuedouble;
  param->sen_ref_p.search_exist.front_ctrl =
      cJSON_GetObjectItem(search_exist, "front_ctrl")->valuedouble;

  gain = cJSON_GetObjectItem(root, "gain");
  param->sensor_gain.l90.a =
      cJSON_GetArrayItem(cJSON_GetObjectItem(gain, "L90"), 0)->valuedouble;
  param->sensor_gain.l90.b =
      cJSON_GetArrayItem(cJSON_GetObjectItem(gain, "L90"), 1)->valuedouble;
  param->sensor_gain.l45.a =
      cJSON_GetArrayItem(cJSON_GetObjectItem(gain, "L45"), 0)->valuedouble;
  param->sensor_gain.l45.b =
      cJSON_GetArrayItem(cJSON_GetObjectItem(gain, "L45"), 1)->valuedouble;
  param->sensor_gain.front.a =
      cJSON_GetArrayItem(cJSON_GetObjectItem(gain, "F"), 0)->valuedouble;
  param->sensor_gain.front.b =
      cJSON_GetArrayItem(cJSON_GetObjectItem(gain, "F"), 1)->valuedouble;
  param->sensor_gain.r45.a =
      cJSON_GetArrayItem(cJSON_GetObjectItem(gain, "R45"), 0)->valuedouble;
  param->sensor_gain.r45.b =
      cJSON_GetArrayItem(cJSON_GetObjectItem(gain, "R45"), 1)->valuedouble;
  param->sensor_gain.r90.a =
      cJSON_GetArrayItem(cJSON_GetObjectItem(gain, "R90"), 0)->valuedouble;
  param->sensor_gain.r90.b =
      cJSON_GetArrayItem(cJSON_GetObjectItem(gain, "R90"), 1)->valuedouble;

  cJSON_free(root);
  cJSON_free(normal);
  cJSON_free(normal_ref);
  cJSON_free(normal_exist);
  cJSON_free(dia);
  cJSON_free(dia_ref);
  cJSON_free(dia_exist);
  cJSON_free(search);
  cJSON_free(search_exist);
  cJSON_free(gain);
}

void MainTask::load_sys_param() {
  FILE *f = fopen("/spiflash/system.txt", "rb");
  if (f == NULL) {
    return;
  }
  char line_buf[LINE_BUF_SIZE];
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
  sys.maze_size = cJSON_GetObjectItem(root, "maze_size")->valueint;
  sys.user_mode = cJSON_GetObjectItem(root, "mode")->valueint;
  test = cJSON_GetObjectItem(root, "test");

  sys.test.v_max = cJSON_GetObjectItem(test, "v_max")->valuedouble;
  sys.test.end_v = cJSON_GetObjectItem(test, "end_v")->valuedouble;
  sys.test.accl = cJSON_GetObjectItem(test, "accl")->valuedouble;
  sys.test.decel = cJSON_GetObjectItem(test, "decel")->valuedouble;
  sys.test.dist = cJSON_GetObjectItem(test, "dist")->valuedouble;
  sys.test.w_max = cJSON_GetObjectItem(test, "w_max")->valuedouble;
  // sys.test.w_end = cJSON_GetObjectItem(test, "w_end")->valuedouble;
  sys.test.alpha = cJSON_GetObjectItem(test, "alpha")->valuedouble;
  sys.test.ang = cJSON_GetObjectItem(test, "ang")->valuedouble;
  sys.test.suction_active =
      cJSON_GetObjectItem(test, "suction_active")->valueint;
  sys.test.suction_duty =
      cJSON_GetObjectItem(test, "suction_duty")->valuedouble;
  sys.test.file_idx = cJSON_GetObjectItem(test, "file_idx")->valueint;
  printf("sys.test.file_idx = %d\n", sys.test.file_idx);
  sys.test.sla_type = cJSON_GetObjectItem(test, "sla_type")->valueint;
  sys.test.sla_return = cJSON_GetObjectItem(test, "sla_return")->valueint;
  sys.test.sla_type2 = cJSON_GetObjectItem(test, "sla_type2")->valueint;
  sys.test.sla_dist = cJSON_GetObjectItem(test, "sla_dist")->valuedouble;
  sys.test.turn_times = cJSON_GetObjectItem(test, "turn_times")->valueint;
  sys.test.ignore_opp_sen =
      cJSON_GetObjectItem(test, "ignore_opp_sen")->valueint;
  cJSON_free(root);
  cJSON_free(goals);
  cJSON_free(test);
}

void MainTask::load_turn_param_profiles() {
  FILE *f = fopen("/spiflash/profiles.txt", "rb");
  if (f == NULL)
    return;
  char line_buf[LINE_BUF_SIZE];
  fgets(line_buf, sizeof(line_buf), f);
  fclose(f);
  printf("%s\n", line_buf);

  cJSON *root = cJSON_CreateObject(), *profile_list, *profile_idx;
  root = cJSON_Parse(line_buf);

  tpp.file_list.clear();
  profile_list = cJSON_GetObjectItem(root, "list");
  int profile_list_size = cJSON_GetArraySize(profile_list);
  printf("profile_list\n");
  tpp.file_list_size = 0;
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
  paramset_list.clear();
  for (const auto file_name : tpp.file_list) {
    const auto path = std::string("/spiflash/" + file_name);

    FILE *f = fopen(path.c_str(), "rb");
    if (f == NULL) {
      return;
    }
    char line_buf[LINE_BUF_SIZE];
    printf("%s\n===================\n", path.c_str());
    fgets(line_buf, sizeof(line_buf), f);
    fclose(f);
    printf("%s\n===================\n", line_buf);

    cJSON *root = cJSON_CreateObject();
    root = cJSON_Parse(line_buf);

    sp.map.clear();

    for (const auto p : turn_name_list) {
      if (p.first == TurnType::None) {
        for (const auto p2 : straight_name_list) {
          str_p.v_max = cJSON_GetObjectItem(
                            cJSON_GetObjectItem(
                                cJSON_GetObjectItem(root, p.second.c_str()),
                                p2.second.c_str()),
                            "v_max")
                            ->valuedouble;
          str_p.accl = cJSON_GetObjectItem(
                           cJSON_GetObjectItem(
                               cJSON_GetObjectItem(root, p.second.c_str()),
                               p2.second.c_str()),
                           "accl")
                           ->valuedouble;
          str_p.decel = cJSON_GetObjectItem(
                            cJSON_GetObjectItem(
                                cJSON_GetObjectItem(root, p.second.c_str()),
                                p2.second.c_str()),
                            "decel")
                            ->valuedouble;
          str_p.w_max = cJSON_GetObjectItem(
                            cJSON_GetObjectItem(
                                cJSON_GetObjectItem(root, p.second.c_str()),
                                p2.second.c_str()),
                            "w_max")
                            ->valuedouble;
          str_p.w_end = cJSON_GetObjectItem(
                            cJSON_GetObjectItem(
                                cJSON_GetObjectItem(root, p.second.c_str()),
                                p2.second.c_str()),
                            "w_end")
                            ->valuedouble;
          str_p.alpha = cJSON_GetObjectItem(
                            cJSON_GetObjectItem(
                                cJSON_GetObjectItem(root, p.second.c_str()),
                                p2.second.c_str()),
                            "alpha")
                            ->valuedouble;
          sp.str_map[p2.first] = str_p;
        }
        continue;
      }
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
      sp2.type = cast_turn_type(p.second);
      sp.map[p.first] = sp2;
    }

    paramset_list.emplace_back(sp);
    cJSON_free(root);
  }
}

void MainTask::load_param() {
  if (!ui->button_state_hold()) {
    load_hw_param();
    load_sensor_param();
    load_sys_param();
    load_turn_param_profiles();
    load_slalom_param();
  }
}
void MainTask::rx_uart_json() {

  load_param();

  uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
  ui->coin(40);
  while (1) {
    int len = uart_read_bytes(UART_NUM_0, data, (BUF_SIZE - 1),
                              250 / portTICK_RATE_MS);
    uart_write_bytes(UART_NUM_0, (const char *)data, len);
    if (len) {
      data[len] = '\0';
      std::string str = std::string((const char *)data);
      save_json_data(str);
    }
    if (ui->button_state_hold()) {
      break;
    }
  }
  free(data);

  ui->coin(100);
  vTaskDelay(100 / portTICK_PERIOD_MS);
}
void MainTask::task() {
  mp->set_userinterface(ui);
  search_ctrl->set_userinterface(ui);
  pt->motor_disable();
  check_battery();
  // ui->init();

  ui->coin(80);
  rx_uart_json(); // uartでファイルを受け取る
  reset_tgt_data();
  reset_ego_data();

  if (sys.user_mode != 0) {
    if (sys.user_mode == 1) {
      printf("test_sla\n");
      test_sla();
    } else if (sys.user_mode == 2) {
      printf("test_run\n");
      test_run();
    } else if (sys.user_mode == 3) {
      printf("test_turn\n");
      test_turn();
    } else if (sys.user_mode == 4) {
      printf("test_run_sla\n");
      test_run_sla();
    } else if (sys.user_mode == 5) {
      printf("test_search_sla\n");
      test_search_sla();
    } else if (sys.user_mode == 6) {
      printf("test_search_sla_walloff\n");
      test_search_sla_walloff();
    } else if (sys.user_mode == 7) {
      printf("test_search_front_ctrl\n");
      test_front_ctrl();
    } else if (sys.user_mode == 13) {
      printf("keep_pivot\n");
      keep_pivot();
    } else if (sys.user_mode == 14) {
      printf("echo_sensor_csv\n");
      dump1();
    } else if (sys.user_mode == 15) {
      printf("echo_printf\n");
      dump2();
    } else if (sys.user_mode == 16) {
      lt->dump_log(slalom_log_file);
      while (1) {
        if (ui->button_state_hold())
          break;
        vTaskDelay(10 / portTICK_RATE_MS);
      }
    } else if (sys.user_mode == 17) {
    }
  } else {
    ui->hello_exia();
    lgc->init(sys.maze_size, sys.maze_size * sys.maze_size - 1);
    lgc->set_goal_pos(sys.goals);
    search_ctrl->set_lgc(lgc);
    search_ctrl->set_motion_plannning(mp);
    pc->set_logic(lgc);
    read_maze_data();
    search_ctrl->print_maze();
    while (1) {
      int mode_num = select_mode();
      printf("%d\n", mode_num);
      if (mode_num == 0) {
        search_ctrl->exec(paramset_list[0], SearchMode::Kata);
        save_maze_data(true);
        while (1) {
          if (ui->button_state_hold())
            break;
          vTaskDelay(10 / portTICK_RATE_MS);
        }
        search_ctrl->print_maze();
      } else if (mode_num == 1) {
        search_ctrl->exec(paramset_list[0], SearchMode::Return);
        save_maze_data(true);
        while (1) {
          if (ui->button_state_hold())
            break;
          vTaskDelay(10 / portTICK_RATE_MS);
        }
        search_ctrl->print_maze();
      } else if (mode_num == 2) {
        pc->path_create(true);
        pc->convert_large_path(true);
        // pc->diagonalPath(true, true);
        pc->print_path();

        mp->exec_path_running(paramset_list[0]);

      } else if (mode_num == 3) {
        pc->path_create(true);
        pc->convert_large_path(true);
        // pc->diagonalPath(true, true);
        pc->print_path();

        mp->exec_path_running(paramset_list[1]);

      } else if (mode_num == 4) {
        pc->path_create(true);
        pc->convert_large_path(true);
        // pc->diagonalPath(true, true);
        pc->print_path();

        mp->exec_path_running(paramset_list[2]);

      } else if (mode_num == 14) {
        dump1(); // taskの最終行に配置すること
      } else if (mode_num == 15) {
        save_maze_data(false);
      }
      vTaskDelay(10 / portTICK_RATE_MS);
    }
  }

  // echo_sensing_result_with_json();
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(xDelay10 / portTICK_RATE_MS);
  }
  dump1(); // taskの最終行に配置すること
  while (1) {
    vTaskDelay(xDelay100);
  }
}

void MainTask::recieve_data() {}

void MainTask::req_error_reset() {
  tgt_val->pl_req.error_vel_reset = 1;
  tgt_val->pl_req.error_gyro_reset = 1;
  tgt_val->pl_req.error_ang_reset = 1;
  tgt_val->pl_req.error_dist_reset = 1;
  tgt_val->pl_req.time_stamp++;
}

void MainTask::test_run() {
  mp->reset_gyro_ref_with_check();

  if (sys.test.suction_active) {
    pt->suction_enable(sys.test.suction_duty);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();
  lt->start_slalom_log();

  ps.v_max = sys.test.v_max;
  ps.v_end = 20;
  ps.dist = sys.test.dist - 5;
  // + param->offset_start_dist;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.motion_type = MotionType::STRAIGHT;
  ps.dia_mode = false;

  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = sys.test.end_v;
  ps.dist = 5;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  mp->go_straight(ps);
  vTaskDelay(100 / portTICK_RATE_MS);
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->suction_disable();

  lt->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  mp->coin();
  lt->save(slalom_log_file);
  ui->coin(120);

  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
}

void MainTask::test_run_sla() {
  mp->reset_gyro_ref_with_check();

  if (sys.test.suction_active) {
    pt->suction_enable(sys.test.suction_duty);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();
  lt->start_slalom_log();
  // pt->active_logging(_f);

  ps.v_max = sys.test.v_max;
  ps.v_end = 20;
  ps.dist = 45 + param->offset_start_dist + 90;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.dia_mode = false;

  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = sys.test.end_v;
  ps.dist = 5;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  mp->go_straight(ps);
  vTaskDelay(100 / portTICK_RATE_MS);
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->suction_disable();

  lt->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  lt->save(slalom_log_file);
  ui->coin(120);

  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
}

void MainTask::test_turn() {
  rorl = ui->select_direction();

  mp->reset_gyro_ref_with_check();
  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();

  lt->start_slalom_log();
  // pt->active_logging();
  pr.w_max = sys.test.w_max;
  pr.alpha = sys.test.alpha;
  pr.w_end = 0;
  pr.ang = sys.test.ang * PI / 180;
  pr.RorL = rorl;

  mp->pivot_turn(pr);
  vTaskDelay(100 / portTICK_RATE_MS);
  pt->motor_disable();
  pt->suction_disable();

  lt->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  lt->save(slalom_log_file);
  ui->coin(120);

  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
}

void MainTask::test_sla() {

  int file_idx = sys.test.file_idx;

  if (file_idx >= tpp.file_list_size) {
    printf("%d %d\n", file_idx, tpp.file_list_size);
    ui->error();
    return;
  }

  auto sla_p =
      paramset_list[file_idx].map[static_cast<TurnType>(sys.test.sla_type)];

  printf("slalom params\n");
  printf("v = %f\n", sla_p.v);
  printf("ang = %f\n", sla_p.ang * 180 / PI);
  printf("radius =  %f\n", sla_p.rad);
  printf("time =  %f\n", sla_p.time);
  printf("n = %d\n", sla_p.pow_n);

  rorl = ui->select_direction();
  rorl2 = (rorl == TurnDirection::Right) ? (TurnDirection::Left)
                                         : (TurnDirection::Right);

  float backup_r = param->sen_ref_p.normal.exist.right45;
  float backup_l = param->sen_ref_p.normal.exist.left45;
  // if (sys.test.ignore_opp_sen) {
  //   if (rorl == TurnDirection::Right) {
  //     param->sen_ref_p.normal.exist.right45 = 1;
  //   } else {
  //     param->sen_ref_p.normal.exist.left45 = 1;
  //   }
  // }
  mp->reset_gyro_ref_with_check();

  if (sys.test.suction_active) {
    pt->suction_enable(sys.test.suction_duty);
    vTaskDelay(xDelay500 / portTICK_PERIOD_MS);
  }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();

  lt->start_slalom_log();

  ps.v_max = sla_p.v;
  ps.v_end = sla_p.v;
  ps.dist = sys.test.sla_dist + param->offset_start_dist;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.motion_type = MotionType::STRAIGHT;
  ps.dia_mode = false;

  mp->go_straight(ps);

  nm.v_max = sla_p.v;
  nm.v_end = sla_p.v;
  nm.accl = sys.test.accl;
  nm.decel = sys.test.decel;
  nm.is_turn = false;

  mp->slalom(sla_p, rorl, nm);

  if (sys.test.sla_return > 0) {
    mp->slalom(
        paramset_list[file_idx].map[static_cast<TurnType>(sys.test.sla_type2)],
        rorl2, nm);
  }

  ps.v_max = sla_p.v;
  ps.v_end = sys.test.end_v;
  ps.dist = 90;

  if (sys.test.sla_return == 0) {
    if (static_cast<TurnType>(sys.test.sla_type) == TurnType::Dia45 ||
        static_cast<TurnType>(sys.test.sla_type) == TurnType::Dia135) {
      ps.dist = 45 * ROOT2;
    }
  } else if (sys.test.sla_return == 1) {
    if (static_cast<TurnType>(sys.test.sla_type) == TurnType::Dia90) {
      ps.dist = 45 * ROOT2;
    }
  }
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::NONE;
  mp->go_straight(ps);

  vTaskDelay(100 / portTICK_RATE_MS);
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->suction_disable();
  lt->stop_slalom_log();

  lt->save(slalom_log_file);
  ui->coin(120);

  param->sen_ref_p.normal.exist.right45 = backup_r;
  param->sen_ref_p.normal.exist.left45 = backup_l;
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
}

void MainTask::test_search_sla() {

  int file_idx = sys.test.file_idx;

  if (file_idx >= tpp.file_list_size) {
    ui->error();
    return;
  }

  auto sla_p = paramset_list[file_idx].map[TurnType::Normal];

  rorl = ui->select_direction();
  float backup_r = param->sen_ref_p.normal.exist.right45;
  float backup_l = param->sen_ref_p.normal.exist.left45;
  if (sys.test.ignore_opp_sen) {
    if (rorl == TurnDirection::Right) {
      param->sen_ref_p.normal.exist.right45 = 9999;
    } else {
      param->sen_ref_p.normal.exist.left45 = 9999;
    }
  }
  rorl2 = (rorl == TurnDirection::Right) ? (TurnDirection::Left)
                                         : (TurnDirection::Right);
  mp->reset_gyro_ref_with_check();

  if (sys.test.suction_active) {
    pt->suction_enable(sys.test.suction_duty);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();

  lt->start_slalom_log();

  ps.v_max = sla_p.v;
  ps.v_end = sla_p.v;
  ps.dist = 45 + param->offset_start_dist;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::Straight;

  mp->go_straight(ps);

  nm.v_max = sla_p.v;
  nm.v_end = sla_p.v;
  nm.accl = sys.test.accl;
  nm.decel = sys.test.decel;
  nm.is_turn = false;

  mp->slalom(sla_p, rorl, nm);
  for (int i = 0; i < sys.test.turn_times; i++) {
    mp->slalom(sla_p, rorl2, nm);
  }

  ps.v_max = sla_p.v;
  ps.v_end = 20;
  ps.dist = 45 - 5;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::NONE;
  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = sys.test.end_v;
  ps.dist = 5;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::NONE;
  mp->go_straight(ps);

  vTaskDelay(100 / portTICK_RATE_MS);
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->suction_disable();
  lt->stop_slalom_log();

  lt->save(slalom_log_file);
  ui->coin(120);

  param->sen_ref_p.normal.exist.right45 = backup_r;
  param->sen_ref_p.normal.exist.left45 = backup_l;
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
}

// 探索中の前壁制御
void MainTask::test_front_ctrl() {
  int file_idx = sys.test.file_idx;

  if (file_idx >= tpp.file_list_size) {
    ui->error();
    return;
  }

  auto sla_p = paramset_list[file_idx].map[TurnType::Normal];

  mp->reset_gyro_ref_with_check();

  if (sys.test.suction_active) {
    pt->suction_enable(sys.test.suction_duty);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();

  lt->start_slalom_log();

  ps.v_max = sla_p.v;
  ps.v_end = sla_p.v;
  ps.dist = 45 + 80 + param->offset_start_dist;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::Straight;

  mp->go_straight(ps);

  mp->search_front_ctrl(ps);

  ps.v_max = sla_p.v;
  ps.v_end = 20;
  ps.dist = 45 - 5;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::NONE;
  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = sys.test.end_v;
  ps.dist = 5;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::NONE;
  mp->go_straight(ps);

  vTaskDelay(100 / portTICK_RATE_MS);
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->suction_disable();
  lt->stop_slalom_log();

  lt->save(slalom_log_file);
  ui->coin(120);

  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
}

void MainTask::test_search_sla_walloff() {
  printf("search_walloff_offset= %f, %f\n",
         param->sen_ref_p.search_exist.offset_l,
         param->sen_ref_p.search_exist.offset_r);

  mp->reset_gyro_ref_with_check();

  if (sys.test.suction_active) {
    pt->suction_enable(sys.test.suction_duty);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();

  req_error_reset();
  lt->start_slalom_log();
  // pt->active_logging(_f);

  ps.v_max = sys.test.v_max;
  ps.v_end = sys.test.v_max;
  ps.dist = 45 + param->offset_start_dist;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.wall_off_req = WallOffReq::NONE;
  ps.wall_off_dist_r = 0;
  ps.wall_off_dist_l = 0;
  ps.dia_mode = false;
  mp->go_straight(ps);

  ps.v_max = sys.test.v_max;
  ps.v_end = sys.test.v_max;
  ps.dist = 90;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.wall_off_req = WallOffReq::SEARCH;
  ps.wall_off_dist_r = param->sen_ref_p.search_exist.offset_r;
  ps.wall_off_dist_l = param->sen_ref_p.search_exist.offset_l;
  ps.dia_mode = false;
  mp->go_straight(ps);

  ps.v_max = sys.test.v_max;
  ps.v_end = 20;
  ps.dist = 40;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  ps.wall_off_req = WallOffReq::NONE;
  mp->go_straight(ps);

  ps.v_max = 20;
  ps.v_end = sys.test.end_v;
  ps.dist = 5;
  ps.accl = sys.test.accl;
  ps.decel = sys.test.decel;
  mp->go_straight(ps);

  vTaskDelay(100 / portTICK_RATE_MS);
  pt->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  pt->suction_disable();

  lt->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  lt->save(slalom_log_file);
  ui->coin(120);

  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
}

void MainTask::save_maze_data(bool write) {
  auto *f = fopen(maze_log_file.c_str(), "wb");
  if (f == NULL)
    return;
  if (write) {
    for (const auto d : lgc->map) {
      fprintf(f, "%d,", d);
    }
  } else {
    printf("delete maze data\n");
    fprintf(f, "null");
  }
  fclose(f);
}

void MainTask::read_maze_data() {
  auto *f = fopen(maze_log_file.c_str(), "rb");
  if (f == NULL)
    return;
  char line_buf[LINE_BUF_SIZE];
  while (fgets(line_buf, sizeof(line_buf), f) != NULL)
    printf("%s\n", line_buf);
  fclose(f);
  std::string str = std::string(line_buf);
  if (str == "null")
    return;
  auto map_list = split(str, ',');
  for (int i = 0; i < map_list.size(); i++) {
    lgc->set_native_wall_data(i, stoi(map_list[i]));
  }
  printf("read maze data!!!\n");
  search_ctrl->print_maze();
}