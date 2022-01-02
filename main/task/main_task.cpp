#include "include/main_task.hpp"

MainTask::MainTask() {}

// MainTask::MainTask(const exia_io_t &_exia) : exia_io(_exia) {}
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
}
void MainTask::set_planning_task(PlanningTask *_pt) { //
  pt = _pt;
}
void MainTask::set_tgt_entity(tgt_entity_t *_tgt) {
  tgt = _tgt;
  ui.set_tgt_entity(_tgt);
  mp.set_tgt_entity(_tgt);
}
void MainTask::check_battery() {
  vTaskDelay(1000 / portTICK_PERIOD_MS); //他モジュールの起動待ち

  if (entity_ro->battery.data > LOW_BATTERY_TH)
    return;
  while (1) {
    ui.music_sync(MUSIC::G5, 250);
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

void MainTask::reset_motion_tgt() {
  tgt->motion_tgt.v_max = 0;
  tgt->motion_tgt.accl = 0;
  tgt->motion_tgt.w_max = 0;
  tgt->motion_tgt.alpha = 0;
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
           ego->dist);
    printf("ego_w: %0.3f, %0.3f, %0.3f deg\n", ego->w, ego->angle,
           ego->angle * 180 / PI);
    printf("duty: %0.3f, %0.3f\n", ego->duty.duty_l, ego->duty.duty_r);

    if (gpio_get_level(SW1)) {
      gpio_set_level(A_CW_CCW, 1);
      gpio_set_level(B_CW_CCW, 1);
      gpio_set_level(SUCTION_PWM, 0);
      pt->suction_disable();

    } else {
      gpio_set_level(A_CW_CCW, 0);
      gpio_set_level(B_CW_CCW, 0);
      gpio_set_level(SUCTION_PWM, 1);
      ego->angle = 0;
      ego->dist = 0;
      pt->suction_enable();
      pt->motor_disable();
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
    ui.LED_bit(lbit.b1, lbit.b2, lbit.b3, lbit.b4);
    bool break_btn = ui.button_state_hold();
    if (break_btn) {
      ui.coin(100);
      break;
    }
    vTaskDelay(xDelay);
  }

  return mode_num;
}
void MainTask::keep_pivot() {
  const TickType_t xDelay = 100 / portTICK_PERIOD_MS;
  ui.motion_check();
  pt->motor_enable();

  tgt_val->v = 0;
  tgt_val->accl = 0;
  tgt_val->w = 0;
  tgt_val->alpha = 0;

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
      {"ego", {{"angle", ego->angle}, {"dist", ego->dist}}}};
}
void MainTask::echo_sensing_result_with_json() {
  const TickType_t xDelay = 50 / portTICK_PERIOD_MS;
  while (1) {
    entity_to_json(json_instance);
    printf("%s\n", json_instance.dump().c_str());
    vTaskDelay(xDelay);
  }
}

void MainTask::task() {
  const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

  ui.coin(100);

  tgt_val->v = 0;
  tgt_val->accl = 0;
  tgt_val->w = 0;
  tgt_val->alpha = 0;

  pt->motor_disable();
  check_battery();
  reset_gyro_ref();
  reset_motion_tgt();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  pt->motor_disable();
  tgt_val->v = 0;
  tgt_val->accl = 0;
  tgt_val->w = 0;
  tgt_val->alpha = 0;
  int mode = select_mode();

  ui.motion_check();
  echo_sensing_result_with_json();

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