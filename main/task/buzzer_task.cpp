#include "include/buzzer_task.hpp"

BuzzerTask::BuzzerTask() {}
BuzzerTask::~BuzzerTask() {}

void BuzzerTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "buzzer_task", 8192, this, 4,
                          &handle, xCoreID);
}
void BuzzerTask::task_entry_point(void *task_instance) {
  static_cast<BuzzerTask *>(task_instance)->task();
}

void BuzzerTask::set_sensing_entity(sensing_entity_t *_entity) {
  entity = _entity; //
}
void BuzzerTask::set_ego_entity(ego_entity_t *_ego) {
  ego = _ego; //
}
void BuzzerTask::set_planning_task(PlanningTask *_pt) { //
  pt = _pt;
}
void BuzzerTask::init_gpio() {
  gpio_config_t io_conf;
  // 割り込みをしない
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // 出力モード
  io_conf.mode = GPIO_MODE_OUTPUT;
  // 設定したいピンのビットマスク
  io_conf.pin_bit_mask = 0;
  io_conf.pin_bit_mask |= 1ULL << LED_R90;
  io_conf.pin_bit_mask |= 1ULL << LED_R45;
  io_conf.pin_bit_mask |= 1ULL << LED_F;
  io_conf.pin_bit_mask |= 1ULL << LED_L45;
  io_conf.pin_bit_mask |= 1ULL << LED_L90;
  io_conf.pin_bit_mask |= 1ULL << A_CW_CCW;
  io_conf.pin_bit_mask |= 1ULL << B_CW_CCW;

  io_conf.pin_bit_mask |= 1ULL << LED1;
  io_conf.pin_bit_mask |= 1ULL << LED2;
  io_conf.pin_bit_mask |= 1ULL << LED3;
  io_conf.pin_bit_mask |= 1ULL << LED4;
  io_conf.pin_bit_mask |= 1ULL << LED5;

  io_conf.pin_bit_mask |= 1ULL << SUCTION_PWM;

  // 内部プルダウンしない
  io_conf.pull_down_en = (gpio_pulldown_t)0;
  // 内部プルアップしない
  io_conf.pull_up_en = (gpio_pullup_t)0;
  // 設定をセットする
  gpio_config(&io_conf);
  // gpio_set_direction((gpio_num_t)GPIO_OUTPUT_IO_8,
}

void BuzzerTask::task() {
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

  gpio_set_level(LED1, 1);
  gpio_set_level(LED2, 1);
  gpio_set_level(LED3, 1);
  gpio_set_level(LED4, 1);
  gpio_set_level(LED5, 1);

  ledc_channel_config_t buzzer_ch;
  ledc_timer_config_t buzzer_timer;

  buzzer_ch.channel = (ledc_channel_t)LEDC_CHANNEL_0;
  buzzer_ch.duty = 50;
  buzzer_ch.gpio_num = LED1;
  buzzer_ch.speed_mode = (ledc_mode_t)LEDC_HIGH_SPEED_MODE;
  buzzer_ch.timer_sel = (ledc_timer_t)LEDC_TIMER_0;

  buzzer_timer.duty_resolution = (ledc_timer_bit_t)LEDC_TIMER_10_BIT;
  buzzer_timer.freq_hz = 440 * 2;
  buzzer_timer.speed_mode = (ledc_mode_t)LEDC_HIGH_SPEED_MODE; // timer mode
  buzzer_timer.timer_num = (ledc_timer_t)LEDC_TIMER_0;         // timer index

  ledc_channel_config(&buzzer_ch);
  ledc_timer_config(&buzzer_timer);
  // //   int battery_cnt = 0;
  // //   bool hold = false;
  //   int duty = 50;
  //   ledc_set_duty(buzzer_ch.speed_mode, buzzer_ch.channel, duty);
  //   ledc_update_duty(buzzer_ch.speed_mode, buzzer_ch.channel);

  while (1) {
    // battery_cnt++;
    // if (battery_cnt > 0 && battery_cnt < BATTERY_BUZZER_MAX_CNT) {
    //   duty = 50;
    //   hold = false;
    // } else {
    //   duty = 0;
    // }
    // if (duty == 0) {
    //   ledc_stop(buzzer_ch.speed_mode, buzzer_ch.channel, 0);
    // } else if (!hold) {
    //   ledc_set_duty(buzzer_ch.speed_mode, buzzer_ch.channel, duty);
    //   ledc_update_duty(buzzer_ch.speed_mode, buzzer_ch.channel);
    // }
    // if (duty == 0) {
    //   hold = false;
    // }

    // if (battery_cnt >= BATTERY_BUZZER_MAX_CNT * 2) {
    //   battery_cnt = 0;
    // }
    vTaskDelay(xDelay);
  }
}
