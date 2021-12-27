
#include "include/planning_task.hpp"

PlanningTask::PlanningTask() {}

PlanningTask::~PlanningTask() {}
void PlanningTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "planning_task", 8192, this, 1,
                          &handle, xCoreID);
}
void PlanningTask::motor_enable() {
  motor_en = true;
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
}
void PlanningTask::suction_enable() {
  suction_en = true;
  mcpwm_start(MCPWM_UNIT_1, MCPWM_TIMER_2);
}
void PlanningTask::motor_disable() {
  motor_en = false;
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
}
void PlanningTask::suction_disable() {
  suction_en = false;
  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
  mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_2); //
}
void PlanningTask::task_entry_point(void *task_instance) {
  static_cast<PlanningTask *>(task_instance)->task();
}
void PlanningTask::set_sensing_entity(sensing_entity_t *_entity) {
  entity_ro = _entity; //
}
void PlanningTask::set_ego_entity(ego_entity_t *_ego) {
  ego = _ego; //
}
void PlanningTask::set_ego_param_entity(ego_param_t *_param) {
  param_ro = _param; //
}
void PlanningTask::set_tgt_entity(tgt_entity_t *_tgt) {
  tgt = _tgt; //
}
void PlanningTask::buzzer(ledc_channel_config_t &buzzer_ch,
                          ledc_timer_config_t &buzzer_timer) {
  int duty = 0;
  if (buzzer_timestamp != tgt->buzzer.timstamp) {
    buzzer_time_cnt = 0;
    buzzer_timestamp = tgt->buzzer.timstamp;
  }
  if (buzzer_time_cnt < tgt->buzzer.time) {
    duty = 50;
    buzzer_time_cnt++;
    buzzer_timer.freq_hz = tgt->buzzer.hz;
  }
  ledc_set_duty(buzzer_ch.speed_mode, buzzer_ch.channel, duty);
  ledc_update_duty(buzzer_ch.speed_mode, buzzer_ch.channel);
}
void PlanningTask::task() {
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  init_gpio();

  ledc_channel_config_t buzzer_ch;
  ledc_timer_config_t buzzer_timer;

  buzzer_ch.channel = (ledc_channel_t)LEDC_CHANNEL_0;
  buzzer_ch.duty = 0;
  buzzer_ch.gpio_num = LED1;
  buzzer_ch.speed_mode = (ledc_mode_t)LEDC_HIGH_SPEED_MODE;
  buzzer_ch.timer_sel = (ledc_timer_t)LEDC_TIMER_0;

  buzzer_timer.duty_resolution = (ledc_timer_bit_t)LEDC_TIMER_10_BIT;
  buzzer_timer.freq_hz = 440;
  buzzer_timer.speed_mode = (ledc_mode_t)LEDC_HIGH_SPEED_MODE; // timer mode
  buzzer_timer.timer_num = (ledc_timer_t)LEDC_TIMER_0;         // timer index
  ledc_channel_config(&buzzer_ch);
  ledc_timer_config(&buzzer_timer);

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, A_PWM);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, B_PWM);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM2A, SUCTION_PWM);
  mcpwm_config_t motor_pwm_conf;
  motor_pwm_conf.frequency = MOTOR_HZ; // PWM周波数= 10kHz,
  motor_pwm_conf.cmpr_a = 0; // デューティサイクルの初期値（0%）
  motor_pwm_conf.cmpr_b = 0; // デューティサイクルの初期値（0%）
  motor_pwm_conf.counter_mode = MCPWM_UP_COUNTER;
  motor_pwm_conf.duty_mode = MCPWM_DUTY_MODE_0; // アクティブハイ
  motor_pwm_conf.cmpr_a = 0; // デューティサイクルの初期値（0%）
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &motor_pwm_conf);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &motor_pwm_conf);
  mcpwm_config_t suction_pwm_conf;
  suction_pwm_conf.frequency = SUCTION_MOTOR_HZ; // PWM周波数= 10kHz,
  suction_pwm_conf.cmpr_a = 0; // デューティサイクルの初期値（0%）
  suction_pwm_conf.cmpr_b = 0; // デューティサイクルの初期値（0%）
  suction_pwm_conf.counter_mode = MCPWM_UP_COUNTER;
  suction_pwm_conf.duty_mode = MCPWM_DUTY_MODE_0; // アクティブハイ
  suction_pwm_conf.cmpr_a = 0; // デューティサイクルの初期値（0%）
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_2, &suction_pwm_conf);
  ego->angle = 0;
  while (1) {
    // 自己位置更新
    update_ego_motion();

    calc_next_tgt_val();
    calc_tgt_duty();

    set_next_duty(tgt_duty.duty_l, tgt_duty.duty_r, tgt_duty.duty_suction);
    buzzer(buzzer_ch, buzzer_timer);
    vTaskDelay(xDelay);
  }
}

void PlanningTask::update_ego_motion() {
  const double dt = param_ro->dt;
  const double tire = param_ro->tire;
  ego->v_r = (double)(PI * tire * entity_ro->encoder.right / 4096.0 / dt / 1);
  ego->v_l = (double)(PI * tire * entity_ro->encoder.left / 4096.0 / dt / 1);
  ego->v_c = (ego->v_l + ego->v_r) / 2;
  ego->rpm.right = 30.0 * ego->v_r / (PI * tire / 2);
  ego->rpm.left = 30.0 * ego->v_l / (PI * tire / 2);

  ego->dist += ego->v_c * dt;
  ego->w = param_ro->gyro_param.gyro_w_gain *
           (entity_ro->gyro.raw - tgt->gyro_zero_p_offset);
  ego->angle += ego->w * dt;
}
void PlanningTask::set_next_duty(const double duty_l, const double duty_r,
                                 const double duty_suction) {
  if (motor_en) {
    if (duty_l >= 0) {
      gpio_set_level(A_CW_CCW, 1);
    } else {
      gpio_set_level(A_CW_CCW, 0);
    }

    if (duty_r >= 0) {
      gpio_set_level(B_CW_CCW, 0);
    } else {
      gpio_set_level(B_CW_CCW, 1);
    }

    double tmp_duty_r = duty_r > 0 ? duty_r : -duty_r;
    double tmp_duty_l = duty_l > 0 ? duty_l : -duty_l;
    // mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, tmp_duty_l);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
    // mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, tmp_duty_r);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
  } else {
    motor_disable();
  }
  if (suction_en) {
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A, ABS(duty_suction));
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
  } else {
    suction_disable();
  }
}

void PlanningTask::init_gpio() {
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
void PlanningTask::calc_next_tgt_val() {
  tgt_val->v += tgt_val->accl * dt; //
}
void PlanningTask::calc_tgt_duty() {

  error_entity.v.error_p = tgt_val->v - ego->v_c;
  error_entity.v.error_i += error_entity.v.error_p;

  error_entity.w.error_p = tgt_val->w - ego->w;
  // error_entity.w.error_p = 0; // tgt_val->w - ego->w;
  error_entity.w.error_i += error_entity.w.error_p;

  // error_entity.v.error_i = 0;

  double duty_c = param_ro->motor_pid.p * error_entity.v.error_p +
                  param_ro->motor_pid.i * error_entity.v.error_i;

  double duty_roll = param_ro->gyro_pid.p * error_entity.w.error_p +
                     param_ro->gyro_pid.i * error_entity.w.error_i;
  // duty_roll = 0;
  tgt_duty.duty_r = duty_c + duty_roll;
  tgt_duty.duty_l = duty_c - duty_roll;

  // tgt_duty.duty_l = 40;
  // tgt_duty.duty_r = 40;

  ego->duty.duty_r = tgt_duty.duty_r;
  ego->duty.duty_l = tgt_duty.duty_l;
  tgt_duty.duty_suction = 0;
}