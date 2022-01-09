
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
void PlanningTask::suction_enable(float duty) {
  suction_en = true;
  tgt_duty.duty_suction = duty;
  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
  mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
                 ABS(tgt_duty.duty_suction));
  mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
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
  tgt_duty.duty_suction = 0;
  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
  mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_2); //
}
void PlanningTask::task_entry_point(void *task_instance) {
  static_cast<PlanningTask *>(task_instance)->task();
}

void PlanningTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_entity_ro) {
  entity_ro = _entity_ro;
}
void PlanningTask::set_ego_param_entity(
    std::shared_ptr<ego_param_t> &_param_ro) {
  param_ro = _param_ro;
}
void PlanningTask::set_ego_entity(std::shared_ptr<ego_entity_t> &_ego) {
  ego = _ego;
}
void PlanningTask::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}

void PlanningTask::active_logging(FILE *_f) {
  log_active = true;
  // *f = *_f;
  // log_list.clear();
  log_list2_size = 0;
}
void PlanningTask::inactive_logging() {
  log_active = false;
  // *f = NULL;
  // fclose(f);
}
void PlanningTask::buzzer(ledc_channel_config_t &buzzer_ch,
                          ledc_timer_config_t &buzzer_timer) {
  int duty = 0;
  if (buzzer_timestamp != tgt_val->buzzer.timstamp) {
    buzzer_time_cnt = 0;
    buzzer_timestamp = tgt_val->buzzer.timstamp;
    buzzer_timer.freq_hz = tgt_val->buzzer.hz;
    ledc_channel_config(&buzzer_ch);
    ledc_timer_config(&buzzer_timer);
  }
  if (buzzer_time_cnt < tgt_val->buzzer.time) {
    duty = 50;
    buzzer_time_cnt++;
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
  buzzer_ch.gpio_num = BUZZER;
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

  motor_en = false;
  set_next_duty(0, 0, 0);
  mpc_tgt_calc.initialize();

  while (1) {
    // 自己位置更新
    update_ego_motion();

    mpc_step = 1;

    // 物理量ベース計算
    mpc_tgt_calc.step(&tgt_val->tgt_in, &tgt_val->ego_in, tgt_val->motion_mode,
                      mpc_step, &mpc_next_ego);

    // 算出結果をコピー
    cp_tgt_val();

    pl_req_activate();

    // Duty計算
    calc_tgt_duty();

    check_fail_safe();

    set_next_duty(tgt_duty.duty_l, tgt_duty.duty_r, tgt_duty.duty_suction);

    buzzer(buzzer_ch, buzzer_timer);

    vTaskDelay(xDelay);
  }
}

void PlanningTask::update_ego_motion() {
  const float dt = param_ro->dt;
  const float tire = param_ro->tire;
  // エンコーダ、ジャイロから速度、角速度、距離、角度更新
  ego->v_r = (float)(PI * tire * entity_ro->encoder.right / 4096.0 / dt / 1);
  ego->v_l = (float)(PI * tire * entity_ro->encoder.left / 4096.0 / dt / 1);
  ego->v_c = (ego->v_l + ego->v_r) / 2;

  ego->rpm.right = 30.0 * ego->v_r / (PI * tire / 2);
  ego->rpm.left = 30.0 * ego->v_l / (PI * tire / 2);

  tgt_val->ego_in.dist += ego->v_c * dt;
  ego->w_raw = param_ro->gyro_param.gyro_w_gain_right *
               (entity_ro->gyro.raw - tgt_val->gyro_zero_p_offset);

  ego->w_lp = ego->w_lp * (1 - param_ro->gyro_param.lp_delay) +
              ego->w_raw * param_ro->gyro_param.lp_delay;

  ego->battery_raw = entity_ro->battery.data;

  ego->battery_lp = ego->battery_lp * (1 - param_ro->battery_param.lp_delay) +
                    ego->battery_raw * param_ro->battery_param.lp_delay;

  tgt_val->ego_in.ang += ego->w_lp * dt;

  ego->right90_raw = entity_ro->led_sen.right90.raw;
  ego->right90_lp = ego->right90_lp * (1 - param_ro->led_param.lp_delay) +
                    ego->right90_raw * param_ro->led_param.lp_delay;
  ego->right45_raw = entity_ro->led_sen.right45.raw;
  ego->right45_lp = ego->right45_lp * (1 - param_ro->led_param.lp_delay) +
                    ego->right45_raw * param_ro->led_param.lp_delay;

  ego->front_raw = entity_ro->led_sen.front.raw;
  ego->front_lp = ego->front_lp * (1 - param_ro->led_param.lp_delay) +
                  ego->front_raw * param_ro->led_param.lp_delay;

  ego->left45_raw = entity_ro->led_sen.left45.raw;
  ego->left45_lp = ego->left45_lp * (1 - param_ro->led_param.lp_delay) +
                   ego->left45_raw * param_ro->led_param.lp_delay;
  ego->left90_raw = entity_ro->led_sen.left90.raw;
  ego->left90_lp = ego->left90_lp * (1 - param_ro->led_param.lp_delay) +
                   ego->left90_raw * param_ro->led_param.lp_delay;

  // コピー
  tgt_val->ego_in.slip_point.w = ego->w_lp;
}

void PlanningTask::set_next_duty(float duty_l, float duty_r,
                                 float duty_suction) {
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

    float tmp_duty_r = duty_r > 0 ? duty_r : -duty_r;
    float tmp_duty_l = duty_l > 0 ? duty_l : -duty_l;
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
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A, duty_suction);
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

  io_conf.pin_bit_mask |= 1ULL << BUZZER;

  io_conf.pin_bit_mask |= 1ULL << SUCTION_PWM;

  // 内部プルダウンしない
  io_conf.pull_down_en = (gpio_pulldown_t)0;
  // 内部プルアップしない
  io_conf.pull_up_en = (gpio_pullup_t)0;
  // 設定をセットする
  gpio_config(&io_conf);
  // gpio_set_direction((gpio_num_t)GPIO_OUTPUT_IO_8,
}

void PlanningTask::pl_req_activate() {
  if (tgt_val->pl_req.time_stamp != pid_req_timestamp) {
    if (tgt_val->pl_req.error_gyro_reset == 1) {
      error_entity.v.error_i = 0;
    }
    if (tgt_val->pl_req.error_vel_reset == 1) {
      error_entity.w.error_i = 0;
    }
    if (tgt_val->pl_req.error_led_reset == 1) {
      // error_entity.led.error_i = 0;
    }
    // if (tgt_val->pl_req.log_start == 1) {
    //   log_active = true;
    // }
    // if (tgt_val->pl_req.log_end == 1) {
    //   log_active = false;
    // }
    pid_req_timestamp = tgt_val->pl_req.time_stamp;
  }
}

void PlanningTask::calc_tgt_duty() {

  error_entity.v.error_p = tgt_val->ego_in.v - ego->v_c;
  error_entity.w.error_p = tgt_val->ego_in.w - ego->w_lp;

  error_entity.v.error_i += error_entity.v.error_p;
  error_entity.w.error_i += error_entity.w.error_p;

  float duty_c = param_ro->motor_pid.p * error_entity.v.error_p +
                 param_ro->motor_pid.i * error_entity.v.error_i;

  float duty_roll = param_ro->gyro_pid.p * error_entity.w.error_p +
                    param_ro->gyro_pid.i * error_entity.w.error_i;
  if (!motor_en) {
    duty_c = 0;
    duty_roll = 0;
    error_entity.v.error_i = 0;
    error_entity.w.error_i = 0;
  }
  tgt_duty.duty_r = duty_c + duty_roll;
  tgt_duty.duty_l = duty_c - duty_roll;

  ego->duty.duty_r = tgt_duty.duty_r;
  ego->duty.duty_l = tgt_duty.duty_l;
}

void PlanningTask::cp_tgt_val() {
  tgt_val->ego_in.accl = mpc_next_ego.accl;
  tgt_val->ego_in.alpha = mpc_next_ego.alpha;
  tgt_val->ego_in.pivot_state = mpc_next_ego.pivot_state;
  tgt_val->ego_in.sla_param = mpc_next_ego.sla_param;
  tgt_val->ego_in.state = mpc_next_ego.state;
  tgt_val->ego_in.v = mpc_next_ego.v;
  tgt_val->ego_in.w = mpc_next_ego.w;
  tgt_val->ego_in.sla_param.state = mpc_next_ego.sla_param.state;
  tgt_val->ego_in.sla_param.counter = mpc_next_ego.sla_param.counter;
  tgt_val->ego_in.sla_param.state = mpc_next_ego.sla_param.state;

  tgt_val->ego_in.img_ang = mpc_next_ego.img_ang;
  tgt_val->ego_in.img_dist = mpc_next_ego.img_dist;

  tgt_val->ego_in.slip_point.slip_angle = mpc_next_ego.slip_point.slip_angle;

  tgt_val->ego_in.cnt_delay_accl_ratio = mpc_next_ego.cnt_delay_accl_ratio;
  tgt_val->ego_in.cnt_delay_decel_ratio = mpc_next_ego.cnt_delay_decel_ratio;
}

void PlanningTask::check_fail_safe() {
  bool no_problem = true;
  if (motor_en) {
    if (ABS(ego->duty.duty_l) > 80) {
      fail_safe.invalid_duty_r_cnt++;
      no_problem = true;
    }
  }

  if (no_problem) {
    fail_safe.invalid_duty_r_cnt = 0;
    fail_safe.invalid_duty_l_cnt = 0;
    tgt_val->fss.error = 0;
  } else {
    if (ABS(fail_safe.invalid_duty_r_cnt) > 10 ||
        ABS(fail_safe.invalid_duty_l_cnt) > 10) {
      tgt_val->fss.error = 1;
    }
  }
}