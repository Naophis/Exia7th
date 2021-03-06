
#include "include/planning_task.hpp"

constexpr int MOTOR_HZ = 200000;
constexpr int SUCTION_MOTOR_HZ = 10000;
PlanningTask::PlanningTask() {}

PlanningTask::~PlanningTask() {}
void PlanningTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "planning_task", 8192 * 2, this, 1,
                          &handle, xCoreID);
}
void PlanningTask::motor_enable() {
  motor_en = true;
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
  tgt_val->nmr.v_max = 0;
  tgt_val->nmr.v_end = 0;
  tgt_val->nmr.accl = 0;
  tgt_val->nmr.decel = 0;
  tgt_val->nmr.dist = 0;
  tgt_val->nmr.motion_mode = RUN_MODE2::NONE_MODE;
  tgt_val->nmr.motion_type = MotionType::NONE;
  tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  tgt_val->nmr.sct = SensorCtrlType::NONE;
  tgt_val->motion_type = MotionType::NONE;
  tgt_val->ego_in.dist = 0;
  tgt_val->ego_in.img_dist = 0;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;

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
  tgt_val->global_pos.ang = 0;
  tgt_val->global_pos.img_ang = 0;
  tgt_val->global_pos.dist = 0;
  tgt_val->global_pos.img_dist = 0;

  tgt_val->nmr.timstamp++;
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
void PlanningTask::motor_disable(bool reset_req) {
  // if (reset_req) {
  motor_en = false;

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
  gpio_set_level(A_PWM, 0);
  gpio_set_level(B_PWM, 0);

  // }
}
void PlanningTask::motor_disable() {
  motor_disable(true); //
  vTaskDelay(1 / portTICK_PERIOD_MS);
}
void PlanningTask::suction_disable() {
  // gpio_set_level(SUCTION_PWM, 0);
  suction_en = false;
  tgt_duty.duty_suction = 0;
  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_B);
  // mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_2); //
  mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_B, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_B,
                      MCPWM_DUTY_MODE_0);
  // gpio_set_level(SUCTION_PWM, 0);
}
void PlanningTask::task_entry_point(void *task_instance) {
  static_cast<PlanningTask *>(task_instance)->task();
}

void PlanningTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
}

void PlanningTask::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param_ro) {
  param_ro = _param_ro;
}

void PlanningTask::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}

void PlanningTask::active_logging(FILE *_f) {
  log_active = true;
  log_list2_size = 0;
}
void PlanningTask::inactive_logging() { log_active = false; }

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

  memset(&buzzer_ch, 0, sizeof(buzzer_ch));
  memset(&buzzer_timer, 0, sizeof(buzzer_timer));
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

  memset(&motor_pwm_conf, 0, sizeof(motor_pwm_conf));
  motor_pwm_conf.frequency = MOTOR_HZ; // PWM?????????= 10kHz,
  motor_pwm_conf.cmpr_a = 0; // ??????????????????????????????????????????0%???
  motor_pwm_conf.cmpr_b = 0; // ??????????????????????????????????????????0%???
  motor_pwm_conf.counter_mode = MCPWM_UP_COUNTER;
  motor_pwm_conf.duty_mode = MCPWM_DUTY_MODE_0; // ?????????????????????
  motor_pwm_conf.cmpr_a = 0; // ??????????????????????????????????????????0%???
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &motor_pwm_conf);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &motor_pwm_conf);

  memset(&suction_pwm_conf, 0, sizeof(suction_pwm_conf));
  suction_pwm_conf.frequency = SUCTION_MOTOR_HZ; // PWM?????????= 10kHz,
  suction_pwm_conf.cmpr_a = 0; // ??????????????????????????????????????????0%???
  suction_pwm_conf.cmpr_b = 0; // ??????????????????????????????????????????0%???
  suction_pwm_conf.counter_mode = MCPWM_UP_COUNTER;
  suction_pwm_conf.duty_mode = MCPWM_DUTY_MODE_0; // ?????????????????????
  suction_pwm_conf.cmpr_a = 0; // ??????????????????????????????????????????0%???
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_2, &suction_pwm_conf);

  motor_en = false;
  set_next_duty(0, 0, 0);
  gpio_set_level(SUCTION_PWM, 0);
  mpc_tgt_calc.initialize();

  while (1) {
    // ??????????????????
    update_ego_motion();
    calc_sensor_dist_all();

    mpc_step = 1;

    cp_request();

    // ????????????????????????
    mpc_tgt_calc.step(&tgt_val->tgt_in, &tgt_val->ego_in, tgt_val->motion_mode,
                      mpc_step, &mpc_next_ego);

    mpc_tgt_calc.step(&tgt_val->tgt_in, &tgt_val->ego_in, tgt_val->motion_mode,
                      param_ro->sakiyomi_time, &mpc_next_ego2);

    // ????????????????????????
    cp_tgt_val();

    pl_req_activate();

    // Duty??????
    calc_tgt_duty();

    check_fail_safe();

    set_next_duty(tgt_duty.duty_l, tgt_duty.duty_r, tgt_duty.duty_suction);

    buzzer(buzzer_ch, buzzer_timer);
    global_msec_timer++;
    vTaskDelay(xDelay);
  }
}

float PlanningTask::calc_sensor_pid() {
  float duty = 0;

  error_entity.sen.error_i += error_entity.sen.error_p;
  error_entity.sen.error_p = check_sen_error();
  if (param_ro->sensor_pid.mode == 1) {
    duty = param_ro->sensor_pid.p * error_entity.sen.error_p +
           param_ro->sensor_pid.i * error_entity.sen.error_i +
           param_ro->sensor_pid.d * error_entity.sen.error_d +
           (error_entity.sen_log.gain_z - error_entity.sen_log.gain_zz) * dt;
    error_entity.sen_log.gain_zz = error_entity.sen_log.gain_z;
    error_entity.sen_log.gain_z = duty;
  } else {
    duty = param_ro->sensor_pid.p * error_entity.sen.error_p +
           param_ro->sensor_pid.i * error_entity.sen.error_i +
           param_ro->sensor_pid.d * error_entity.sen.error_d;
  }

  return duty;
}
float PlanningTask::calc_sensor_pid_dia() {
  float duty = 0;

  error_entity.sen_dia.error_i += error_entity.sen_dia.error_p;
  error_entity.sen_dia.error_p = check_sen_error_dia();
  if (param_ro->sensor_pid_dia.mode == 1) {
    duty =
        param_ro->sensor_pid_dia.p * error_entity.sen_dia.error_p +
        param_ro->sensor_pid_dia.i * error_entity.sen_dia.error_i +
        param_ro->sensor_pid_dia.d * error_entity.sen_dia.error_d +
        (error_entity.sen_log_dia.gain_z - error_entity.sen_log_dia.gain_zz) *
            dt;
    error_entity.sen_log_dia.gain_zz = error_entity.sen_log_dia.gain_z;
    error_entity.sen_log_dia.gain_z = duty;
  } else {
    duty = param_ro->sensor_pid_dia.p * error_entity.sen_dia.error_p +
           param_ro->sensor_pid_dia.i * error_entity.sen_dia.error_i +
           param_ro->sensor_pid_dia.d * error_entity.sen_dia.error_d;
  }

  return duty;
}
float PlanningTask::check_sen_error() {
  float error = 0;
  int check = 0;
  float dist_mod = (int)(tgt_val->ego_in.dist / 90);
  float tmp_dist = tgt_val->ego_in.dist - 90 * dist_mod;

  if (tmp_dist < 10) {
    sensing_result->ego.exist_r_wall = sensing_result->ego.right45_dist <
                                       param_ro->sen_ref_p.search_exist.right45;
    sensing_result->ego.exist_l_wall = sensing_result->ego.left45_dist <
                                       param_ro->sen_ref_p.search_exist.left45;
  }

  //?????????????????????????????????????????????
  if (!(sensing_result->ego.left90_dist <
            param_ro->sen_ref_p.normal.exist.front &&
        sensing_result->ego.right90_dist <
            param_ro->sen_ref_p.normal.exist.front)) {
    if (ABS(sensing_result->ego.right45_dist -
            sensing_result->ego.right45_dist_old) <
        param_ro->sen_ref_p.normal.ref.kireme_r) {
      if ((1 < sensing_result->ego.right45_dist &&
           sensing_result->ego.right45_dist <
               param_ro->sen_ref_p.normal.exist.right45)) {
        error += param_ro->sen_ref_p.normal.ref.right45 -
                 sensing_result->ego.right45_dist;
        check++;
      }
    }
    if (ABS(sensing_result->ego.left45_dist -
            sensing_result->ego.left45_dist_old) <
        param_ro->sen_ref_p.normal.ref.kireme_l) {
      if ((1 < sensing_result->ego.left45_dist &&
           sensing_result->ego.left45_dist <
               param_ro->sen_ref_p.normal.exist.left45)) {
        error -= param_ro->sen_ref_p.normal.ref.left45 -
                 sensing_result->ego.left45_dist;
        check++;
      }
    }
  }
  if (check == 0) {
    error_entity.sen.error_i = 0;
    error_entity.sen_log.gain_zz = 0;
    error_entity.sen_log.gain_z = 0;

    if (!(sensing_result->ego.left90_dist <
              param_ro->sen_ref_p.normal.exist.front &&
          sensing_result->ego.right90_dist <
              param_ro->sen_ref_p.normal.exist.front)) {
      if (sensing_result->ego.right45_dist >
              param_ro->sen_ref_p.normal2.ref.kireme_r &&
          sensing_result->ego.left45_dist >
              param_ro->sen_ref_p.normal2.ref.kireme_l) {
        if ((1 < sensing_result->sen.r45.sensor_dist &&
             sensing_result->sen.r45.sensor_dist <
                 param_ro->sen_ref_p.normal2.exist.right45)) {
          error += param_ro->sen_ref_p.normal2.ref.right45 -
                   sensing_result->sen.r45.sensor_dist;
          check++;
        }
      }
      if (sensing_result->ego.right45_dist >
              param_ro->sen_ref_p.normal2.ref.kireme_r &&
          sensing_result->ego.left45_dist >
              param_ro->sen_ref_p.normal2.ref.kireme_l) {
        if ((1 < sensing_result->sen.l45.sensor_dist &&
             sensing_result->sen.l45.sensor_dist <
                 param_ro->sen_ref_p.normal2.exist.left45)) {
          error -= param_ro->sen_ref_p.normal2.ref.left45 -
                   sensing_result->sen.l45.sensor_dist;
          check++;
        }
      }
      error *= param_ro->sen_ref_p.normal2.exist.front;
    }

  } else {
    // TODO U???????????????????????????????????????
    if (tgt_val->tgt_in.tgt_dist >= param_ro->clear_dist_order) {
      if (!(param_ro->clear_dist_ragne_from <= tmp_dist &&
            tmp_dist <= param_ro->clear_dist_ragne_to)) {
        if ((ABS(tgt_val->ego_in.ang - tgt_val->ego_in.img_ang) * 180 / PI) <
            param_ro->clear_angle) {
          tgt_val->global_pos.ang = tgt_val->global_pos.img_ang;
          error_entity.w.error_i = 0;
          error_entity.w.error_d = 0;
          error_entity.ang.error_i = 0;
          error_entity.ang.error_d = 0;
        }
      } else {
        // error_entity.sen.error_i = 0;
        // error_entity.sen_log.gain_zz = 0;
        // error_entity.sen_log.gain_z = 0;
        // if (check == 2) {
        //   return error;
        // } else if (check == 1) {
        //   return error * 2;
        // }
      }
    }
  }

  if (check == 2) {
    return error;
  } else if (check == 1) {
    return error * 2;
  }
  error_entity.sen.error_i = 0;
  error_entity.sen_log.gain_zz = 0;
  error_entity.sen_log.gain_z = 0;
  return 0;
}
float PlanningTask::check_sen_error_dia() {
  float error = 0;
  int check = 0;

  //?????????????????????????????????????????????
  if (tgt_val->tgt_in.tgt_dist > 45 &&
      (tgt_val->tgt_in.tgt_dist - tgt_val->ego_in.dist) > 40) {
    // if (ABS(sensing_result->ego.right90_dist -
    //         sensing_result->ego.right90_dist_old) <
    //     param_ro->sen_ref_p.dia.ref.kireme_r) {
    if (1 < sensing_result->ego.right90_dist &&
        sensing_result->ego.right90_dist <
            param_ro->sen_ref_p.dia.exist.right90) {
      error += param_ro->sen_ref_p.dia.ref.right90 -
               sensing_result->ego.right90_dist;
      check++;
    }
    // }
    // if (ABS(sensing_result->ego.left90_dist -
    //         sensing_result->ego.left90_dist_old) <
    //     param_ro->sen_ref_p.dia.ref.kireme_l) {
    if (1 < sensing_result->ego.left90_dist &&
        sensing_result->ego.left90_dist <
            param_ro->sen_ref_p.dia.exist.left90) {
      error -=
          param_ro->sen_ref_p.dia.ref.left90 - sensing_result->ego.left90_dist;
      check++;
    }
  }
  // }
  if (check == 0) {
    error_entity.sen_dia.error_i = 0;
    error_entity.sen_log_dia.gain_zz = 0;
    error_entity.sen_log_dia.gain_z = 0;
  } else {
    // TODO U???????????????????????????????????????
    if (tgt_val->tgt_in.tgt_dist >= param_ro->clear_dist_order) {
      if ((ABS(tgt_val->ego_in.ang - tgt_val->ego_in.img_ang) * 180 / PI) <
          param_ro->clear_angle) {
        tgt_val->global_pos.ang = tgt_val->global_pos.img_ang;
        // error_entity.w.error_i = 0;
        // error_entity.w.error_d = 0;
        // error_entity.ang.error_i = 0;
        // error_entity.ang.error_d = 0;
      } else {
        // error_entity.sen.error_i = 0;
        // error_entity.sen_log.gain_zz = 0;
        // error_entity.sen_log.gain_z = 0;
        // if (check == 2) {
        //   return error;
        // } else if (check == 1) {
        //   return error * 2;
        // }
      }
    }
  }

  if (check == 2) {
    return error;
  } else if (check == 1) {
    return error * 2;
  }
  return 0;
}

void PlanningTask::update_ego_motion() {
  const float dt = param_ro->dt;
  const float tire = param_ro->tire;

  if (!motor_en) {
    tgt_val->ego_in.v = 0;
    tgt_val->ego_in.w = 0;
  }

  // ??????????????????????????????????????????????????????????????????????????????
  sensing_result->ego.v_r =
      (float)(PI * tire * sensing_result->encoder.right / 4096.0 / dt / 1);
  sensing_result->ego.v_l =
      (float)(PI * tire * sensing_result->encoder.left / 4096.0 / dt / 1);
  sensing_result->ego.v_c =
      (sensing_result->ego.v_l + sensing_result->ego.v_r) / 2;

  sensing_result->ego.rpm.right =
      30.0 * sensing_result->ego.v_r / (PI * tire / 2);
  sensing_result->ego.rpm.left =
      30.0 * sensing_result->ego.v_l / (PI * tire / 2);

  tgt_val->ego_in.dist += sensing_result->ego.v_c * dt;
  tgt_val->global_pos.dist += sensing_result->ego.v_c * dt;

  if (GY_MODE) {
    sensing_result->gyro.data = 0;
    for (int i = 0; i < GY_DQ_SIZE; i++) {
      sensing_result->gyro.data += sensing_result->gyro_list[i];
    }
    sensing_result->gyro.data /= GY_DQ_SIZE;
  }

  if (GY_MODE) {
    if (tgt_val->motion_dir == MotionDirection::LEFT) {
      sensing_result->ego.w_raw =
          param_ro->gyro_param.gyro_w_gain_left *
          (sensing_result->gyro.data - tgt_val->gyro_zero_p_offset);
    } else {
      sensing_result->ego.w_raw =
          param_ro->gyro_param.gyro_w_gain_right *
          (sensing_result->gyro.data - tgt_val->gyro_zero_p_offset);
    }
  } else {
    if (tgt_val->motion_dir == MotionDirection::LEFT) {
      sensing_result->ego.w_raw =
          param_ro->gyro_param.gyro_w_gain_left *
          (sensing_result->gyro.raw - tgt_val->gyro_zero_p_offset);
    } else {
      sensing_result->ego.w_raw =
          param_ro->gyro_param.gyro_w_gain_right *
          (sensing_result->gyro.raw - tgt_val->gyro_zero_p_offset);
    }
  }

  sensing_result->ego.w_lp =
      sensing_result->ego.w_lp * (1 - param_ro->gyro_param.lp_delay) +
      sensing_result->ego.w_raw * param_ro->gyro_param.lp_delay;

  sensing_result->ego.battery_raw = sensing_result->battery.data;

  sensing_result->ego.battery_lp =
      sensing_result->ego.battery_lp * (1 - param_ro->battery_param.lp_delay) +
      sensing_result->ego.battery_raw * param_ro->battery_param.lp_delay;

  tgt_val->ego_in.ang += sensing_result->ego.w_lp * dt;
  tgt_val->global_pos.ang += sensing_result->ego.w_lp * dt;

  sensing_result->ego.left45_lp_old = sensing_result->ego.left45_lp;
  sensing_result->ego.left90_lp_old = sensing_result->ego.left90_lp;
  sensing_result->ego.front_lp_old = sensing_result->ego.front_lp;
  sensing_result->ego.right45_lp_old = sensing_result->ego.right45_lp;
  sensing_result->ego.right90_lp_old = sensing_result->ego.right90_lp;

  sensing_result->ego.right90_raw = sensing_result->led_sen.right90.raw;
  sensing_result->ego.right90_lp =
      sensing_result->ego.right90_lp * (1 - param_ro->led_param.lp_delay) +
      sensing_result->ego.right90_raw * param_ro->led_param.lp_delay;
  sensing_result->ego.right45_raw = sensing_result->led_sen.right45.raw;
  sensing_result->ego.right45_lp =
      sensing_result->ego.right45_lp * (1 - param_ro->led_param.lp_delay) +
      sensing_result->ego.right45_raw * param_ro->led_param.lp_delay;

  sensing_result->ego.front_raw = sensing_result->led_sen.front.raw;
  sensing_result->ego.front_lp =
      sensing_result->ego.front_lp * (1 - param_ro->led_param.lp_delay) +
      sensing_result->ego.front_raw * param_ro->led_param.lp_delay;

  sensing_result->ego.left45_raw = sensing_result->led_sen.left45.raw;
  sensing_result->ego.left45_lp =
      sensing_result->ego.left45_lp * (1 - param_ro->led_param.lp_delay) +
      sensing_result->ego.left45_raw * param_ro->led_param.lp_delay;
  sensing_result->ego.left90_raw = sensing_result->led_sen.left90.raw;
  sensing_result->ego.left90_lp =
      sensing_result->ego.left90_lp * (1 - param_ro->led_param.lp_delay) +
      sensing_result->ego.left90_raw * param_ro->led_param.lp_delay;

  // ?????????
  tgt_val->ego_in.slip_point.w = sensing_result->ego.w_lp;
}

void PlanningTask::set_next_duty(float duty_l, float duty_r,
                                 float duty_suction) {
  if (motor_en) {
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
    if (duty_r < 0) {
      gpio_set_level(A_CW_CCW, 0);
    } else {
      gpio_set_level(A_CW_CCW, 1);
    }

    if (duty_l < 0) {
      gpio_set_level(B_CW_CCW, 1);
    } else {
      gpio_set_level(B_CW_CCW, 0);
    }

    float tmp_duty_r = duty_r > 0 ? duty_r : -duty_r;
    float tmp_duty_l = duty_l > 0 ? duty_l : -duty_l;
    // mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, tmp_duty_r);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
    // mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, tmp_duty_l);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
  } else {
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
  }
  if (suction_en) {
    // mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
    float duty_suction_in =
        tgt_duty.duty_suction / sensing_result->ego.battery_lp * 100;
    if (duty_suction_in > 100) {
      duty_suction_in = 100;
    }
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A, duty_suction_in);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
  } else {
    // mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
    // mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
    //                     MCPWM_DUTY_MODE_0);
    suction_disable();
  }
}

void PlanningTask::init_gpio() {
  gpio_config_t io_conf;
  // ????????????????????????
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // ???????????????
  io_conf.mode = GPIO_MODE_OUTPUT;
  // ??????????????????????????????????????????
  io_conf.pin_bit_mask = 0;
  io_conf.pin_bit_mask |= 1ULL << LED_R90;
  io_conf.pin_bit_mask |= 1ULL << LED_R45;
  io_conf.pin_bit_mask |= 1ULL << LED_L45;
  io_conf.pin_bit_mask |= 1ULL << LED_L90;
  io_conf.pin_bit_mask |= 1ULL << A_CW_CCW;
  io_conf.pin_bit_mask |= 1ULL << B_CW_CCW;

  io_conf.pin_bit_mask |= 1ULL << LED1;
  io_conf.pin_bit_mask |= 1ULL << LED2;
  io_conf.pin_bit_mask |= 1ULL << LED3;
  io_conf.pin_bit_mask |= 1ULL << LED4;
  io_conf.pin_bit_mask |= 1ULL << LED5;

  // io_conf.pin_bit_mask |= 1ULL << BUZZER;

  // io_conf.pin_bit_mask |= 1ULL << SUCTION_PWM;

  // ??????????????????????????????
  io_conf.pull_down_en = (gpio_pulldown_t)0;
  // ??????????????????????????????
  io_conf.pull_up_en = (gpio_pullup_t)0;
  // ????????????????????????
  gpio_config(&io_conf);
  gpio_set_level(SUCTION_PWM, 0);
  // gpio_set_direction((gpio_num_t)GPIO_OUTPUT_IO_8,
}

void PlanningTask::pl_req_activate() {
  if (tgt_val->pl_req.time_stamp != pid_req_timestamp) {
    if (tgt_val->pl_req.error_gyro_reset == 1) {
      error_entity.v.error_i = 0;
    }
    if (tgt_val->pl_req.error_vel_reset == 1) {
      error_entity.dist.error_i = 0;
    }
    if (tgt_val->pl_req.error_dist_reset == 1) {
      error_entity.w.error_i = 0;
    }
    if (tgt_val->pl_req.error_ang_reset == 1) {
      error_entity.ang.error_i = 0;
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

float PlanningTask::get_feadforward_front(TurnDirection td) {
  // return 0;
  if (param_ro->FF_front == 0)
    return 0;
  const float tread = 38;
  if (td == TurnDirection::Right) {

    return (param_ro->Mass * mpc_next_ego2.accl / 1000 * param_ro->tire / 2000 +
            param_ro->Ke * (mpc_next_ego2.v + tread / 2 * mpc_next_ego2.w) /
                (param_ro->tire / 2) * 30.0 / PI) *
           (param_ro->Resist / param_ro->Km) / 2 /
           (param_ro->gear_a / param_ro->gear_b);
  }
  return (param_ro->Mass * mpc_next_ego2.accl / 1000 * param_ro->tire / 2000 +
          param_ro->Ke * (mpc_next_ego2.v - tread / 2 * mpc_next_ego2.w) /
              (param_ro->tire / 2) * 30.0 / PI) *
         (param_ro->Resist / param_ro->Km) / 2 /
         (param_ro->gear_a / param_ro->gear_b);
}
float PlanningTask::get_feadforward_front() {
  // return 0;
  if (param_ro->FF_front == 0)
    return 0;
  return param_ro->Mass * mpc_next_ego2.accl / 1000 * param_ro->tire / 1000 *
         param_ro->Resist /
         ((param_ro->gear_a / param_ro->gear_b) * param_ro->Km) / 2;
}
float PlanningTask::get_feadforward_roll() {
  // return 0;
  const float tread = 38;
  if (param_ro->FF_roll == 0)
    return 0;
  return param_ro->Lm * mpc_next_ego2.alpha * param_ro->tire * //
         param_ro->Resist /
         ((param_ro->gear_a / param_ro->gear_b) * param_ro->Km) / tread;
}
float PlanningTask::get_rpm_ff_val(TurnDirection td) {
  // return 0;
  if (param_ro->FF_keV == 0)
    return 0;
  if (td == TurnDirection::Right)
    return param_ro->Ke *
           (tgt_val->ego_in.v + param_ro->tread / 2 * tgt_val->ego_in.w) /
           (param_ro->tire / 2) * 30.0 / PI;
  return param_ro->Ke *
         (tgt_val->ego_in.v - param_ro->tread / 2 * tgt_val->ego_in.w) /
         (param_ro->tire / 2) * 30.0 / PI;
}
void PlanningTask::calc_tgt_duty() {

  float duty_sen = 0;
  if (tgt_val->nmr.sct == SensorCtrlType::Straight) {
    duty_sen = calc_sensor_pid();
    error_entity.sen_dia.error_i = 0;
    error_entity.sen_log_dia.gain_zz = 0;
    error_entity.sen_log_dia.gain_z = 0;
  } else if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
    duty_sen = calc_sensor_pid_dia();
    error_entity.sen.error_i = 0;
    error_entity.sen_log.gain_zz = 0;
    error_entity.sen_log.gain_z = 0;
  } else if (tgt_val->nmr.sct == SensorCtrlType::NONE) {
    error_entity.sen.error_i = 0;
    error_entity.sen_log.gain_zz = 0;
    error_entity.sen_log.gain_z = 0;
    error_entity.sen_dia.error_i = 0;
    error_entity.sen_log_dia.gain_zz = 0;
    error_entity.sen_log_dia.gain_z = 0;
  }
  sensing_result->ego.duty.sen = duty_sen;

  error_entity.v.error_d = error_entity.v.error_p;
  error_entity.dist.error_d = error_entity.dist.error_p;

  error_entity.w.error_d = error_entity.w.error_p;
  error_entity.ang.error_d = error_entity.ang.error_p;

  error_entity.v.error_p = tgt_val->ego_in.v - sensing_result->ego.v_c;
  error_entity.w.error_p = tgt_val->ego_in.w - sensing_result->ego.w_lp;

  // tgt_val->global_pos.img_ang += mpc_next_ego.w;
  // tgt_val->global_pos.img_dist += mpc_next_ego.v;

  // error_entity.dist.error_p = tgt_val->ego_in.img_dist -
  // tgt_val->ego_in.dist; error_entity.ang.error_p = tgt_val->ego_in.img_ang
  // - tgt_val->ego_in.ang;

  error_entity.dist.error_p =
      tgt_val->global_pos.img_dist - tgt_val->global_pos.dist;
  error_entity.ang.error_p =
      tgt_val->global_pos.img_ang - tgt_val->global_pos.ang;

  if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    error_entity.v.error_i = error_entity.v.error_d = 0;
    error_entity.w.error_i = error_entity.w.error_d = 0;
    // error_entity.v.error_i = 0;
    // error_entity.w.error_i = 0;
    if (sensing_result->ego.front_dist < 90) {
      error_entity.dist.error_p = sensing_result->ego.front_dist -
                                  param_ro->sen_ref_p.search_exist.front_ctrl;
      error_entity.ang.error_p =
          (sensing_result->ego.right90_dist - sensing_result->ego.left90_dist) /
              2 -
          param_ro->sen_ref_p.search_exist.kireme_r;
      // error_entity.v.error_p = sensing_result->ego.front_dist - 45;
      // error_entity.w.error_p =
      //     (sensing_result->ego.right90_dist -
      //     sensing_result->ego.left90_dist) / 2;
    } else {
      error_entity.dist.error_p = error_entity.dist.error_i =
          error_entity.dist.error_d = 0;
      error_entity.ang.error_p = error_entity.ang.error_i =
          error_entity.ang.error_d = 0;
      // error_entity.v.error_p = error_entity.v.error_i =
      // error_entity.v.error_d =
      //     0;
      // error_entity.w.error_p = error_entity.w.error_i =
      // error_entity.w.error_d =
      //     0;
    }
  }

  error_entity.v.error_d = error_entity.v.error_p - error_entity.v.error_d;
  error_entity.dist.error_d =
      error_entity.dist.error_p - error_entity.dist.error_d;
  error_entity.w.error_d = error_entity.w.error_p - error_entity.w.error_d;
  error_entity.ang.error_d =
      error_entity.ang.error_p - error_entity.ang.error_d;

  error_entity.v.error_i += error_entity.v.error_p;
  error_entity.dist.error_i += error_entity.dist.error_p;
  error_entity.w.error_i += error_entity.w.error_p;
  error_entity.ang.error_i += error_entity.ang.error_p;

  float duty_rpm_r = get_rpm_ff_val(TurnDirection::Right);
  float duty_rpm_l = get_rpm_ff_val(TurnDirection::Left);
  // float duty_rpm_r = get_feadforward_front(TurnDirection::Right);
  // float duty_rpm_l = get_feadforward_front(TurnDirection::Left);
  float duty_ff_front = 0;
  float duty_ff_roll = 0;
  duty_ff_front = get_feadforward_front();
  duty_ff_roll = get_feadforward_roll();

  duty_c = 0;
  duty_c2 = 0;
  duty_roll = 0;
  duty_roll2 = 0;

  if (param_ro->motor_pid.mode == 1) {
    duty_c = param_ro->motor_pid.p * error_entity.v.error_p +
             param_ro->motor_pid.i * error_entity.v.error_i +
             param_ro->motor_pid.d * error_entity.v.error_d +
             (error_entity.v_log.gain_z - error_entity.v_log.gain_zz) * dt;
    error_entity.v_log.gain_zz = error_entity.v_log.gain_z;
    error_entity.v_log.gain_z = duty_c;
  } else {
    duty_c = param_ro->motor_pid.p * error_entity.v.error_p +
             param_ro->motor_pid.i * error_entity.v.error_i +
             param_ro->motor_pid.d * error_entity.v.error_d;
    error_entity.v_log.gain_zz = 0;
    error_entity.v_log.gain_z = 0;
  }
  if (param_ro->gyro_pid.mode == 1) {
    duty_roll = param_ro->gyro_pid.p * error_entity.w.error_p +
                param_ro->gyro_pid.i * error_entity.w.error_i +
                param_ro->gyro_pid.d * error_entity.w.error_d +
                (error_entity.w_log.gain_z - error_entity.w_log.gain_zz) * dt;
    error_entity.w_log.gain_zz = error_entity.w_log.gain_z;
    error_entity.w_log.gain_z = duty_roll;
  } else {
    duty_roll = param_ro->gyro_pid.p * error_entity.w.error_p +
                param_ro->gyro_pid.i * error_entity.w.error_i +
                param_ro->gyro_pid.d * error_entity.w.error_d;
    error_entity.w_log.gain_zz = 0;
    error_entity.w_log.gain_z = 0;
  }

  if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    // duty_roll = 0;
    // duty_c = 0;
    if (param_ro->dist_pid.mode == 1) {
      duty_c2 =
          param_ro->dist_pid.p * error_entity.dist.error_p +
          param_ro->dist_pid.i * error_entity.dist.error_i +
          param_ro->dist_pid.d * error_entity.dist.error_d +
          (error_entity.dist_log.gain_z - error_entity.dist_log.gain_zz) * dt;
      error_entity.dist_log.gain_zz = error_entity.dist_log.gain_z;
      error_entity.dist_log.gain_z = duty_c;
    } else {
      duty_c2 = param_ro->dist_pid.p * error_entity.dist.error_p +
                param_ro->dist_pid.i * error_entity.dist.error_i +
                param_ro->dist_pid.d * error_entity.dist.error_d;
      error_entity.dist_log.gain_zz = 0;
      error_entity.dist_log.gain_z = 0;
    }
    if (param_ro->angle_pid.mode == 1) {
      duty_roll2 =
          param_ro->angle_pid.p * error_entity.ang.error_p +
          param_ro->angle_pid.i * error_entity.ang.error_i +
          param_ro->angle_pid.d * error_entity.ang.error_d +
          (error_entity.ang_log.gain_z - error_entity.ang_log.gain_zz) * dt;
      error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
      error_entity.ang_log.gain_z = duty_roll2;
    } else {
      duty_roll2 = param_ro->angle_pid.p * error_entity.ang.error_p +
                   param_ro->angle_pid.i * error_entity.ang.error_i +
                   param_ro->angle_pid.d * error_entity.ang.error_d;
      error_entity.ang_log.gain_zz = 0;
      error_entity.ang_log.gain_z = 0;
    }
  }

  if (tgt_val->motion_type == MotionType::PIVOT) {
    if (param_ro->angle_pid.mode == 1) {
      duty_roll2 =
          param_ro->angle_pid.p * error_entity.ang.error_p +
          param_ro->angle_pid.i * error_entity.ang.error_i +
          param_ro->angle_pid.d * error_entity.ang.error_d +
          (error_entity.ang_log.gain_z - error_entity.ang_log.gain_zz) * dt;
      error_entity.ang_log.gain_zz = error_entity.ang_log.gain_z;
      error_entity.ang_log.gain_z = duty_roll2;
    } else {
      duty_roll2 = param_ro->angle_pid.p * error_entity.ang.error_p +
                   param_ro->angle_pid.i * error_entity.ang.error_i +
                   param_ro->angle_pid.d * error_entity.ang.error_d;
      error_entity.ang_log.gain_zz = 0;
      error_entity.ang_log.gain_z = 0;
    }
  }

  tgt_duty.duty_r = (duty_c + duty_c2 + duty_roll + duty_roll2 + duty_rpm_r +
                     duty_ff_front + duty_ff_roll + duty_sen) /
                    sensing_result->ego.battery_lp * 100;

  tgt_duty.duty_l = (duty_c + duty_c2 - duty_roll - duty_roll2 + duty_rpm_l +
                     duty_ff_front - duty_ff_roll - duty_sen) /
                    sensing_result->ego.battery_lp * 100;
  const auto max_duty = param_ro->sen_ref_p.search_exist.offset_l;
  if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    if (tgt_duty.duty_r > max_duty) {
      tgt_duty.duty_r = max_duty;
    } else if (tgt_duty.duty_r < -max_duty) {
      tgt_duty.duty_r = -max_duty;
    }
    if (tgt_duty.duty_l > max_duty) {
      tgt_duty.duty_l = max_duty;
    } else if (tgt_duty.duty_l < -max_duty) {
      tgt_duty.duty_l = -max_duty;
    }
  }
  if (tgt_val->motion_type == MotionType::NONE) {
    tgt_duty.duty_l = tgt_duty.duty_r = 0;
  }
  // printf("%0.3f, %0.3f %0.3f\n", duty_rpm_l, duty_rpm_r,
  // sensing_result->ego.battery_lp );
  if (!motor_en) {
    duty_c = 0;
    duty_c2 = 0;
    duty_roll = 0;
    duty_roll2 = 0;
    duty_sen = 0;
    error_entity.v.error_i = 0;
    error_entity.dist.error_i = 0;
    error_entity.w.error_i = 0;
    error_entity.ang.error_i = 0;
    error_entity.sen.error_i = 0;
    error_entity.sen_dia.error_i = 0;
    tgt_duty.duty_r = tgt_duty.duty_l = 0;
    duty_ff_front = duty_ff_roll = 0;
    duty_rpm_r = duty_rpm_l = 0;
    error_entity.v_log.gain_zz = 0;
    error_entity.v_log.gain_z = 0;
    error_entity.dist_log.gain_zz = 0;
    error_entity.dist_log.gain_z = 0;
    error_entity.w_log.gain_zz = 0;
    error_entity.w_log.gain_z = 0;
    error_entity.ang_log.gain_zz = 0;
    error_entity.ang_log.gain_z = 0;
    error_entity.sen_log.gain_z = 0;
    error_entity.sen_log.gain_zz = 0;
    tgt_val->global_pos.ang = 0;
    tgt_val->global_pos.img_ang = 0;
    tgt_val->global_pos.dist = 0;
    tgt_val->global_pos.img_dist = 0;
  }
  sensing_result->ego.duty.duty_r = tgt_duty.duty_r;
  sensing_result->ego.duty.duty_l = tgt_duty.duty_l;
  sensing_result->ego.ff_duty.front = duty_ff_front;
  sensing_result->ego.ff_duty.roll = duty_ff_roll;
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

  tgt_val->global_pos.img_ang += mpc_next_ego.w * dt;
  tgt_val->global_pos.img_dist += mpc_next_ego.v * dt;

  tgt_val->ego_in.slip_point.slip_angle = mpc_next_ego.slip_point.slip_angle;

  tgt_val->ego_in.cnt_delay_accl_ratio = mpc_next_ego.cnt_delay_accl_ratio;
  tgt_val->ego_in.cnt_delay_decel_ratio = mpc_next_ego.cnt_delay_decel_ratio;
}

void PlanningTask::check_fail_safe() {
  bool no_problem = true;
  if (motor_en) {
    if (ABS(sensing_result->ego.duty.duty_r) > 100) {
      fail_safe.invalid_duty_r_cnt++;
      no_problem = false;
    }
    if (ABS(sensing_result->ego.duty.duty_l) > 100) {
      fail_safe.invalid_duty_l_cnt++;
      no_problem = false;
    }

    if (ABS(tgt_val->ego_in.v - sensing_result->ego.v_c) > 50) {
      fail_safe.invalid_v_cnt++;
      no_problem = false;
    }
    if (ABS(tgt_val->ego_in.w - sensing_result->ego.w_lp) > 5) {
      fail_safe.invalid_w_cnt++;
      no_problem = false;
    }
  }

  if (no_problem) {
    fail_safe.invalid_duty_r_cnt = 0;
    fail_safe.invalid_duty_l_cnt = 0;
    fail_safe.invalid_v_cnt = 0;
    fail_safe.invalid_w_cnt = 0;
    tgt_val->fss.error = 0;
  } else {
    if (ABS(fail_safe.invalid_duty_r_cnt) > 100 ||
        ABS(fail_safe.invalid_duty_l_cnt) > 100 ||
        ABS(fail_safe.invalid_v_cnt) > 100 ||
        ABS(fail_safe.invalid_w_cnt) > 50) {
      tgt_val->fss.error = 1;
    }
  }
}

void PlanningTask::cp_request() {
  if (motion_req_timestamp == tgt_val->nmr.timstamp) {
    return;
  }
  const float dt = param_ro->dt;
  motion_req_timestamp = tgt_val->nmr.timstamp;

  tgt_val->tgt_in.v_max = tgt_val->nmr.v_max;
  tgt_val->tgt_in.end_v = tgt_val->nmr.v_end;
  tgt_val->tgt_in.accl = tgt_val->nmr.accl;
  tgt_val->tgt_in.decel = tgt_val->nmr.decel;
  tgt_val->tgt_in.w_max = tgt_val->nmr.w_max;
  tgt_val->tgt_in.end_w = tgt_val->nmr.w_end;
  tgt_val->tgt_in.alpha = tgt_val->nmr.alpha;

  tgt_val->tgt_in.tgt_dist = tgt_val->nmr.dist;
  tgt_val->tgt_in.tgt_angle = tgt_val->nmr.ang;

  tgt_val->motion_mode = (int)(tgt_val->nmr.motion_mode);
  tgt_val->motion_type = tgt_val->nmr.motion_type;

  tgt_val->ego_in.sla_param.base_alpha = tgt_val->nmr.sla_alpha;
  tgt_val->ego_in.sla_param.base_time = tgt_val->nmr.sla_time;
  tgt_val->ego_in.sla_param.limit_time_count = tgt_val->nmr.sla_time * 2 / dt;
  tgt_val->ego_in.sla_param.pow_n = tgt_val->nmr.sla_pow_n;

  tgt_val->ego_in.state = 0;
  tgt_val->ego_in.pivot_state = 0;

  if (!(tgt_val->motion_type == MotionType::NONE ||
        tgt_val->motion_type == MotionType::STRAIGHT ||
        tgt_val->motion_type == MotionType::PIVOT_PRE ||
        tgt_val->motion_type == MotionType::PIVOT_AFTER ||
        tgt_val->motion_type == MotionType::READY ||
        tgt_val->motion_type == MotionType::WALL_OFF)) {
    tgt_val->ego_in.ang -= tgt_val->ego_in.img_ang;
    tgt_val->ego_in.img_ang = 0;
  }

  if (tgt_val->motion_type == MotionType::NONE ||
      tgt_val->motion_type == MotionType::READY) {
    tgt_val->global_pos.ang -= tgt_val->global_pos.img_ang;
    tgt_val->global_pos.img_ang = 0;

    tgt_val->global_pos.dist -= tgt_val->global_pos.img_dist;
    tgt_val->global_pos.img_dist = 0;
  }

  if (tgt_val->tgt_in.tgt_angle != 0) {
    tgt_val->ego_in.img_ang -= tgt_val->ego_in.ang;
    tgt_val->ego_in.ang = 0;
  }
  if (tgt_val->tgt_in.tgt_dist != 0) {
    tgt_val->ego_in.img_dist -= tgt_val->ego_in.dist;
    tgt_val->ego_in.dist = 0;
  }
  tgt_val->ego_in.sla_param.counter = 1;
  tgt_val->ego_in.sla_param.state = 0;

  tgt_val->motion_dir = tgt_val->nmr.motion_dir;
  tgt_val->dia_mode = tgt_val->nmr.dia_mode;
}
float PlanningTask::calc_sensor(float data, float a, float b) {
  auto res = a / std::log(data) - b;
  if (res < 5 || res > 180)
    return 180;
  return res;
}

void PlanningTask::calc_sensor_dist_all() {
  if (!(tgt_val->motion_type == MotionType::NONE ||
        tgt_val->motion_type == MotionType::PIVOT)) {

    sensing_result->ego.left90_dist_old = sensing_result->ego.left90_dist;
    sensing_result->ego.left45_dist_old = sensing_result->ego.left45_dist;
    sensing_result->ego.front_dist_old = sensing_result->ego.front_dist;
    sensing_result->ego.right45_dist_old = sensing_result->ego.right45_dist;
    sensing_result->ego.right90_dist_old = sensing_result->ego.right90_dist;

    sensing_result->ego.left90_dist =
        calc_sensor(sensing_result->ego.left90_lp, param_ro->sensor_gain.l90.a,
                    param_ro->sensor_gain.l90.b);
    sensing_result->ego.left45_dist =
        calc_sensor(sensing_result->ego.left45_lp, param_ro->sensor_gain.l45.a,
                    param_ro->sensor_gain.l45.b);
    sensing_result->ego.right45_dist =
        calc_sensor(sensing_result->ego.right45_lp, param_ro->sensor_gain.r45.a,
                    param_ro->sensor_gain.r45.b);
    sensing_result->ego.right90_dist =
        calc_sensor(sensing_result->ego.right90_lp, param_ro->sensor_gain.r90.a,
                    param_ro->sensor_gain.r90.b);
    sensing_result->ego.front_dist =
        (sensing_result->ego.left90_dist + sensing_result->ego.right90_dist) /
        2;
  } else {
    sensing_result->ego.left90_dist        //
        = sensing_result->ego.left45_dist  //
        = sensing_result->ego.front_dist   //
        = sensing_result->ego.right45_dist //
        = sensing_result->ego.right90_dist = 180;
  }
  // ?????????????????????????????????????????????????????????
  calc_sensor_dist_diff();
}

void PlanningTask::calc_sensor_dist_diff() {
  if (sensing_result->sen.l45.sensor_dist > sensing_result->ego.left45_dist) {
    sensing_result->sen.l45.sensor_dist = sensing_result->ego.left45_dist;
    sensing_result->sen.l45.global_run_dist = tgt_val->global_pos.dist;
  } else {
    if (((tgt_val->global_pos.dist - sensing_result->sen.l45.global_run_dist) >
         param_ro->wall_off_hold_dist) &&
        sensing_result->ego.left45_dist <
            param_ro->sen_ref_p.normal2.exist.left90) {
      sensing_result->sen.l45.sensor_dist = sensing_result->ego.left45_dist;
    }
  }

  if (sensing_result->sen.r45.sensor_dist > sensing_result->ego.right45_dist) {
    sensing_result->sen.r45.sensor_dist = sensing_result->ego.right45_dist;
    sensing_result->sen.r45.global_run_dist = tgt_val->global_pos.dist;
  } else {
    if (((tgt_val->global_pos.dist - sensing_result->sen.r45.global_run_dist) >
         param_ro->wall_off_hold_dist) &&
        sensing_result->ego.right45_dist <
            param_ro->sen_ref_p.normal2.exist.right90) {
      sensing_result->sen.r45.sensor_dist = sensing_result->ego.right45_dist;
    }
  }
  sen_log.r45_dist = sensing_result->sen.r45.sensor_dist;
  sen_log.l45_dist = sensing_result->sen.r45.sensor_dist;
  sen_log.global_run_dist = tgt_val->global_pos.dist;
  // sensing_result->sen_dist_log.list.push_back(sen_log);
  // if (sensing_result->sen_dist_log.list.size() > param_ro->sen_log_size) {
  //   sensing_result->sen_dist_log.list.pop_front();
  // }
}