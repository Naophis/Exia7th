
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
  mcpwm_start(MCPWM_UNIT_1, MCPWM_TIMER_2); //
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
  entity = _entity; //
}
void PlanningTask::set_ego_entity(ego_entity_t *_ego) {
  ego = _ego; //
}
void PlanningTask::set_ego_param_entity(ego_param_t *_param) {
  param = _param; //
}
void PlanningTask::task() {
//   init_gpio();
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

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
  while (1) {
    // 自己位置更新
    update_ego_motion();
    float duty_r = 20;
    float duty_l = 20;
    float duty_suction = 20;
    set_next_duty(duty_l, duty_r, duty_suction);
    vTaskDelay(xDelay);
  }
}

void PlanningTask::update_ego_motion() {
  const double dt = param->dt;
  const double tire = param->tire;
  ego->v_r = (double)(PI * tire * entity->encoder.right / 4096.0 / dt / 2);
  ego->v_l = (double)(PI * tire * entity->encoder.left / 4096.0 / dt / 2);
  ego->v_c = (ego->v_l + ego->v_r) / 2;
  ego->rpm.right = 30.0 * ego->v_r / (PI * tire / 2);
  ego->rpm.left = 30.0 * ego->v_l / (PI * tire / 2);

  ego->dist += ego->v_c * dt;
  ego->w = param->gyro_w_gain * entity->gyro.raw;
  ego->angle += ego->w * dt;
}
void PlanningTask::set_next_duty(const double duty_l, const double duty_r,
                                 const double duty_suction) {
  if (motor_en) {
    if (duty_l >= 0)
      gpio_set_level(A_CW_CCW, 1);
    else
      gpio_set_level(A_CW_CCW, 0);

    if (duty_r >= 0)
      gpio_set_level(B_CW_CCW, 0);
    else
      gpio_set_level(B_CW_CCW, 1);

    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_r);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_l);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
  } else {
    motor_disable();
  }
  if (suction_en) {
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A, duty_suction);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
  } else {
    suction_disable();
  }
}
