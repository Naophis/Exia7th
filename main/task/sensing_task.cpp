#include "include/sensing_task.hpp"

SensingTask::SensingTask() {}

SensingTask::~SensingTask() {}

void SensingTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "sensing_task", 8192, this, 2,
                          &handle, xCoreID);
}

void SensingTask::task_entry_point(void *task_instance) {
  static_cast<SensingTask *>(task_instance)->task();
}

void SensingTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
}

void SensingTask::task() {
  // timer_init_grp0_timer0();
  gyro_if.init();
  gyro_if.setup();
  ready = true;

  // sensing init
  adc2_config_channel_atten(SEN_R90, atten);
  adc2_config_channel_atten(SEN_R45, atten);
  adc2_config_channel_atten(SEN_L45, atten);
  adc2_config_channel_atten(SEN_L90, atten);
  adc2_config_channel_atten(BATTERY, atten);
  encoder_init(PCNT_UNIT_0, ENC_R_A, ENC_R_B);
  encoder_init(PCNT_UNIT_1, ENC_L_A, ENC_L_B);

  while (1) {
    adc2_get_raw(BATTERY, width, &sensing_result->battery.raw);

    // LED_OFF ADC
    adc2_get_raw(SEN_R90, width, &sensing_result->led_sen_before.right90.raw);
    adc2_get_raw(SEN_R45, width, &sensing_result->led_sen_before.right45.raw);
    adc2_get_raw(SEN_L45, width, &sensing_result->led_sen_before.left45.raw);
    adc2_get_raw(SEN_L90, width, &sensing_result->led_sen_before.left90.raw);

    // 超信地旋回中は発光をサボる
    if (!(tgt_val->motion_type == MotionType::NONE ||
          tgt_val->motion_type == MotionType::PIVOT)) {
    }
    gpio_set_level(LED_R90, 1);
    gpio_set_level(LED_R45, 1);
    gpio_set_level(LED_L45, 1);
    gpio_set_level(LED_L90, 1);
    for (int i = 0; i < 10000; i++)
      ;
    adc2_get_raw(SEN_R90, width, &sensing_result->led_sen_after.right90.raw);
    adc2_get_raw(SEN_R45, width, &sensing_result->led_sen_after.right45.raw);
    adc2_get_raw(SEN_L45, width, &sensing_result->led_sen_after.left45.raw);
    adc2_get_raw(SEN_L90, width, &sensing_result->led_sen_after.left90.raw);

    sensing_result->led_sen.right90.raw =
        std::max(sensing_result->led_sen_after.right90.raw -
                     sensing_result->led_sen_before.right90.raw,
                 0);
    sensing_result->led_sen.right45.raw =
        std::max(sensing_result->led_sen_after.right45.raw -
                     sensing_result->led_sen_before.right45.raw,
                 0);
    // sensing_result->led_sen.front.raw =
    //     std::max(sensing_result->led_sen_after.front.raw -
    //                  sensing_result->led_sen_before.front.raw,
    //              0);
    sensing_result->led_sen.left45.raw =
        std::max(sensing_result->led_sen_after.left45.raw -
                     sensing_result->led_sen_before.left45.raw,
                 0);
    sensing_result->led_sen.left90.raw =
        std::max(sensing_result->led_sen_after.left90.raw -
                     sensing_result->led_sen_before.left90.raw,
                 0);
    sensing_result->led_sen.front.raw = (sensing_result->led_sen.left90.raw +
                                         sensing_result->led_sen.right90.raw) /
                                        2;
    gpio_set_level(LED_R90, 0);
    gpio_set_level(LED_R45, 0);
    gpio_set_level(LED_L45, 0);
    gpio_set_level(LED_L90, 0);

    pcnt_get_counter_value(PCNT_UNIT_0, &sensing_result->encoder_raw.right);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_get_counter_value(PCNT_UNIT_1, &sensing_result->encoder_raw.left);
    pcnt_counter_clear(PCNT_UNIT_1);

    if (GY_MODE == 0) {
      sensing_result->gyro.raw = gyro_if.read_gyro_z();
    } else {
      if (gyro_q.size() == GY_DQ_SIZE) {
        sensing_result->gyro.raw = gyro_q[0];
        for (int i = 0; i < GY_DQ_SIZE; i++) {
          sensing_result->gyro_list[i] = gyro_q[i];
        }
      } else {
        sensing_result->gyro.raw = 0;
      }
    }

    sensing_result->battery.data =
        BATTERY_GAIN * 2 * sensing_result->battery.raw / 4096;
    sensing_result->encoder.right = -sensing_result->encoder_raw.right;
    sensing_result->encoder.left = -sensing_result->encoder_raw.left; //反転必須
    vTaskDelay(xDelay);
  }
}
void SensingTask::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}
void SensingTask::encoder_init(const pcnt_unit_t unit, const gpio_num_t pinA,
                               const gpio_num_t pinB) {
  pcnt_config_0.pulse_gpio_num = pinA;
  pcnt_config_0.ctrl_gpio_num = pinB;
  pcnt_config_0.unit = unit;

  pcnt_config_1.pulse_gpio_num = pinB;
  pcnt_config_1.ctrl_gpio_num = pinA;
  pcnt_config_1.unit = unit;

  pcnt_unit_config(&pcnt_config_0);
  pcnt_unit_config(&pcnt_config_1);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);

  pcnt_counter_resume(unit);
}