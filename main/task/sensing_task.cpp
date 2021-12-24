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

void SensingTask::set_sensing_entity(sensing_entity_t *_entity) {
  entity = _entity; //
}
void SensingTask::task() {
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
  static const adc_atten_t atten = ADC_ATTEN_DB_11;

  gyro_if.init();
  gyro_if.setup();

  // sensing init
  adc2_config_channel_atten(SEN_R90, atten);
  adc2_config_channel_atten(SEN_R45, atten);
  adc2_config_channel_atten(SEN_F, atten);
  adc2_config_channel_atten(SEN_L45, atten);
  adc2_config_channel_atten(SEN_L90, atten);
  adc2_config_channel_atten(BATTERY, atten);
  encoder_init(PCNT_UNIT_0, ENC_R_A, ENC_R_B);
  encoder_init(PCNT_UNIT_1, ENC_L_A, ENC_L_B);

  while (1) {
    adc2_get_raw(BATTERY, width, &entity->battery.raw);

    // LED_OFF ADC
    adc2_get_raw(SEN_R90, width, &entity->led_sen_before.right90.raw);
    adc2_get_raw(SEN_R45, width, &entity->led_sen_before.right45.raw);
    adc2_get_raw(SEN_F, width, &entity->led_sen_before.front.raw);
    adc2_get_raw(SEN_L45, width, &entity->led_sen_before.left45.raw);
    adc2_get_raw(SEN_L90, width, &entity->led_sen_before.left90.raw);

    // LED_ON ADC
    gpio_set_level(LED_R90, 1);
    gpio_set_level(LED_R45, 1);
    gpio_set_level(LED_F, 1);
    gpio_set_level(LED_L45, 1);
    gpio_set_level(LED_L90, 1);
    for (int i = 0; i < 10000; i++)
      ;
    adc2_get_raw(SEN_R90, width, &entity->led_sen_after.right90.raw);
    adc2_get_raw(SEN_R45, width, &entity->led_sen_after.right45.raw);
    adc2_get_raw(SEN_F, width, &entity->led_sen_after.front.raw);
    adc2_get_raw(SEN_L45, width, &entity->led_sen_after.left45.raw);
    adc2_get_raw(SEN_L90, width, &entity->led_sen_after.left90.raw);

    entity->led_sen.right90.raw =
        entity->led_sen_after.right90.raw - entity->led_sen_before.right90.raw;
    entity->led_sen.right45.raw =
        entity->led_sen_after.right45.raw - entity->led_sen_before.right45.raw;
    entity->led_sen.front.raw =
        entity->led_sen_after.front.raw - entity->led_sen_before.front.raw;
    entity->led_sen.left45.raw =
        entity->led_sen_after.left45.raw - entity->led_sen_before.left45.raw;
    entity->led_sen.left90.raw =
        entity->led_sen_after.left90.raw - entity->led_sen_before.left90.raw;

    gpio_set_level(LED_R90, 0);
    gpio_set_level(LED_R45, 0);
    gpio_set_level(LED_F, 0);
    gpio_set_level(LED_L45, 0);
    gpio_set_level(LED_L90, 0);

    pcnt_get_counter_value(PCNT_UNIT_0, &entity->encoder_raw.right);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_get_counter_value(PCNT_UNIT_1, &entity->encoder_raw.left);
    pcnt_counter_clear(PCNT_UNIT_1);

    entity->gyro.raw = gyro_if.read_gyro_z();
    entity->battery.data = BATTERY_GAIN * 2 * entity->battery.raw / 4096;
    entity->encoder.right = entity->encoder_raw.right;
    entity->encoder.left = -1 * entity->encoder_raw.left; //反転必須
    vTaskDelay(xDelay);
  }
}

void SensingTask::encoder_init(const pcnt_unit_t unit, const gpio_num_t pinA,
                               const gpio_num_t pinB) {
  pcnt_config_t pcnt_config_0 = {
      .pulse_gpio_num = pinA,
      .ctrl_gpio_num = pinB,
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_REVERSE,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DEC,
      .counter_h_lim = ENCODER_H_LIM_VAL,
      .counter_l_lim = ENCODER_L_LIM_VAL,
      .unit = unit,
      .channel = PCNT_CHANNEL_0,
  };
  pcnt_config_t pcnt_config_1 = {
      .pulse_gpio_num = pinB,
      .ctrl_gpio_num = pinA,
      .lctrl_mode = PCNT_MODE_REVERSE,
      .hctrl_mode = PCNT_MODE_KEEP,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DEC,
      .counter_h_lim = ENCODER_H_LIM_VAL,
      .counter_l_lim = ENCODER_L_LIM_VAL,
      .unit = unit,
      .channel = PCNT_CHANNEL_1,
  };

  pcnt_unit_config(&pcnt_config_0);
  pcnt_unit_config(&pcnt_config_1);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);

  pcnt_counter_resume(unit);
}