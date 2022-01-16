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

void IRAM_ATTR SensingTask::isr_entry_point(void *task_instance) {
  static_cast<SensingTask *>(task_instance)->timer_isr();
}

void SensingTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
}

void IRAM_ATTR SensingTask::timer_isr() {
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
  if (itr_state) {
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 2500); // 250 nsec
    itr_state = false;
    c++;
  } else {
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 7500); // 1000 nsec
    itr_state = true;
    d++;
  }
}
void IRAM_ATTR SensingTask::timerCallback(void *arg) {
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
  ets_printf("Hey:\n");
}
void SensingTask::timer_init_grp0_timer0() {
  timer_config_t config;
  config.alarm_en = TIMER_ALARM_EN;
  config.counter_en = TIMER_PAUSE;
  config.clk_src = TIMER_SRC_CLK_APB;
  config.auto_reload = TIMER_AUTORELOAD_EN;
  config.counter_dir = TIMER_COUNT_UP;
  config.divider = 8; // 80Mhz / divider
  timer_init(TIMER_GROUP_0, TIMER_0, &config);

  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 250); // 1000 nsec
  timer_isr_register(TIMER_GROUP_0, TIMER_0, isr_entry_point, NULL,
                     ESP_INTR_FLAG_IRAM, &handle_isr);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_start(TIMER_GROUP_0, TIMER_0);
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
    adc2_get_raw(BATTERY, width, &sensing_result->battery.raw);

    // LED_OFF ADC
    adc2_get_raw(SEN_R90, width, &sensing_result->led_sen_before.right90.raw);
    adc2_get_raw(SEN_R45, width, &sensing_result->led_sen_before.right45.raw);
    adc2_get_raw(SEN_F, width, &sensing_result->led_sen_before.front.raw);
    adc2_get_raw(SEN_L45, width, &sensing_result->led_sen_before.left45.raw);
    adc2_get_raw(SEN_L90, width, &sensing_result->led_sen_before.left90.raw);

    // LED_ON ADC
    gpio_set_level(LED_R90, 1);
    gpio_set_level(LED_R45, 1);
    gpio_set_level(LED_F, 1);
    gpio_set_level(LED_L45, 1);
    gpio_set_level(LED_L90, 1);

    for (int i = 0; i < 10000; i++)
      ;
    adc2_get_raw(SEN_R90, width, &sensing_result->led_sen_after.right90.raw);
    adc2_get_raw(SEN_R45, width, &sensing_result->led_sen_after.right45.raw);
    adc2_get_raw(SEN_F, width, &sensing_result->led_sen_after.front.raw);
    adc2_get_raw(SEN_L45, width, &sensing_result->led_sen_after.left45.raw);
    adc2_get_raw(SEN_L90, width, &sensing_result->led_sen_after.left90.raw);

    sensing_result->led_sen.right90.raw =
        sensing_result->led_sen_after.right90.raw -
        sensing_result->led_sen_before.right90.raw;
    sensing_result->led_sen.right45.raw =
        sensing_result->led_sen_after.right45.raw -
        sensing_result->led_sen_before.right45.raw;
    sensing_result->led_sen.front.raw =
        sensing_result->led_sen_after.front.raw -
        sensing_result->led_sen_before.front.raw;
    sensing_result->led_sen.left45.raw =
        sensing_result->led_sen_after.left45.raw -
        sensing_result->led_sen_before.left45.raw;
    sensing_result->led_sen.left90.raw =
        sensing_result->led_sen_after.left90.raw -
        sensing_result->led_sen_before.left90.raw;

    gpio_set_level(LED_R90, 0);
    gpio_set_level(LED_R45, 0);
    gpio_set_level(LED_F, 0);
    gpio_set_level(LED_L45, 0);
    gpio_set_level(LED_L90, 0);

    pcnt_get_counter_value(PCNT_UNIT_0, &sensing_result->encoder_raw.right);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_get_counter_value(PCNT_UNIT_1, &sensing_result->encoder_raw.left);
    pcnt_counter_clear(PCNT_UNIT_1);

    sensing_result->gyro.raw = gyro_if.read_gyro_z();
    sensing_result->battery.data =
        BATTERY_GAIN * 2 * sensing_result->battery.raw / 4096;
    sensing_result->encoder.right = sensing_result->encoder_raw.right;
    sensing_result->encoder.left =
        -1 * sensing_result->encoder_raw.left; //反転必須
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