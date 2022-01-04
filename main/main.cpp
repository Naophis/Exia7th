#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "driver/pcnt.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "driver/uart.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/defines.hpp"
#include "sdkconfig.h"
#include "soc/adc_channel.h"
#include "soc/ledc_periph.h"
#include "xtensa/core-macros.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "include/adachi.hpp"
// #include "VL53L0X.h"
#include "esp_efuse_rtc_calib.h"

#include "include/buzzer_task.hpp"
#include "include/main_task.hpp"
#include "include/planning_task.hpp"
#include "include/sensing_task.hpp"

#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "rom/uart.h"

ego_param_t param = {0};
sensing_result_entity_t sensing_entity = {0};
ego_entity_t ego = {0};
tgt_entity_t tgt = {0};
motion_tgt_val_t tgt_val = {0};

void init_uart() {
  uart_config_t uart_config;
  uart_config.baud_rate = 2 * 1000 * 1000;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  // uart_config.rx_flow_ctrl_thresh = 122;
  uart_config.source_clk = UART_SCLK_APB;
  int intr_alloc_flags = 0;
  uart_param_config(UART_NUM_0, &uart_config);
  uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags);
  uart_param_config(UART_NUM_0, &uart_config);
  uart_set_pin(UART_NUM_0, TXD, RXD, RTS, CTS);
}

void init_gpio() {
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
#define TIMER_RESOLUTION_HZ 1000000 // 1MHz resolution
bool itr_state = true;
ICM20689 gyro_if;
const adc_bits_width_t width = ADC_WIDTH_BIT_12;
const adc_atten_t atten = ADC_ATTEN_DB_11;

int isr_cnt = 0;

void encoder_init(const pcnt_unit_t unit, const gpio_num_t pinA,
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

void init_sensing_device() {

  // gyro_if.init();
  // gyro_if.setup();

  // sensing init
  adc2_config_channel_atten(SEN_R90, atten);
  adc2_config_channel_atten(SEN_R45, atten);
  adc2_config_channel_atten(SEN_F, atten);
  adc2_config_channel_atten(SEN_L45, atten);
  adc2_config_channel_atten(SEN_L90, atten);
  adc2_config_channel_atten(BATTERY, atten);
  encoder_init(PCNT_UNIT_0, ENC_R_A, ENC_R_B);
  encoder_init(PCNT_UNIT_1, ENC_L_A, ENC_L_B);
}

void isr_a() {
  if (isr_cnt == 0) {
    // adc2_get_raw(BATTERY, width, &sensing_entity.battery.raw);
    gpio_set_level(LED3, 1);
  } else if (isr_cnt == 1) {
  } else if (isr_cnt == 2) {
  } else if (isr_cnt == 3) {
  } else if (isr_cnt == 4) {
  }
}
void isr_b() {
  if (isr_cnt == 0) {
  } else if (isr_cnt == 1) {
  } else if (isr_cnt == 2) {
  } else if (isr_cnt == 3) {
  } else if (isr_cnt == 4) {
    gpio_set_level(LED3, 0);
  }
  isr_cnt++;
  if (isr_cnt == 5) {
    isr_cnt = 0;
  }
}

void timer_isr(void *parameters) {
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
  if (itr_state) {
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 2500); // 250 nsec
    isr_a();
  } else {
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 7500); // 1000 nsec
    isr_b();
  }
  itr_state ^= itr_state;
}

void init_sensing_timer() {
  auto group = TIMER_GROUP_0;
  auto timer = TIMER_0;

  timer_config_t config;
  config.alarm_en = TIMER_ALARM_EN;
  config.counter_en = TIMER_PAUSE;
  config.clk_src = TIMER_SRC_CLK_APB;
  config.auto_reload = TIMER_AUTORELOAD_EN;
  config.counter_dir = TIMER_COUNT_UP;
  config.divider = 8; // 80Mhz / divider
  timer_init(group, timer, &config);

  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 250); // 1000 nsec
  timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_isr, NULL, 0, NULL);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_start(TIMER_GROUP_0, TIMER_0);
}
void set_hardware_param(ego_param_t &ep) {
  const char *base_path = "/spiflash";
  esp_vfs_fat_mount_config_t mount_config;
  wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

  mount_config.max_files = 8;
  mount_config.format_if_mount_failed = true;
  mount_config.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;

  esp_err_t err = esp_vfs_fat_spiflash_mount(base_path, "storage",
                                             &mount_config, &s_wl_handle);
  if (err != ESP_OK) {
    printf("Failed to mount FATFS (%s)\n", esp_err_to_name(err));
    return;
  } else {
    printf("mount OK\n");
  }

  FILE *f = fopen("/spiflash/hardware.txt", "rb");
  char line[1024];
  fgets(line, sizeof(line), f);
  fclose(f);
  std::string json = std::string(line);
  printf("%s\n", json.c_str());
}

extern "C" void app_main() {
  // Adachi adachi;

  init_gpio();
  init_uart();
  // init_sensing_device();
  // set_hardware_param(param);
  param.tire = 12.0;
  param.dt = 0.001;
  param.motor_pid.p = 0.175;
  param.motor_pid.i = 0.0175;
  param.motor_pid.d = 0.0;
  param.gyro_pid.p = 2.5;
  param.gyro_pid.i = 0.75;
  param.gyro_pid.d = 0.0;
  tgt_val.ego_in.v = 0;
  tgt_val.ego_in.w = 0;
  param.gyro_param.gyro_w_gain_left = 0.0002645;

  // xTaskCreate(echo_task, "uart_echo_task", 8192, NULL, 10, NULL);
  // init_sensing_timer();

  SensingTask st;
  st.set_sensing_entity(&sensing_entity);
  st.create_task(0);

  PlanningTask pt;
  pt.set_sensing_entity(&sensing_entity);
  pt.set_ego_param_entity(&param);
  pt.set_ego_entity(&ego);
  pt.set_tgt_entity(&tgt);
  pt.set_tgt_val(&tgt_val);
  pt.create_task(0);
  // IntegratedEntity ie;

  MainTask mt;
  mt.set_sensing_entity(&sensing_entity);
  mt.set_ego_param_entity(&param);
  mt.set_ego_entity(&ego);
  mt.set_planning_task(&pt);
  mt.set_tgt_entity(&tgt);
  mt.set_tgt_val(&tgt_val);

  mt.create_task(1);

  /* Set the GPIO as a push/pull output */

  gpio_set_level(LED1, 0);
  gpio_set_level(LED2, 0);
  gpio_set_level(LED3, 0);
  gpio_set_level(LED4, 0);
  gpio_set_level(LED5, 0);

  while (1) {
    vTaskDelay(100 / portTICK_RATE_MS);
  }
}