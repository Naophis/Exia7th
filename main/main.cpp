#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "driver/pcnt.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
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

void init_uart() {
  uart_config_t uart_config;
  uart_config.baud_rate = 2 * 1000 * 1000;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_RTS;
  uart_config.rx_flow_ctrl_thresh = 122;
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
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

  io_conf.pin_bit_mask |= 1ULL << SUCTION_PWM;

  // 内部プルダウンしない
  io_conf.pull_down_en = (gpio_pulldown_t)0;
  // 内部プルアップしない
  io_conf.pull_up_en = (gpio_pullup_t)0;
  // 設定をセットする
  gpio_config(&io_conf);
  // gpio_set_direction((gpio_num_t)GPIO_OUTPUT_IO_8,
}

extern "C" void app_main() {
  // Adachi adachi;

  ego_param_t param = {0};
  sensing_result_entity_t sensing_entity = {0};
  ego_entity_t ego = {0};
  tgt_entity_t tgt = {0};
  motion_tgt_val_t tgt_val = {0};
  init_gpio();
  init_uart();

  param.tire = 12.0;
  param.dt = 0.001;
  param.motor_pid.p = 0.15;
  param.motor_pid.i = 0.01;
  param.gyro_pid.p = 0.5;
  param.gyro_pid.i = 0.25;

  param.gyro_param.gyro_w_gain = 0.00025;

  SensingTask st ;
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
