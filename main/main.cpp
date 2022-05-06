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
// #include "esp_efuse_rtc_calib.h"

#include "include/logging_task.hpp"
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

#include "esp_debug_helpers.h"
#include "rom/uart.h"
SensingTask st;

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

  // 内部プルダウンしない
  io_conf.pull_down_en = (gpio_pulldown_t)0;
  // 内部プルアップしない
  io_conf.pull_up_en = (gpio_pullup_t)0;
  // 設定をセットする
  gpio_config(&io_conf);
  gpio_set_level(SUCTION_PWM, 0);
  // gpio_set_direction((gpio_num_t)GPIO_OUTPUT_IO_8,
}
int gyro_mode = 0;
ICM20689 gyro_if;
void timer_isr(void *parameters) {
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
  if (st.is_ready()) {
    gyro_mode++;
    if (gyro_mode == 1 || gyro_mode == 3) {
      gyro_if.req_read2byte_itr(0x47);
    } else if (gyro_mode == 2 || gyro_mode == 4) {
      st.gyro_q.push_back(gyro_if.read_2byte_itr());
    }
    if (st.gyro_q.size() > GY_DQ_SIZE) {
      st.gyro_q.pop_front();
    }
    if (gyro_mode == 4) {
      gyro_mode = 0;
    }
  }
}
void hwtimer_init(void) {
  timer_config_t config;
  config.alarm_en = TIMER_ALARM_EN;
  config.counter_en = TIMER_PAUSE;
  config.clk_src = TIMER_SRC_CLK_APB;
  config.auto_reload = TIMER_AUTORELOAD_EN;
  config.counter_dir = TIMER_COUNT_UP;
  config.divider = 8; // 80Mhz / divider
  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, GY_CYCLE);
  timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_isr, NULL, 0, NULL);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_start(TIMER_GROUP_0, TIMER_0);
}
extern "C" void app_main() {
  // Adachi adachi;

  std::shared_ptr<input_param_t> param = std::make_shared<input_param_t>();
  std::shared_ptr<sensing_result_entity_t> sensing_entity =
      std::make_shared<sensing_result_entity_t>();
  std::shared_ptr<motion_tgt_val_t> tgt_val =
      std::make_shared<motion_tgt_val_t>();

  init_gpio();
  init_uart();
  if (GY_MODE) {
    gyro_if.init();
    gyro_if.setup();
    gyro_mode = 0;
    hwtimer_init();
  }

  esp_vfs_fat_mount_config_t mount_config;
  mount_config.max_files = 8;
  mount_config.format_if_mount_failed = true;
  mount_config.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;
  const char *base_path = "/spiflash";
  wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

  printf("storage0: try mount\n");
  esp_err_t err = esp_vfs_fat_spiflash_mount(base_path, "storage0",
                                             &mount_config, &s_wl_handle);
  if (err != ESP_OK) {
    printf("storage0: Failed to mount FATFS (%s)\n", esp_err_to_name(err));
    return;
  } else {
    printf("storage0: mount OK\n");
  }

  gpio_set_level(SUCTION_PWM, 1);
  param->tire = 12.0;
  param->dt = 0.001;
  param->motor_pid.p = 0.175;
  param->motor_pid.i = 0.0175;
  param->motor_pid.d = 0.0;
  param->gyro_pid.p = 2.5;
  param->gyro_pid.i = 0.75;
  param->gyro_pid.d = 0.0;
  tgt_val->ego_in.v = 0;
  tgt_val->ego_in.w = 0;
  param->gyro_param.gyro_w_gain_left = 0.0002645;

  st.set_sensing_entity(sensing_entity);
  st.set_tgt_val(tgt_val);
  st.set_input_param_entity(param);
  st.create_task(0);

  std::shared_ptr<PlanningTask> pt = std::make_shared<PlanningTask>();
  pt->set_sensing_entity(sensing_entity);
  pt->set_input_param_entity(param);
  // pt->set_ego_entity(ego);
  pt->set_tgt_val(tgt_val);
  pt->create_task(0);

  std::shared_ptr<LoggingTask> lt = std::make_shared<LoggingTask>();
  lt->set_sensing_entity(sensing_entity);
  lt->set_input_param_entity(param);
  // lt->set_ego_entity(ego);
  lt->set_tgt_val(tgt_val);
  lt->create_task(1);

  MainTask mt;
  mt.set_sensing_entity(sensing_entity);
  mt.set_input_param_entity(param);
  // mt.set_ego_entity(ego);
  mt.set_tgt_val(tgt_val);
  mt.set_planning_task(pt);
  mt.set_logging_task(lt);

  mt.create_task(1);

  /* Set the GPIO as a push/pull output */

  gpio_set_level(LED1, 0);
  gpio_set_level(LED2, 0);
  gpio_set_level(LED3, 0);
  gpio_set_level(LED4, 0);
  gpio_set_level(LED5, 0);

  while (1) {
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}