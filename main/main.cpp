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

#include "include/planning_task.hpp"
#include "include/sensing_task.hpp"

void init_uart() {
  const int uart_num = UART_NUM_0;
  uart_config_t uart_config = {.baud_rate = 2000000,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_RTS,
                               .rx_flow_ctrl_thresh = 122};
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
}

void planning_task(void *arg) {
  const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
  int i = 0;
  while (1) {
    i++;
    if (i % 2 == 0) {
      gpio_set_level(LED1, 1);
      gpio_set_level(LED2, 1);
      gpio_set_level(LED3, 1);
    } else {
      gpio_set_level(LED1, 0);
      gpio_set_level(LED2, 0);
      gpio_set_level(LED3, 0);
    }
    vTaskDelay(xDelay);
  }
}
// void sensing_task(void *arg) {
//   SensingTask st;
//   st.task();
// }
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
  init_gpio();
  init_uart();
  ego_param_t param;
  param.tire = 12.0;
  param.dt = 0.001;
  param.gyro_w_gain = 0.00025;

  sensing_entity_t sensing_entity;
  ego_entity_t ego;

  SensingTask st;
  st.set_sensing_entity(&sensing_entity);
  st.create_task(0);

  PlanningTask pk;
  pk.set_sensing_entity(&sensing_entity);
  pk.set_ego_param_entity(&param);
  pk.set_ego_entity(&ego);
  pk.create_task(0);

  /* Set the GPIO as a push/pull output */

  gpio_set_level(LED1, 0);
  gpio_set_level(LED2, 0);
  gpio_set_level(LED3, 0);
  gpio_set_level(LED4, 0);
  gpio_set_level(LED5, 0);

  // adc1_init();
  while (1) {

    printf("%c[2J", ESC);   /* 画面消去 */
    printf("%c[0;0H", ESC); /* 戦闘戻す*/
    printf("SW1 %d \n", gpio_get_level(SW1));

    printf("gyro: %d\n", sensing_entity.gyro.raw);
    printf("battery: %0.3f\n", sensing_entity.battery.data);
    printf("encoder: %d, %d\n", sensing_entity.encoder.left,
           sensing_entity.encoder.right);
    printf("sensor: %d, %d, %d, %d, %d\n", sensing_entity.led_sen.left90.raw,
           sensing_entity.led_sen.left45.raw, sensing_entity.led_sen.front.raw,
           sensing_entity.led_sen.right45.raw,
           sensing_entity.led_sen.right90.raw);

    printf("ego_v: %0.3f, %0.3f, %0.3f, %0.3f\n", ego.v_l, ego.v_c, ego.v_r,
           ego.dist);
    printf("ego_w: %0.3f, %0.3f, %0.3f deg\n", ego.w, ego.angle,
           ego.angle * 180 / PI);

    if (gpio_get_level(SW1)) {
      gpio_set_level(A_CW_CCW, 1);
      gpio_set_level(B_CW_CCW, 1);
      gpio_set_level(SUCTION_PWM, 0);

      pk.suction_disable();
      pk.motor_disable();
    } else {
      gpio_set_level(A_CW_CCW, 0);
      gpio_set_level(B_CW_CCW, 0);
      gpio_set_level(SUCTION_PWM, 1);
      ego.angle = 0;
      ego.dist = 0;
      pk.suction_enable();
      pk.motor_enable();
    }

    vTaskDelay(100 / portTICK_RATE_MS);
  }
}
