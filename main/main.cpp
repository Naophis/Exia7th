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

// #include "VL53L0X.h"
#include "esp_efuse_rtc_calib.h"

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
void init_pwm(mcpwm_config_t &pwm_config) {
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_15);
  pwm_config.frequency = 10 * 10000; // PWM周波数= 10kHz,
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // アクティブハイ
}
#define ENCODER_H_LIM_VAL 32767
#define ENCODER_L_LIM_VAL -32767
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

  /* Initialize PCNT unit */

  pcnt_unit_config(&pcnt_config_0);
  pcnt_unit_config(&pcnt_config_1);

  /* Configure and enable the input filter */
  // pcnt_set_filter_value(unit, 100);
  // pcnt_filter_enable(unit);

  /* Initialize PCNT's counter */
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);

  /* Everything is set up, now go to counting */
  pcnt_counter_resume(unit);
}
#define DEFAULT_VREF 3300
// eFuseメモリのVrefを使うため、このデフォルト値は使用されない
void init_adc_config(esp_adc_cal_characteristics_t &adc1Char) {
  static const adc_unit_t unit = ADC_UNIT_1;
  static const adc_unit_t unit2 = ADC_UNIT_2;
  static const adc_channel_t channel = ADC_CHANNEL_0;
  static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
  static const adc_atten_t atten = ADC_ATTEN_DB_11;
  adc1_config_width(width);
  adc1_config_channel_atten(ADC1_CHANNEL_1, atten); // R90
  adc1_config_channel_atten(ADC1_CHANNEL_4, atten); // R45
  adc1_config_channel_atten(ADC1_CHANNEL_6, atten); // Front
  adc1_config_channel_atten(ADC1_CHANNEL_8, atten); // L45

  esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, &adc1Char);
}
#define READ_FLAG 0x80
uint8_t mpu9250_read1byte(spi_device_handle_t spi, const uint8_t address) {
  // １バイト読み込み

  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.length = 16;            // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  t.flags = SPI_TRANS_USE_RXDATA;

  uint16_t tx_data = (address | READ_FLAG) << 8;
  tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
  t.tx_buffer = &tx_data;

  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);                      // Should have had no issues.

  printf("read: %d %d %d %d %d\n", t.rx_data[0], t.rx_data[1], t.rx_data[2],
         t.rx_data[3], address);
  uint8_t data =
      SPI_SWAP_DATA_RX(*(uint16_t *)t.rx_data, 16) & 0x00FF; // FF + Data

  return data;
}
uint8_t mpu9250_write1byte(spi_device_handle_t spi, const uint8_t address,
                           const uint8_t data) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.length = 16;            // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  t.flags = SPI_TRANS_USE_RXDATA;

  uint16_t tx_data = (address) << 8 | (0x0f & data);
  tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
  t.tx_buffer = &tx_data;

  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);                      // Should have had no issues.

  // uint16_t data = SPI_SWAP_DATA_RX(*(uint16_t *)t.rx_data, 16) & 0x00FF; //
  // FF + Data
  printf("write: %d %d %d %d %d\n", t.rx_data[0], t.rx_data[1], t.rx_data[2],
         t.rx_data[3], address);

  return 0;
}
uint8_t mpu9250_read2byte(spi_device_handle_t spi, const uint8_t address) {
  // １バイト読み込み

  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.length = 24;            // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  t.flags = SPI_TRANS_USE_RXDATA;

  uint16_t tx_data = (address | READ_FLAG) << 8;
  tx_data = SPI_SWAP_DATA_TX(tx_data, 24);
  t.tx_buffer = &tx_data;

  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);                      // Should have had no issues.

  printf("read2: %d %d %d %d %d\n", t.rx_data[0], t.rx_data[1], t.rx_data[2],
         t.rx_data[3], address);
  uint8_t data =
      SPI_SWAP_DATA_RX(*(uint16_t *)t.rx_data, 24) & 0x00FF; // FF + Data

  return data;
}
void mpu9250_init(spi_device_handle_t &spi) {
  // Who AM I
  uint8_t mpu_id = mpu9250_read1byte(spi, 0x75);

  printf("initial MPU ID: %02X\n", mpu_id);
  mpu9250_write1byte(spi, 0x6B, 0x80); //スリープ解除?
  vTaskDelay(100 / portTICK_PERIOD_MS);
  mpu9250_write1byte(spi, 0x68, 0x04); //ジャイロリセット
  vTaskDelay(100 / portTICK_PERIOD_MS);
  mpu9250_write1byte(spi, 0x6A, 0x10); // uercontrol i2c=disable
  vTaskDelay(100 / portTICK_PERIOD_MS);
  mpu9250_write1byte(spi, 0x1B, 0x18); // 2000
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

void init_spi(spi_device_handle_t &spi) {
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .mosi_io_num = EN_MOSI,
      .miso_io_num = EN_MISO,
      .sclk_io_num = EN_CLK,
      .quadwp_io_num = -1,  // unused
      .quadhd_io_num = -1,  // unused
      .max_transfer_sz = 2, // bytes
      .flags = SPICOMMON_BUSFLAG_MASTER,
      .intr_flags = 0 // 割り込みをしない
  };

  spi_device_interface_config_t devcfg = {
      .mode = 3,
      .clock_speed_hz = 7 * 1000 * 1000, // aaaaaaaaaaa
      .spics_io_num = EN_GN_SSL,
      .queue_size = 7,
      //
      // .input_delay_ns = 0,
      // .command_bits = 1, // aaaaaaaaaaaaaaaasdasdas
      // .address_bits = 7,
      // .dummy_bits = 0,
      // .duty_cycle_pos = 0,
      // .cs_ena_pretrans = 0,
      // .cs_ena_posttrans = 0,
      // .flags = 0,
      // .pre_cb = NULL,
      // .post_cb = NULL //
  };
  // Initialize the SPI bus
  ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
  ESP_ERROR_CHECK(ret);
  // Attach the LCD to the SPI bus
  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);
}
static void print_char_val_type(esp_adc_cal_value_t val_type) {
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    printf("Characterized using Two Point Value\n");
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    printf("Characterized using eFuse Vref\n");
  } else {
    printf("Characterized using Default Vref\n");
  }
}
static esp_adc_cal_characteristics_t adc_chars;
static const auto channel = ADC1_CHANNEL_1;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

#include "include/adachi.hpp"

extern "C" void app_main() {
  // Adachi adachi;
  // xTaskCreatePinnedToCore(task0, "Task0", 8192, NULL, 1, NULL, 1);

  SensingTask st;
  st.create_task();

  init_gpio();
  init_uart();
  spi_device_handle_t spi;
  init_spi(spi);
  mpu9250_init(spi);
  // icm20689 driver(EN_MOSI, EN_MISO, EN_CLK, EN_GN_SSL);

  int i = 0;
  /* Set the GPIO as a push/pull output */

  gpio_set_level(LED1, 0);
  gpio_set_level(LED2, 0);
  gpio_set_level(LED3, 0);
  gpio_set_level(LED4, 0);
  gpio_set_level(LED5, 0);

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, A_PWM);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, B_PWM);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM2A, SUCTION_PWM);

  // init_pwm(pwm_config);
  static const mcpwm_duty_type_t DUTY_MODE = MCPWM_DUTY_MODE_0;

  mcpwm_config_t motor_pwm_conf;
  motor_pwm_conf.frequency = MOTOR_HZ; // PWM周波数= 10kHz,
  motor_pwm_conf.cmpr_a = 0; // デューティサイクルの初期値（0%）
  motor_pwm_conf.cmpr_b = 0; // デューティサイクルの初期値（0%）
  motor_pwm_conf.counter_mode = MCPWM_UP_COUNTER;
  motor_pwm_conf.duty_mode = DUTY_MODE; // アクティブハイ
  motor_pwm_conf.cmpr_a = 25; // デューティサイクルの初期値（0%）

  mcpwm_config_t suction_pwm_conf;
  suction_pwm_conf.frequency = SUCTION_MOTOR_HZ; // PWM周波数= 10kHz,
  suction_pwm_conf.cmpr_a = 0; // デューティサイクルの初期値（0%）
  suction_pwm_conf.cmpr_b = 0; // デューティサイクルの初期値（0%）
  suction_pwm_conf.counter_mode = MCPWM_UP_COUNTER;
  suction_pwm_conf.duty_mode = DUTY_MODE; // アクティブハイ
  suction_pwm_conf.cmpr_a = 25; // デューティサイクルの初期値（0%）

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &motor_pwm_conf);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &motor_pwm_conf);
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_2, &suction_pwm_conf);

  encoder_init(PCNT_UNIT_0, ENC_R_A, ENC_R_B);
  encoder_init(PCNT_UNIT_1, ENC_L_A, ENC_L_B);

  // adc1_init();

  adc2_config_channel_atten(SEN_R90, atten);
  adc2_config_channel_atten(SEN_R45, atten);
  adc2_config_channel_atten(SEN_F, atten);
  adc2_config_channel_atten(SEN_L45, atten);
  adc2_config_channel_atten(SEN_L90, atten);
  adc2_config_channel_atten(BATTERY, atten);

  int battery;
  int sen_r90;
  int sen_r45;
  int sen_front;
  int sen_l45;
  int sen_l90;

  while (1) {
    i++;
    printf("SW1 %d \n", gpio_get_level(SW1));
    if (i % 2 == 0) {
      gpio_set_level(LED1, 1);
      gpio_set_level(LED2, 1);
      gpio_set_level(LED3, 1);
    } else {
      gpio_set_level(LED1, 0);
      gpio_set_level(LED2, 0);
      gpio_set_level(LED3, 0);
    }
    int16_t count_L;
    int16_t count_R;
    pcnt_get_counter_value(PCNT_UNIT_0, &count_R);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_get_counter_value(PCNT_UNIT_1, &count_L);
    pcnt_counter_clear(PCNT_UNIT_1);
    // uint16_t gyro = driver.readWhoAmI();
    // printf("%u\n", gyro);
    uint8_t who_am_i = mpu9250_read1byte(spi, 0x75);
    uint8_t gyro_z_h = mpu9250_read1byte(spi, 0x47);
    uint8_t gyro_z_l = mpu9250_read1byte(spi, 0x48);
    signed short gyro = (signed short)(gyro_z_h << 8 | gyro_z_l);
    printf("%d %d %d %d\n", who_am_i, gyro_z_h, gyro_z_l, gyro);
    adc2_get_raw(SEN_R90, width, &sen_r90);
    adc2_get_raw(SEN_R45, width, &sen_r45);
    adc2_get_raw(SEN_F, width, &sen_front);
    adc2_get_raw(SEN_L45, width, &sen_l45);
    adc2_get_raw(SEN_L90, width, &sen_l90);
    adc2_get_raw(BATTERY, width, &battery);

    float battery_v = 3.33 * 2 * battery / 4096;

    printf("gyro: %u, %d, %d, %0.3f\n", 0, count_L, count_R, battery_v);
    printf("%d, %d, %d, %d, %d\n", sen_l90, sen_l45, sen_front, sen_r45,
           sen_r90);
    float duty = 20;
    if (gpio_get_level(SW1)) {
      gpio_set_level(A_CW_CCW, 1);
      gpio_set_level(B_CW_CCW, 1);
    } else {
      gpio_set_level(A_CW_CCW, 0);
      gpio_set_level(B_CW_CCW, 0);
      gpio_set_level(SUCTION_PWM, 1);
      duty = 100;
    }
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, DUTY_MODE);

    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, DUTY_MODE);

    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A, duty);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A, DUTY_MODE);

    vTaskDelay(10 / portTICK_RATE_MS);
  }
}
