#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/pcnt.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "soc/adc_channel.h"
#include "xtensa/core-macros.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// #include "VL53L0X.h"
#include "esp_efuse_rtc_calib.h"
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
  io_conf.pin_bit_mask = 1ULL << GPIO_NUM_18;
  // io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_1;
  // io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_4;
  // // io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_6;
  // io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_8;
  // io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_10;
  io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_18;
  io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_19;
  io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_20;
  io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_21;
  io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_26;
  io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_38;
  io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_39;
  io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_40;
  io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_42;
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
  // esp_adc_cal_characteristics_t *adc_chars =
  //     calloc(1, sizeof(esp_adc_cal_characteristics_t));
  // esp_adc_cal_characterize(unit, ADC1_CHANNEL_1, width, DEFAULT_VREF,
  // adc_chars); esp_adc_cal_characterize(unit, ADC1_CHANNEL_4, width,
  // DEFAULT_VREF, adc_chars); esp_adc_cal_characterize(unit, ADC1_CHANNEL_6,
  // width, DEFAULT_VREF, adc_chars); esp_adc_cal_characterize(unit,
  // ADC1_CHANNEL_8, width, DEFAULT_VREF, adc_chars);

  // adc2_config_width(width);
  // adc2_config_channel_atten(ADC2_CHANNEL_0, atten);
  // adc2_config_channel_atten(ADC2_CHANNEL_1, atten);
  // esp_adc_cal_characteristics_t *adc_chars2 =
  //     calloc(1, sizeof(esp_adc_cal_characteristics_t));
  // esp_adc_cal_characterize(unit2, atten, width, DEFAULT_VREF, adc_chars2);
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

  uint8_t data =
      SPI_SWAP_DATA_RX(*(uint16_t *)t.rx_data, 16) & 0x00FF; // FF + Data

  return data;
}
uint8_t mpu9250_write1byte(spi_device_handle_t spi, const uint8_t address,
                           const uint8_t data) {
  // １バイト読み込み

  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.length = 16;            // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  t.flags = SPI_TRANS_USE_RXDATA;

  uint16_t tx_data = (READ_FLAG) << 8 | (0x0f & data);
  tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
  t.tx_buffer = &tx_data;

  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);                      // Should have had no issues.

  // uint16_t data = SPI_SWAP_DATA_RX(*(uint16_t *)t.rx_data, 16) & 0x00FF; //
  // FF + Data

  return 0;
}
void mpu9250_init(spi_device_handle_t spi) {
  // Who AM I
  uint8_t mpu_id = mpu9250_read1byte(spi, 0x75);

  printf("MPU ID: %02X\n", mpu_id);
  mpu9250_write1byte(spi, 0x6B, 0x80); //スリープ解除?
  vTaskDelay(100 / portTICK_RATE_MS);
  mpu9250_write1byte(spi, 0x68, 0x04); //ジャイロリセット
  vTaskDelay(100 / portTICK_RATE_MS);
  mpu9250_write1byte(spi, 0x6A, 0x10); // uercontrol i2c=disable
  vTaskDelay(100 / portTICK_RATE_MS);
  mpu9250_write1byte(spi, 0x1B,
                     0x18); // gyro config ジャイロのフルスケールを±2000°/s
  vTaskDelay(100 / portTICK_RATE_MS);
}

void init_spi(spi_device_handle_t &spi) {
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .mosi_io_num = GPIO_NUM_13,
      .miso_io_num = GPIO_NUM_16,
      .sclk_io_num = GPIO_NUM_14,
      .quadwp_io_num = -1,  // unused
      .quadhd_io_num = -1,  // unused
      .max_transfer_sz = 4, // bytes
  };
  spi_device_interface_config_t devcfg = {
      .mode = 3,                     // SPI mode 3
      .clock_speed_hz = 1000 * 1000, // Clock out at 500 kHz
      .spics_io_num = GPIO_NUM_15,   // CS pin
      .queue_size = 7, // We want to be able to queue 7 transactions at a
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
void adc1_init(void) {
  auto res = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);
  printf("aaaaaa %d", (int)res);

  adc1_config_width(width);
  adc1_config_channel_atten(channel, atten);
  // adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type =
      esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, &adc_chars);
  print_char_val_type(val_type);
}
static void check_efuse(void) {
  // Check TP is burned into eFuse
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
    printf("eFuse Two Point: Supported\n");
  } else {
    printf("eFuse Two Point: NOT supported\n");
  }

  // Check Vref is burned into eFuse
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
    printf("eFuse Vref: Supported\n");
  } else {
    printf("eFuse Vref: NOT supported\n");
  }
}
#include "include/adachi.hpp"

void task0(void *arg) {}

extern "C" void app_main() {
  Adachi adachi;
  xTaskCreatePinnedToCore(task0, "Task0", 4096, NULL, 1, NULL, 0);
  init_gpio();
  init_uart();
  spi_device_handle_t spi;
  init_spi(spi);
  mpu9250_init(spi);
  int i = 0;
  /* Set the GPIO as a push/pull output */

  // gpio_set_level(GPIO_NUM_1, 0);
  // gpio_set_level(GPIO_NUM_4, 0);
  // gpio_set_level(GPIO_NUM_6, 0);
  // gpio_set_level(GPIO_NUM_8, 0);
  // gpio_set_level(GPIO_NUM_10, 0);

  gpio_set_level(GPIO_NUM_18, 0);
  gpio_set_level(GPIO_NUM_19, 0);
  gpio_set_level(GPIO_NUM_20, 0);
  gpio_set_level(GPIO_NUM_21, 0);
  gpio_set_level(GPIO_NUM_26, 0);

  // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_15);
  // // init_pwm(pwm_config);
  // static const mcpwm_operator_t OPR_PH = MCPWM_OPR_A;
  // static const mcpwm_duty_type_t DUTY_MODE = MCPWM_DUTY_MODE_0;
  // mcpwm_config_t pwm_config;
  // pwm_config.frequency = 20 * 1000; // PWM周波数= 10kHz,
  // pwm_config.cmpr_a = 0; // デューティサイクルの初期値（0%）
  // pwm_config.cmpr_b = 0; // デューティサイクルの初期値（0%）
  // pwm_config.counter_mode = MCPWM_UP_COUNTER;
  // pwm_config.duty_mode = DUTY_MODE; // アクティブハイ
  // pwm_config.cmpr_a = 25; // デューティサイクルの初期値（0%）
  // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  //  　uint16_t analog_data;
  //   while(1){
  //       analog_data = adc1_get_raw( ADC1_CHANNEL_4);
  //   	printf("ADC=%d",analog_data);
  //   }
  encoder_init(PCNT_UNIT_0, GPIO_NUM_35, GPIO_NUM_36);
  encoder_init(PCNT_UNIT_1, GPIO_NUM_33, GPIO_NUM_34);

  // adc1_init();
  adc1_config_width(width);
  adc1_config_channel_atten(ADC1_CHANNEL_1, atten); // R90
  adc1_config_channel_atten(ADC1_CHANNEL_4, atten); // R45
  // adc1_config_channel_atten(ADC1_CHANNEL_6, atten); // Front
  adc1_config_channel_atten(ADC1_CHANNEL_8, atten); // L45
  adc2_config_channel_atten(ADC2_CHANNEL_1, atten); // L90
  adc2_config_channel_atten(ADC2_CHANNEL_0, atten); // battery
  int battery;
  esp_adc_cal_characteristics_t adcChar;

  esp_adc_cal_value_t val_type =
      esp_adc_cal_characterize(ADC_UNIT_1, atten, width, 1100, &adcChar);
  esp_err_t res = esp_adc_cal_check_efuse(val_type);
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    printf("eFuse Vref");
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    printf("Two Point");
  } else {
    printf("Default");
  }
  // printf("%d %d %d \n", ESP_ADC_CAL_VAL_EFUSE_VREF, ESP_ADC_CAL_VAL_EFUSE_TP,
  //        ESP_ADC_CAL_VAL_DEFAULT_VREF);

  printf("result = %d\n version=%d\n", res, 0);
  check_efuse();

  while (1) {
    // printf("%d mV\n", voltage);
    i++;
    // printf("hello world\n");

    if (i % 2 == 0) {
      // gpio_set_level(GPIO_NUM_38, 1);
      gpio_set_level(GPIO_NUM_39, 1);
      gpio_set_level(GPIO_NUM_40, 1);
      gpio_set_level(GPIO_NUM_42, 1);
    } else {
      // gpio_set_level(GPIO_NUM_38, 0);
      gpio_set_level(GPIO_NUM_39, 0);
      gpio_set_level(GPIO_NUM_40, 0);
      gpio_set_level(GPIO_NUM_42, 0);
    }
    int16_t count_L;
    int16_t count_R;
    pcnt_get_counter_value(PCNT_UNIT_0, &count_L);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_get_counter_value(PCNT_UNIT_1, &count_R);
    pcnt_counter_clear(PCNT_UNIT_1);
    uint32_t adc_reading = 0;

    int sen_r90 = adc1_get_raw(ADC1_CHANNEL_1);
    int sen_r45 = adc1_get_raw(ADC1_CHANNEL_4);
    int sen_front = adc1_get_raw(ADC1_CHANNEL_6);
    int sen_l45 = adc1_get_raw(ADC1_CHANNEL_8);
    int sen_l90 = 0;
    // adc_reading = adc1_get_raw((adc1_channel_t)channel);
    // int sen_r90 = adc1_get_raw(ADC1_CHANNEL_1);
    // = adc1_get_raw(ADC1_CHANNEL_4);
    // = adc1_get_raw(ADC1_CHANNEL_6);
    // = adc1_get_raw(ADC1_CHANNEL_8);
    // uint32_t sen_r90 = 0;
    // uint32_t sen_r45 = 0;
    // uint32_t sen_front = 0;
    // uint32_t sen_l45 = 0;
    uint8_t mpu_id = mpu9250_read1byte(spi, 0x75);
    adc2_get_raw(ADC2_CHANNEL_0, width, &sen_l90);
    adc2_get_raw(ADC2_CHANNEL_1, width, &battery);
    // esp_adc_cal_get_voltage(ADC_CHANNEL_1, &adcChar, &sen_r90);
    // esp_adc_cal_get_voltage(ADC_CHANNEL_4, &adcChar, &sen_r45);
    // esp_adc_cal_get_voltage(ADC_CHANNEL_6, &adcChar, &sen_front);
    // esp_adc_cal_get_voltage(ADC_CHANNEL_8, &adcChar, &sen_l45);
    printf("MPU ID: %02X, %d, %d, %d\n", mpu_id, count_L, count_R, adc_reading);
    printf("%u, %u, %u, %u, %u  %d\n", sen_l90, sen_l45, sen_front, sen_r45,
           sen_r90, battery);
    printf("GPIO_NUM_MAX = %d\n", (int)GPIO_NUM_MAX);
    // uint16_t result_mm = 0;
    // bool result = vl.read(&result_mm);
    // if (result) {
    //   printf("took_ms = %d\r\n", result_mm);
    // } else {
    //   printf("faild took_ms = %d\r\n", result_mm);
    // }
    // printf("hello world %d %d %d\n", i, count_L, adc_ch1_4);
    // if (i % 2 == 0) {
    //   gpio_set_level(GPIO_NUM_1, 1);
    //   gpio_set_level(GPIO_NUM_2, 1);
    // } else {
    //   gpio_set_level(GPIO_NUM_1, 0);
    //   gpio_set_level(GPIO_NUM_2, 0);
    // }
    // mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, OPR_PH);
    // float duty = 5;
    // if (gpio_get_level(GPIO_NUM_4)) {
    //   gpio_set_level(GPIO_NUM_18, 1);
    //   duty = 5;
    // } else {
    //   gpio_set_level(GPIO_NUM_18, 0);
    //   duty = 15;
    // }
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
    // mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, DUTY_MODE);

    vTaskDelay(125 / portTICK_RATE_MS);
  }
}
