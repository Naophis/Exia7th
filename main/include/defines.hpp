
#ifndef DEFINES_HPP
#define DEFINES_HPP

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "gen_code/bus.h"
#include "include/maze_solver.hpp"
#include "sdkconfig.h"
#include "soc/adc_channel.h"
#include "soc/ledc_periph.h"

#include "include/enums.hpp"
#include "include/structs.hpp"
#include <initializer_list>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#define ABS(IN) ((IN) < 0 ? -(IN) : (IN))

constexpr gpio_num_t LED_R90 = GPIO_NUM_9;
constexpr gpio_num_t LED_R45 = GPIO_NUM_10;
constexpr gpio_num_t LED_F = GPIO_NUM_13;
constexpr gpio_num_t LED_L45 = GPIO_NUM_16;
constexpr gpio_num_t LED_L90 = GPIO_NUM_18;

constexpr gpio_num_t TXD = GPIO_NUM_43;
constexpr gpio_num_t RXD = GPIO_NUM_44;
constexpr gpio_num_t RTS = GPIO_NUM_15;
constexpr gpio_num_t CTS = GPIO_NUM_16;

constexpr gpio_num_t A_CW_CCW = GPIO_NUM_39;
constexpr gpio_num_t B_CW_CCW = GPIO_NUM_41;
constexpr gpio_num_t A_PWM = GPIO_NUM_40;
constexpr gpio_num_t B_PWM = GPIO_NUM_42;

constexpr gpio_num_t BUZZER = GPIO_NUM_34;

constexpr gpio_num_t LED1 = GPIO_NUM_1;
constexpr gpio_num_t LED2 = GPIO_NUM_46;
constexpr gpio_num_t LED3 = GPIO_NUM_45;
constexpr gpio_num_t LED4 = GPIO_NUM_8;
constexpr gpio_num_t LED5 = GPIO_NUM_21;
constexpr gpio_num_t LED_R = GPIO_NUM_8;
constexpr gpio_num_t LED_L = GPIO_NUM_21;

constexpr gpio_num_t SW1 = GPIO_NUM_38;

constexpr gpio_num_t EN_MOSI = GPIO_NUM_2;
constexpr gpio_num_t EN_CLK = GPIO_NUM_3;
constexpr gpio_num_t EN_GN_SSL = GPIO_NUM_4;
constexpr gpio_num_t EN_MISO = GPIO_NUM_5;

constexpr gpio_num_t ENC_R_A = GPIO_NUM_6;
constexpr gpio_num_t ENC_R_B = GPIO_NUM_7;
constexpr gpio_num_t ENC_L_A = GPIO_NUM_35;
constexpr gpio_num_t ENC_L_B = GPIO_NUM_36;

constexpr gpio_num_t SUCTION_PWM = GPIO_NUM_37;

#define SEN_R90 ADC2_CHANNEL_0
#define SEN_R45 ADC2_CHANNEL_1
#define SEN_F ADC2_CHANNEL_4
#define SEN_L45 ADC2_CHANNEL_6
#define SEN_L90 ADC2_CHANNEL_8
#define BATTERY ADC2_CHANNEL_9
// constexpr adc2_channel_t SEN_R90 = ADC2_CHANNEL_0;
// constexpr adc2_channel_t SEN_R45 = ADC2_CHANNEL_1;
// constexpr adc2_channel_t SEN_F = ADC2_CHANNEL_4;
// constexpr adc2_channel_t SEN_L45 = ADC2_CHANNEL_6;
// constexpr adc2_channel_t SEN_L90 = ADC2_CHANNEL_8;
// constexpr adc2_channel_t BATTERY = ADC2_CHANNEL_9;

constexpr int MOTOR_HZ = 100000;
constexpr int SUCTION_MOTOR_HZ = 10000;
constexpr int16_t ENCODER_H_LIM_VAL = 32767;
constexpr int16_t ENCODER_L_LIM_VAL = -32767;
constexpr uint8_t READ_FLAG = 0x80;
constexpr uint8_t ESC = 0x1B;
constexpr uint16_t BUF_SIZE = 4096;

constexpr uint16_t MOTION_CHECK_TH = 1000;
constexpr uint16_t ENC_OPE_V_R_TH = 175;

constexpr uint16_t LOG_SIZE = 2000;
constexpr uint16_t LINE_BUF_SIZE = 1500;
constexpr float BATTERY_GAIN = 2.97324;

constexpr uint8_t LEDC_HIGH_SPEED_MODE = 0;
constexpr float LOW_BATTERY_TH = 3.95;

static const std::initializer_list<std::pair<TurnType, std::string>>
    turn_name_list = {
        {TurnType::Normal, "normal"},    //
        {TurnType::Large, "large"},      //
        {TurnType::Orval, "orval"},      //
        {TurnType::Dia45, "dia45"},      //
        {TurnType::Dia135, "dia135"},    //
        {TurnType::Dia90, "dia90"},      //
        {TurnType::Dia45_2, "dia45_2"},  //
        {TurnType::Dia135_2, "dia135_2"} //
};

static const std::string slalom_log_file("/spiflash/sla.log");

static const std::string format1("%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,");
static const std::string format2("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,");
static const std::string
    format3("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d\n");

#endif