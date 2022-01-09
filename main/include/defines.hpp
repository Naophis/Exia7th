
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

#define LED_R90 GPIO_NUM_9
#define LED_R45 GPIO_NUM_10
#define LED_F GPIO_NUM_13
#define LED_L45 GPIO_NUM_16
#define LED_L90 GPIO_NUM_18

#define TXD GPIO_NUM_43
#define RXD GPIO_NUM_44
#define RTS GPIO_NUM_15
#define CTS GPIO_NUM_16

#define A_CW_CCW GPIO_NUM_39
#define B_CW_CCW GPIO_NUM_41
#define A_PWM GPIO_NUM_40
#define B_PWM GPIO_NUM_42

#define BUZZER GPIO_NUM_34

#define LED1 GPIO_NUM_1
#define LED2 GPIO_NUM_46
#define LED3 GPIO_NUM_45
#define LED4 GPIO_NUM_8
#define LED5 GPIO_NUM_21
#define LED_R GPIO_NUM_8
#define LED_L GPIO_NUM_21

#define SW1 GPIO_NUM_38

#define EN_MOSI GPIO_NUM_2
#define EN_CLK GPIO_NUM_3
#define EN_GN_SSL GPIO_NUM_4
#define EN_MISO GPIO_NUM_5

#define ENC_R_A GPIO_NUM_6
#define ENC_R_B GPIO_NUM_7
#define ENC_L_A GPIO_NUM_35
#define ENC_L_B GPIO_NUM_36

#define SEN_R90 ADC2_CHANNEL_0
#define SEN_R45 ADC2_CHANNEL_1
#define SEN_F ADC2_CHANNEL_4
#define SEN_L45 ADC2_CHANNEL_6
#define SEN_L90 ADC2_CHANNEL_8
#define BATTERY ADC2_CHANNEL_9

#define MOTOR_HZ 100000
#define SUCTION_MOTOR_HZ 10000

#define SUCTION_PWM GPIO_NUM_37
#define LEDC_AUTO_CLK 0
#define LEDC_TIMER_10_BIT 10
#define LEDC_TIMER_14_BIT 14
// #define LEDC_TIMER_0 0
#define LEDC_CHANNEL_0 0
#define LEDC_INTR_DISABLE 0
// #define LEDC_HIGH_SPEED_MODE 0

#define ENCODER_H_LIM_VAL 32767
#define ENCODER_L_LIM_VAL -32767
#define READ_FLAG 0x80
#define ESC 0x1B
#define BUF_SIZE (4096)

// constexpr float MOTION_CHECK_TH = 1000;
#define MOTION_CHECK_TH 1000
#define ENC_OPE_V_R_TH 175

#define LOG_SIZE 2000
#define LINE_BUF_SIZE 1500

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