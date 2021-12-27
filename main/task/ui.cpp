#include "include/ui.hpp"

void UserInterface::set_sensing_entity(sensing_result_entity_t *_entity) {
  entity_ro = _entity;
}

void UserInterface::set_tgt_entity(tgt_entity_t *_tgt) { tgt = _tgt; }

void UserInterface::set_ego_entity(ego_entity_t *_ego) { ego = _ego; }

bool UserInterface::button_state() {
  return !gpio_get_level(SW1); //
}

bool UserInterface::button_state_hold() {
  if (!gpio_get_level(SW1)) {
    while (!gpio_get_level(SW1))
      ;
    return true;
  } else {
    return false;
  }
}

int UserInterface::encoder_operation() {
  double v_r = ego->v_r;
  if (v_r > ENC_OPE_V_R_TH) {
    tgt->buzzer.hz = 880;
    tgt->buzzer.time = 100;
    int buzzer_timestamp = tgt->buzzer.timstamp;
    tgt->buzzer.timstamp = ++buzzer_timestamp;
    vTaskDelay(tgt->buzzer.time / portTICK_PERIOD_MS);
    return 1;
  }
  if (v_r < -ENC_OPE_V_R_TH) {
    tgt->buzzer.hz = 440;
    tgt->buzzer.time = 100;
    int buzzer_timestamp = tgt->buzzer.timstamp;
    tgt->buzzer.timstamp = ++buzzer_timestamp;
    vTaskDelay(tgt->buzzer.time / portTICK_PERIOD_MS);
    return -1;
  }
  return 0;
}

void UserInterface::motion_check() {
  int c = 0;
  while (1) {
    c++;
    if (c % 2 == 0) {
      LED_on_all();
    } else {
      LED_off_all();
    }
    double front_sensor_data = entity_ro->led_sen.front.raw;
    if (front_sensor_data > MOTION_CHECK_TH) {
      LED_off_all();
      for (int i = 0; i < 2; i++) {
        tgt->buzzer.hz = 880;
        tgt->buzzer.time = 100;
        int buzzer_timestamp = tgt->buzzer.timstamp;
        tgt->buzzer.timstamp = ++buzzer_timestamp;
        vTaskDelay(tgt->buzzer.time / portTICK_PERIOD_MS);
        vTaskDelay(tgt->buzzer.time / portTICK_PERIOD_MS);
      }
      break;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void UserInterface::LED_on_off(gpio_num_t gpio_num, int state) {
  gpio_set_level(gpio_num, state);
}
void UserInterface::LED_on(int byte) {
  int led1 = byte & 0x01;
  int led2 = byte & 0x02;
  int led3 = byte & 0x03;
  int led4 = byte & 0x04;
  int led5 = byte & 0x05;
  const int state = 1;
  if (led1)
    LED_on_off(LED1, state);
  if (led2)
    LED_on_off(LED2, state);
  if (led3)
    LED_on_off(LED3, state);
  if (led4)
    LED_on_off(LED4, state);
  if (led5)
    LED_on_off(LED5, state);
}

void UserInterface::LED_off(int byte) {
  int led1 = byte & 0x01;
  int led2 = byte & 0x02;
  int led3 = byte & 0x03;
  int led4 = byte & 0x04;
  int led5 = byte & 0x05;
  const int state = 0;
  if (led1)
    LED_on_off(LED1, state);
  if (led2)
    LED_on_off(LED2, state);
  if (led3)
    LED_on_off(LED3, state);
  if (led4)
    LED_on_off(LED4, state);
  if (led5)
    LED_on_off(LED5, state);
}
void UserInterface::LED_off_all() {
  const int state = 0;
  LED_on_off(LED1, state);
  LED_on_off(LED2, state);
  LED_on_off(LED3, state);
  LED_on_off(LED4, state);
  LED_on_off(LED5, state);
}
void UserInterface::LED_on_all() {
  const int state = 1;
  LED_on_off(LED1, state);
  LED_on_off(LED2, state);
  LED_on_off(LED3, state);
  LED_on_off(LED4, state);
  LED_on_off(LED5, state);
}