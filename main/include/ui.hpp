#ifndef UI_HPP
#define UI_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/defines.hpp"

class UserInterface {
public:
  UserInterface(){};
  virtual ~UserInterface(){};

  void motion_check();
  void set_sensing_entity(sensing_result_entity_t *_entity);
  void set_tgt_entity(tgt_entity_t *_tgt);
  void set_ego_entity(ego_entity_t *_ego);

  void LED_on(int byte);
  void LED_off(int byte);
  void LED_off_all();
  void LED_on_all();

  int encoder_operation();
  bool button_state();
  bool button_state_hold();

private:
  sensing_result_entity_t *entity_ro;
  tgt_entity_t *tgt;
  ego_entity_t *ego;
  void LED_on_off(gpio_num_t gpio_num, int on_off);
};

#endif