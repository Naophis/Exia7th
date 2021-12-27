#ifndef UI_HPP
#define UI_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/defines.hpp"

constexpr double MOTION_CHECK_TH = 1000;
class UserInterface {
public:
  UserInterface(){};
  virtual ~UserInterface(){};

  void motion_check();
  void set_sensing_entity(sensing_entity_t *_entity);
  void set_tgt_entity(tgt_entity_t *_tgt);

  void LED_on(int byte);
  void LED_off(int byte);
  void LED_off_all();
  void LED_on_all();

private:
  sensing_entity_t *entity_ro;
  tgt_entity_t *tgt;
  void LED_on_off(gpio_num_t gpio_num, int on_off);
};

#endif