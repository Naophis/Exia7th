#ifndef SENSING_TASK_HPP
#define SENSING_TASK_HPP

#include "base_task.hpp"
#include "defines.hpp"

class SensingTask : public BaseTask {
public:
  SensingTask() : BaseTask("sensing task", 1, 8192) {}

  virtual ~SensingTask() {}

private:
  virtual void task() {
    const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
    int i = 0;
    while (1) {
      if (i % 2 == 0) {
        gpio_set_level(LED5, 1);
        gpio_set_level(LED4, 0);
      } else {
        gpio_set_level(LED4, 1);
        gpio_set_level(LED5, 0);
      }
      i++;
      vTaskDelay(xDelay);
    }
  }
};

#endif