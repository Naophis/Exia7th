#include "include/main_task.hpp"

MainTask::MainTask() {}

MainTask::~MainTask() {}
void MainTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "main_task", 8192, this, 2, &handle,
                          xCoreID);
}
void MainTask::task_entry_point(void *task_instance) {
  static_cast<MainTask *>(task_instance)->task();
}

void MainTask::set_sensing_entity(sensing_entity_t *_entity) {
  entity = _entity; //
}
void MainTask::set_ego_entity(ego_entity_t *_ego) {
  ego = _ego; //
}
void MainTask::set_planning_task(PlanningTask *_pt) { //
  pt = _pt;
}
void MainTask::task() {
  const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

  while (1) {
    printf("%c[2J", ESC);   /* 画面消去 */
    printf("%c[0;0H", ESC); /* 戦闘戻す*/
    printf("SW1 %d \n", gpio_get_level(SW1));

    printf("gyro: %d\n", entity->gyro.raw);
    printf("battery: %0.3f\n", entity->battery.data);
    printf("encoder: %d, %d\n", entity->encoder.left, entity->encoder.right);
    printf("sensor: %d, %d, %d, %d, %d\n", entity->led_sen.left90.raw,
           entity->led_sen.left45.raw, entity->led_sen.front.raw,
           entity->led_sen.right45.raw, entity->led_sen.right90.raw);

    printf("ego_v: %0.3f, %0.3f, %0.3f, %0.3f\n", ego->v_l, ego->v_c, ego->v_r,
           ego->dist);
    printf("ego_w: %0.3f, %0.3f, %0.3f deg\n", ego->w, ego->angle,
           ego->angle * 180 / PI);

    if (gpio_get_level(SW1)) {
      gpio_set_level(A_CW_CCW, 1);
      gpio_set_level(B_CW_CCW, 1);
      gpio_set_level(SUCTION_PWM, 0);
      pt->suction_disable();
      pt->motor_disable();
    } else {
      gpio_set_level(A_CW_CCW, 0);
      gpio_set_level(B_CW_CCW, 0);
      gpio_set_level(SUCTION_PWM, 1);
      ego->angle = 0;
      ego->dist = 0;
      pt->suction_enable();
      pt->motor_enable();
    }

    vTaskDelay(xDelay);
  }
}
