#ifndef INTEGRATED_ENTITY_HPP
#define INTEGRATED_ENTITY_HPP

#include "defines.hpp"
#include "include/main_task.hpp"
#include "include/planning_task.hpp"
#include "include/sensing_task.hpp"

class IntegratedEntity {
public:
  IntegratedEntity();
  virtual ~IntegratedEntity();
  // MainTask getMainTask();
  // PlanningTask getPlanningTask();
  // SensingTask getSensingTask();

  // MainTask *getMainTaskPtr();
  // PlanningTask *getPlanningTaskPtr();
  // SensingTask *getSensingTaskPtr();

private:
  // MainTask mt;
  // PlanningTask pt;
  // SensingTask st;
};

#endif