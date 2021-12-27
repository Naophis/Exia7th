#ifndef MotionPlanning_HPP
#define MotionPlanning_HPP

#include "include/defines.hpp"
class MotionPlanning {
public:
  MotionPlanning() {}
  virtual ~MotionPlanning() {}
  void set_tgt_entity(tgt_entity_t *_tgt) { tgt = _tgt; }

private:
  tgt_entity_t *tgt;
};
#endif