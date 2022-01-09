#ifndef ENUMS_HPP
#define ENUMS_HPP

enum class LogFileType : int {
  SLALOM = 0,
  STRAIGHT = 1,
};

enum class RUN_MODE2 : int {
  NONE_MODE = 0,
  KEEP = 0,
  SLAROM_RUN = 1,
  PIVOT_TURN = 2,
  ST_RUN = 3,
  SLALOM_RUN2 = 4,
};

enum class MotionType : char {
  NONE = 0,
  STRAIGHT = 1,
  PIVOT = 2,
  SLA_FRONT_STR = 3,
  SLALOM = 4,
  SLA_BACK_STR = 5,
  WALL_OFF = 6,
};

enum class FailSafe : int {
  NONE = 0,
  ERROR = 1,
};
enum class MotionResult : int {
  NONE = 0,
  ERROR = 1,
};

#endif