#ifndef ENUMS_HPP
#define ENUMS_HPP

enum class SearchMode : int {
  ALL = 0,
  Kata = 1,
  Return = 2,
};

enum class LogFileType : int {
  SLALOM = 0,
  STRAIGHT = 1,
};

enum class SensorCtrlType : int {
  NONE = 0,
  Straight = 1,
  Dia = 2,
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

enum class MotionDirection : int {
  NONE = 0,
  RIGHT = 1,
  LEFT = 2,
};

#endif