#ifndef MAZE_SOLVER_H
#define MAZE_SOLVER_H

#include "stdio.h"
#include <vector>
#include <cmath>

using namespace std;
constexpr unsigned char MAX_MAZE_SIZE = 32;
constexpr double ROOT2 = sqrt(2);
constexpr double dt = 0.001;
constexpr double eps = 0.001;
constexpr double PI = 3.141592653589793238;

typedef struct {
  float n;
  float e;
  float w;
  float s;
  unsigned int v : 8;
  unsigned int N1 : 4;
  unsigned int NE : 4;
  unsigned int E1 : 4;
  unsigned int SE : 4;
  unsigned int S1 : 4;
  unsigned int SW : 4;
  unsigned int W1 : 4;
  unsigned int NW : 4;
  unsigned int step : 4;
} vector_map_t;

enum class Direction : int {
  North = 1,
  East = 2,
  NorthEast = 3,
  West = 4,
  NorthWest = 5,
  SouthEast = 6,
  SouthWest = 7,
  South = 8,
  Undefined = 255,
  Null = 0
};

enum class Motion : int {
  NONE = 0,
  Straight = 1,
  TurnRight = 2,
  TurnLeft = 3,
  Back = 4,
};

enum class TurnDirection : int {
  None = 0,
  Right = 1,
  Left = 2,
  Pivot180 = 128,
  End = 256,
};

enum class PathMotion : int {
  Right = 1,
  Left = 2,
  Pivot180 = 128,
  End = 255,
};

enum class TurnType : int {
  None = 0,
  Normal = 1,
  Orval = 2,
  Large = 3,
  Dia45 = 4,
  Dia135 = 5,
  Dia90 = 6,
  Kojima = 7,
  Finish = 255,
};

typedef struct {
  vector<float> s;
  vector<int> t;
  int size;
} path_type;

typedef struct {
  unsigned char x;
  unsigned char y;
} point_t;

typedef struct {
  unsigned char x;
  unsigned char y;
  Direction dir;
  float dist2;
} dir_pt_t;

typedef struct {
  unsigned char x;
  unsigned char y;
  Direction dir;
  unsigned char prev_motion;
} ego_t;

typedef struct {
  float s;
  unsigned char t;
} path_element;

typedef struct {
  std::vector<path_element> paths;
  int size;
} path_struct;

typedef struct {
  float x;
  float y;
  float ang;
  float base_ang_accl;
  char type;
} trajectory_point_t;

constexpr initializer_list<Direction> direction_list = {
    Direction::North, Direction::East, Direction::West, Direction::South};

typedef struct {
  float x;
  float y;
  float ang;
  Direction dir;
} ego_odom_t;

typedef struct {
  float radius;
  float time;
  float n;
  float front;
  float back;
} slalom_param_t;

typedef struct {
  slalom_param_t normal;
  slalom_param_t orval;
  slalom_param_t large;
  slalom_param_t dia45;
  slalom_param_t dia45_2;
  slalom_param_t dia135;
  slalom_param_t dia135_2;
  slalom_param_t dia90;
} slalom_data_t;

typedef struct {
  float v_max;
  float accl;
  float decel;
  float turn_v;
} run_param_t;

typedef struct {
  vector<trajectory_point_t> normal;
  vector<trajectory_point_t> orval;
  vector<trajectory_point_t> large;
  vector<trajectory_point_t> dia45;
  vector<trajectory_point_t> dia45_2;
  vector<trajectory_point_t> dia135;
  vector<trajectory_point_t> dia135_2;
  vector<trajectory_point_t> dia90;
} base_trajectory_pattern_t;
#endif
