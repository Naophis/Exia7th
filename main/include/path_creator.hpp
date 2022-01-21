#ifndef PATH_CREATOR_H
#define PATH_CREATOR_H

#include "include/defines.hpp"
#include "stdio.h"
#include <vector>

#include "logic.hpp"
#include "maze_solver.hpp"

// constexpr int checkQlength = 256;
constexpr int R = 1;
constexpr int L = 2;
constexpr unsigned int vector_max_step_val = 180 * 1024;
constexpr float MAX = vector_max_step_val;
class PathCreator {
private:
  int get_dist_val(int x, int y);
  void updateVectorMap(const bool isSearch);

  void add_path_s(int idx, float val);
  void append_path_s(float val);
  void append_path_t(int val);
  void setNextRootDirectionPath(int x, int y, Direction now_dir, Direction dir,
                                float &val, Direction &next_dir);
  Direction get_next_pos(int &x, int &y, Direction dir,
                         Direction next_direction);

  Motion get_next_motion(Direction now_dir, Direction next_direction);

  void priorityStraight2(int x, int y, Direction now_dir, Direction dir,
                         float &dist_val, Direction &next_dir);

  void pathOffset();

public:
  std::shared_ptr<MazeSolverBaseLgc> lgc;
  void set_logic(std::shared_ptr<MazeSolverBaseLgc> &_lgc);

  std::vector<float> path_s;
  std::vector<int> path_t;
  int path_size;

  PathCreator(/* args */);
  ~PathCreator();
  void path_create(bool is_search);
  void path_reflash();
  void convert_large_path(bool b1);
  void diagonalPath(bool isFull, bool a1);
  void print_path();
};

#endif