#ifndef Adachi_H
#define Adachi_H

#include "include/defines.hpp"
#include "logic.hpp"
#include "maze_solver.hpp"
#include "stdio.h"
#include <unordered_map>
#include <vector>

using namespace std;

class Adachi {

public:
  Adachi();
  ~Adachi();
  void set_logic(std::shared_ptr<MazeSolverBaseLgc> &_lgc);
  void set_ego(std::shared_ptr<ego_t> &_ego);
  Direction detect_next_direction();
  Motion get_next_motion(Direction next_dir);
  void get_next_pos(Direction next_dir);
  // int exec(path_type &path);
  Motion exec();
  void back_home();
  bool is_go_home();
  void deadEnd(int egox, int egoy);
  bool is_goal(int x, int y);
  bool is_goaled();
  void reset_goal() { goaled = false; }

private:
  void setNextDirection(int x2, int y2, Direction dir, Direction &next_dir,
                        int &val);
  void setNextDirection2(int x2, int y2, Direction dir, Direction &next_dir,
                         int &val);
  bool goaled = false;
  bool goal_startpos_lock = false;
  char next_direction;
  int dist_val;
  int limit = 256;
  int limit2 = 25;
  int cost_mode = 0;

  point_t next_goal_pt;

  std::shared_ptr<MazeSolverBaseLgc> lgc;
  std::shared_ptr<ego_t> ego;

  vector<point_t> pt_list;
  vector<point_t> start_pt_list;
  unordered_map<unsigned int, unsigned char> subgoal_list;
};

#endif