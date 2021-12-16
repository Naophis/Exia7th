#ifndef Adachi_H
#define Adachi_H

#include "logic.hpp"
#include "maze_solver.hpp"
#include "stdio.h"
#include <unordered_map>
#include <vector>

using namespace std;

class Adachi {
private:
  char next_direction;
  int dist_val;
  // void setNextDirection(int x, int y, int x2, int y2, int dir, int &next_dir,
  //                       int &val);
  void setNextDirection(int x2, int y2, Direction dir, Direction &next_dir,
                        int &val);
  void setNextDirection2(int x2, int y2, Direction dir, Direction &next_dir,
                         int &val);
  bool goaled = false;
  bool goal_startpos_lock = false;
  point_t next_goal_pt ;
  int limit = 256;
  int limit2 = 25;

  // void create_path(path_type &path, Motion motion);
  // void clear_path(path_type &path);

public:
  Adachi(/* args */);
  ~Adachi();
  void set_logic(MazeSolverBaseLgc *_logic);
  void set_ego(ego_t *_ego);
  Direction detect_next_direction();
  Motion get_next_motion(Direction next_dir);
  void get_next_pos(Direction next_dir);
  int exec(path_type &path);
  void back_home();
  ego_t *ego;
  vector<point_t> pt_list;
  vector<point_t> start_pt_list;
  bool is_go_home();
  void deadEnd(int egox, int egoy);
  bool is_goal(int x, int y);
  bool is_goaled();
  MazeSolverBaseLgc *lgc;
  int cost_mode = 0;
  unordered_map<unsigned int, unsigned char> subgoal_list;
};

#endif