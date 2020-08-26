//
// Created by noorall on 2020/8/13.
//
#include "multi_planner.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "multi_planner_service");
  multi_planner::MultiPlanner multiPlanner;
  multiPlanner.run();
  return 0;
}