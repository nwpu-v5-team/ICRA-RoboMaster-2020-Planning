/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H
#define ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "proto/a_star_planner_config.pb.h"

#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"
#include "costmap/costmap_interface.h"

#include "../global_planner_base.h"

#include "roborts_msgs/CellInfo.h"
#include "roborts_msgs/UpdateCellInfos.h"
#include "roborts_msgs/GetCellInfos.h"
#include "roborts_msgs/CleanCellInfos.h"
#include "roborts_msgs/GetCurrentIndex.h"
#include "roborts_msgs/GetCurrentPath.h"

namespace roborts_global_planner {

/**
 * @brief Global planner alogorithm class for A star under the representation of costmap
 */
class AStarPlanner : public GlobalPlannerBase {
 public:
  /**
   * @brief Constructor of A star planner, set the costmap pointer and relevant
   * costmap size.
   * @param costmap_ptr The shared pointer of costmap interface
   */
  AStarPlanner(CostmapPtr costmap_ptr);

  virtual ~AStarPlanner();

  /**
   * @brief Main Plan function(override the base-class function)
   * @param start Start pose input
   * @param goal Goal pose input
   * @param path Global plan path output
   * @return ErrorInfo which is OK if succeed
   */
  roborts_common::ErrorInfo Plan(const geometry_msgs::PoseStamped &start,
                                 const geometry_msgs::PoseStamped &goal,
                                 std::vector<geometry_msgs::PoseStamped> &path,
                                 const std::string &robot_id,
                                 const std::string &syne_robot_id,
                                 const bool is_clean);

 private:
  /**
   * @brief State enumerate for the cell.
   */
  enum SearchState {
    NOT_HANDLED, /**< The cell is not handled.*/
    OPEN,        /**< The cell is in open priority queue.*/
    CLOSED       /**< The cell is in close queue.*/
  };

  /**
   * @brief Plan based on 1D Costmap list. Input the index in the costmap and
   * get the plan path.
   * @param start_index start pose index in the 1D costmap list
   * @param goal_index goal pose index in the 1D costmap list
   * @param path plan path output
   * @return ErrorInfo which is OK if succeed
   */
  roborts_common::ErrorInfo SearchPath(
      const int &start_index, const int &goal_index,
      std::vector<geometry_msgs::PoseStamped> &path,
      const std::string &robot_id, const std::string &syne_robot_id,
      const bool is_clean);

  /**
   * @brief Calculate the cost for the diagonal or parallel movement.
   * @param current_index Index of the current cell as input
   * @param neighbor_index Index of the neighbor cell as input
   * @param move_cost Movement cost as output
   * @return ErrorInfo which is OK if succeed
   */
  roborts_common::ErrorInfo GetMoveCost(const int &current_index,
                                        const int &neighbor_index,
                                        int &move_cost) const;

  /**
   * @brief Calculate the Manhattan distance between two cell index used as the
   * heuristic function of A star algorithm.
   * @param index1 Index of the first cell as input
   * @param index2 Index of the second cell as input
   * @param manhattan_distance The Manhattan distance as output
   */
  void GetManhattanDistance(const int &index1, const int &index2,
                            int &manhattan_distance) const;

  /**
   * @brief Get the index of nine neighbor cell from the current cell
   * @param current_index Index of the current cell as input
   * @param neighbors_index Index of the neighbor cells as out
   */
  void GetNineNeighbors(const int &current_index,
                        std::vector<int> &neighbors_index) const;

  /**
   * @brief Get the index of the nearest cell that the robot passed
   * @param robot_id Id of the robot
   */
  int GetHandledIndex(const std::string &robot_id);

  /**
   * @brief Check whether there is a path conflict between the current robot and
   * the collaborative robot.
   * @return True will be returned when the path does not conflict.
   */
  bool CheckPath(const int &handled_index, const int &current_index);

  /**
   * @brief Get the Euclidean distance of our two robots.
   * @param my_index The index of our robot in the global map.
   * @param syne_index Index of synergy robot in global map.
   * @return the Euclidean distance of our two robots.
   * */
  double GetEuclideanDistance(const int &my_index, const int &syne_index);

  /**
   * @brief Used for priority queue compare process.
   */
  struct Compare {
    bool operator()(const int &index1, const int &index2) {
      return AStarPlanner::f_score_.at(index1) >
             AStarPlanner::f_score_.at(index2);
    }
  };

  //! heuristic_factor_
  float heuristic_factor_;
  //! inaccessible_cost
  unsigned int inaccessible_cost_;
  //! goal_search_tolerance
  unsigned int goal_search_tolerance_;
  //! gridmap height size
  unsigned int gridmap_height_;
  //! gridmap height width
  unsigned int gridmap_width_;
  //! gridmap cost array
  unsigned char *cost_;
  //! search algorithm related f score, f_score = g_score +
  //! heuristic_cost_estimate
  static std::vector<int> f_score_;
  //! search algorithm related g score, which refers to the score from start
  //! cell to current cell
  std::vector<int> g_score_;
  //! vector that indicates the parent cell index of each cell
  std::vector<int> parent_;
  //! vector that indicates the state of each cell
  std::vector<AStarPlanner::SearchState> state_;
  //! vector that indicates the cost and index information of each cell;
  std::vector<roborts_msgs::CellInfo> cell_infos_;
  //! vector that indicates the path info;
  std::vector<roborts_msgs::CellInfo> path_info_;
  //! nodehandle of ros;
  ros::NodeHandle nh_;
  //! client of clean_cell_infos service;
  ros::ServiceClient clean_client_;
  //! client of get_cell_infos service;
  ros::ServiceClient get_client_;
  //! client of update_cell_infos service;
  ros::ServiceClient update_client_;
  //! client of get_current_index service;
  ros::ServiceClient get_index_client_;
  //! client of get_current_path service;
  ros::ServiceClient get_path_client_;
  //! srv of clean_cell_infos service;
  roborts_msgs::CleanCellInfos clean_srv_;
  //! srv of get_cell_infos service;
  roborts_msgs::GetCellInfos get_srv_;
  //! srv of update_cell_infos service;
  roborts_msgs::UpdateCellInfos update_srv_;
  //! srv of get_current_index service;
  roborts_msgs::GetCurrentIndex get_index_srv_;
  //! srv of get_current_path service;
  roborts_msgs::GetCurrentPath get_path_srv_;
  //! store the intermediate variable of cell info;
  roborts_msgs::CellInfo cell_info_;
  //! store path information of synergy robots;
  std::vector<roborts_msgs::CellInfo> syne_path_;
};

std::vector<int> AStarPlanner::f_score_;
roborts_common::REGISTER_ALGORITHM(
    GlobalPlannerBase, "a_star_planner", AStarPlanner,
    std::shared_ptr<roborts_costmap::CostmapInterface>);

}  // namespace roborts_global_planner

#endif  // ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H
