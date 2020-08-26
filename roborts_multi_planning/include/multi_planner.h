//
// Created by noorall on 2020/8/13.
//

#ifndef PATH_PLANING_WS_MULTI_PLANNER_H
#define PATH_PLANING_WS_MULTI_PLANNER_H

#include <vector>
#include "ros/ros.h"
#include "roborts_msgs/UpdateCellInfos.h"
#include "roborts_msgs/GetCellInfos.h"
#include "roborts_msgs/CleanCellInfos.h"
#include "roborts_msgs/UpdateCurrentIndex.h"
#include "roborts_msgs/GetCurrentIndex.h"
#include "roborts_msgs/GetCurrentPath.h"

namespace multi_planner {
class MultiPlanner {
 public:
  MultiPlanner();

  ~MultiPlanner();

  void run();

 private:
  /**
   * @brief Update the path information of the specified robot to the shared
   * array.
   *
   * @param request The request of the service.
   * @param response The response of the service.
   * @return True will be returned when the information is successfully updated.
   * */
  bool UpdateCellInfos(roborts_msgs::UpdateCellInfos::Request &request,
                       roborts_msgs::UpdateCellInfos::Response &response);

  /**
   * @brief Get the occupancy information of the global map.
   *
   * @param request The request of the service.
   * @param response The response of the service.
   * @return True will be returned when the information is successfully gotten.
   * */
  bool GetCellInfos(roborts_msgs::GetCellInfos::Request &request,
                    roborts_msgs::GetCellInfos::Response &response);

  /**
   * @brief Clear the path information of the specified robot.
   *
   * @param request The request of the service.
   * @param response The response of the service.
   * @return True will be returned when the information is successfully cleared.
   * */
  bool CleanCellInfos(roborts_msgs::CleanCellInfos::Request &request,
                      roborts_msgs::CleanCellInfos::Response &response);

  /**
   * @brief Update the index information of the designated robot on the path.
   *
   * @param request The request of the service.
   * @param response The response of the service.
   * @return True will be returned when the information is successfully updated.
   * */
  bool UpdateCurrentIndex(roborts_msgs::UpdateCurrentIndex::Request &request,
                          roborts_msgs::UpdateCurrentIndex::Response &response);

  /**
   * @brief Get the current index information of the specified robot.
   *
   * @param request The request of the service.
   * @param response The response of the service.
   * @return True will be returned when the information is successfully gotten.
   * */
  bool GetCurrentIndex(roborts_msgs::GetCurrentIndex::Request &request,
                       roborts_msgs::GetCurrentIndex::Response &response);

  /**
   * @brief Get the current path information of the specified robot.
   *
   * @param request The request of the service.
   * @param response The response of the service.
   * @return True will be returned when the information is successfully gotten.
   * */
  bool GetCurrentPath(roborts_msgs::GetCurrentPath::Request &request,
                      roborts_msgs::GetCurrentPath::Response &response);

  //! The width of the global map.
  int width_;
  //! The cell with index and cost of the global map.
  std::vector<roborts_msgs::CellInfo> cell_info_;
  //! The current path info of robots.
  std::map<std::string, std::vector<roborts_msgs::CellInfo>> path_info_;
  //! Robots' current index
  std::map<std::string, int> current_index_;
};
}
#endif //PATH_PLANING_WS_MULTI_PLANNER_H
