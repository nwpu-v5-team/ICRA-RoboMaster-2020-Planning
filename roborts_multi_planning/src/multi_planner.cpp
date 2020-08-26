//
// Created by noorall on 2020/8/13.
//

#include "multi_planner.h"

namespace multi_planner {
  MultiPlanner::MultiPlanner() {
    cell_info_.resize(415161);
    width_ = 849;
  }

  MultiPlanner::~MultiPlanner() {}

  bool MultiPlanner::UpdateCellInfos(
          roborts_msgs::UpdateCellInfos::Request &request,
          roborts_msgs::UpdateCellInfos::Response &response) {
    for (auto cell_info : request.cell_infos) {
      cell_info_.at(cell_info.index) = cell_info;
    }
    if (path_info_.find(request.robot_id) == path_info_.end()) {
      path_info_.insert(
              std::pair<std::string, std::vector<roborts_msgs::CellInfo>>(
                      request.robot_id, request.cell_infos));
    } else {
      path_info_.at(request.robot_id) = request.cell_infos;
    }
    if (current_index_.find(request.robot_id) != current_index_.end()) {
      current_index_.at(request.robot_id) = -1;
    }
    response.result = 1;
    return true;
  }

  bool MultiPlanner::GetCellInfos(
          roborts_msgs::GetCellInfos::Request &request,
          roborts_msgs::GetCellInfos::Response &response) {
    response.cell_infos = cell_info_;
    return true;
  }

  bool MultiPlanner::CleanCellInfos(
          roborts_msgs::CleanCellInfos::Request &request,
          roborts_msgs::CleanCellInfos::Response &response) {
    if (path_info_.find(request.robot_id) != path_info_.end()) {
      for (auto cell_info : path_info_.at(request.robot_id)) {
        cell_info_.at(cell_info.index).index = cell_info.index;
        cell_info_.at(cell_info.index).cost_g = 0;
      }
    }
    response.result = 1;
    return true;
  }

  bool MultiPlanner::UpdateCurrentIndex(
          roborts_msgs::UpdateCurrentIndex::Request &request,
          roborts_msgs::UpdateCurrentIndex::Response &response) {
    auto get_euclidean_distance = [&](int &my_index, int &syne_index) {
      int x = abs(static_cast<long>(my_index) / width_ - syne_index / width_);
      int y = abs(static_cast<long>(my_index) % width_ - syne_index % width_);
      return sqrt(x * x + y * y);
    };
    if (path_info_.find(request.robot_id) != path_info_.end()) {
      int i = 0;
      for (auto cell : path_info_.at(request.robot_id)) {
        i++;
        if (get_euclidean_distance(cell.index, request.index) < 70) {
          if (current_index_.find(request.robot_id) == current_index_.end()) {
            current_index_.insert(
                    std::pair<std::string, int>(request.robot_id, cell.index));
          } else {
            current_index_.at(request.robot_id) = cell.index;
          }
          for (int j = i; j >= 0; j--) {
            cell_info_.at(path_info_.at(request.robot_id).at(i).index).cost_g
                    = 0;
          }
          path_info_.at(request.robot_id).erase(path_info_.at(request.robot_id).begin(),
                                                path_info_.at(request.robot_id).begin()
                                                + i);
          if (path_info_.at(request.robot_id).size() < 10) {
            current_index_.at(request.robot_id) = -1;
            path_info_.clear();
          }
          break;
        }
      }
    }
    response.result = 1;
    return true;
  }

  bool MultiPlanner::GetCurrentIndex(
          roborts_msgs::GetCurrentIndex::Request &request,
          roborts_msgs::GetCurrentIndex::Response &response) {
    if (current_index_.find(request.robot_id) == current_index_.end()) {
      response.result = -1;
    } else {
      response.result = current_index_.at(request.robot_id);
      ROS_INFO("%s:%d", request.robot_id.c_str(), response.result);
    }
    return true;
  }

  bool MultiPlanner::GetCurrentPath(
          roborts_msgs::GetCurrentPath::Request &request,
          roborts_msgs::GetCurrentPath::Response &response) {
    if (path_info_.find(request.robot_id) != path_info_.end()) {
      response.path_info = path_info_.at(request.robot_id);
    }
    return true;
  }

  void MultiPlanner::run() {
    ros::NodeHandle nh;
    auto update_cell_infos_server = nh.advertiseService(
            "update_cell_infos", &MultiPlanner::UpdateCellInfos, this);
    auto get_cell_infos_server =
            nh.advertiseService("get_cell_infos", &MultiPlanner::GetCellInfos, this);
    auto clean_cell_infos_server = nh.advertiseService(
            "clean_cell_infos", &MultiPlanner::CleanCellInfos, this);
    auto update_current_index = nh.advertiseService(
            "update_current_index", &MultiPlanner::UpdateCurrentIndex, this);
    auto get_current_index = nh.advertiseService(
            "get_current_index", &MultiPlanner::GetCurrentIndex, this);
    auto get_current_path = nh.advertiseService(
            "get_current_path", &MultiPlanner::GetCurrentPath, this);
    ROS_INFO("Service start!");
    ros::spin();
  }
}  // namespace multi_planner
