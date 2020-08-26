//
// Created by heihei on 2020/5/8.
//

#ifndef ROBORTS_COSTMAP_INCLUDE_COSTMAP_BUFFER_LAYER_H_
#define ROBORTS_COSTMAP_INCLUDE_COSTMAP_BUFFER_LAYER_H_

#include "costmap_layer.h"
#include "map_common.h"
#include <map>
#include <nav_msgs/OccupancyGrid.h>
#include <roborts_msgs/BuffZoneStatus.h>

namespace roborts_costmap {

class BufferLayer : public CostmapLayer {

public:
  BufferLayer() {}
  virtual ~BufferLayer() {}
  virtual void OnInitialize();
  virtual void Activate();
  virtual void Deactivate();
  virtual void Reset();
  virtual void UpdateCosts(Costmap2D &master_grid, int min_i, int min_j,
                           int max_i, int max_j);

  virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw,
                            double *min_x, double *min_y, double *max_x,
                            double *max_y);

  void InComingMap(const nav_msgs::OccupancyGridConstPtr &new_map);

  void UpdateBuffZoneStatus(
      const roborts_msgs::BuffZoneStatus &new_buff_zone_status);

private:
  std::string global_frame_;
  std::string map_frame_;
  std::string map_topic_;
  bool subscribe_to_updates_;
  bool map_received_;
  bool has_updated_data_;
  unsigned int staic_layer_x_, staic_layer_y_, width_, height_;
  bool use_maximum_;
  bool first_map_only_;
  ros::Subscriber map_sub_, map_update_sub_;
  ros::Subscriber buff_zone_sub_;

  std::map<int, BuffZone> map_buff_zones_;
  std::string robot_color_;
  double buff_zone_smaller_;

  std::vector<uint8_t> vec_last_buff_debuff_status_;
  std::vector<bool> vec_last_buff_active_;
};

} // namespace roborts_costmap

#endif // ROBORTS_COSTMAP_INCLUDE_COSTMAP_BUFFER_LAYER_H_
