/*
 * Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _OCTOMAP_SERVER_HPP_
#define _OCTOMAP_SERVER_HPP_

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <octomap/OcTreeNode.h>
#include <octomap/octomap_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_srvs/srv/empty.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <pcl_ros/transforms.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.h>
#include <octomap_msgs/srv/bounding_box_query.hpp>

#include <octomap_server2/conversions.h>

#include <laser_geometry/laser_geometry.hpp>

#include <fog_lib/params.h>
#include <octomap_server2/octomap_server.hpp>

#include <cmath>
//}

namespace octomap_server
{

/* defines //{ */

using vec3s_t = Eigen::Matrix<float, 3, -1>;
using vec3_t  = Eigen::Vector3f;

struct xyz_lut_t
{
  vec3s_t directions;  // a matrix of normalized direction column vectors
  vec3s_t offsets;     // a matrix of offset vectors
};

typedef struct
{
  double max_range;
  int    horizontal_rays;
} SensorParams2DLidar_t;

typedef struct
{
  double max_range;
  double vertical_fov;
  int    vertical_rays;
  int    horizontal_rays;
} SensorParams3DLidar_t;

typedef struct
{
  double max_range;
  double vertical_fov;
  double horizontal_fov;
  int    vertical_rays;
  int    horizontal_rays;
} SensorParamsDepthCam_t;

#ifdef COLOR_OCTOMAP_SERVER
using PCLPoint      = pcl::PointXYZRGB;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTree_t      = octomap::ColorOcTree;
#else
using PCLPoint      = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTree_t      = octomap::OcTree;
#endif

typedef enum
{

  LIDAR_3D,
  LIDAR_2D,
  LIDAR_1D,
  DEPTH_CAMERA,
  ULTRASOUND,

} SensorType_t;

//}

/* class OctomapServer //{ */

class OctomapServer : public rclcpp::Node {

public:
  OctomapServer(rclcpp::NodeOptions options);
  bool callbackResetMap(std_srvs::srv::Empty::Request& req, std_srvs::srv::Empty::Response& resp);

  void callback3dLidarCloud2(const sensor_msgs::msg::PointCloud2::UniquePtr msg, const SensorType_t sensor_type, const int sensor_id);
  void callbackLaserScan(const sensor_msgs::msg::LaserScan::UniquePtr msg);

private:
  // | --------------------------- TF ----------------------------|

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // | -------------------- topic subscribers ------------------- |

  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> sub_3dlaser_pc2_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> sub_depth_cam_pc2_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   sub_laser_scan_;

  // | ----------------------- publishers ----------------------- |

  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_map_global_full_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_map_global_binary_;

  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_map_local_full_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_map_local_binary_;

  // | -------------------- service serviers -------------------- |

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_reset_map_;

  // | ------------------------- timers ------------------------- |

  rclcpp::TimerBase::SharedPtr timer_global_map_;
  double                       _global_map_rate_;
  void                         timerGlobalMap();

  rclcpp::TimerBase::SharedPtr timer_local_map_;
  void                         timerLocalMap();

  rclcpp::TimerBase::SharedPtr timer_persistency_;
  void                         timerPersistency();

  rclcpp::TimerBase::SharedPtr timer_altitude_alignment_;
  void                         timerAltitudeAlignment();

  // | ----------------------- parameters ----------------------- |

  std::atomic<bool> is_initialized_     = false;
  std::atomic<bool> getting_laser_scan_ = false;
  rclcpp::Time      last_time_depth_cam_;
  rclcpp::Time      last_time_3d_laser;
  rclcpp::Time      last_time_laser_scan_;

  // | -------------------- callback groups ----------------------|

  // a shared pointer to each callback group has to be saved or the callbacks will never get called
  std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups_;
  // new callback groups have to be initialized using this function to be saved into callback_groups_
  rclcpp::CallbackGroup::SharedPtr new_cbk_grp();

  bool        _simulation_;
  std::string _uav_name_;

  double _robot_height_;

  bool        _persistency_enabled_;
  std::string _persistency_map_name_;
  double      _persistency_save_time_;

  bool   _persistency_align_altitude_enabled_;
  double _persistency_align_altitude_distance_;

  bool _global_map_publish_full_;
  bool _global_map_publish_binary_;

  bool _map_while_grounded_;

  double _local_map_size_;
  bool   _local_map_enabled_;
  bool   _local_map_publish_full_;
  bool   _local_map_publish_binary_;

  std::shared_ptr<OcTree_t> octree_;
  std::mutex                mutex_octree_;
  std::atomic<bool>         octree_initialized_;

  std::shared_ptr<OcTree_t> octree_local_;
  std::mutex                mutex_octree_local_;

  double     avg_time_cloud_insertion_ = 0;
  std::mutex mutex_avg_time_cloud_insertion_;

  double     time_last_local_map_processing_ = 0;
  std::mutex mutex_time_local_map_processing_;

  std::string _world_frame_;
  std::string _robot_frame_;
  double      octree_resolution_;
  bool        _global_map_compress_;
  std::string _map_path_;

  double _local_map_horizontal_distance_;
  double _local_map_vertical_distance_;
  double _local_map_rate_;
  double _local_map_max_computation_duty_cycle_;

  double local_map_horizontal_offset_ = 0;
  double local_map_vertical_offset_   = 0;

  bool   _unknown_rays_update_free_space_;
  bool   _unknown_rays_clear_occupied_;
  double _unknown_rays_distance_;

  int        resolution_fractor_;
  std::mutex mutex_resolution_fractor_;

  laser_geometry::LaserProjection projector_;

  bool copyInsideBBX2(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to, const octomap::point3d& p_min, const octomap::point3d& p_max);

  octomap::OcTreeNode* touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key, unsigned int depth,
                                       unsigned int max_depth);

  octomap::OcTreeNode* touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth);

  void expandNodeRecursive(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const unsigned int node_depth);

  std::optional<double> getGroundZ(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y);

  bool translateMap(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y, const double& z);

  bool createLocalMap(const std::string frame_id, const double horizontal_distance, const double vertical_distance, std::shared_ptr<OcTree_t>& octree);

  virtual void insertPointCloud(const geometry_msgs::msg::Vector3& sensorOrigin, const PCLPointCloud::ConstPtr& cloud,
                                const PCLPointCloud::ConstPtr& free_cloud);

  void initialize3DLidarLUT(xyz_lut_t& lut, const SensorParams3DLidar_t sensor_params);
  void initializeDepthCamLUT(xyz_lut_t& lut, const SensorParamsDepthCam_t sensor_params);

  int n_sensors_2d_lidar_;
  int n_sensors_3d_lidar_;
  int n_sensors_depth_cam_;

  std::vector<xyz_lut_t> sensor_2d_lidar_xyz_lut_;

  std::vector<xyz_lut_t> sensor_3d_lidar_xyz_lut_;

  std::vector<xyz_lut_t> sensor_depth_camera_xyz_lut_;

  std::vector<SensorParams2DLidar_t> sensor_params_2d_lidar_;

  std::vector<SensorParams3DLidar_t> sensor_params_3d_lidar_;

  std::vector<SensorParamsDepthCam_t> sensor_params_depth_cam_;

  // sensor model
  double _probHit_;
  double _probMiss_;
  double _thresMin_;
  double _thresMax_;
};

//}

}  // namespace octomap_server

#endif  // _OCTOMAP_SERVER_HPP_
