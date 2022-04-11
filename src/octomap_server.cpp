#include <octomap_server2/octomap_server.hpp>

using namespace fog_lib;
namespace octomap_server
{

/* OctomapServer constructor //{ */
OctomapServer::OctomapServer(rclcpp::NodeOptions options) : Node("octomap_server2", options) {

  RCLCPP_INFO(get_logger(), "[OctomapServer]: Initializing...");
  RCLCPP_INFO(get_logger(), "-------------- Loading parameters --------------");
  bool loaded_successfully = true;

  /* parse params from config file //{ */
  loaded_successfully &= parse_param("map_while_grounded", _map_while_grounded_, *this);

  loaded_successfully &= parse_param("global_map.rate", _global_map_rate_, *this);
  loaded_successfully &= parse_param("global_map.compress", _global_map_compress_, *this);
  loaded_successfully &= parse_param("global_map.publish_full", _global_map_publish_full_, *this);
  loaded_successfully &= parse_param("global_map.publish_binary", _global_map_publish_binary_, *this);

  loaded_successfully &= parse_param("local_map.enabled", _local_map_enabled_, *this);
  loaded_successfully &= parse_param("local_map.horizontal_distance", _local_map_horizontal_distance_, *this);
  loaded_successfully &= parse_param("local_map.vertical_distance", _local_map_vertical_distance_, *this);
  loaded_successfully &= parse_param("local_map.rate", _local_map_rate_, *this);
  loaded_successfully &= parse_param("local_map.max_computation_duty_cycle", _local_map_max_computation_duty_cycle_, *this);
  loaded_successfully &= parse_param("local_map.publish_full", _local_map_publish_full_, *this);
  loaded_successfully &= parse_param("local_map.publish_binary", _local_map_publish_binary_, *this);

  loaded_successfully &= parse_param("mapping.resolution", octree_resolution_, *this);
  loaded_successfully &= parse_param("mapping.initial_fractor", resolution_fractor_, *this);
  loaded_successfully &= parse_param("world_frame_id", _world_frame_, *this);
  loaded_successfully &= parse_param("robot_frame_id", _robot_frame_, *this);

  loaded_successfully &= parse_param("unknown_rays.update_free_space", _unknown_rays_update_free_space_, *this);
  loaded_successfully &= parse_param("unknown_rays.clear_occupied", _unknown_rays_clear_occupied_, *this);
  loaded_successfully &= parse_param("unknown_rays.ray_distance", _unknown_rays_distance_, *this);

  loaded_successfully &= parse_param("sensor_params.2d_lidar.n_sensors", n_sensors_2d_lidar_, *this);
  loaded_successfully &= parse_param("sensor_params.3d_lidar.n_sensors", n_sensors_3d_lidar_, *this);
  loaded_successfully &= parse_param("sensor_params.depth_camera.n_sensors", n_sensors_depth_cam_, *this);

  for (int i = 0; i < n_sensors_2d_lidar_; i++) {

    std::stringstream max_range_param_name;
    max_range_param_name << "sensor_params.2d_lidar.sensor_" << i << ".max_range";

    std::stringstream horizontal_rays_param_name;
    horizontal_rays_param_name << "sensor_params.2d_lidar.sensor_" << i << ".horizontal_rays";

    SensorParams2DLidar_t params;

    loaded_successfully &= parse_param(max_range_param_name.str(), params.max_range, *this);
    loaded_successfully &= parse_param(horizontal_rays_param_name.str(), params.horizontal_rays, *this);

    sensor_params_2d_lidar_.push_back(params);
  }

  for (int i = 0; i < n_sensors_depth_cam_; i++) {

    std::stringstream max_range_param_name;
    max_range_param_name << "sensor_params.depth_camera.sensor_" << i << ".max_range";

    std::stringstream horizontal_rays_param_name;
    horizontal_rays_param_name << "sensor_params.depth_camera.sensor_" << i << ".horizontal_rays";

    std::stringstream vertical_rays_param_name;
    vertical_rays_param_name << "sensor_params.depth_camera.sensor_" << i << ".vertical_rays";

    std::stringstream hfov_param_name;
    hfov_param_name << "sensor_params.depth_camera.sensor_" << i << ".horizontal_fov_angle";

    std::stringstream vfov_param_name;
    vfov_param_name << "sensor_params.depth_camera.sensor_" << i << ".vertical_fov_angle";

    SensorParamsDepthCam_t params;

    loaded_successfully &= parse_param(max_range_param_name.str(), params.max_range, *this);
    loaded_successfully &= parse_param(horizontal_rays_param_name.str(), params.horizontal_rays, *this);
    loaded_successfully &= parse_param(vertical_rays_param_name.str(), params.vertical_rays, *this);
    loaded_successfully &= parse_param(hfov_param_name.str(), params.horizontal_fov, *this);
    loaded_successfully &= parse_param(vfov_param_name.str(), params.vertical_fov, *this);

    sensor_params_depth_cam_.push_back(params);
  }

  for (int i = 0; i < n_sensors_3d_lidar_; i++) {

    std::stringstream max_range_param_name;
    max_range_param_name << "sensor_params.3d_lidar.sensor_" << i << ".max_range";

    std::stringstream horizontal_rays_param_name;
    horizontal_rays_param_name << "sensor_params.3d_lidar.sensor_" << i << ".horizontal_rays";

    std::stringstream vertical_rays_param_name;
    vertical_rays_param_name << "sensor_params.3d_lidar.sensor_" << i << ".vertical_rays";

    std::stringstream vfov_param_name;
    vfov_param_name << "sensor_params.3d_lidar.sensor_" << i << ".vertical_fov_angle";

    SensorParams3DLidar_t params;

    loaded_successfully &= parse_param(max_range_param_name.str(), params.max_range, *this);
    loaded_successfully &= parse_param(horizontal_rays_param_name.str(), params.horizontal_rays, *this);
    loaded_successfully &= parse_param(vertical_rays_param_name.str(), params.vertical_rays, *this);
    loaded_successfully &= parse_param(vfov_param_name.str(), params.vertical_fov, *this);

    sensor_params_3d_lidar_.push_back(params);
  }

  loaded_successfully &= parse_param("sensor_model.hit", _probHit_, *this);
  loaded_successfully &= parse_param("sensor_model.miss", _probMiss_, *this);
  loaded_successfully &= parse_param("sensor_model.min", _thresMin_, *this);
  loaded_successfully &= parse_param("sensor_model.max", _thresMax_, *this);

  /* /1* check parameters //{ *1/ */

  /* if (m_useHeightMap && m_useColoredMap) { */
  /*   std::string msg = std::string("You enabled both height map and RGBcolor registration.") + " This is contradictory. " + "Defaulting to height map."; */
  /*   RCLCPP_WARN(this->get_logger(), msg.c_str()); */
  /*   m_useColoredMap = false; */
  /* } */

  /* if (m_useColoredMap) { */
  /* #ifdef COLOR_OCTOMAP_SERVER */
  /*   RCLCPP_WARN(this->get_logger(), "Using RGB color registration (if information available)"); */
  /* #else */
  /*   std::string msg = std::string("Colored map requested in launch file") + " - node not running/compiled to support colors, " + */
  /*                     "please define COLOR_OCTOMAP_SERVER and recompile or launch " + "the octomap_color_server node"; */
  /*   RCLCPP_WARN(this->get_logger(), msg.c_str()); */
  /* #endif */
  /* } */

  /* if (m_updateFreeSpaceUsingMissingData && m_maxRange < 0.0) { */
  /*   std::string msg = std::string("You enabled updating free space using missing data in measurements. ") + */
  /*                     "However, the maximal sensor range is not limited. " + "Disabling this feature."; */
  /*   RCLCPP_WARN(this->get_logger(), msg.c_str()); */
  /*   m_updateFreeSpaceUsingMissingData = false; */
  /* } */

  /* if (m_localMapping && m_localMapDistance < m_maxRange) { */
  /*   std::string msg = std::string("You enabled using only the local map. ") + */
  /*                     "However, the local distance for the map is lower than the maximal sensor range. " + */
  /*                     "Defaulting the local distance for the map to the maximal sensor range."; */
  /*   RCLCPP_WARN(this->get_logger(), msg.c_str()); */
  /*   m_localMapDistance = m_maxRange; */
  /* } */


  /* //} */

  if (!loaded_successfully) {
    const std::string str = "Could not load all non-optional parameters. Shutting down.";
    RCLCPP_ERROR(get_logger(), "[Octomap_server]: %s", str.c_str());
    rclcpp::shutdown();
    return;
  }

  //}

  /* initialize sensor LUT model //{ */

  for (int i = 0; i < n_sensors_3d_lidar_; i++) {
    xyz_lut_t lut_table;
    sensor_3d_lidar_xyz_lut_.push_back(lut_table);
    initialize3DLidarLUT(sensor_3d_lidar_xyz_lut_[i], sensor_params_3d_lidar_[i]);
  }

  for (int i = 0; i < n_sensors_depth_cam_; i++) {
    xyz_lut_t lut_table;
    sensor_depth_camera_xyz_lut_.push_back(lut_table);
    initializeDepthCamLUT(sensor_depth_camera_xyz_lut_[i], sensor_params_depth_cam_[i]);
  }

  //}

  /* initialize octomap object & params //{ */

  octree_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_->setProbHit(_probHit_);
  octree_->setProbMiss(_probMiss_);
  octree_->setClampingThresMin(_thresMin_);
  octree_->setClampingThresMax(_thresMax_);

  octree_local_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_local_->setProbHit(_probHit_);
  octree_local_->setProbMiss(_probMiss_);
  octree_local_->setClampingThresMin(_thresMin_);
  octree_local_->setClampingThresMax(_thresMax_);

  octree_initialized_ = true;

  //}

  /* tf //{ */

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  //}

  /* publishers //{ */

  rclcpp::QoS qos(rclcpp::KeepLast(3));
  pub_map_global_full_   = create_publisher<octomap_msgs::msg::Octomap>("octomap_global_full_out", qos);
  pub_map_global_binary_ = create_publisher<octomap_msgs::msg::Octomap>("octomap_global_binary_out", qos);

  pub_map_local_full_   = create_publisher<octomap_msgs::msg::Octomap>("octomap_local_full_out", qos);
  pub_map_local_binary_ = create_publisher<octomap_msgs::msg::Octomap>("octomap_local_binary_out", qos);

  //}

  /* subscribers //{ */

  rclcpp::SubscriptionOptions subopts;

  /* 3D lidars //{ */
  for (int i = 0; i < n_sensors_3d_lidar_; i++) {

    std::stringstream ss;
    ss << "lidar_3d_" << i << "_in";

    subopts.callback_group = new_cbk_grp();
    std::function<void(const sensor_msgs::msg::PointCloud2::UniquePtr)> boundCallback3dLidarCloud2 =
        std::bind(&OctomapServer::callback3dLidarCloud2, this, std::placeholders::_1, LIDAR_3D, i);

    auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(ss.str(), rclcpp::SystemDefaultsQoS(), boundCallback3dLidarCloud2, subopts);
    sub_3dlaser_pc2_.push_back(sub);
  }
  //}

  /* depth cameras //{ */
  for (int i = 0; i < n_sensors_depth_cam_; i++) {

    std::stringstream ss;
    ss << "depth_camera_" << i << "_in";

    subopts.callback_group = new_cbk_grp();
    std::function<void(const sensor_msgs::msg::PointCloud2::UniquePtr)> boundCallback3dLidarCloud2 =
        std::bind(&OctomapServer::callback3dLidarCloud2, this, std::placeholders::_1, DEPTH_CAMERA, i);

    auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(ss.str(), rclcpp::SystemDefaultsQoS(), boundCallback3dLidarCloud2, subopts);
    sub_depth_cam_pc2_.push_back(sub);
  }
  //}

  /* 2D lidar //{ */
  subopts.callback_group = new_cbk_grp();
  std::function<void(const sensor_msgs::msg::LaserScan::UniquePtr)> boundCallbackLaserScan =
      std::bind(&OctomapServer::callbackLaserScan, this, std::placeholders::_1);
  sub_laser_scan_ = create_subscription<sensor_msgs::msg::LaserScan>("laser_scan_in", rclcpp::SystemDefaultsQoS(), boundCallbackLaserScan, subopts);
  //}

  //}

  /* services //{ */

  const auto qos_profile    = qos.get_rmw_qos_profile();
  const auto action_grp_ptr = new_cbk_grp();
  service_reset_map_        = create_service<std_srvs::srv::Empty>(
      "reset_map_in", std::bind(&OctomapServer::callbackResetMap, this, std::placeholders::_1, std::placeholders::_2), qos_profile, action_grp_ptr);

  //}

  /* timers //{ */

  timer_global_map_ = create_wall_timer(std::chrono::duration<double>(1.0 / _global_map_rate_), std::bind(&OctomapServer::timerGlobalMap, this), new_cbk_grp());

  if (_local_map_enabled_) {
    timer_local_map_ = create_wall_timer(std::chrono::duration<double>(1.0 / _local_map_rate_), std::bind(&OctomapServer::timerLocalMap, this), new_cbk_grp());
  }

  time_last_local_map_processing_ = (1.0 / _local_map_rate_) * _local_map_max_computation_duty_cycle_;

  is_initialized_ = true;
  RCLCPP_INFO(get_logger(), "[OctomapServer]: Initialized");

  //}
}
//}

// | --------------------- topic callbacks -------------------- |

/* callbackLaserScan() //{ */

void OctomapServer::callbackLaserScan(const sensor_msgs::msg::LaserScan::UniquePtr msg) {

  if (!is_initialized_) {
    return;
  }

  if (!octree_initialized_) {
    return;
  }

  if (!_map_while_grounded_) {

    if (getting_laser_scan_) {

      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: missing control manager diagnostics, can not integrate data!");
      return;

    } else {
      /* if ((ros::Time::now() - last_time).toSec() > 1.0) { */
      /*   RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: control manager diagnostics too old, can not integrate data!"); */
      /*   return; */
      /* } */

      // TODO is this the best option?
      /* if (!sh_control_manager_diag_.getMsg()->flying_normally) { */
      /*   RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: not flying normally, therefore, not integrating data"); */
      /*   return; */
      /* } */
    }
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: callbackLaserScan()");

  PCLPointCloud::Ptr pc              = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr free_vectors_pc = boost::make_shared<PCLPointCloud>();

  Eigen::Matrix4f                      sensorToWorld;
  geometry_msgs::msg::TransformStamped sensorToWorldTf;

  try {
    sensorToWorldTf = tf_buffer_->lookupTransform(_world_frame_, msg->header.frame_id, rclcpp::Time(0));
  }
  catch (...) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: callbackLaserScan(): could not find tf from %s to %s", msg->header.frame_id.c_str(),
                         _world_frame_.c_str());
    return;
  }

  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

  // laser scan to point cloud
  sensor_msgs::msg::PointCloud2 ros_cloud;
  projector_.projectLaser(*msg, ros_cloud);
  pcl::fromROSMsg(ros_cloud, *pc);

  // compute free rays, if required
  if (_unknown_rays_update_free_space_) {

    sensor_msgs::msg::LaserScan free_scan = *msg;

    /* double free_scan_distance = (msg->range_max - 1.0) < _unknown_rays_distance_ ? (msg->range_max - 1.0) : _unknown_rays_distance_; */

    for (size_t i = 0; i < msg->ranges.size(); i++) {
      if (msg->ranges[i] > msg->range_max || msg->ranges[i] < msg->range_min) {
        free_scan.ranges[i] = msg->range_max - 1.0;  // valid under max range
      } else {
        free_scan.ranges[i] = msg->range_min - 1.0;  // definitely invalid
      }
    }

    sensor_msgs::msg::PointCloud2 free_cloud;

    projector_.projectLaser(free_scan, free_cloud);

    pcl::fromROSMsg(free_cloud, *free_vectors_pc);
  }

  free_vectors_pc->header = pc->header;

  // transform to the map frame

  pcl::transformPointCloud(*pc, *pc, sensorToWorld);
  pcl::transformPointCloud(*free_vectors_pc, *free_vectors_pc, sensorToWorld);

  pc->header.frame_id              = _world_frame_;
  free_vectors_pc->header.frame_id = _world_frame_;

  insertPointCloud(sensorToWorldTf.transform.translation, pc, free_vectors_pc);

  /* const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorToWorldTf.transform.translation); */
  last_time_laser_scan_ = msg->header.stamp;
}

//}

/* callback3dLidarCloud2() //{ */

void OctomapServer::callback3dLidarCloud2(const sensor_msgs::msg::PointCloud2::UniquePtr msg, const SensorType_t sensor_type, const int sensor_id) {

  if (!is_initialized_) {
    return;
  }

  if (!octree_initialized_) {
    return;
  }

  if (!_map_while_grounded_) {

    /*     if (!sh_control_manager_diag_.hasMsg()) { */

    /*       RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: missing control manager diagnostics, can not integrate data!"); */
    /*       return; */

    /*     } else { */

    /*       ros::Time last_time = sh_control_manager_diag_.lastMsgTime(); */

    /*       if ((ros::Time::now() - last_time).toSec() > 1.0) { */
    /*         RCLCPP_WARN_THROTTLE(get_logger(), get_clock()*,1.0, "[OctomapServer]: control manager diagnostics too old, can not integrate data!"); */
    /*         return; */
    /*       } */

    /*       // TODO is this the best option? */
    /*       if (!sh_control_manager_diag_.getMsg()->flying_normally) { */
    /*         ROS_INFO_THROTTLE(1.0, "[OctomapServer]: not flying normally, therefore, not integrating data"); */
    /*         return; */
    /*       } */
    /*     } */
  }

  rclcpp::Time time_start = get_clock()->now();

  PCLPointCloud::Ptr pc              = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr free_vectors_pc = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr hit_pc          = boost::make_shared<PCLPointCloud>();

  pcl::fromROSMsg(*msg, *pc);

  Eigen::Matrix4f                      sensorToWorld;
  geometry_msgs::msg::TransformStamped sensorToWorldTf;
  try {
    sensorToWorldTf = tf_buffer_->lookupTransform(_world_frame_, msg->header.frame_id, rclcpp::Time(0));
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
  }
  catch (...) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: callback3dLidarCloud2(): could not find tf from %s to %s",
                         msg->header.frame_id.c_str(), _world_frame_.c_str());
    return;
  }

  double max_range;

  switch (sensor_type) {
    case LIDAR_3D: {
      max_range = sensor_params_3d_lidar_[sensor_id].max_range;
      break;
    }
    case DEPTH_CAMERA: {
      max_range = sensor_params_depth_cam_[sensor_id].max_range;
      break;
    }
    default: {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: callback3dLidarCloud2(): unsupported sensor type");
      return;
    }
  }

  for (size_t i = 0; i < pc->size(); i++) {
    pcl::PointXYZ pt = pc->at(i);
    if ((std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) && ((pow(pt.x, 2) + pow(pt.y, 2) + pow(pt.z, 2)) < pow(max_range, 2))) {
      hit_pc->push_back(pt);
    } else {
      if (_unknown_rays_update_free_space_) {
        vec3_t ray_vec;

        switch (sensor_type) {
          case LIDAR_3D: {
            ray_vec = sensor_3d_lidar_xyz_lut_[sensor_id].directions.col(i);
            break;
          }
          case DEPTH_CAMERA: {
            ray_vec = sensor_depth_camera_xyz_lut_[sensor_id].directions.col(i);
            break;
          }
          default: {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: callback3dLidarCloud2(): unsupported sensor type");
            return;
          }
        }
        if (ray_vec(2) > 0.0) {

          pcl::PointXYZ temp_pt;

          temp_pt.x = ray_vec(0) * float(max_range);
          temp_pt.y = ray_vec(1) * float(max_range);
          temp_pt.z = ray_vec(2) * float(max_range);

          free_vectors_pc->push_back(temp_pt);
        }
      }
    }
  }

  free_vectors_pc->header = pc->header;

  // Voxelize data
  /* if (hit_pc->size() > 0) { */

  /*   ROS_INFO("[OctomapServer]: size %d", hit_pc->size()); */

  /*   for (int i = 0; i < hit_pc->size(); i++) { */

  /*     const pcl::PointXYZ pt = hit_pc->at(i); */
  /*     const double        x  = pt.x; */
  /*     const double        y  = pt.y; */
  /*     const double        z  = pt.z; */

  /*     if (!std::isfinite(x)) { */
  /*       ROS_ERROR("NaN detected in variable \"x\"!!!"); */
  /*     } */
  /*   } */

  /*   pcl::VoxelGrid<PCLPoint> vg; */
  /*   vg.setInputCloud(hit_pc); */
  /*   vg.setLeafSize(0.2, 0.2, 0.2); */
  /*   PCLPointCloud::Ptr temp_pc; */
  /*   ROS_INFO("[OctomapServer]: voxel grid in"); */
  /*   vg.filter(*hit_pc); */
  /*   ROS_INFO("[OctomapServer]: voxel grid out"); */
  /* } */

  /* if (free_vectors_pc->size() > 0) { */
  /*   pcl::VoxelGrid<PCLPoint> vg; */
  /*   vg.setInputCloud(free_vectors_pc); */
  /*   vg.setLeafSize(1.0, 1.0, 1.0); */
  /*   vg.filter(*free_vectors_pc); */
  /* } */

  /* // filter lone pixels */
  /* if (hit_pc->size() > 0) { */
  /*   pcl::StatisticalOutlierRemoval<PCLPoint> sor(true); */
  /*   sor.setInputCloud(hit_pc); */
  /*   sor.setMeanK(50.0); */
  /*   sor.setStddevMulThresh(1.0); */
  /*   sor.filter(*hit_pc); */
  /* } */

  // transform to the map frame

  pcl::transformPointCloud(*hit_pc, *hit_pc, sensorToWorld);
  pcl::transformPointCloud(*free_vectors_pc, *free_vectors_pc, sensorToWorld);

  hit_pc->header.frame_id          = _world_frame_;
  free_vectors_pc->header.frame_id = _world_frame_;

  insertPointCloud(sensorToWorldTf.transform.translation, hit_pc, free_vectors_pc);

  /* const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorToWorldTf.transform.translation); */

  {
    std::scoped_lock lock(mutex_avg_time_cloud_insertion_);

    rclcpp::Time time_end = get_clock()->now();

    double exec_duration = (time_end - time_start).seconds();

    double coef               = 0.5;
    avg_time_cloud_insertion_ = coef * avg_time_cloud_insertion_ + (1.0 - coef) * exec_duration;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: avg cloud insertion time = %.3f sec", avg_time_cloud_insertion_);
  }
}  // namespace mrs_octomap_server

//}

// | -------------------- service callbacks ------------------- |

/* callbackResetMap() //{ */

bool OctomapServer::callbackResetMap([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                     [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response>      response) {
  {
    std::scoped_lock lock(mutex_octree_);
    octree_->clear();
    octree_initialized_ = true;
  }

  RCLCPP_INFO(get_logger(), "[OctomapServer]: octomap cleared");
  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerGlobalMap() //{ */

void OctomapServer::timerGlobalMap() {

  if (!is_initialized_) {
    return;
  }

  if (!octree_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(get_logger(), "[OctomapServer]: full map timer spinning");

  std::scoped_lock lock(mutex_octree_);

  size_t octomap_size = octree_->size();
  if (octomap_size <= 1) {
    RCLCPP_WARN(get_logger(), "[OctomapServer]: Nothing to publish, octree is empty");
    return;
  }

  if (_global_map_compress_) {
    octree_->prune();
  }

  if (_global_map_publish_full_) {

    octomap_msgs::msg::Octomap om;
    om.header.frame_id = _world_frame_;
    om.header.stamp    = get_clock()->now();  // TODO

    if (octomap_msgs::fullMapToMsg(*octree_, om)) {
      pub_map_global_full_->publish(om);
    } else {
      RCLCPP_ERROR(get_logger(), "[OctomapServer]: error serializing global octomap to full representation");
    }
  }

  if (_global_map_publish_binary_) {

    octomap_msgs::msg::Octomap om;
    om.header.frame_id = _world_frame_;
    om.header.stamp    = get_clock()->now();  // TODO

    if (octomap_msgs::binaryMapToMsg(*octree_, om)) {
      pub_map_global_binary_->publish(om);
    } else {
      RCLCPP_ERROR(get_logger(), "[OctomapServer]: error serializing global octomap to binary representation");
    }
  }
}

//}

/* timerLocalMap() //{ */

void OctomapServer::timerLocalMap() {

  if (!is_initialized_) {
    return;
  }

  if (!octree_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(get_logger(), "[OctomapServer]: local map timer spinning");

  std::scoped_lock lock(mutex_octree_local_);


  /* double time_local_map_processing; */

  /* { */
  /*   std::scoped_lock lck(mutex_time_local_map_processing_); */
  /*   time_local_map_processing = time_last_local_map_processing_; */
  /* } */

  double duty_factor = time_last_local_map_processing_ / (_local_map_max_computation_duty_cycle_ * (1.0 / _local_map_rate_));

  if (duty_factor >= 1.0) {

    local_map_horizontal_offset_ -= 0.5;
    local_map_vertical_offset_ -= 0.25;

  } else if (duty_factor <= 0.5) {

    local_map_horizontal_offset_ += 0.5;
    local_map_vertical_offset_ += 0.25;

    if (local_map_vertical_offset_ >= 0) {
      local_map_horizontal_offset_ = 0;
    }

    if (local_map_vertical_offset_ >= 0) {
      local_map_vertical_offset_ = 0;
    }
  }

  double horizontal_distance = _local_map_horizontal_distance_ + local_map_horizontal_offset_;
  double vertical_distance   = _local_map_vertical_distance_ + local_map_vertical_offset_;

  if (horizontal_distance < 10) {
    horizontal_distance = 10;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: saturating local map size to 10, your computer is probably not very powerfull");
  }

  if (vertical_distance < 5) {
    vertical_distance = 5;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1.0,
                          "[OctomapServer]: saturating local map vertical size to 5, your computer is probably not very powerfull");
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5.0, "[OctomapServer]: local map size: hor %d, ver %d", int(horizontal_distance), int(vertical_distance));

  bool success = createLocalMap(_robot_frame_, horizontal_distance, vertical_distance, octree_local_);

  if (!success) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: failed to create the local map");
    return;
  }

  size_t octomap_size = octree_local_->size();

  if (octomap_size <= 1) {
    RCLCPP_WARN(get_logger(), "[OctomapServer]: Nothing to publish, octree is empty");
    return;
  }

  if (_local_map_publish_full_) {

    octomap_msgs::msg::Octomap om;
    om.header.frame_id = _world_frame_;
    om.header.stamp    = get_clock()->now();  // TODO

    if (octomap_msgs::fullMapToMsg(*octree_local_, om)) {
      pub_map_local_full_->publish(om);
    } else {
      RCLCPP_ERROR(get_logger(), "[OctomapServer]: error serializing local octomap to full representation");
    }
  }

  if (_local_map_publish_binary_) {

    octomap_msgs::msg::Octomap om;
    om.header.frame_id = _world_frame_;
    om.header.stamp    = get_clock()->now();  // TODO

    if (octomap_msgs::binaryMapToMsg(*octree_local_, om)) {
      pub_map_local_binary_->publish(om);
    } else {
      RCLCPP_ERROR(get_logger(), "[OctomapServer]: error serializing local octomap to binary representation");
    }
  }
}

//}

// | ------------------------ routines ------------------------ |

/* insertPointCloud() //{ */

void OctomapServer::insertPointCloud(const geometry_msgs::msg::Vector3& sensorOriginTf, const PCLPointCloud::ConstPtr& cloud,
                                     const PCLPointCloud::ConstPtr& free_vectors_cloud) {

  std::scoped_lock lock(mutex_octree_);

  double resolution_fractor;
  {
    std::scoped_lock lck(mutex_resolution_fractor_);
    resolution_fractor = resolution_fractor_;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: insertPointCloud()");

  const octomap::point3d sensor_origin      = octomap::pointTfToOctomap(sensorOriginTf);
  const float            free_space_ray_len = float(_unknown_rays_distance_);
  double                 coarse_res         = octree_->getResolution() * pow(2.0, resolution_fractor);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: current resolution = %.2f m", coarse_res);

  octomap::KeySet occupied_cells;
  octomap::KeySet free_cells;
  octomap::KeySet free_ends;

  /* const bool free_space_bounded = free_space_ray_len > 0.0f; */

  // all measured points: make it free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = cloud->begin(); it != cloud->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    octomap::point3d measured_point(it->x, it->y, it->z);
    const float      point_distance = float((measured_point - sensor_origin).norm());

    octomap::OcTreeKey key;
    if (octree_->coordToKeyChecked(measured_point, key)) {
      occupied_cells.insert(key);
    }

    // move end point to distance min(free space ray len, current distance)
    measured_point = sensor_origin + (measured_point - sensor_origin).normalize() * std::min(free_space_ray_len, point_distance);

    octomap::OcTreeKey measured_key = octree_->coordToKey(measured_point);

    free_ends.insert(measured_key);
  }

  for (PCLPointCloud::const_iterator it = free_vectors_cloud->begin(); it != free_vectors_cloud->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    octomap::point3d measured_point(it->x, it->y, it->z);
    octomap::KeyRay  keyRay;

    // check if the ray intersects a cell in the occupied list
    if (octomap_tools::computeRayKeys(octree_, sensor_origin, measured_point, keyRay, resolution_fractor)) {

      octomap::KeyRay::iterator alterantive_ray_end = keyRay.end();

      for (octomap::KeyRay::iterator it2 = keyRay.begin(), end = keyRay.end(); it2 != end; ++it2) {

        if (!_unknown_rays_clear_occupied_) {

          // check if the cell is occupied in the map
          auto node = octree_->search(*it2);

          if (node && octree_->isNodeOccupied(node)) {

            if (it2 == keyRay.begin()) {
              alterantive_ray_end = keyRay.begin();  // special case
            } else {
              alterantive_ray_end = it2 - 1;
            }

            break;
          }
        }
      }

      free_cells.insert(keyRay.begin(), alterantive_ray_end);
    }
  }

  // for FREE RAY ENDS
  for (octomap::KeySet::iterator it = free_ends.begin(), end = free_ends.end(); it != end; ++it) {

    octomap::point3d coords = octree_->keyToCoord(*it);

    octomap::KeyRay key_ray;
    if (octomap_tools::computeRayKeys(octree_, sensor_origin, coords, key_ray, resolution_fractor)) {

      for (octomap::KeyRay::iterator it2 = key_ray.begin(), end = key_ray.end(); it2 != end; ++it2) {

        if (occupied_cells.count(*it2)) {

          octomap::KeyRay::iterator last_key = it2 != key_ray.begin() ? it2 - 1 : key_ray.begin();

          free_cells.insert(key_ray.begin(), last_key);
          break;

        } else {
          free_cells.insert(key_ray.begin(), key_ray.end());
        }
      }
    }
  }

  octomap::OcTreeNode* root = octree_->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {
    octomap::OcTreeKey key = octree_->coordToKey(0, 0, 0, octree_->getTreeDepth());
    octree_->setNodeValue(key, 0.0);
  }

  // FREE CELLS
  for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
    octomap::OcTreeNode* node = touchNode(octree_, *it, octree_->getTreeDepth() - resolution_fractor);
    octree_->updateNodeLogOdds(node, octree_->getProbMissLog());
  }


  // OCCUPIED CELLS
  for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++) {
    octomap::OcTreeNode* node = touchNode(octree_, *it, octree_->getTreeDepth() - resolution_fractor);
    octree_->updateNodeLogOdds(node, octree_->getProbHitLog());
  }
}

//}

/* initialize3DLidarLUT() //{ */

void OctomapServer::initialize3DLidarLUT(xyz_lut_t& lut, const SensorParams3DLidar_t sensor_params) {

  const int                                       rangeCount         = sensor_params.horizontal_rays;
  const int                                       verticalRangeCount = sensor_params.vertical_rays;
  std::vector<std::tuple<double, double, double>> coord_coeffs;
  const double                                    minAngle = 0.0;
  const double                                    maxAngle = 2.0 * M_PI;

  const double verticalMinAngle = -sensor_params.vertical_fov / 2.0;
  const double verticalMaxAngle = sensor_params.vertical_fov / 2.0;

  const double yDiff = maxAngle - minAngle;
  const double pDiff = verticalMaxAngle - verticalMinAngle;

  double yAngle_step = yDiff / (rangeCount - 1);

  double pAngle_step;
  if (verticalRangeCount > 1)
    pAngle_step = pDiff / (verticalRangeCount - 1);
  else
    pAngle_step = 0;

  coord_coeffs.reserve(rangeCount * verticalRangeCount);

  for (int i = 0; i < rangeCount; i++) {
    for (int j = 0; j < verticalRangeCount; j++) {

      // Get angles of ray to get xyz for point
      const double yAngle = i * yAngle_step + minAngle;
      const double pAngle = j * pAngle_step + verticalMinAngle;

      const double x_coeff = cos(pAngle) * cos(yAngle);
      const double y_coeff = cos(pAngle) * sin(yAngle);
      const double z_coeff = sin(pAngle);
      coord_coeffs.push_back({x_coeff, y_coeff, z_coeff});
    }
  }

  int it = 0;
  lut.directions.resize(3, rangeCount * verticalRangeCount);
  lut.offsets.resize(3, rangeCount * verticalRangeCount);

  for (int row = 0; row < verticalRangeCount; row++) {
    for (int col = 0; col < rangeCount; col++) {
      const auto [x_coeff, y_coeff, z_coeff] = coord_coeffs.at(col * verticalRangeCount + row);
      lut.directions.col(it)                 = vec3_t(x_coeff, y_coeff, z_coeff);
      lut.offsets.col(it)                    = vec3_t(0, 0, 0);
      it++;
    }
  }
}

//}

/* initializeDepthCamLUT() //{ */

void OctomapServer::initializeDepthCamLUT(xyz_lut_t& lut, const SensorParamsDepthCam_t sensor_params) {

  const int horizontalRangeCount = sensor_params.horizontal_rays;
  const int verticalRangeCount   = sensor_params.vertical_rays;

  RCLCPP_INFO(get_logger(), "[OctomapServer]: initializing depth camera lut, res %d x %d = %d points", horizontalRangeCount, verticalRangeCount,
              horizontalRangeCount * verticalRangeCount);

  std::vector<std::tuple<double, double, double>> coord_coeffs;

  // yes it's flipped, pixel [0,0] is top-left
  const double horizontalMinAngle = sensor_params.horizontal_fov / 2.0;
  const double horizontalMaxAngle = -sensor_params.horizontal_fov / 2.0;

  const double verticalMinAngle = sensor_params.vertical_fov / 2.0;
  const double verticalMaxAngle = -sensor_params.vertical_fov / 2.0;

  const double yDiff = horizontalMaxAngle - horizontalMinAngle;
  const double pDiff = verticalMaxAngle - verticalMinAngle;

  Eigen::Quaterniond rot = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ());

  double yAngle_step = yDiff / (horizontalRangeCount - 1);

  double pAngle_step;
  if (verticalRangeCount > 1) {
    pAngle_step = pDiff / (verticalRangeCount - 1);
  } else {
    pAngle_step = 0;
  }

  coord_coeffs.reserve(horizontalRangeCount * verticalRangeCount);

  for (int j = 0; j < verticalRangeCount; j++) {
    for (int i = 0; i < horizontalRangeCount; i++) {

      // Get angles of ray to get xyz for point
      const double yAngle = i * yAngle_step + horizontalMinAngle;
      const double pAngle = j * pAngle_step + verticalMinAngle;

      const double x_coeff = cos(pAngle) * cos(yAngle);
      const double y_coeff = cos(pAngle) * sin(yAngle);
      const double z_coeff = sin(pAngle);

      Eigen::Vector3d p(x_coeff, y_coeff, z_coeff);

      p = rot * p;

      /* double r = (double)(i) / horizontalRangeCount; */
      /* double g = (double)(j) / horizontalRangeCount; */

      coord_coeffs.push_back({p.x(), p.y(), p.z()});
    }
  }

  int it = 0;
  lut.directions.resize(3, horizontalRangeCount * verticalRangeCount);
  lut.offsets.resize(3, horizontalRangeCount * verticalRangeCount);

  for (int row = 0; row < verticalRangeCount; row++) {
    for (int col = 0; col < horizontalRangeCount; col++) {
      const auto [x_coeff, y_coeff, z_coeff] = coord_coeffs.at(col + horizontalRangeCount * row);
      lut.directions.col(it)                 = vec3_t(x_coeff, y_coeff, z_coeff);
      lut.offsets.col(it)                    = vec3_t(0, 0, 0);
      it++;
    }
  }
}

//}

/* copyInsideBBX2() //{ */

bool OctomapServer::copyInsideBBX2(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to, const octomap::point3d& p_min,
                                   const octomap::point3d& p_max) {

  octomap::OcTreeKey minKey, maxKey;

  if (!from->coordToKeyChecked(p_min, minKey) || !from->coordToKeyChecked(p_max, maxKey)) {
    return false;
  }

  octomap::OcTreeNode* root = to->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {
    octomap::OcTreeKey key = to->coordToKey(p_min.x() - to->getResolution() * 2.0, p_min.y(), p_min.z(), to->getTreeDepth());
    to->setNodeValue(key, 1.0);
  }

  for (OcTree_t::leaf_bbx_iterator it = from->begin_leafs_bbx(p_min, p_max, from->getTreeDepth() - resolution_fractor_), end = from->end_leafs_bbx(); it != end;
       ++it) {

    octomap::OcTreeNode* orig_node = it.operator->();

    octomap_tools::eatChildren(from, orig_node);

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = touchNode(to, k, it.getDepth());
    node->setValue(orig_node->getValue());
  }

  if (!got_root) {
    octomap::OcTreeKey key = to->coordToKey(p_min.x() - to->getResolution() * 2.0, p_min.y(), p_min.z(), to->getTreeDepth());
    to->deleteNode(key, to->getTreeDepth());
  }

  return true;
}

/* //} */

/* touchNode() //{ */

octomap::OcTreeNode* OctomapServer::touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth = 0) {

  return touchNodeRecurs(octree, octree->getRoot(), key, 0, target_depth);
}

//}

/* touchNodeRecurs() //{ */

octomap::OcTreeNode* OctomapServer::touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key,
                                                    unsigned int depth, unsigned int max_depth = 0) {

  assert(node);

  // follow down to last level
  if (depth < octree->getTreeDepth() && (max_depth == 0 || depth < max_depth)) {

    unsigned int pos = octomap::computeChildIdx(key, int(octree->getTreeDepth() - depth - 1));

    /* ROS_INFO("pos: %d", pos); */
    if (!octree->nodeChildExists(node, pos)) {

      // not a pruned node, create requested child
      octree->createNodeChild(node, pos);
    }

    return touchNodeRecurs(octree, octree->getNodeChild(node, pos), key, depth + 1, max_depth);
  }

  // at last level, update node, end of recursion
  else {

    octomap_tools::eatChildren(octree, node);

    // destroy all children
    /* for (int i = 0; i < 8; i++) { */

    /*   if (octree->nodeChildExists(node, i)) { */

    /*     auto child = octree->getNodeChild(node, i); */

    /*     octree->deleteNodeChild(node, i); */
    /*   } */
    /* } */

    return node;
  }
}

//}

/* expandNodeRecursive() //{ */

void OctomapServer::expandNodeRecursive(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const unsigned int node_depth) {

  if (node_depth < octree->getTreeDepth()) {

    octree->expandNode(node);

    for (int i = 0; i < 8; i++) {
      auto child = octree->getNodeChild(node, i);

      expandNodeRecursive(octree, child, node_depth + 1);
    }

  } else {
    return;
  }
}

//}

/* translateMap() //{ */

bool OctomapServer::translateMap(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y, const double& z) {

  RCLCPP_INFO(get_logger(), "[OctomapServer]: translating map by %.2f, %.2f, %.2f", x, y, z);

  octree->expand();

  // allocate the new future octree
  std::shared_ptr<OcTree_t> octree_new = std::make_shared<OcTree_t>(octree_resolution_);
  octree_new->setProbHit(octree->getProbHit());
  octree_new->setProbMiss(octree->getProbMiss());
  octree_new->setClampingThresMin(octree->getClampingThresMin());
  octree_new->setClampingThresMax(octree->getClampingThresMax());

  for (OcTree_t::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {

    auto coords = it.getCoordinate();

    coords.x() += float(x);
    coords.y() += float(y);
    coords.z() += float(z);

    auto value = it->getValue();
    /* auto key   = it.getKey(); */

    auto new_key = octree_new->coordToKey(coords);

    octree_new->setNodeValue(new_key, value);
  }

  octree_new->prune();

  octree = octree_new;

  RCLCPP_INFO(get_logger(), "[OctomapServer]: map translated");

  return true;
}

//}

/* createLocalMap() //{ */

bool OctomapServer::createLocalMap(const std::string frame_id, const double horizontal_distance, const double vertical_distance,
                                   std::shared_ptr<OcTree_t>& octree) {

  std::scoped_lock lock(mutex_octree_);

  rclcpp::Time time_start = get_clock()->now();

  geometry_msgs::msg::TransformStamped sensorToWorldTf;
  try {
    sensorToWorldTf = tf_buffer_->lookupTransform(_world_frame_, frame_id, rclcpp::Time(0));
  }
  catch (...) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, "[OctomapServer]: createLocalMap(): could not find tf from %s to %s", frame_id.c_str(),
                         _world_frame_.c_str());
    return false;
  }

  double robot_x = sensorToWorldTf.transform.translation.x;
  double robot_y = sensorToWorldTf.transform.translation.y;
  double robot_z = sensorToWorldTf.transform.translation.z;

  bool success = true;

  // clear the old local map
  octree->clear();

  const octomap::point3d p_min =
      octomap::point3d(float(robot_x - horizontal_distance), float(robot_y - horizontal_distance), float(robot_z - vertical_distance));
  const octomap::point3d p_max =
      octomap::point3d(float(robot_x + horizontal_distance), float(robot_y + horizontal_distance), float(robot_z + vertical_distance));

  success = copyInsideBBX2(octree_, octree, p_min, p_max);

  octree->setProbHit(octree->getProbHit());
  octree->setProbMiss(octree->getProbMiss());
  octree->setClampingThresMin(octree->getClampingThresMinLog());
  octree->setClampingThresMax(octree->getClampingThresMaxLog());

  {
    std::scoped_lock lock(mutex_time_local_map_processing_);
    rclcpp::Time     time_end       = get_clock()->now();
    time_last_local_map_processing_ = (time_end - time_start).seconds();

    if (time_last_local_map_processing_ > ((1.0 / _local_map_rate_) * _local_map_max_computation_duty_cycle_)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5.0, "[OctomapServer]: local map creation time = %.3f sec", time_last_local_map_processing_);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5.0, "[OctomapServer]: local map creation time = %.3f sec", time_last_local_map_processing_);
    }
  }

  return success;
}

//}

/* new_cbk_grp() //{ */
// just a util function that returns a new mutually exclusive callback group to shorten the call
rclcpp::CallbackGroup::SharedPtr OctomapServer::new_cbk_grp() {
  const rclcpp::CallbackGroup::SharedPtr new_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_groups_.push_back(new_group);
  return new_group;
}
//}

}  // namespace octomap_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::OctomapServer)
