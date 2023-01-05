#include <octomap_server2/octomap_server.hpp>

#include <octomap_ros/conversions.hpp>

namespace octomap_server
{

/* OctomapServer constructor //{ */
OctomapServer::OctomapServer(rclcpp::NodeOptions options) : Node("octomap_server2", options) {

  RCLCPP_INFO(get_logger(), "[OctomapServer]: Initializing...");
  RCLCPP_INFO(get_logger(), "-------------- Loading parameters --------------");
  bool loaded_successfully = true;

  /* parse params from config file //{ */
  loaded_successfully &= parse_param("map_while_grounded", _map_while_grounded_, *this);
  loaded_successfully &= parse_param("local_map.size.width", _local_map_width_, *this);
  loaded_successfully &= parse_param("local_map.size.height", _local_map_height_, *this);
  loaded_successfully &= parse_param("local_map.publisher_rate", _local_map_publisher_rate_, *this);
  loaded_successfully &= parse_param("local_map.publish_full", _local_map_publish_full_, *this);
  loaded_successfully &= parse_param("local_map.compress", _local_map_compress_, *this);
  loaded_successfully &= parse_param("local_map.publish_binary", _local_map_publish_binary_, *this);
  
  loaded_successfully &= parse_param("global_map.publisher_rate", _global_map_publisher_rate_, *this);
  loaded_successfully &= parse_param("global_map.creation_rate", _global_map_creator_rate_, *this);
  loaded_successfully &= parse_param("global_map.enabled", _global_map_enabled_, *this);
  loaded_successfully &= parse_param("global_map.compress", _global_map_compress_, *this);
  loaded_successfully &= parse_param("global_map.publish_full", _global_map_publish_full_, *this);
  loaded_successfully &= parse_param("global_map.publish_binary", _global_map_publish_binary_, *this);

  local_map_width_ = _local_map_width_;
  local_map_height_ = _local_map_height_;

  loaded_successfully &= parse_param("resolution", octree_resolution_, *this);
  loaded_successfully &= parse_param("world_frame_id", _world_frame_, *this);
  loaded_successfully &= parse_param("robot_frame_id", _robot_frame_, *this);

  loaded_successfully &= parse_param("unknown_rays.update_free_space", _unknown_rays_update_free_space_, *this);
  loaded_successfully &= parse_param("unknown_rays.clear_occupied", _unknown_rays_clear_occupied_, *this);
  loaded_successfully &= parse_param("unknown_rays.ray_distance", _unknown_rays_distance_, *this);

  loaded_successfully &= parse_param("sensor_model.hit", _probHit_, *this);
  loaded_successfully &= parse_param("sensor_model.miss", _probMiss_, *this);
  loaded_successfully &= parse_param("sensor_model.min", _thresMin_, *this);
  loaded_successfully &= parse_param("sensor_model.max", _thresMax_, *this);
  loaded_successfully &= parse_param("sensor_model.min_range", _rangeMin_, *this);
  loaded_successfully &= parse_param("sensor_model.max_range", _rangeMax_, *this);

  if (!loaded_successfully) {
    const std::string str = "Could not load all non-optional parameters. Shutting down.";
    RCLCPP_ERROR(get_logger(), "[Octomap_server]: %s", str.c_str());
    rclcpp::shutdown();
    return;
  }

  //}

  /* initialize octomap object & params //{ */

  octree_global_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_global_->setProbHit(_probHit_);
  octree_global_->setProbMiss(_probMiss_);
  octree_global_->setClampingThresMin(_thresMin_);
  octree_global_->setClampingThresMax(_thresMax_);

  octree_local_0_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_local_0_->setProbHit(_probHit_);
  octree_local_0_->setProbMiss(_probMiss_);
  octree_local_0_->setClampingThresMin(_thresMin_);
  octree_local_0_->setClampingThresMax(_thresMax_);

  octree_local_1_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_local_1_->setProbHit(_probHit_);
  octree_local_1_->setProbMiss(_probMiss_);
  octree_local_1_->setClampingThresMin(_thresMin_);
  octree_local_1_->setClampingThresMax(_thresMax_);

  octree_local_ = octree_local_0_;

  octrees_initialized_ = true;

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
  if (_global_map_enabled_) {
    timer_global_map_publisher_ = create_wall_timer(std::chrono::duration<double>(1.0 / _global_map_publisher_rate_), std::bind(&OctomapServer::timerGlobalMapPublisher, this), new_cbk_grp());
    timer_global_map_creator_ = create_wall_timer(std::chrono::duration<double>(1.0 / _global_map_creator_rate_), std::bind(&OctomapServer::timerGlobalMapCreator, this), new_cbk_grp());
  }

  timer_local_map_publisher_ = create_wall_timer(std::chrono::duration<double>(1.0 / _local_map_publisher_rate_), std::bind(&OctomapServer::timerLocalMapPublisher, this), new_cbk_grp());
  timer_local_map_resizer_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&OctomapServer::timerLocalMapResizer, this), new_cbk_grp());

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

  if (!octrees_initialized_) {
    return;
  }

  if (!_map_while_grounded_) {

    if (getting_laser_scan_) {

      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[OctomapServer]: missing control manager diagnostics, can not integrate data!");
      return;
    }
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "[OctomapServer]: callbackLaserScan()");

  PCLPointCloud::Ptr pc              = std::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr free_vectors_pc = std::make_shared<PCLPointCloud>();

  Eigen::Matrix4f                      sensorToWorld;
  geometry_msgs::msg::TransformStamped sensorToWorldTf;
  geometry_msgs::msg::TransformStamped robotToWorldTf;

  try {
    robotToWorldTf = tf_buffer_->lookupTransform(_world_frame_, _robot_frame_, rclcpp::Time(0));
  }
  catch (...) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[OctomapServer]: callbackLaserScan(): could not find tf from %s to %s", _robot_frame_.c_str(),
                         _world_frame_.c_str());
    return;
  }

  try {
    sensorToWorldTf = tf_buffer_->lookupTransform(_world_frame_, msg->header.frame_id, rclcpp::Time(0));
  }
  catch (...) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[OctomapServer]: callbackLaserScan(): could not find tf from %s to %s", msg->header.frame_id.c_str(),
                         _world_frame_.c_str());
    return;
  }

  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

  // clamp ranges to parametrized values
  msg->range_max = std::min((double)msg->range_max, _rangeMax_);
  msg->range_min = std::max((double)msg->range_min, _rangeMin_);

  // laser scan to point cloud
  sensor_msgs::msg::PointCloud2 ros_cloud;
  projector_.projectLaser(*msg, ros_cloud);
  pcl::fromROSMsg(ros_cloud, *pc);

  // compute free rays, if required
  if (_unknown_rays_update_free_space_) {

    sensor_msgs::msg::LaserScan free_scan = *msg;

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

  insertPointCloud(robotToWorldTf.transform.translation, sensorToWorldTf.transform.translation, pc, free_vectors_pc);

  last_time_laser_scan_ = msg->header.stamp;
}

//}

// | -------------------- service callbacks ------------------- |

/* callbackResetMap() //{ */

bool OctomapServer::callbackResetMap([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                     [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response>      response) {
  {
    std::scoped_lock lock(mutex_octree_global_, mutex_octree_local_);
    octree_global_->clear();
    octree_local_->clear();
  }
  octrees_initialized_ = true;

  RCLCPP_INFO(get_logger(), "[OctomapServer]: octomap cleared");
  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerGlobalMapPublisher() //{ */

void OctomapServer::timerGlobalMapPublisher() {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(get_logger(), "[OctomapServer]: full map publisher timer spinning");

  size_t octomap_size; 
  {
    std::scoped_lock lock(mutex_octree_global_);

    octomap_size = octree_global_->size();
  }
  if (octomap_size <= 1) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[OctomapServer]: Nothing to publish, octree is empty");
    return;
  }

  if (_global_map_publish_full_) {

    octomap_msgs::msg::Octomap om;
    om.header.frame_id = _world_frame_;
    om.header.stamp    = get_clock()->now();  // TODO

    bool success = false;
    {
      std::scoped_lock lock(mutex_octree_global_);
      success = octomap_msgs::fullMapToMsg(*octree_global_, om);
    }

    if (success) {
      pub_map_global_full_->publish(om);
    } else {
      RCLCPP_ERROR(get_logger(), "[OctomapServer]: error serializing global octomap to full representation");
    }
  }

  if (_global_map_publish_binary_) {

    octomap_msgs::msg::Octomap om;
    om.header.frame_id = _world_frame_;
    om.header.stamp    = get_clock()->now();  // TODO

    bool success = false;
    {
      std::scoped_lock lock(mutex_octree_global_);
      success = octomap_msgs::binaryMapToMsg(*octree_global_, om);
    }

    if (success) {
      pub_map_global_binary_->publish(om);
    } else {
      RCLCPP_ERROR(get_logger(), "[OctomapServer]: error serializing global octomap to binary representation");
    }
  }
}

//}

/* timerGlobalMapCreator() //{ */

void OctomapServer::timerGlobalMapCreator() {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(get_logger(), "[OctomapServer]: global map creator timer spinning");

  // copy the local map into a buffer

  std::shared_ptr<OcTree_t> local_map_tmp_;
  {
    std::scoped_lock lock(mutex_octree_local_);

    local_map_tmp_ = std::make_shared<OcTree_t>(*octree_local_);
  }

  {
    std::scoped_lock lock(mutex_octree_global_);

    copyLocalMap(local_map_tmp_, octree_global_);

    if (_global_map_compress_) {
      octree_global_->prune();
    }

  }
}

//}

/* timerLocalMapPublisher() //{ */

void OctomapServer::timerLocalMapPublisher() {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(get_logger(), "[OctomapServer]: local map publisher timer spinning");

  size_t octomap_size = octree_local_->size();

  if (octomap_size <= 1) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[OctomapServer]: Nothing to publish, octree_local_, octree is empty");
    return;
  }

  if (_local_map_publish_full_) {

    octomap_msgs::msg::Octomap om;
    om.header.frame_id = _world_frame_;
    om.header.stamp    = get_clock()->now();  // TODO

    bool success = false;
    {
      std::scoped_lock lock(mutex_octree_local_);
      success = octomap_msgs::fullMapToMsg(*octree_local_, om);
    }

    if (success) {
      pub_map_local_full_->publish(om);
    } else {
      RCLCPP_ERROR(get_logger(), "[OctomapServer]: error serializing local octomap to full representation");
    }
  }

  if (_local_map_publish_binary_) {

    octomap_msgs::msg::Octomap om;
    om.header.frame_id = _world_frame_;
    om.header.stamp    = get_clock()->now();  // TODO

    bool success = false;
    {
      std::scoped_lock lock(mutex_octree_local_);
      success = octomap_msgs::binaryMapToMsg(*octree_local_, om);
    }

    if (success) {
      pub_map_local_binary_->publish(om);
    } else {
      RCLCPP_ERROR(get_logger(), "[OctomapServer]: error serializing local octomap to binary representation");
    }
  }
}

//}

/* timerLocalMapResizer() //{ */

void OctomapServer::timerLocalMapResizer() {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(get_logger(), "[OctomapServer]: local map resizer timer spinning");

  double local_map_duty;
  {
    std::scoped_lock lock(mutex_local_map_duty_);
    local_map_duty = local_map_duty_;
  }

  RCLCPP_INFO(get_logger(), "[OctomapServer]: local map duty time: %.3f s", local_map_duty);

  {
    std::scoped_lock lock(mutex_local_map_dimensions_);

    if (local_map_duty > 0.9) {
      local_map_width_ -= int(ceil(10.0 * (local_map_duty - 0.9)));
      local_map_height_ -= int(ceil(10.0 * (local_map_duty - 0.9)));
    } else if (local_map_duty < 0.8) {
      local_map_width_++;
      local_map_height_++;
    }

    if (local_map_width_ < 10) {
      local_map_width_ = 10;
    } else if (local_map_width_ > _local_map_width_) {
      local_map_width_ = _local_map_width_;
    }

    if (local_map_height_ < 10) {
      local_map_height_ = 10;
    } else if (local_map_height_ > _local_map_height_) {
      local_map_height_ = _local_map_height_;
    }

    local_map_duty = 0;
  }

  RCLCPP_INFO(get_logger(), "[OctomapServer]: local map size %d %d", local_map_width_, local_map_height_);

  {
    std::scoped_lock lock(mutex_local_map_duty_);
    local_map_duty_ = local_map_duty;
  }
}

//}

// | ------------------------ routines ------------------------ |

/* insertPointCloud() //{ */

void OctomapServer::insertPointCloud(const geometry_msgs::msg::Vector3& robotOriginTf, const geometry_msgs::msg::Vector3& sensorOriginTf, 
                                     const PCLPointCloud::ConstPtr& cloud,const PCLPointCloud::ConstPtr& free_vectors_cloud) {

  std::scoped_lock lock(mutex_octree_local_);
  rclcpp::Time time_start = get_clock()->now();

  int local_map_width;
  int local_map_height;

  {
    std::scoped_lock lck(mutex_local_map_dimensions_);
    local_map_width = local_map_width_;
    local_map_height = local_map_height_;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "[OctomapServer]: insertPointCloud()");

  geometry_msgs::msg::Point msg_sensorOriginTf;
  msg_sensorOriginTf.x = sensorOriginTf.x;
  msg_sensorOriginTf.y = sensorOriginTf.y;
  msg_sensorOriginTf.z = sensorOriginTf.z;

  const octomap::point3d sensor_origin      = octomap::pointMsgToOctomap(msg_sensorOriginTf);
  const float            free_space_ray_len = std::min(float(_unknown_rays_distance_), float(sqrt(2 * pow(local_map_width / 2.0, 2) + pow(local_map_height / 2.0, 2))));

  octomap::KeySet occupied_cells;
  octomap::KeySet free_cells;
  octomap::KeySet free_ends;

  // all measured points: make it free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = cloud->begin(); it != cloud->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    octomap::point3d measured_point(it->x, it->y, it->z);
    const float      point_distance = float((measured_point - sensor_origin).norm());

    octomap::OcTreeKey key;
    if (octree_local_->coordToKeyChecked(measured_point, key)) {
      occupied_cells.insert(key);
    }

    // move end point to distance min(free space ray len, current distance)
    measured_point = sensor_origin + (measured_point - sensor_origin).normalize() * std::min(free_space_ray_len, point_distance);

    octomap::OcTreeKey measured_key = octree_local_->coordToKey(measured_point);

    free_ends.insert(measured_key);
  }

  for (PCLPointCloud::const_iterator it = free_vectors_cloud->begin(); it != free_vectors_cloud->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    octomap::point3d measured_point(it->x, it->y, it->z);
    const float      point_distance = float((measured_point - sensor_origin).norm());

    octomap::KeyRay keyRay;

    // move end point to distance min(free space ray len, current distance)
    measured_point = sensor_origin + (measured_point - sensor_origin).normalize() * std::min(free_space_ray_len, point_distance);

    // check if the ray intersects a cell in the occupied list
    if (computeRayKeys(octree_local_, sensor_origin, measured_point, keyRay)) {

      octomap::KeyRay::iterator alternative_ray_end = keyRay.end();

      // check if the cell is occupied in the map
      if (!_unknown_rays_clear_occupied_) {
        for (octomap::KeyRay::iterator it2 = keyRay.begin(), end = keyRay.end(); it2 != end; ++it2) {
          auto node = octree_local_->search(*it2);
          if (node && octree_local_->isNodeOccupied(node)) {
            if (it2 == keyRay.begin()) {
              alternative_ray_end = keyRay.begin();  // special case
            } else {
              alternative_ray_end = it2 - 1;
            }
            break;
          }
        }
      }

      free_cells.insert(keyRay.begin(), alternative_ray_end);
    }
  }

  // for FREE RAY ENDS
  for (octomap::KeySet::iterator it = free_ends.begin(), end = free_ends.end(); it != end; ++it) {

    octomap::point3d coords = octree_local_->keyToCoord(*it);

    octomap::KeyRay key_ray;
    if (computeRayKeys(octree_local_, sensor_origin, coords, key_ray)) {

      octomap::KeyRay::iterator alternative_ray_end = key_ray.end();

      for (octomap::KeyRay::iterator it2 = key_ray.begin(), end = key_ray.end(); it2 != end; ++it2) {
        if (occupied_cells.count(*it2)) {

            if (it2 == key_ray.begin()) {
              alternative_ray_end = key_ray.begin();  // special case
            } else {
              alternative_ray_end = it2 - 1;
            }
            break;
        }
      }
      free_cells.insert(key_ray.begin(), alternative_ray_end);
    }
  }

  octomap::OcTreeNode* root = octree_local_->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {
    octomap::OcTreeKey key = octree_local_->coordToKey(0, 0, 0, octree_local_->getTreeDepth());
    octree_local_->setNodeValue(key, 1.0);
  }

  // FREE CELLS
  for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
    octree_local_->updateNode(*it, octree_local_->getProbMissLog());
  }


  // OCCUPIED CELLS
  for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++) {
    octree_local_->updateNode(*it, octree_local_->getProbHitLog());
  }

  // CLAIM THAT CELL, IN WHICH ROBOT IS, IS FREE 
  octomap::OcTreeKey robot_key = octree_local_->coordToKey(robotOriginTf.x, robotOriginTf.y, robotOriginTf.z);
  octree_local_->updateNode(robot_key, false);

  // CROP THE MAP AROUND THE ROBOT
  float x        = sensor_origin.x();
  float y        = sensor_origin.y();
  float z        = sensor_origin.z();
  float width_2  = float(local_map_width) / float(2.0);
  float height_2 = float(local_map_height) / float(2.0);

  octomap::point3d roi_min(x - width_2, y - width_2, z - height_2);
  octomap::point3d roi_max(x + width_2, y + width_2, z + height_2);

  // The First way how to clear data outside of bounding box is to iterate throught all leafs and delete ones 
  // that are outside. This is efficient for tree that has most of the data inside of the bounding box.
  // This is more efficient for Rplidar sensor
  {
    /* rclcpp::Time time_start_crop = get_clock()->now(); */

    clearOutsideBBX(octree_local_, roi_min, roi_max);

    /* rclcpp::Time time_end_crop = get_clock()->now(); */
    /* RCLCPP_INFO(this->get_logger(), "[map]: crop time: %.8f",(time_end_crop - time_start_crop).seconds()); */
  }

  // The second way how to clear data outside of bounding box is to copy all leafs inside of the bouding box 
  // into new tree. This is efficient for tree that has the most of data outside of the bounding box.
  /* { */
    /* rclcpp::Time time_start_copy = get_clock()->now(); */
    /* std::shared_ptr<OcTree_t> from; */

    /* if (octree_local_idx_ == 0) { */
    /*   from              = octree_local_0_; */
    /*   octree_local_     = octree_local_1_; */
    /*   octree_local_idx_ = 1; */
    /* } else { */
    /*   from              = octree_local_1_; */
    /*   octree_local_     = octree_local_0_; */
    /*   octree_local_idx_ = 0; */
    /* } */

    /* octree_local_->clear(); */

    /* copyInsideBBX2(from, octree_local_, roi_min, roi_max); */
  
    /* rclcpp::Time time_end_copy = get_clock()->now(); */
    /* RCLCPP_INFO(this->get_logger(), "[map]: copy time: %.8f",(time_end_copy - time_start_copy).seconds()); */
  /* } */

  if (_local_map_compress_) {
    octree_local_->prune();
  }

  rclcpp::Time time_end = get_clock()->now();
  {
    std::scoped_lock lock(mutex_local_map_duty_);

    local_map_duty_ += (time_end - time_start).seconds();
  }
}

//}

/* clearOutsideBBX() //{ */

bool OctomapServer::clearOutsideBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey minKey, maxKey;
  if (!octree->coordToKeyChecked(p_min, minKey) || !octree->coordToKeyChecked(p_max, maxKey)) {
    return false;
  }

  std::vector<std::pair<octomap::OcTreeKey, unsigned int>> keys;
  for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
    // check if outside of bbx:
    octomap::OcTreeKey k = it.getKey();
    if (k[0] < minKey[0] || k[1] < minKey[1] || k[2] < minKey[2] || k[0] > maxKey[0] || k[1] > maxKey[1] || k[2] > maxKey[2]) {
      keys.push_back(std::make_pair(k, it.getDepth()));
    }
  }

  for (auto k : keys) {
    octree->deleteNode(k.first, k.second);
  }

  /* RCLCPP_INFO(this->get_logger(), "Number of voxels removed outside local area: %li", keys.size()); */
  return true;
}

//}

/* copyInsideBBX2() //{ */

bool OctomapServer::copyInsideBBX2(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to, const octomap::point3d& p_min, const octomap::point3d& p_max) {

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

  // iterate over leafs of the original "from" tree (up to the desired fractor depth)
  for (OcTree_t::leaf_bbx_iterator it = from->begin_leafs_bbx(p_min, p_max, from->getTreeDepth()), end = from->end_leafs_bbx(); it != end; ++it) {

    octomap::OcTreeNode* orig_node = it.operator->();

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = touchNode(to, k, it.getDepth());
    node->setValue(orig_node->getValue());
  }

  return true;
}

/* //} */

/* computeRayKeys() //{ */
bool OctomapServer::computeRayKeys(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& origin, const octomap::point3d& end, octomap::KeyRay& ray) {

  // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
  // basically: DDA in 3D

  ray.reset();

  double resolution      = octree->getResolution();

  octomap::OcTreeKey key_origin, key_end;
  if (!octree->coordToKeyChecked(origin, key_origin) || !octree->coordToKeyChecked(end, key_end)) {
    OCTOMAP_WARNING_STR("coordinates ( " << origin << " -> " << end << ") out of bounds in computeRayKeys");
    return false;
  }


  if (key_origin == key_end)
    return true;  // same tree cell, we're done.

  ray.addKey(key_origin);

  // Initialization phase -------------------------------------------------------

  octomap::point3d direction = (end - origin);
  float            length    = (float)direction.norm();
  direction /= length;  // normalize vector

  int    step[3];
  double tMax[3];
  double tDelta[3];

  octomap::OcTreeKey current_key = key_origin;

  for (unsigned int i = 0; i < 3; ++i) {
    // compute step direction
    if (direction(i) > 0.0)
      step[i] = 1;
    else if (direction(i) < 0.0)
      step[i] = -1;
    else
      step[i] = 0;

    // compute tMax, tDelta
    if (step[i] != 0) {
      // corner point of voxel (in direction of ray)
      double voxelBorder = octree->keyToCoord(current_key[i]);
      voxelBorder += (float)(step[i] * resolution * 0.5);

      tMax[i]   = (voxelBorder - origin(i)) / direction(i);
      tDelta[i] = resolution / fabs(direction(i));
    } else {
      tMax[i]   = std::numeric_limits<double>::max();
      tDelta[i] = std::numeric_limits<double>::max();
    }
  }

  // Incremental phase  ---------------------------------------------------------

  bool done = false;
  while (!done) {

    unsigned int dim;

    // find minimum tMax:
    if (tMax[0] < tMax[1]) {
      if (tMax[0] < tMax[2])
        dim = 0;
      else
        dim = 2;
    } else {
      if (tMax[1] < tMax[2])
        dim = 1;
      else
        dim = 2;
    }

    // advance in direction "dim"
    current_key[dim] += step[dim];
    tMax[dim] += tDelta[dim];


    /* assert(current_key[dim] < octree->size()); */

    // reached endpoint, key equv?
    if (current_key == key_end) {
      done = true;
      break;
    } else {

      // reached endpoint world coords?
      // dist_from_origin now contains the length of the ray when traveled until the border of the current voxel
      double dist_from_origin = std::min(std::min(tMax[0], tMax[1]), tMax[2]);
      // if this is longer than the expected ray length, we should have already hit the voxel containing the end point with the code above (key_end).
      // However, we did not hit it due to accumulating discretization errors, so this is the point here to stop the ray as we would never reach the voxel
      // key_end
      if (dist_from_origin > length) {
        done = true;
        break;
      }

      else {  // continue to add freespace cells
        ray.addKey(current_key);
      }
    }

    assert(ray.size() < ray.sizeMax() - 1);

  }  // end while

  return true;
}
//}

/* copyLocalMap() //{ */

bool OctomapServer::copyLocalMap(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to) {

  octomap::OcTreeKey minKey, maxKey;

  octomap::OcTreeNode* root = to->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {
    octomap::OcTreeKey key = to->coordToKey(0, 0, 0, to->getTreeDepth());
    to->setNodeValue(key, 1.0);
  }

  // iterate over leafs of the original "from" tree (up to the desired fractor depth)
  for (OcTree_t::leaf_iterator it = from->begin_leafs(from->getTreeDepth()), end = from->end_leafs(); it != end; ++it) {

    octomap::OcTreeNode* orig_node = it.operator->();

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = touchNode(to, k, it.getDepth());
    node->setValue(orig_node->getValue());
  }

  return true;
}

//}

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

    if (!octree->nodeChildExists(node, pos)) {

      // not a pruned node, create requested child
      octree->createNodeChild(node, pos);
    }

    return touchNodeRecurs(octree, octree->getNodeChild(node, pos), key, depth + 1, max_depth);
  }

  // at last level, update node, end of recursion
  else {

    return node;
  }
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

/* parse_param() //{ */
// just a util function for loading parameters
  template <class T>
bool OctomapServer::parse_param(const std::string &param_name, T &param_dest, rclcpp::Node& node)
{
  node.declare_parameter<T>(param_name); 
  if (!node.get_parameter(param_name, param_dest))
  {
    RCLCPP_ERROR(node.get_logger(), "Could not load param '%s'", param_name.c_str());
    return false;
  }
  else
  {
    RCLCPP_INFO_STREAM(node.get_logger(), "Loaded '" << param_name << "' = '" << param_dest << "'");
  }
  return true;
}

}  // namespace octomap_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::OctomapServer)
