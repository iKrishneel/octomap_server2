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

#include <eigen3/Eigen/Core>
#include <octomap_server2/octomap_server.hpp>

namespace ph = std::placeholders;

namespace octomap_server
{

/* Class OctomapServer: constructor  //{ */

OctomapServer::OctomapServer(const rclcpp::NodeOptions& options, const std::string node_name)
    : Node(node_name, options),
      m_octree(NULL),
      m_maxRange(20),
      m_minRange(0.4),
      m_worldFrameId("/map"),
      m_useHeightMap(true),
      m_colorFactor(0.8),
      m_publishFreeSpace(false),
      m_res(0.05),
      m_treeDepth(0),
      m_maxTreeDepth(0),
      m_occupancyMinZ(std::numeric_limits<double>::lowest()),
      m_occupancyMaxZ(std::numeric_limits<double>::max()),
      m_minSizeX(0.0),
      m_minSizeY(0.0),
      m_filterSpeckles(false),
      m_compressMap(true),
      m_filterGroundPlane(false),
      m_ZGroundFilterDistance(0.5),
      m_localMapping(false),
      m_localMapDistance(20.0),
      m_updateFreeSpaceUsingMissingData(true),
      m_incrementalUpdate(false),
      m_useColoredMap(false),
      m_isInitialized(false) {

  // | ------------------- parameters loading ------------------- |

  RCLCPP_INFO(this->get_logger(), "Initializing...");
  RCLCPP_INFO(this->get_logger(), "-------------- Loading parameters --------------");
  bool loaded_successfully = true;

  /* parse params from config file //{ */
  loaded_successfully &= parse_param("resolution", m_res);
  loaded_successfully &= parse_param("frame_id", m_worldFrameId);
  loaded_successfully &= parse_param("compress_map", m_compressMap);
  loaded_successfully &= parse_param("update_free_space_using_missing_data", m_updateFreeSpaceUsingMissingData);

  double probHit, probMiss, thresMin, thresMax;
  loaded_successfully &= parse_param("sensor_model.hit", probHit);
  loaded_successfully &= parse_param("sensor_model.miss", probMiss);
  loaded_successfully &= parse_param("sensor_model.min", thresMin);
  loaded_successfully &= parse_param("sensor_model.max", thresMax);
  loaded_successfully &= parse_param("sensor_model.max_range", m_maxRange);
  loaded_successfully &= parse_param("sensor_model.min_range", m_minRange);

  loaded_successfully &= parse_param("filter_speckles", m_filterSpeckles);

  loaded_successfully &= parse_param("ground_filter.enable", m_filterGroundPlane);
  loaded_successfully &= parse_param("ground_filter.distance", m_ZGroundFilterDistance);

  loaded_successfully &= parse_param("local_mapping.enable", m_localMapping);
  loaded_successfully &= parse_param("local_mapping.distance", m_localMapDistance);

  loaded_successfully &= parse_param("visualization.occupancy_min_z", m_occupancyMinZ);
  loaded_successfully &= parse_param("visualization.occupancy_max_z", m_occupancyMaxZ);

  loaded_successfully &= parse_param("visualization.colored_map.enabled", m_useColoredMap);

  loaded_successfully &= parse_param("visualization.height_map.enabled", m_useHeightMap);
  loaded_successfully &= parse_param("visualization.height_map.color_factor", m_colorFactor);
  double r, g, b, a;
  loaded_successfully &= parse_param("visualization.height_map.color.r", r);
  loaded_successfully &= parse_param("visualization.height_map.color.g", g);
  loaded_successfully &= parse_param("visualization.height_map.color.b", b);
  loaded_successfully &= parse_param("visualization.height_map.color.a", a);
  m_color.r = r;
  m_color.g = g;
  m_color.b = b;
  m_color.a = a;

  loaded_successfully &= parse_param("visualization.publish_free_space", m_publishFreeSpace);
  loaded_successfully &= parse_param("visualization.color_free.r", r);
  loaded_successfully &= parse_param("visualization.color_free.g", g);
  loaded_successfully &= parse_param("visualization.color_free.b", b);
  loaded_successfully &= parse_param("visualization.color_free.a", a);
  m_colorFree.r = r;
  m_colorFree.g = g;
  m_colorFree.b = b;
  m_colorFree.a = a;

  loaded_successfully &= parse_param("visualization.downprojected_2D_map.min_x_size", m_minSizeX);
  loaded_successfully &= parse_param("visualization.downprojected_2D_map.min_y_size", m_minSizeY);
  loaded_successfully &= parse_param("visualization.downprojected_2D_map.incremental_projection", m_incrementalUpdate);


  /* check parameters //{ */

  if (m_useHeightMap && m_useColoredMap) {
    std::string msg = std::string("You enabled both height map and RGBcolor registration.") + " This is contradictory. " + "Defaulting to height map.";
    RCLCPP_WARN(this->get_logger(), msg);
    m_useColoredMap = false;
  }

  if (m_useColoredMap) {
#ifdef COLOR_OCTOMAP_SERVER
    RCLCPP_WARN(this->get_logger(), "Using RGB color registration (if information available)");
#else
    std::string msg = std::string("Colored map requested in launch file") + " - node not running/compiled to support colors, " +
                      "please define COLOR_OCTOMAP_SERVER and recompile or launch " + "the octomap_color_server node";
    RCLCPP_WARN(this->get_logger(), msg);
#endif
  }

  if (m_updateFreeSpaceUsingMissingData && m_maxRange < 0.0) {
    std::string msg = std::string("You enabled updating free space using missing data in measurements. ") +
                      "However, the maximal sensor range is not limited. " + "Disabling this feature.";
    RCLCPP_WARN(this->get_logger(), msg);
    m_updateFreeSpaceUsingMissingData = false;
  }

  if (m_localMapping && m_localMapDistance < m_maxRange) {
    std::string msg = std::string("You enabled using only the local map. ") +
                      "However, the local distance for the map is lower than the maximal sensor range. " +
                      "Defaulting the local distance for the map to the maximal sensor range.";
    RCLCPP_WARN(this->get_logger(), msg);
    m_localMapDistance = m_maxRange;
  }


  //}

  if (!loaded_successfully) {
    const std::string str = "Could not load all non-optional parameters. Shutting down.";
    RCLCPP_ERROR(this->get_logger(), str);
    rclcpp::shutdown();
    return;
  }

  //}

  /* initialize octomap object & params //{ */

  m_octree = std::make_shared<OcTreeT>(m_res);
  m_octree->setProbHit(probHit);
  m_octree->setProbMiss(probMiss);
  m_octree->setClampingThresMin(thresMin);
  m_octree->setClampingThresMax(thresMax);
  m_treeDepth               = m_octree->getTreeDepth();
  m_maxTreeDepth            = m_treeDepth;
  m_gridmap.info.resolution = m_res;

  //}

  // --------------------------------------------------------------
  // |                         tf listener                        |
  // --------------------------------------------------------------

  /* tf_listener //{ */

  this->m_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->m_buffer->setUsingDedicatedThread(true);

  auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
  this->m_buffer->setCreateTimerInterface(create_timer_interface);

  this->m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_buffer, this, false);

  //}

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  /* publishers //{ */

  rclcpp::QoS qos(rclcpp::KeepLast(3));
  this->m_markerPub             = this->create_publisher<visualization_msgs::msg::MarkerArray>("occupied_cells_vis_array_out", qos);
  this->m_binaryMapPub          = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_binary_out", qos);
  this->m_fullMapPub            = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_full_out", qos);
  this->m_occupiedPointCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("octomap_point_cloud_centers_out", qos);
  this->m_freePointCloudPub     = this->create_publisher<sensor_msgs::msg::PointCloud2>("octomap_free_centers_out", qos);
  this->m_mapPub                = this->create_publisher<nav_msgs::msg::OccupancyGrid>("projected_map_out", qos);
  this->m_fmarkerPub            = this->create_publisher<visualization_msgs::msg::MarkerArray>("free_cells_vis_array_out", qos);

  //}

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  /* subscribers //{ */

  // Point Cloud
  this->m_pointCloudSub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "cloud_in", rmw_qos_profile_sensor_data);

  this->m_tfPointCloudSub = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
      *m_buffer, m_worldFrameId, 5, this->get_node_logging_interface(), this->get_node_clock_interface(), std::chrono::seconds(1));
  this->m_tfPointCloudSub->connectInput(*m_pointCloudSub);
  this->m_tfPointCloudSub->registerCallback(std::bind(&OctomapServer::insertCloudCallback, this, ph::_1));

  // Laser scan
  this->m_laserScanSub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, "laser_scan_in", rmw_qos_profile_sensor_data);

  this->m_tfLaserScanSub = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
      *m_buffer, m_worldFrameId, 5, this->get_node_logging_interface(), this->get_node_clock_interface(), std::chrono::seconds(1));
  this->m_tfLaserScanSub->connectInput(*m_laserScanSub);
  this->m_tfLaserScanSub->registerCallback(std::bind(&OctomapServer::insertLaserScanCallback, this, ph::_1));

  //}

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  /* service servers //{ */

  this->m_octomapBinaryService = this->create_service<OctomapSrv>("octomap_binary", std::bind(&OctomapServer::octomapBinarySrv, this, ph::_1, ph::_2));
  this->m_octomapFullService   = this->create_service<OctomapSrv>("octomap_full", std::bind(&OctomapServer::octomapFullSrv, this, ph::_1, ph::_2));
  this->m_clearBBXService      = this->create_service<BBXSrv>("clear_bbx", std::bind(&OctomapServer::clearBBXSrv, this, ph::_1, ph::_2));
  this->m_resetService         = this->create_service<std_srvs::srv::Empty>("reset", std::bind(&OctomapServer::resetSrv, this, ph::_1, ph::_2));

  //}

  m_isInitialized = true;
  RCLCPP_INFO(this->get_logger(), "Initialized");
}

//}

/* Class OctomapServer: destructor  //{ */

OctomapServer::~OctomapServer() {
}

//}

/* OctomapServer::openFile() //{ */

bool OctomapServer::openFile(const std::string& filename) {
  if (filename.length() <= 3)
    return false;

  std::string suffix = filename.substr(filename.length() - 3, 3);
  if (suffix == ".bt") {
    if (!m_octree->readBinary(filename)) {
      return false;
    }
  } else if (suffix == ".ot") {
    auto tree = octomap::AbstractOcTree::read(filename);
    if (!tree) {
      return false;
    }

    OcTreeT* octree = dynamic_cast<OcTreeT*>(tree);
    m_octree        = std::shared_ptr<OcTreeT>(octree);

    if (!m_octree) {
      std::string msg = "Could not read OcTree in file";
      RCLCPP_ERROR(this->get_logger(), msg.c_str());
      return false;
    }
  } else {
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Octomap file %s loaded (%zu nodes).", filename.c_str(), m_octree->size());

  m_treeDepth               = m_octree->getTreeDepth();
  m_maxTreeDepth            = m_treeDepth;
  m_res                     = m_octree->getResolution();
  m_gridmap.info.resolution = m_res;
  double minX, minY, minZ;
  double maxX, maxY, maxZ;
  m_octree->getMetricMin(minX, minY, minZ);
  m_octree->getMetricMax(maxX, maxY, maxZ);

  m_updateBBXMin[0] = m_octree->coordToKey(minX);
  m_updateBBXMin[1] = m_octree->coordToKey(minY);
  m_updateBBXMin[2] = m_octree->coordToKey(minZ);

  m_updateBBXMax[0] = m_octree->coordToKey(maxX);
  m_updateBBXMax[1] = m_octree->coordToKey(maxY);
  m_updateBBXMax[2] = m_octree->coordToKey(maxZ);

  publishAll(this->now());
  return true;
}

//}

/* OctomapServer::insertLaserScanCallback() //{ */

void OctomapServer::insertLaserScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan) {
  if (!m_isInitialized) {
    return;
  }

  PCLPointCloud::Ptr pc              = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr free_vectors_pc = boost::make_shared<PCLPointCloud>();

  Eigen::Matrix4f                      sensorToWorld;
  geometry_msgs::msg::TransformStamped sensorToWorldTf;
  try {
    if (!this->m_buffer->canTransform(m_worldFrameId, scan->header.frame_id, scan->header.stamp)) {
      throw "Failed";
    }

    sensorToWorldTf = this->m_buffer->lookupTransform(m_worldFrameId, scan->header.frame_id, scan->header.stamp);
    sensorToWorld   = pcl_ros::transformAsMatrix(sensorToWorldTf);
  }
  catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  sensor_msgs::msg::PointCloud2 ros_cloud;
  projector_.projectLaser(*scan, ros_cloud);
  pcl::fromROSMsg(ros_cloud, *pc);

  // directly transform to map frame:
  pcl::transformPointCloud(*pc, *pc, sensorToWorld);
  pc->header.frame_id = m_worldFrameId;

  // compute free rays, if required
  if (m_updateFreeSpaceUsingMissingData) {
    sensor_msgs::msg::LaserScan free_scan = *scan;
    for (auto& range : free_scan.ranges) {
      if (std::isfinite(range)) {
        range = std::numeric_limits<double>::infinity();
      } else {
        range = 1;
      }
    }

    projector_.projectLaser(free_scan, ros_cloud);
    pcl::fromROSMsg(ros_cloud, *free_vectors_pc);

    pcl::transformPointCloud(*free_vectors_pc, *free_vectors_pc, sensorToWorld);
  }
  free_vectors_pc->header.frame_id = m_worldFrameId;

  insertData(sensorToWorldTf.transform.translation, pc, free_vectors_pc);

  if (m_localMapping) {

    const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorToWorldTf.transform.translation);
    const octomap::point3d p_min         = sensor_origin - octomap::point3d(m_localMapDistance, m_localMapDistance, m_localMapDistance);
    const octomap::point3d p_max         = sensor_origin + octomap::point3d(m_localMapDistance, m_localMapDistance, m_localMapDistance);
    clearOutsideBBX(p_min, p_max);
  }

  publishAll(scan->header.stamp);
}

//}

/* OctomapServer::insertCloudCallback() //{ */

void OctomapServer::insertCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud) {
  if (!m_isInitialized) {
    return;
  }

  PCLPointCloud::Ptr pc              = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr free_vectors_pc = boost::make_shared<PCLPointCloud>();
  pcl::fromROSMsg(*cloud, *pc);

  Eigen::Matrix4f                      sensorToWorld;
  geometry_msgs::msg::TransformStamped sensorToWorldTf;
  try {
    if (!this->m_buffer->canTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp)) {
      throw "Failed";
    }

    sensorToWorldTf = this->m_buffer->lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp);
    sensorToWorld   = pcl_ros::transformAsMatrix(sensorToWorldTf);
  }
  catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  // directly transform to map frame:
  pcl::transformPointCloud(*pc, *pc, sensorToWorld);
  pc->header.frame_id = m_worldFrameId;

  // compute free rays, if required
  if (m_updateFreeSpaceUsingMissingData) {
    std::string text("For using free rays to update data, update insertCloudCallback according to available data.");
    RCLCPP_WARN(this->get_logger(), "%s", text.c_str());
  }
  free_vectors_pc->header.frame_id = m_worldFrameId;

  insertData(sensorToWorldTf.transform.translation, pc, free_vectors_pc);

  if (m_localMapping) {
    const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorToWorldTf.transform.translation);
    const octomap::point3d p_min         = sensor_origin - octomap::point3d(m_localMapDistance, m_localMapDistance, m_localMapDistance);
    const octomap::point3d p_max         = sensor_origin + octomap::point3d(m_localMapDistance, m_localMapDistance, m_localMapDistance);
    clearOutsideBBX(p_min, p_max);
  }

  publishAll(cloud->header.stamp);
}

//}

/* OctomapServer::insertData() //{ */

void OctomapServer::insertData(const geometry_msgs::msg::Vector3& sensorOriginTf, const PCLPointCloud::ConstPtr& cloud,
                               const PCLPointCloud::ConstPtr& free_vectors_cloud) {
  octomap::point3d    sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

  if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin) || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax)) {

    RCLCPP_WARN(this->get_logger(), "Could not generate Key for origin");
  }

#ifdef COLOR_OCTOMAP_SERVER
  unsigned char* colors = new unsigned char[3];
#endif

  octomap::KeySet free_cells, occupied_cells;
  // update free space using missing data
  for (auto it = free_vectors_cloud->begin(); it != free_vectors_cloud->end(); ++it) {
    octomap::point3d point(it->x, it->y, it->z);
    // maxrange check
    point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;

    if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {
      free_cells.insert(m_keyRay.begin(), m_keyRay.end());
    }

    octomap::OcTreeKey endKey;
    if (m_octree->coordToKeyChecked(point, endKey)) {
      updateMinKey(endKey, m_updateBBXMin);
      updateMaxKey(endKey, m_updateBBXMax);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not generate Key for endpoint");
    }
  }

  for (auto it = cloud->begin(); it != cloud->end(); ++it) {
    octomap::point3d point(it->x, it->y, it->z);
    // filter ground if neccessary
    if (m_filterGroundPlane && it->z <= m_ZGroundFilterDistance) {
      // maxrange check
      if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange)) {
        // free cells
        if (((point - sensorOrigin).norm() <= m_minRange) || m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {
          free_cells.insert(m_keyRay.begin(), m_keyRay.end());
        }

        // occupied endpoint
        octomap::OcTreeKey key;
        if (m_octree->coordToKeyChecked(point, key)) {
          free_cells.insert(key);

          updateMinKey(key, m_updateBBXMin);
          updateMaxKey(key, m_updateBBXMax);
        }
      } else {
        // ray longer than maxrange:;
        octomap::point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
        if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)) {
          free_cells.insert(m_keyRay.begin(), m_keyRay.end());

          octomap::OcTreeKey endKey;
          if (m_octree->coordToKeyChecked(new_end, endKey)) {
            free_cells.insert(endKey);
            updateMinKey(endKey, m_updateBBXMin);
            updateMaxKey(endKey, m_updateBBXMax);
          } else {
            RCLCPP_ERROR(this->get_logger(), "Could not generate Key for endpoint");
          }
        }
      }
      // all other points: free on ray, occupied on endpoint:
    } else {
      // range check
      if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange)) {
        // free cells
        if (((point - sensorOrigin).norm() <= m_minRange) || m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {
          free_cells.insert(m_keyRay.begin(), m_keyRay.end());
        }

        // occupied endpoint
        octomap::OcTreeKey key;
        if (m_octree->coordToKeyChecked(point, key)) {
          occupied_cells.insert(key);

          updateMinKey(key, m_updateBBXMin);
          updateMaxKey(key, m_updateBBXMax);

#ifdef COLOR_OCTOMAP_SERVER
          // NB: Only read and interpret color if it's an occupied node
          m_octree->averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
#endif
        }
      } else {
        // ray longer than maxrange:;
        octomap::point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
        if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)) {
          free_cells.insert(m_keyRay.begin(), m_keyRay.end());

          octomap::OcTreeKey endKey;
          if (m_octree->coordToKeyChecked(new_end, endKey)) {
            free_cells.insert(endKey);
            updateMinKey(endKey, m_updateBBXMin);
            updateMaxKey(endKey, m_updateBBXMax);
          } else {
            RCLCPP_ERROR(this->get_logger(), "Could not generate Key for endpoint");
          }
        }
      }
    }
  }

  // mark free cells only if not seen occupied in this cloud
  for (auto it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
    if (occupied_cells.find(*it) == occupied_cells.end()) {
      m_octree->updateNode(*it, false);
    }
  }

  // now mark all occupied cells:
  for (auto it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++) {
    m_octree->updateNode(*it, true);
  }

  if (m_compressMap) {
    m_octree->prune();
  }

#ifdef COLOR_OCTOMAP_SERVER
  if (colors) {
    delete[] colors;
    colors = NULL;
  }
#endif
}

//}

/* OctomapServer::publishAll() //{ */

void OctomapServer::publishAll(const rclcpp::Time& rostime) {

  // ros::WallTime startTime = ros::WallTime::now();

  size_t octomap_size = m_octree->size();
  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octomap_size <= 1) {
    RCLCPP_WARN(this->get_logger(), "Nothing to publish, octree is empty");
    return;
  }

  bool publishFreeMarkerArray = m_publishFreeSpace && m_fmarkerPub->get_subscription_count() > 0;
  bool publishMarkerArray     = m_markerPub->get_subscription_count() > 0;
  bool publishPointCloud      = m_occupiedPointCloudPub->get_subscription_count() > 0 || m_freePointCloudPub->get_subscription_count() > 0;
  bool publishBinaryMap       = m_binaryMapPub->get_subscription_count() > 0;
  bool publishFullMap         = m_fullMapPub->get_subscription_count() > 0;
  m_publish2DMap              = m_mapPub->get_subscription_count() > 0;

  // init markers for free space:
  visualization_msgs::msg::MarkerArray freeNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  freeNodesVis.markers.resize(m_treeDepth + 1);

  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, 0.0);
  geometry_msgs::msg::Pose pose;
  pose.orientation = tf2::toMsg(quaternion);

  // init markers:
  visualization_msgs::msg::MarkerArray occupiedNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth + 1);

  // init pointcloud:
  pcl::PointCloud<PCLPoint> occupied_pclCloud;
  pcl::PointCloud<PCLPoint> free_pclCloud;

  // call pre-traversal hook:
  handlePreNodeTraversal(rostime);

  // now, traverse all leafs in the tree:
  for (auto it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it) {
    bool inUpdateBBX = isInUpdateBBX(it);

    // call general hook:
    handleNode(it);
    if (inUpdateBBX) {
      handleNodeInBBX(it);
    }

    if (m_octree->isNodeOccupied(*it)) {
      double z         = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {
        double x = it.getX();
        double y = it.getY();
#ifdef COLOR_OCTOMAP_SERVER
        int r = it->getColor().r;
        int g = it->getColor().g;
        int b = it->getColor().b;
#endif

        // Ignore speckles in the map:
        if (m_filterSpeckles && (it.getDepth() == m_treeDepth + 1) && isSpeckleNode(it.getKey())) {
          RCLCPP_INFO(this->get_logger(), "Ignoring single speckle at (%f,%f,%f)", x, y, z);
          continue;
        }  // else: current octree node is no speckle, send it out


        handleOccupiedNode(it);
        if (inUpdateBBX) {
          handleOccupiedNodeInBBX(it);
        }

        // create marker:
        if (publishMarkerArray) {
          unsigned idx = it.getDepth();
          assert(idx < occupiedNodesVis.markers.size());

          geometry_msgs::msg::Point cubeCenter;
          cubeCenter.x = x;
          cubeCenter.y = y;
          cubeCenter.z = z;

          occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
          if (m_useHeightMap) {
            double minX, minY, minZ, maxX, maxY, maxZ;
            m_octree->getMetricMin(minX, minY, minZ);
            m_octree->getMetricMax(maxX, maxY, maxZ);

            double h = (1.0 - std::min(std::max((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * m_colorFactor;
            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
          }

#ifdef COLOR_OCTOMAP_SERVER
          if (m_useColoredMap) {
            // TODO
            // potentially use occupancy as measure for alpha channel?
            std_msgs::msg::ColorRGBA _color;
            _color.r = (r / 255.);
            _color.g = (g / 255.);
            _color.b = (b / 255.);
            _color.a = 1.0;
            occupiedNodesVis.markers[idx].colors.push_back(_color);
          }
#endif
        }

        // insert into pointcloud:
        if (publishPointCloud) {
#ifdef COLOR_OCTOMAP_SERVER
          PCLPoint _point = PCLPoint();
          _point.x        = x;
          _point.y        = y;
          _point.z        = z;
          _point.r        = r;
          _point.g        = g;
          _point.b        = b;
          occupied_pclCloud.push_back(_point);
#else
          occupied_pclCloud.push_back(PCLPoint(x, y, z));
#endif
        }
      }
    } else {
      // node not occupied => mark as free in 2D map if unknown so far
      double z         = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {
        handleFreeNode(it);
        if (inUpdateBBX) {
          handleFreeNodeInBBX(it);
        }

        if (m_publishFreeSpace) {
          double x = it.getX();
          double y = it.getY();

          // create marker for free space:
          if (publishFreeMarkerArray) {
            unsigned idx = it.getDepth();
            assert(idx < freeNodesVis.markers.size());

            geometry_msgs::msg::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            freeNodesVis.markers[idx].points.push_back(cubeCenter);
          }
        }

        // insert into pointcloud:
        if (publishPointCloud) {
          double x = it.getX();
          double y = it.getY();
          free_pclCloud.push_back(PCLPoint(x, y, z));
        }
      }
    }
  }

  // call post-traversal hook:
  handlePostNodeTraversal(rostime);

  // finish MarkerArray:
  if (publishMarkerArray) {
    for (size_t i = 0; i < occupiedNodesVis.markers.size(); ++i) {
      double size = m_octree->getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
      occupiedNodesVis.markers[i].header.stamp    = rostime;
      occupiedNodesVis.markers[i].ns              = "map";
      occupiedNodesVis.markers[i].id              = i;
      occupiedNodesVis.markers[i].type            = visualization_msgs::msg::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x         = size;
      occupiedNodesVis.markers[i].scale.y         = size;
      occupiedNodesVis.markers[i].scale.z         = size;
      if (!m_useColoredMap)
        occupiedNodesVis.markers[i].color = m_color;


      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::msg::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::msg::Marker::DELETE;
    }
    m_markerPub->publish(occupiedNodesVis);
  }

  // finish FreeMarkerArray:
  if (publishFreeMarkerArray) {
    for (size_t i = 0; i < freeNodesVis.markers.size(); ++i) {
      double size = m_octree->getNodeSize(i);

      freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
      freeNodesVis.markers[i].header.stamp    = rostime;
      freeNodesVis.markers[i].ns              = "map";
      freeNodesVis.markers[i].id              = i;
      freeNodesVis.markers[i].type            = visualization_msgs::msg::Marker::CUBE_LIST;
      freeNodesVis.markers[i].scale.x         = size;
      freeNodesVis.markers[i].scale.y         = size;
      freeNodesVis.markers[i].scale.z         = size;
      freeNodesVis.markers[i].color           = m_colorFree;

      if (freeNodesVis.markers[i].points.size() > 0)
        freeNodesVis.markers[i].action = visualization_msgs::msg::Marker::ADD;
      else
        freeNodesVis.markers[i].action = visualization_msgs::msg::Marker::DELETE;
    }
    m_fmarkerPub->publish(freeNodesVis);
  }


  // finish pointcloud:
  if (publishPointCloud) {
    sensor_msgs::msg::PointCloud2 cloud;
    // occupied
    pcl::toROSMsg(occupied_pclCloud, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp    = rostime;
    m_occupiedPointCloudPub->publish(cloud);

    // free
    pcl::toROSMsg(free_pclCloud, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp    = rostime;
    m_freePointCloudPub->publish(cloud);
  }

  if (publishBinaryMap) {
    publishBinaryOctoMap(rostime);
  }

  if (publishFullMap) {
    publishFullOctoMap(rostime);
  }

  /*
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);
  */
}

//}

/* OctomapServer::octomapBinarySrv() //{ */

bool OctomapServer::octomapBinarySrv([[maybe_unused]] const std::shared_ptr<OctomapSrv::Request> req, std::shared_ptr<OctomapSrv::Response> res) {
  // ros::WallTime startTime = ros::WallTime::now();
  RCLCPP_INFO(this->get_logger(), "Sending binary map data on service request");
  res->map.header.frame_id = m_worldFrameId;
  res->map.header.stamp    = this->get_clock()->now();
  if (!octomap_msgs::binaryMapToMsg(*m_octree, res->map)) {
    return false;
  }

  /*
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
  */
  return true;
}

//}

/* OctomapServer::octomapFullSrv() //{ */

bool OctomapServer::octomapFullSrv([[maybe_unused]] const std::shared_ptr<OctomapSrv::Request> req, std::shared_ptr<OctomapSrv::Response> res) {
  RCLCPP_INFO(this->get_logger(), "Sending full map data on service request");
  res->map.header.frame_id = m_worldFrameId;
  res->map.header.stamp    = this->get_clock()->now();

  if (!octomap_msgs::fullMapToMsg(*m_octree, res->map)) {
    return false;
  }
  return true;
}

//}

/* OctomapServer::clearBBXSrv() //{ */

bool OctomapServer::clearBBXSrv(const std::shared_ptr<BBXSrv::Request> req, [[maybe_unused]] std::shared_ptr<BBXSrv::Response> resp) {
  octomap::point3d min = octomap::pointMsgToOctomap(req->min);
  octomap::point3d max = octomap::pointMsgToOctomap(req->max);

  double thresMin = m_octree->getClampingThresMin();
  for (auto it = m_octree->begin_leafs_bbx(min, max), end = m_octree->end_leafs_bbx(); it != end; ++it) {
    it->setLogOdds(octomap::logodds(thresMin));
  }
  m_octree->updateInnerOccupancy();

  publishAll(this->now());

  return true;
}

//}

/* OctomapServer::resetSrv() //{ */

bool OctomapServer::resetSrv([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                             [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response>      resp) {
  visualization_msgs::msg::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(m_treeDepth + 1);
  auto rostime = this->now();

  m_octree->clear();
  // clear 2D map:
  m_gridmap.data.clear();
  m_gridmap.info.height            = 0.0;
  m_gridmap.info.width             = 0.0;
  m_gridmap.info.resolution        = 0.0;
  m_gridmap.info.origin.position.x = 0.0;
  m_gridmap.info.origin.position.y = 0.0;

  RCLCPP_INFO(this->get_logger(), "Cleared octomap");
  publishAll(rostime);

  publishBinaryOctoMap(rostime);
  for (size_t i = 0; i < occupiedNodesVis.markers.size(); ++i) {
    occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
    occupiedNodesVis.markers[i].header.stamp    = rostime;
    occupiedNodesVis.markers[i].ns              = "map";
    occupiedNodesVis.markers[i].id              = i;
    occupiedNodesVis.markers[i].type            = visualization_msgs::msg::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].action          = visualization_msgs::msg::Marker::DELETE;
  }

  m_markerPub->publish(occupiedNodesVis);

  visualization_msgs::msg::MarkerArray freeNodesVis;
  freeNodesVis.markers.resize(m_treeDepth + 1);

  for (size_t i = 0; i < freeNodesVis.markers.size(); ++i) {
    freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
    freeNodesVis.markers[i].header.stamp    = rostime;
    freeNodesVis.markers[i].ns              = "map";
    freeNodesVis.markers[i].id              = i;
    freeNodesVis.markers[i].type            = visualization_msgs::msg::Marker::CUBE_LIST;
    freeNodesVis.markers[i].action          = visualization_msgs::msg::Marker::DELETE;
  }
  m_fmarkerPub->publish(freeNodesVis);
  return true;
}

//}

/* OctomapServer::clearOutsideBBX() //{ */

bool OctomapServer::clearOutsideBBX(const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey minKey, maxKey;
  if (!m_octree->coordToKeyChecked(p_min, minKey) || !m_octree->coordToKeyChecked(p_max, maxKey)) {
    return false;
  }

  std::vector<std::pair<octomap::OcTreeKey, unsigned int>> keys;
  for (OcTreeT::leaf_iterator it = m_octree->begin_leafs(), end = m_octree->end_leafs(); it != end; ++it) {
    // check if outside of bbx:
    octomap::OcTreeKey k = it.getKey();
    if (k[0] < minKey[0] || k[1] < minKey[1] || k[2] < minKey[2] || k[0] > maxKey[0] || k[1] > maxKey[1] || k[2] > maxKey[2]) {
      keys.push_back(std::make_pair(k, it.getDepth()));
    }
  }

  for (auto k : keys) {
    m_octree->deleteNode(k.first, k.second);
  }

  RCLCPP_INFO(this->get_logger(), "Number of voxels removed outside local area: %i", keys.size());
  return true;
}

//}

/* OctomapServer::publishBinaryOctoMap() //{ */

void OctomapServer::publishBinaryOctoMap(const rclcpp::Time& rostime) const {

  octomap_msgs::msg::Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp    = rostime;

  if (octomap_msgs::binaryMapToMsg(*m_octree, map)) {
    m_binaryMapPub->publish(map);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Error serializing OctoMap");
  }
}

//}

/*  OctomapServer::publishFullOctoMap() //{ */

void OctomapServer::publishFullOctoMap(const rclcpp::Time& rostime) const {

  octomap_msgs::msg::Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp    = rostime;

  if (octomap_msgs::fullMapToMsg(*m_octree, map)) {
    m_fullMapPub->publish(map);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Error serializing OctoMap");
  }
}

//}

/* OctomapServer::handlePreNodeTraversal() //{ */

void OctomapServer::handlePreNodeTraversal(const rclcpp::Time& rostime) {
  if (m_publish2DMap) {
    // init projected 2D map:
    m_gridmap.header.frame_id             = m_worldFrameId;
    m_gridmap.header.stamp                = rostime;
    nav_msgs::msg::MapMetaData oldMapInfo = m_gridmap.info;

    // TODO:
    // move most of this stuff into c'tor and init map only once(adjust if size changes)
    double minX, minY, minZ, maxX, maxY, maxZ;
    m_octree->getMetricMin(minX, minY, minZ);
    m_octree->getMetricMax(maxX, maxY, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);

    [[maybe_unused]] octomap::OcTreeKey minKey = m_octree->coordToKey(minPt, m_maxTreeDepth);
    [[maybe_unused]] octomap::OcTreeKey maxKey = m_octree->coordToKey(maxPt, m_maxTreeDepth);

    /* RCLCPP_INFO(this->get_logger(), "MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]); */

    // add padding if requested (= new min/maxPts in x&y):
    if (m_minSizeX > 0.0 || m_minSizeY > 0.0) {
      const double sizeX = maxX - minX;
      const double sizeY = maxY - minY;
      if (m_minSizeX > sizeX) {
        const double centerX = maxX - sizeX / 2;
        minX                 = centerX - m_minSizeX / 2;
        maxX                 = centerX + m_minSizeX / 2;
      }
      if (m_minSizeY > sizeY) {
        const double centerY = maxY - sizeY / 2;
        minY                 = centerY - m_minSizeY / 2;
        maxY                 = centerY + m_minSizeY / 2;
      }
      minPt = octomap::point3d(minX, minY, minZ);
      maxPt = octomap::point3d(maxX, maxY, maxZ);
    }

    octomap::OcTreeKey paddedMaxKey;
    if (!m_octree->coordToKeyChecked(minPt, m_maxTreeDepth, m_paddedMinKey)) {
      RCLCPP_ERROR(this->get_logger(), "Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
      return;
    }
    if (!m_octree->coordToKeyChecked(maxPt, m_maxTreeDepth, paddedMaxKey)) {
      RCLCPP_ERROR(this->get_logger(), "Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
      return;
    }

    /* RCLCPP_INFO(this->get_logger(), "Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2], */
    /*             paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]); */
    assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

    m_multires2DScale     = 1 << (m_treeDepth - m_maxTreeDepth);
    m_gridmap.info.width  = (paddedMaxKey[0] - m_paddedMinKey[0]) / m_multires2DScale + 1;
    m_gridmap.info.height = (paddedMaxKey[1] - m_paddedMinKey[1]) / m_multires2DScale + 1;

    [[maybe_unused]] int mapOriginX = minKey[0] - m_paddedMinKey[0];
    [[maybe_unused]] int mapOriginY = minKey[1] - m_paddedMinKey[1];
    assert(mapOriginX >= 0 && mapOriginY >= 0);

    // might not exactly be min / max of octree:
    octomap::point3d origin  = m_octree->keyToCoord(m_paddedMinKey, m_treeDepth);
    double           gridRes = m_octree->getNodeSize(m_maxTreeDepth);
    m_projectCompleteMap     = (!m_incrementalUpdate || (std::abs(gridRes - m_gridmap.info.resolution) > 1e-6));

    m_gridmap.info.resolution        = gridRes;
    m_gridmap.info.origin.position.x = origin.x() - gridRes * 0.5;
    m_gridmap.info.origin.position.y = origin.y() - gridRes * 0.5;
    if (m_maxTreeDepth != m_treeDepth) {
      m_gridmap.info.origin.position.x -= m_res / 2.0;
      m_gridmap.info.origin.position.y -= m_res / 2.0;
    }

    // workaround for  multires. projection not working properly for inner nodes:
    // force re-building complete map
    if (m_maxTreeDepth < m_treeDepth) {
      m_projectCompleteMap = true;
    }

    if (m_projectCompleteMap) {
      /* RCLCPP_INFO(this->get_logger(), "Rebuilding complete 2D map"); */
      m_gridmap.data.clear();
      // init to unknown:
      m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);

    } else {
      if (mapChanged(oldMapInfo, m_gridmap.info)) {
        RCLCPP_INFO(this->get_logger(), "2D grid map size changed to %dx%d", m_gridmap.info.width, m_gridmap.info.height);
        adjustMapData(m_gridmap, oldMapInfo);
      }
      /* nav_msgs::msg::OccupancyGrid::_data_type::iterator startIt; */
      auto mapUpdateBBXMinX = std::max(0, (int(m_updateBBXMin[0]) - int(m_paddedMinKey[0])) / int(m_multires2DScale));
      auto mapUpdateBBXMinY = std::max(0, (int(m_updateBBXMin[1]) - int(m_paddedMinKey[1])) / int(m_multires2DScale));
      auto mapUpdateBBXMaxX = std::min(int(m_gridmap.info.width - 1), (int(m_updateBBXMax[0]) - int(m_paddedMinKey[0])) / int(m_multires2DScale));
      auto mapUpdateBBXMaxY = std::min(int(m_gridmap.info.height - 1), (int(m_updateBBXMax[1]) - int(m_paddedMinKey[1])) / int(m_multires2DScale));

      assert(mapUpdateBBXMaxX > mapUpdateBBXMinX);
      assert(mapUpdateBBXMaxY > mapUpdateBBXMinY);

      auto numCols = mapUpdateBBXMaxX - mapUpdateBBXMinX + 1;

      // test for max idx:
      auto max_idx = m_gridmap.info.width * mapUpdateBBXMaxY + mapUpdateBBXMaxX;
      if (max_idx >= m_gridmap.data.size()) {
        RCLCPP_ERROR(this->get_logger(), std::string("BBX index not valid:") + "%d (max index %zu for size %d x %d) update-BBX is: " + "[%zu %zu]-[%zu %zu]",
                     max_idx, m_gridmap.data.size(), m_gridmap.info.width, m_gridmap.info.height, mapUpdateBBXMinX, mapUpdateBBXMinY, mapUpdateBBXMaxX,
                     mapUpdateBBXMaxY);
      }

      // reset proj. 2D map in bounding box:
      for (int j = mapUpdateBBXMinY; j <= mapUpdateBBXMaxY; ++j) {
        std::fill_n(m_gridmap.data.begin() + m_gridmap.info.width * j + mapUpdateBBXMinX, numCols, -1);
      }
    }
  }
}

//}

/* OctomapServer::handlePostNodeTraversal() //{ */

void OctomapServer::handlePostNodeTraversal([[maybe_unused]] const rclcpp::Time& rostime) {
  if (m_publish2DMap) {
    m_mapPub->publish(m_gridmap);
  }
}

//}

/* OctomapServer::handleOccupiedNode() //{ */

void OctomapServer::handleOccupiedNode(const OcTreeT::iterator& it) {
  if (m_publish2DMap && m_projectCompleteMap) {
    update2DMap(it, true);
  }
}

//}

/* OctomapServer::handleFreeNode() //{ */

void OctomapServer::handleFreeNode(const OcTreeT::iterator& it) {
  if (m_publish2DMap && m_projectCompleteMap) {
    update2DMap(it, false);
  }
}

//}

/* OctomapServer::handleOccupiedNodeInBBX() //{ */

void OctomapServer::handleOccupiedNodeInBBX(const OcTreeT::iterator& it) {
  if (m_publish2DMap && !m_projectCompleteMap) {
    update2DMap(it, true);
  }
}

//}

/* OctomapServer::handleFreeNodeInBBX() //{ */

void OctomapServer::handleFreeNodeInBBX(const OcTreeT::iterator& it) {
  if (m_publish2DMap && !m_projectCompleteMap) {
    update2DMap(it, false);
  }
}

//}

/* OctomapServer::update2DMap() //{ */

void OctomapServer::update2DMap(const OcTreeT::iterator& it, bool occupied) {
  if (it.getDepth() == m_maxTreeDepth) {
    auto idx = mapIdx(it.getKey());
    if (occupied) {
      m_gridmap.data[mapIdx(it.getKey())] = 100;
    } else if (m_gridmap.data[idx] == -1) {
      m_gridmap.data[idx] = 0;
    }
  } else {
    int                intSize = 1 << (m_maxTreeDepth - it.getDepth());
    octomap::OcTreeKey minKey  = it.getIndexKey();
    for (int dx = 0; dx < intSize; dx++) {
      int i = (minKey[0] + dx - m_paddedMinKey[0]) / m_multires2DScale;
      for (int dy = 0; dy < intSize; dy++) {
        auto idx = mapIdx(i, (minKey[1] + dy - m_paddedMinKey[1]) / m_multires2DScale);
        if (occupied) {
          m_gridmap.data[idx] = 100;
        } else if (m_gridmap.data[idx] == -1) {
          m_gridmap.data[idx] = 0;
        }
      }
    }
  }
}

//}

/* OctomapServer::isSpeckleNode() //{ */

bool OctomapServer::isSpeckleNode(const octomap::OcTreeKey& nKey) const {
  octomap::OcTreeKey key;
  bool               neighborFound = false;
  for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]) {
    for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]) {
      for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]) {
        if (key != nKey) {
          auto node = m_octree->search(key);
          if (node && m_octree->isNodeOccupied(node)) {
            neighborFound = true;
          }
        }
      }
    }
  }
  return neighborFound;
}

//}

/* OctomapServer::adjustMapData() //{ */

void OctomapServer::adjustMapData(nav_msgs::msg::OccupancyGrid& map, const nav_msgs::msg::MapMetaData& oldMapInfo) const {
  if (map.info.resolution != oldMapInfo.resolution) {
    RCLCPP_ERROR(this->get_logger(), "Resolution of map changed, cannot be adjusted");
    return;
  }

  int i_off = int((oldMapInfo.origin.position.x - map.info.origin.position.x) / map.info.resolution + 0.5);
  int j_off = int((oldMapInfo.origin.position.y - map.info.origin.position.y) / map.info.resolution + 0.5);

  if (i_off < 0 || j_off < 0 || oldMapInfo.width + i_off > map.info.width || oldMapInfo.height + j_off > map.info.height) {
    RCLCPP_ERROR(this->get_logger(), "New 2D map does not contain old map area, this case is not implemented");
    return;
  }

  // nav_msgs::msg::OccupancyGrid::_data_type oldMapData =
  // map.data;
  auto oldMapData = map.data;

  map.data.clear();
  // init to unknown:
  map.data.resize(map.info.width * map.info.height, -1);
  nav_msgs::msg::OccupancyGrid::_data_type::iterator fromStart, fromEnd, toStart;

  for (int j = 0; j < int(oldMapInfo.height); ++j) {
    // copy chunks, row by row:
    fromStart = oldMapData.begin() + j * oldMapInfo.width;
    fromEnd   = fromStart + oldMapInfo.width;
    toStart   = map.data.begin() + ((j + j_off) * m_gridmap.info.width + i_off);
    copy(fromStart, fromEnd, toStart);
  }
}

//}

/* OctomapServer::heightMapColor() //{ */

std_msgs::msg::ColorRGBA OctomapServer::heightMapColor(double h) {
  std_msgs::msg::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int    i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:
      color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:
      color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:
      color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:
      color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:
      color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:
      color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }
  return color;
}

//}

/* OctomapServer::parse_param() //{ */
template <class T>
bool OctomapServer::parse_param(const std::string& param_name, T& param_dest) {
  this->declare_parameter(param_name);
  if (!this->get_parameter(param_name, param_dest)) {
    RCLCPP_ERROR(this->get_logger(), "Could not load param '%s'", param_name.c_str());
    return false;
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded '" << param_name << "' = '" << param_dest << "'");
  }
  return true;
}
//}

/* OctomapServer::parse_param impl //{ */
template bool OctomapServer::parse_param<int>(const std::string& param_name, int& param_dest);
template bool OctomapServer::parse_param<double>(const std::string& param_name, double& param_dest);
template bool OctomapServer::parse_param<float>(const std::string& param_name, float& param_dest);
template bool OctomapServer::parse_param<std::string>(const std::string& param_name, std::string& param_dest);
template bool OctomapServer::parse_param<bool>(const std::string& param_name, bool& param_dest);
template bool OctomapServer::parse_param<unsigned int>(const std::string& param_name, unsigned int& param_dest);
//}

}  // namespace octomap_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::OctomapServer)
