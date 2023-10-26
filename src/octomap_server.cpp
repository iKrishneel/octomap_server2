
#include <octomap_server2/octomap_server.hpp>

namespace octomap_server {
    OctomapServer::OctomapServer(
        const rclcpp::NodeOptions &options,
        const std::string node_name):
        Node(node_name, options),
        m_octree(NULL),
        m_maxRange(20),
        m_worldFrameId("/map"),
        m_baseFrameId("base_footprint"),
        m_useHeightMap(true),
        m_useColoredMap(false),
        m_colorFactor(0.8),
        m_publishFreeSpace(false),
        m_res(0.05),
        m_treeDepth(0),
        m_maxTreeDepth(0),
        m_pointcloudMinX(-std::numeric_limits<double>::max()),
        m_pointcloudMaxX(std::numeric_limits<double>::max()),
        m_pointcloudMinY(-std::numeric_limits<double>::max()),
        m_pointcloudMaxY(std::numeric_limits<double>::max()),
        m_pointcloudMinZ(-std::numeric_limits<double>::max()),
        m_pointcloudMaxZ(std::numeric_limits<double>::max()),
        m_occupancyMinZ(-std::numeric_limits<double>::max()),
        m_occupancyMaxZ(std::numeric_limits<double>::max()),
        m_minSizeX(0.0),
        m_minSizeY(0.0),
        m_filterSpeckles(false),
        m_filterGroundPlane(false),
        m_groundFilterDistance(0.04),
        m_groundFilterAngle(0.15),
        m_groundFilterPlaneDistance(0.07),
        m_compressMap(true),
        m_incrementalUpdate(false) {

        rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
        this->buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
        this->buffer_->setUsingDedicatedThread(true);
        this->m_tfListener = std::make_shared<tf2_ros::TransformListener>(
            *buffer_, this, false);
        
        m_worldFrameId = this->declare_parameter("frame_id", m_worldFrameId);
        m_baseFrameId = this->declare_parameter("base_frame_id", m_baseFrameId);
        m_useHeightMap = this->declare_parameter("height_map", m_useHeightMap);
        m_useColoredMap = this->declare_parameter("colored_map", m_useColoredMap);
        m_colorFactor = this->declare_parameter("color_factor", m_colorFactor);

        m_pointcloudMinX = this->declare_parameter(
            "pointcloud_min_x", m_pointcloudMinX);
        m_pointcloudMaxX = this->declare_parameter(
            "pointcloud_max_x", m_pointcloudMaxX);
        m_pointcloudMinY = this->declare_parameter(
            "pointcloud_min_y", m_pointcloudMinY);
        m_pointcloudMaxY = this->declare_parameter(
            "pointcloud_max_y", m_pointcloudMaxY);
        m_pointcloudMinZ = this->declare_parameter(
            "pointcloud_min_z", m_pointcloudMinZ);
        m_pointcloudMaxZ = this->declare_parameter(
            "pointcloud_max_z", m_pointcloudMaxZ);
        m_occupancyMinZ = this->declare_parameter(
            "occupancy_min_z", m_occupancyMinZ);
        m_occupancyMaxZ = this->declare_parameter(
            "occupancy_max_z", m_occupancyMaxZ);
        m_minSizeX = this->declare_parameter(
            "min_x_size", m_minSizeX);
        m_minSizeY = this->declare_parameter(
            "min_y_size", m_minSizeY);

        m_filterSpeckles = this->declare_parameter(
            "filter_speckles", m_filterSpeckles);
        m_filterGroundPlane = this->declare_parameter(
            "filter_ground", m_filterGroundPlane);
        // distance of points from plane for RANSAC
        m_groundFilterDistance = this->declare_parameter(
            "ground_filter/distance", m_groundFilterDistance);
        // angular derivation of found plane:
        m_groundFilterAngle = this->declare_parameter(
            "ground_filter/angle", m_groundFilterAngle);
        m_groundFilterPlaneDistance = this->declare_parameter(
            "ground_filter/plane_distance", m_groundFilterPlaneDistance);

        m_maxRange = this->declare_parameter(
            "sensor_model/max_range", m_maxRange);

        m_res = this->declare_parameter("resolution", m_res);
        double probHit = this->declare_parameter("sensor_model/hit", 0.7);
        double probMiss = this->declare_parameter("sensor_model/miss", 0.4);
        double thresMin = this->declare_parameter("sensor_model/min", 0.12);
        double thresMax = this->declare_parameter("sensor_model/max", 0.97);
        m_compressMap = this->declare_parameter("compress_map", m_compressMap);
        m_incrementalUpdate = this->declare_parameter(
            "incremental_2D_projection", false);

        if (m_filterGroundPlane &&
            (m_pointcloudMinZ > 0.0 || m_pointcloudMaxZ < 0.0)) {
            std::string msg = "You enabled ground filtering but incoming pointclouds " +
                std::string("will be pre-filtered in [%ld, %ld], excluding the") +
                std::string("ground level z=0 This will not work.");
            RCLCPP_WARN(this->get_logger(), msg.c_str(), m_pointcloudMinZ, m_pointcloudMaxZ);
        }

        if (m_useHeightMap && m_useColoredMap) {
            std::string msg = std::string("You enabled both height map and RGB") +
                "color registration. This is contradictory. Defaulting to height map."; 
            RCLCPP_WARN(this->get_logger(), msg.c_str());
            m_useColoredMap = false;
        }

        if (m_useColoredMap) {
#ifdef COLOR_OCTOMAP_SERVER
            RCLCPP_WARN(
                this->get_logger(),
                "Using RGB color registration (if information available)");
#else
            std::string msg = std::string("Colored map requested in launch file") +
                " - node not running/compiled to support colors, " +
                "please define COLOR_OCTOMAP_SERVER and recompile or launch " +
                "the octomap_color_server node";
            RCLCPP_WARN(this->get_logger(), msg.c_str());
#endif
        }

        // initialize octomap object & params
        m_octree = std::make_shared<OcTreeT>(m_res);
        m_octree->setProbHit(probHit);
        m_octree->setProbMiss(probMiss);
        m_octree->setClampingThresMin(thresMin);
        m_octree->setClampingThresMax(thresMax);
        m_treeDepth = m_octree->getTreeDepth();
        m_maxTreeDepth = m_treeDepth;
        m_gridmap.info.resolution = m_res;

        double r = this->declare_parameter("color/r", 0.0);
        double g = this->declare_parameter("color/g", 0.0);
        double b = this->declare_parameter("color/b", 1.0);
        double a = this->declare_parameter("color/a", 1.0);
        m_color.r = r;
        m_color.g = g;
        m_color.b = b;
        m_color.a = a;

        r = this->declare_parameter("color_free/r", 0.0);
        g = this->declare_parameter("color_free/g", 1.0);
        b = this->declare_parameter("color_free/b", 0.0);
        a = this->declare_parameter("color_free/a", 1.0);
        m_colorFree.r = r;
        m_colorFree.g = g;
        m_colorFree.b = b;
        m_colorFree.a = a;
        
        m_publishFreeSpace = this->declare_parameter(
            "publish_free_space", m_publishFreeSpace);
        std::string msg = std::string("Publishing non-latched (topics are only)") +
                    "prepared as needed, will only be re-published on map change";
        RCLCPP_INFO(this->get_logger(), msg.c_str());

        RCLCPP_INFO(this->get_logger(), "Frame Id %s", m_worldFrameId.c_str());
        RCLCPP_INFO(this->get_logger(), "Resolution %.2f", m_res);
        
        this->onInit();
    }

    OctomapServer::~OctomapServer() {

    }

    void OctomapServer::onInit() {
        this->subscribe();

        rclcpp::QoS qos(rclcpp::KeepLast(3));
        this->m_markerPub = this->create_publisher<
            visualization_msgs::msg::MarkerArray>(
                "occupied_cells_vis_array", qos);
        this->m_binaryMapPub = this->create_publisher<
            octomap_msgs::msg::Octomap>("octomap_binary", qos);
        this->m_fullMapPub = this->create_publisher<
            octomap_msgs::msg::Octomap>("octomap_full", qos);
        this->m_pointCloudPub = this->create_publisher<
            sensor_msgs::msg::PointCloud2>(
                "octomap_point_cloud_centers", qos);
        this->m_mapPub = this->create_publisher<
            nav_msgs::msg::OccupancyGrid>("projected_map", qos);
        this->m_fmarkerPub = this->create_publisher<
            visualization_msgs::msg::MarkerArray>(
                "free_cells_vis_array", qos);
    }

    void OctomapServer::subscribe() {
        this->m_pointCloudSub = std::make_shared<
            message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
                this, "cloud_in", rmw_qos_profile_sensor_data);

        auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface());
        this->buffer_->setCreateTimerInterface(create_timer_interface);
        
        this->m_tfPointCloudSub = std::make_shared<tf2_ros::MessageFilter<
            sensor_msgs::msg::PointCloud2>>(
                *buffer_, m_worldFrameId, 5,
                this->get_node_logging_interface(),
                this->get_node_clock_interface(),
                std::chrono::seconds(1));
        this->m_tfPointCloudSub->connectInput(*m_pointCloudSub);
        this->m_tfPointCloudSub->registerCallback(
            std::bind(&OctomapServer::insertCloudCallback, this, ph::_1));

        this->m_octomapBinaryService = this->create_service<OctomapSrv>(
            "octomap_binary",
            std::bind(&OctomapServer::octomapBinarySrv, this, ph::_1, ph::_2));
        this->m_octomapFullService = this->create_service<OctomapSrv>(
            "octomap_full",
            std::bind(&OctomapServer::octomapFullSrv, this, ph::_1, ph::_2));
        this->m_clearBBXService = this->create_service<BBXSrv>(
            "clear_bbx",
            std::bind(&OctomapServer::clearBBXSrv, this, ph::_1, ph::_2));
        this->m_resetService = this->create_service<std_srvs::srv::Empty>(
            "reset", std::bind(&OctomapServer::resetSrv, this, ph::_1, ph::_2));

        RCLCPP_INFO(this->get_logger(), "Setup completed!");
    }

    bool OctomapServer::openFile(const std::string &filename){
        if (filename.length() <= 3)
            return false;

        std::string suffix = filename.substr(filename.length()-3, 3);
        if (suffix== ".bt") {
            if (!m_octree->readBinary(filename)) {
                return false;
            }
        } else if (suffix == ".ot") {
            auto tree = octomap::AbstractOcTree::read(filename);
            if (!tree){
                return false;
            }

            OcTreeT *octree = dynamic_cast<OcTreeT*>(tree);
            m_octree = std::shared_ptr<OcTreeT>(octree);
            
            if (!m_octree) {
                std::string msg = "Could not read OcTree in file";
                RCLCPP_ERROR(this->get_logger(), msg.c_str());
                return false;
            }
        } else {
            return false;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Octomap file %s loaded (%zu nodes).",
                    filename.c_str(), m_octree->size());

        m_treeDepth = m_octree->getTreeDepth();
        m_maxTreeDepth = m_treeDepth;
        m_res = m_octree->getResolution();
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
        
        rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>();
        publishAll(clock->now());
        return true;
    }

    void OctomapServer::insertCloudCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud){
        auto start = std::chrono::steady_clock::now();
        
        //
        // ground filtering in base frame
        //
        PCLPointCloud pc; // input cloud for filtering and ground-detection
        pcl::fromROSMsg(*cloud, pc);
        
        Eigen::Matrix4f sensorToWorld;
        geometry_msgs::msg::TransformStamped sensorToWorldTf;
        try {
            if (!this->buffer_->canTransform(
                    m_worldFrameId, cloud->header.frame_id,
                    cloud->header.stamp)) {
                throw "Failed";
            }
            
            // RCLCPP_INFO(this->get_logger(), "Can transform");

            sensorToWorldTf = this->buffer_->lookupTransform(
                m_worldFrameId, cloud->header.frame_id,
                cloud->header.stamp);
            sensorToWorld = pcl_ros::transformAsMatrix(sensorToWorldTf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s",ex.what());
            return;
        }

        // set up filter for height range, also removes NANs:
        pcl::PassThrough<PCLPoint> pass_x;
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
        pcl::PassThrough<PCLPoint> pass_y;
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
        pcl::PassThrough<PCLPoint> pass_z;
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

        PCLPointCloud pc_ground; // segmented ground plane
        PCLPointCloud pc_nonground; // everything else
        
        if (m_filterGroundPlane) {
            geometry_msgs::msg::TransformStamped baseToWorldTf;
            geometry_msgs::msg::TransformStamped sensorToBaseTf;
            
            try {
                if (!this->buffer_->canTransform(
                    m_baseFrameId, cloud->header.frame_id,
                    cloud->header.stamp)) {
                    throw "Failed";
                }

                sensorToBaseTf = this->buffer_->lookupTransform(
                    m_baseFrameId, cloud->header.frame_id,
                    cloud->header.stamp);
                baseToWorldTf = this->buffer_->lookupTransform(
                    m_worldFrameId, m_baseFrameId, cloud->header.stamp);
            } catch (tf2::TransformException& ex) {
                std::string msg = std::string("Transform error for ground plane filter") +
                    "You need to set the base_frame_id or disable filter_ground.";
                RCLCPP_ERROR(this->get_logger(), "%s %", msg, ex.what());
                return;
            }

            Eigen::Matrix4f sensorToBase =
                pcl_ros::transformAsMatrix(sensorToBaseTf);
            Eigen::Matrix4f baseToWorld = 
                pcl_ros::transformAsMatrix(baseToWorldTf);

            // transform pointcloud from sensor frame to fixed robot frame
            pcl::transformPointCloud(pc, pc, sensorToBase);
            pass_x.setInputCloud(pc.makeShared());
            pass_x.filter(pc);
            pass_y.setInputCloud(pc.makeShared());
            pass_y.filter(pc);
            pass_z.setInputCloud(pc.makeShared());
            pass_z.filter(pc);
            filterGroundPlane(pc, pc_ground, pc_nonground);

            // transform clouds to world frame for insertion
            pcl::transformPointCloud(pc_ground, pc_ground, baseToWorld);
            pcl::transformPointCloud(pc_nonground, pc_nonground, baseToWorld);
        } else {
            // directly transform to map frame:
            pcl::transformPointCloud(pc, pc, sensorToWorld);
            
            // just filter height range:
            pass_x.setInputCloud(pc.makeShared());
            pass_x.filter(pc);
            pass_y.setInputCloud(pc.makeShared());
            pass_y.filter(pc);
            pass_z.setInputCloud(pc.makeShared());
            pass_z.filter(pc);

            pc_nonground = pc;
            // pc_nonground is empty without ground segmentation
            pc_ground.header = pc.header;
            pc_nonground.header = pc.header;
        }
        
        insertScan(sensorToWorldTf.transform.translation,
                   pc_ground, pc_nonground);

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        RCLCPP_INFO(this->get_logger(), "Time lapse %f", elapsed_seconds.count());
        
        publishAll(cloud->header.stamp);
    }

    void OctomapServer::insertScan(
        const geometry_msgs::msg::Vector3 &sensorOriginTf,
        const PCLPointCloud& ground,
        const PCLPointCloud& nonground) {
        octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);
        
        if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
            || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax)) {

            RCLCPP_WARN(this->get_logger(),
                        "Could not generate Key for origin");
        }

#ifdef COLOR_OCTOMAP_SERVER
        unsigned char* colors = new unsigned char[3];
#endif
        
        // instead of direct scan insertion, compute update to filter ground:
        octomap::KeySet free_cells, occupied_cells;
        // insert ground points only as free:
        for (auto it = ground.begin(); it != ground.end(); ++it) {
            octomap::point3d point(it->x, it->y, it->z);
            // maxrange check
            if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange) ) {
                point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            }

            // only clear space (ground points)
            if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }

            octomap::OcTreeKey endKey;
            if (m_octree->coordToKeyChecked(point, endKey)) {
                updateMinKey(endKey, m_updateBBXMin);
                updateMaxKey(endKey, m_updateBBXMax);
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "Could not generate Key for endpoint");
            }
        }

        auto start = std::chrono::steady_clock::now();
        
        // all other points: free on ray, occupied on endpoint:
        for (auto it = nonground.begin(); it != nonground.end(); ++it) {
            octomap::point3d point(it->x, it->y, it->z);
            // maxrange check            
            if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange)) {
                // free cells
                if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {
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
                    m_octree->averageNodeColor(it->x, it->y, it->z,
                                               it->r, it->g, it->b);
#endif
                }
            } else {
                // ray longer than maxrange:;
                octomap::point3d new_end = sensorOrigin +
                    (point - sensorOrigin).normalized() * m_maxRange;
                if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)) {
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                    octomap::OcTreeKey endKey;
                    if (m_octree->coordToKeyChecked(new_end, endKey)) {
                        free_cells.insert(endKey);
                        updateMinKey(endKey, m_updateBBXMin);
                        updateMaxKey(endKey, m_updateBBXMax);
                    } else {
                        RCLCPP_ERROR(this->get_logger(),
                                     "Could not generate Key for endpoint");
                    }
                }
            }
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        RCLCPP_INFO(this->get_logger(), "Time lapse[insert] %f", elapsed_seconds.count());
        
        // mark free cells only if not seen occupied in this cloud
        for(auto it = free_cells.begin(), end=free_cells.end();
            it!= end; ++it){
            if (occupied_cells.find(*it) == occupied_cells.end()){
                m_octree->updateNode(*it, false);
            }
        }

        // now mark all occupied cells:
        for (auto it = occupied_cells.begin(),
                 end=occupied_cells.end(); it!= end; it++) {
            m_octree->updateNode(*it, true);
        }

        // TODO: eval lazy+updateInner vs. proper insertion
        // non-lazy by default (updateInnerOccupancy() too slow for large maps)
        //m_octree->updateInnerOccupancy();
        octomap::point3d minPt, maxPt;
        /* todo
        ROS_DEBUG_STREAM("Bounding box keys (before): "
                         << m_updateBBXMin[0] << " "
                         << m_updateBBXMin[1] << " "
                         << m_updateBBXMin[2] << " / "
                         <<m_updateBBXMax[0] << " "
                         << m_updateBBXMax[1] << " "
                         << m_updateBBXMax[2]);
        */
        
        // TODO: we could also limit the bbx to be within the map bounds here
        // (see publishing check)
        
        minPt = m_octree->keyToCoord(m_updateBBXMin);
        maxPt = m_octree->keyToCoord(m_updateBBXMax);
        /* todo
        ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
        ROS_DEBUG_STREAM("Bounding box keys (after): "
                         << m_updateBBXMin[0] << " " << m_updateBBXMin[1]
                         << " " << m_updateBBXMin[2] << " / "
                         << m_updateBBXMax[0] << " "<< m_updateBBXMax[1]
                         << " "<< m_updateBBXMax[2]);
        */

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

    void OctomapServer::publishAll(
        const rclcpp::Time &rostime) {

        // ros::WallTime startTime = ros::WallTime::now();
        
        size_t octomap_size = m_octree->size();
        // TODO: estimate num occ. voxels for size of arrays (reserve)
        if (octomap_size <= 1) {
            RCLCPP_WARN(
                this->get_logger(),
                "Nothing to publish, octree is empty");
            return;
        }

        bool publishFreeMarkerArray = m_publishFreeSpace &&
            m_fmarkerPub->get_subscription_count()  > 0;
        bool publishMarkerArray = m_markerPub->get_subscription_count() > 0;
        bool publishPointCloud = m_pointCloudPub->get_subscription_count() > 0;
        bool publishBinaryMap = m_binaryMapPub->get_subscription_count() > 0;
        bool publishFullMap = m_fullMapPub->get_subscription_count() > 0;
        m_publish2DMap = m_mapPub->get_subscription_count() > 0;

        // init markers for free space:
        visualization_msgs::msg::MarkerArray freeNodesVis;
        // each array stores all cubes of a different size, one for each depth level:
        freeNodesVis.markers.resize(m_treeDepth+1);

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, 0.0);        
        geometry_msgs::msg::Pose pose;
        pose.orientation = tf2::toMsg(quaternion);

        // init markers:
        visualization_msgs::msg::MarkerArray occupiedNodesVis;
        // each array stores all cubes of a different size, one for each depth level:
        occupiedNodesVis.markers.resize(m_treeDepth+1);

        // init pointcloud:
        pcl::PointCloud<PCLPoint> pclCloud;

        // call pre-traversal hook:
        handlePreNodeTraversal(rostime);
        
        // now, traverse all leafs in the tree:
        for (auto it = m_octree->begin(m_maxTreeDepth),
                 end = m_octree->end(); it != end; ++it) {
            bool inUpdateBBX = isInUpdateBBX(it);

            // call general hook:
            handleNode(it);
            if (inUpdateBBX) {
                handleNodeInBBX(it);
            }

            if (m_octree->isNodeOccupied(*it)) {
                double z = it.getZ();
                double half_size = it.getSize() / 2.0;
                if (z + half_size > m_occupancyMinZ &&
                    z - half_size < m_occupancyMaxZ) {
                    double size = it.getSize();
                    double x = it.getX();
                    double y = it.getY();
#ifdef COLOR_OCTOMAP_SERVER
                    int r = it->getColor().r;
                    int g = it->getColor().g;
                    int b = it->getColor().b;
#endif

                    // Ignore speckles in the map:
                    if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) &&
                        isSpeckleNode(it.getKey())){
                        RCLCPP_INFO(this->get_logger(),
                            "Ignoring single speckle at (%f,%f,%f)", x, y, z);
                        continue;
                    } // else: current octree node is no speckle, send it out

                    
                    handleOccupiedNode(it);
                    if (inUpdateBBX) {
                        handleOccupiedNodeInBBX(it);
                    }

                    //create marker:
                    if (publishMarkerArray){
                        unsigned idx = it.getDepth();
                        assert(idx < occupiedNodesVis.markers.size());

                        geometry_msgs::msg::Point cubeCenter;
                        cubeCenter.x = x;
                        cubeCenter.y = y;
                        cubeCenter.z = z;

                        occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
                        if (m_useHeightMap){
                            double minX, minY, minZ, maxX, maxY, maxZ;
                            m_octree->getMetricMin(minX, minY, minZ);
                            m_octree->getMetricMax(maxX, maxY, maxZ);

                            double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
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
                        _point.x = x;
                        _point.y = y;
                        _point.z = z;
                        _point.r = r;
                        _point.g = g;
                        _point.b = b;
                        pclCloud.push_back(_point);
#else
                        pclCloud.push_back(PCLPoint(x, y, z));
#endif
                    }

                }
            } else {
                // node not occupied => mark as free in 2D map if unknown so far
                double z = it.getZ();
                double half_size = it.getSize() / 2.0;
                if (z + half_size > m_occupancyMinZ &&
                    z - half_size < m_occupancyMaxZ) {
                    handleFreeNode(it);
                    if (inUpdateBBX) {
                        handleFreeNodeInBBX(it);
                    }

                    if (m_publishFreeSpace) {
                        double x = it.getX();
                        double y = it.getY();

                        //create marker for free space:
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
                }
            }
        }

        // call post-traversal hook:
        handlePostNodeTraversal(rostime);

        // finish MarkerArray:
        if (publishMarkerArray) {
            for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
                double size = m_octree->getNodeSize(i);

                occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
                occupiedNodesVis.markers[i].header.stamp = rostime;
                occupiedNodesVis.markers[i].ns = "map";
                occupiedNodesVis.markers[i].id = i;
                occupiedNodesVis.markers[i].type =
                    visualization_msgs::msg::Marker::CUBE_LIST;
                occupiedNodesVis.markers[i].scale.x = size;
                occupiedNodesVis.markers[i].scale.y = size;
                occupiedNodesVis.markers[i].scale.z = size;
                if (!m_useColoredMap)
                    occupiedNodesVis.markers[i].color = m_color;


                if (occupiedNodesVis.markers[i].points.size() > 0)
                    occupiedNodesVis.markers[i].action =
                        visualization_msgs::msg::Marker::ADD;
                else
                    occupiedNodesVis.markers[i].action =
                        visualization_msgs::msg::Marker::DELETE;
            }
            m_markerPub->publish(occupiedNodesVis);
        }

        // finish FreeMarkerArray:
        if (publishFreeMarkerArray) {
            for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){
                double size = m_octree->getNodeSize(i);

                freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
                freeNodesVis.markers[i].header.stamp = rostime;
                freeNodesVis.markers[i].ns = "map";
                freeNodesVis.markers[i].id = i;
                freeNodesVis.markers[i].type =
                    visualization_msgs::msg::Marker::CUBE_LIST;
                freeNodesVis.markers[i].scale.x = size;
                freeNodesVis.markers[i].scale.y = size;
                freeNodesVis.markers[i].scale.z = size;
                freeNodesVis.markers[i].color = m_colorFree;

                if (freeNodesVis.markers[i].points.size() > 0)
                    freeNodesVis.markers[i].action =
                        visualization_msgs::msg::Marker::ADD;
                else
                    freeNodesVis.markers[i].action =
                        visualization_msgs::msg::Marker::DELETE;
            }
            m_fmarkerPub->publish(freeNodesVis);
        }


        // finish pointcloud:
        if (publishPointCloud) {
            sensor_msgs::msg::PointCloud2 cloud;
            pcl::toROSMsg(pclCloud, cloud);
            cloud.header.frame_id = m_worldFrameId;
            cloud.header.stamp = rostime;
            m_pointCloudPub->publish(cloud);
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

    bool OctomapServer::octomapBinarySrv(
        const std::shared_ptr<OctomapSrv::Request> req,
        std::shared_ptr<OctomapSrv::Response> res) {
        // ros::WallTime startTime = ros::WallTime::now();
        RCLCPP_INFO(this->get_logger(),
                    "Sending binary map data on service request");
        res->map.header.frame_id = m_worldFrameId;
        res->map.header.stamp = this->get_clock()->now();
        if (!octomap_msgs::binaryMapToMsg(*m_octree, res->map)) {
            return false;
        }
        
        /*
        double total_elapsed = (ros::WallTime::now() - startTime).toSec();
        ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
        */
        return true;
    }

    bool OctomapServer::octomapFullSrv(
        const std::shared_ptr<OctomapSrv::Request> req,
        std::shared_ptr<OctomapSrv::Response> res) {
        RCLCPP_INFO(this->get_logger(),
                    "Sending full map data on service request");
        res->map.header.frame_id = m_worldFrameId;
        res->map.header.stamp = this->get_clock()->now();

        if (!octomap_msgs::fullMapToMsg(*m_octree, res->map)) {
            return false;            
        }
        return true;
    }

    bool OctomapServer::clearBBXSrv(
        const std::shared_ptr<BBXSrv::Request> req,
        std::shared_ptr<BBXSrv::Response> resp) {
        octomap::point3d min = octomap::pointMsgToOctomap(req->min);
        octomap::point3d max = octomap::pointMsgToOctomap(req->max);

        double thresMin = m_octree->getClampingThresMin();
        for(auto it = m_octree->begin_leafs_bbx(min,max),
                end=m_octree->end_leafs_bbx(); it!= end; ++it) {
            it->setLogOdds(octomap::logodds(thresMin));
        }
        m_octree->updateInnerOccupancy();

        rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>();
        publishAll(clock->now());

        return true;
    }

    bool OctomapServer::resetSrv(
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> resp) {
        visualization_msgs::msg::MarkerArray occupiedNodesVis;
        occupiedNodesVis.markers.resize(m_treeDepth + 1);
        rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>();
        auto rostime = clock->now();
        
        m_octree->clear();
        // clear 2D map:
        m_gridmap.data.clear();
        m_gridmap.info.height = 0.0;
        m_gridmap.info.width = 0.0;
        m_gridmap.info.resolution = 0.0;
        m_gridmap.info.origin.position.x = 0.0;
        m_gridmap.info.origin.position.y = 0.0;

        RCLCPP_INFO(this->get_logger(), "Cleared octomap");
        publishAll(rostime);

        publishBinaryOctoMap(rostime);
        for (auto i = 0; i < occupiedNodesVis.markers.size(); ++i){
            occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
            occupiedNodesVis.markers[i].header.stamp = rostime;
            occupiedNodesVis.markers[i].ns = "map";
            occupiedNodesVis.markers[i].id = i;
            occupiedNodesVis.markers[i].type =
                visualization_msgs::msg::Marker::CUBE_LIST;
            occupiedNodesVis.markers[i].action =
                visualization_msgs::msg::Marker::DELETE;
        }

        m_markerPub->publish(occupiedNodesVis);

        visualization_msgs::msg::MarkerArray freeNodesVis;
        freeNodesVis.markers.resize(m_treeDepth + 1);

        for (auto i = 0; i < freeNodesVis.markers.size(); ++i) {
            freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
            freeNodesVis.markers[i].header.stamp = rostime;
            freeNodesVis.markers[i].ns = "map";
            freeNodesVis.markers[i].id = i;
            freeNodesVis.markers[i].type =
                visualization_msgs::msg::Marker::CUBE_LIST;
            freeNodesVis.markers[i].action =
                visualization_msgs::msg::Marker::DELETE;
        }
        m_fmarkerPub->publish(freeNodesVis);
        return true;
    }

    void OctomapServer::publishBinaryOctoMap(
        const rclcpp::Time& rostime) const{

        octomap_msgs::msg::Octomap map;
        map.header.frame_id = m_worldFrameId;
        map.header.stamp = rostime;

        if (octomap_msgs::binaryMapToMsg(*m_octree, map)) {
            m_binaryMapPub->publish(map);
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Error serializing OctoMap");
        }
    }

    void OctomapServer::publishFullOctoMap(
        const rclcpp::Time& rostime) const{

        octomap_msgs::msg::Octomap map;
        map.header.frame_id = m_worldFrameId;
        map.header.stamp = rostime;

        if (octomap_msgs::fullMapToMsg(*m_octree, map)) {
            m_fullMapPub->publish(map);
        } else {            
            RCLCPP_ERROR(this->get_logger(),
                         "Error serializing OctoMap");
        }
    }

    void OctomapServer::filterGroundPlane(
        const PCLPointCloud& pc,
        PCLPointCloud& ground,
        PCLPointCloud& nonground) const {
        ground.header = pc.header;
        nonground.header = pc.header;

        if (pc.size() < 50){
            RCLCPP_WARN(this->get_logger(),
                "Pointcloud in OctomapServer too small, skipping ground plane extraction");
            nonground = pc;
        } else {
            // plane detection for ground plane removal:
            pcl::ModelCoefficients::Ptr coefficients(
                new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(
                new pcl::PointIndices);

            // Create the segmentation object and set up:
            pcl::SACSegmentation<PCLPoint> seg;
            seg.setOptimizeCoefficients (true);
            // TODO:
            // maybe a filtering based on the surface normals might be more robust / accurate?
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(200);
            seg.setDistanceThreshold (m_groundFilterDistance);
            seg.setAxis(Eigen::Vector3f(0, 0, 1));
            seg.setEpsAngle(m_groundFilterAngle);

            PCLPointCloud cloud_filtered(pc);
            // Create the filtering object
            pcl::ExtractIndices<PCLPoint> extract;
            bool groundPlaneFound = false;

            while (cloud_filtered.size() > 10 && !groundPlaneFound) {
                seg.setInputCloud(cloud_filtered.makeShared());
                seg.segment (*inliers, *coefficients);
                if (inliers->indices.size () == 0) {
                    RCLCPP_INFO(this->get_logger(),
                                "PCL segmentation did not find any plane.");                    
                    break;
                }

                extract.setInputCloud(cloud_filtered.makeShared());
                extract.setIndices(inliers);

                if (std::abs(coefficients->values.at(3)) < m_groundFilterPlaneDistance) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f",
                        inliers->indices.size(), cloud_filtered.size(),
                        coefficients->values.at(0), coefficients->values.at(1),
                        coefficients->values.at(2), coefficients->values.at(3));
                    extract.setNegative(false);
                    extract.filter(ground);

                    // remove ground points from full pointcloud:
                    // workaround for PCL bug:
                    if(inliers->indices.size() != cloud_filtered.size()) {
                        extract.setNegative(true);
                        PCLPointCloud cloud_out;
                        extract.filter(cloud_out);
                        nonground += cloud_out;
                        cloud_filtered = cloud_out;
                    }
                    groundPlaneFound = true;
                } else {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f",
                                inliers->indices.size(), cloud_filtered.size(),
                              coefficients->values.at(0), coefficients->values.at(1),
                                coefficients->values.at(2), coefficients->values.at(3));
                    pcl::PointCloud<PCLPoint> cloud_out;
                    extract.setNegative (false);
                    extract.filter(cloud_out);
                    nonground +=cloud_out;
   
                    // remove current plane from scan for next iteration:
                    // workaround for PCL bug:
                    if(inliers->indices.size() != cloud_filtered.size()){
                        extract.setNegative(true);
                        cloud_out.points.clear();
                        extract.filter(cloud_out);
                        cloud_filtered = cloud_out;
                    } else{
                        cloud_filtered.points.clear();
                    }
                }
            }
            // TODO: also do this if overall starting pointcloud too small?
            if (!groundPlaneFound){ // no plane found or remaining points too small
                RCLCPP_WARN(this->get_logger(),
                            "No ground plane found in scan");

                // do a rough fitlering on height to prevent spurious obstacles
                pcl::PassThrough<PCLPoint> second_pass;
                second_pass.setFilterFieldName("z");
                second_pass.setFilterLimits(-m_groundFilterPlaneDistance,
                                            m_groundFilterPlaneDistance);
                second_pass.setInputCloud(pc.makeShared());
                second_pass.filter(ground);

                second_pass.setNegative(true);
                second_pass.filter(nonground);
            }
        }
    }

    void OctomapServer::handlePreNodeTraversal(
        const rclcpp::Time& rostime) {
        if (m_publish2DMap){
            // init projected 2D map:
            m_gridmap.header.frame_id = m_worldFrameId;
            m_gridmap.header.stamp = rostime;
            nav_msgs::msg::MapMetaData oldMapInfo = m_gridmap.info;

            // TODO:
            // move most of this stuff into c'tor and init map only once(adjust if size changes)
            double minX, minY, minZ, maxX, maxY, maxZ;
            m_octree->getMetricMin(minX, minY, minZ);
            m_octree->getMetricMax(maxX, maxY, maxZ);

            octomap::point3d minPt(minX, minY, minZ);
            octomap::point3d maxPt(maxX, maxY, maxZ);
            octomap::OcTreeKey minKey = m_octree->coordToKey(minPt, m_maxTreeDepth);
            octomap::OcTreeKey maxKey = m_octree->coordToKey(maxPt, m_maxTreeDepth);

            RCLCPP_INFO(
                this->get_logger(),
                "MinKey: %d %d %d / MaxKey: %d %d %d",
                minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

            // add padding if requested (= new min/maxPts in x&y):
            double halfPaddedX = 0.5*m_minSizeX;
            double halfPaddedY = 0.5*m_minSizeY;
            minX = std::min(minX, -halfPaddedX);
            maxX = std::max(maxX, halfPaddedX);
            minY = std::min(minY, -halfPaddedY);
            maxY = std::max(maxY, halfPaddedY);
            minPt = octomap::point3d(minX, minY, minZ);
            maxPt = octomap::point3d(maxX, maxY, maxZ);

            octomap::OcTreeKey paddedMaxKey;
            if (!m_octree->coordToKeyChecked(
                    minPt, m_maxTreeDepth, m_paddedMinKey)) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Could not create padded min OcTree key at %f %f %f",
                    minPt.x(), minPt.y(), minPt.z());
                return;
            }
            if (!m_octree->coordToKeyChecked(
                    maxPt, m_maxTreeDepth, paddedMaxKey)) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Could not create padded max OcTree key at %f %f %f",
                    maxPt.x(), maxPt.y(), maxPt.z());
                return;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Padded MinKey: %d %d %d / padded MaxKey: %d %d %d",
                m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2],
                paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
            assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);
            
            m_multires2DScale = 1 << (m_treeDepth - m_maxTreeDepth);
            m_gridmap.info.width = (paddedMaxKey[0] - m_paddedMinKey[0]) /
                m_multires2DScale + 1;
            m_gridmap.info.height = (paddedMaxKey[1] - m_paddedMinKey[1]) /
                m_multires2DScale + 1;

            int mapOriginX = minKey[0] - m_paddedMinKey[0];
            int mapOriginY = minKey[1] - m_paddedMinKey[1];
            assert(mapOriginX >= 0 && mapOriginY >= 0);

            // might not exactly be min / max of octree:
            octomap::point3d origin = m_octree->keyToCoord(m_paddedMinKey, m_treeDepth);
            double gridRes = m_octree->getNodeSize(m_maxTreeDepth);
            m_projectCompleteMap = (!m_incrementalUpdate ||
                                    (std::abs(gridRes-m_gridmap.info.resolution) > 1e-6));
            
            m_gridmap.info.resolution = gridRes;
            m_gridmap.info.origin.position.x = origin.x() - gridRes*0.5;
            m_gridmap.info.origin.position.y = origin.y() - gridRes*0.5;
            if (m_maxTreeDepth != m_treeDepth) {
                m_gridmap.info.origin.position.x -= m_res/2.0;
                m_gridmap.info.origin.position.y -= m_res/2.0;
            }

            // workaround for  multires. projection not working properly for inner nodes:
            // force re-building complete map
            if (m_maxTreeDepth < m_treeDepth) {
                m_projectCompleteMap = true;
            }

            if(m_projectCompleteMap) {
                RCLCPP_INFO(this->get_logger(), "Rebuilding complete 2D map");
                m_gridmap.data.clear();
                // init to unknown:
                m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);

            } else {
                if (mapChanged(oldMapInfo, m_gridmap.info)){
                    RCLCPP_INFO(
                        this->get_logger(),
                        "2D grid map size changed to %dx%d",
                        m_gridmap.info.width, m_gridmap.info.height);
                    adjustMapData(m_gridmap, oldMapInfo);
                }
                nav_msgs::msg::OccupancyGrid::_data_type::iterator startIt;
                auto mapUpdateBBXMinX = std::max(
                    0, (int(m_updateBBXMin[0]) - int(m_paddedMinKey[0])) /
                    int(m_multires2DScale));
                auto mapUpdateBBXMinY = std::max(
                    0, (int(m_updateBBXMin[1]) - int(m_paddedMinKey[1])) /
                    int(m_multires2DScale));
                auto mapUpdateBBXMaxX = std::min(
                    int(m_gridmap.info.width-1),
                    (int(m_updateBBXMax[0]) - int(m_paddedMinKey[0])) /
                    int(m_multires2DScale));
                auto mapUpdateBBXMaxY = std::min(
                    int(m_gridmap.info.height-1),
                    (int(m_updateBBXMax[1]) - int(m_paddedMinKey[1])) /
                    int(m_multires2DScale));

                assert(mapUpdateBBXMaxX > mapUpdateBBXMinX);
                assert(mapUpdateBBXMaxY > mapUpdateBBXMinY);

                auto numCols = mapUpdateBBXMaxX-mapUpdateBBXMinX +1;

                // test for max idx:
                auto max_idx = m_gridmap.info.width*mapUpdateBBXMaxY + mapUpdateBBXMaxX;
                if (max_idx  >= m_gridmap.data.size()) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        std::string("BBX index not valid:").c_str(),
                        "%d (max index %zu for size %d x %d) update-BBX is: ",
                        "[%zu %zu]-[%zu %zu]",
                        max_idx, m_gridmap.data.size(), m_gridmap.info.width,
                        m_gridmap.info.height, mapUpdateBBXMinX,
                        mapUpdateBBXMinY, mapUpdateBBXMaxX, mapUpdateBBXMaxY);
                }

                // reset proj. 2D map in bounding box:
                for (unsigned int j = mapUpdateBBXMinY; j <= mapUpdateBBXMaxY; ++j) {
                    std::fill_n(
                        m_gridmap.data.begin() + m_gridmap.info.width*j+mapUpdateBBXMinX,
                        numCols, -1);
                }
            }
        }
    }

    void OctomapServer::handlePostNodeTraversal(
        const rclcpp::Time& rostime){
        if (m_publish2DMap) {
            m_mapPub->publish(m_gridmap);
        }
    }

    void OctomapServer::handleOccupiedNode(
        const OcTreeT::iterator& it){
        if (m_publish2DMap && m_projectCompleteMap){
            update2DMap(it, true);
        }
    }

    void OctomapServer::handleFreeNode(
        const OcTreeT::iterator& it){
        if (m_publish2DMap && m_projectCompleteMap){
            update2DMap(it, false);
        }
    }

    void OctomapServer::handleOccupiedNodeInBBX(
        const OcTreeT::iterator& it){
        if (m_publish2DMap && !m_projectCompleteMap){
            update2DMap(it, true);
        }
    }

    void OctomapServer::handleFreeNodeInBBX(
        const OcTreeT::iterator& it){
        if (m_publish2DMap && !m_projectCompleteMap){
            update2DMap(it, false);
        }
    }

    void OctomapServer::update2DMap(
        const OcTreeT::iterator& it, bool occupied) {
        if (it.getDepth() == m_maxTreeDepth){
            auto idx = mapIdx(it.getKey());
            if (occupied) {
                m_gridmap.data[mapIdx(it.getKey())] = 100;
            } else if (m_gridmap.data[idx] == -1){
                m_gridmap.data[idx] = 0;
            }
        } else {
            int intSize = 1 << (m_maxTreeDepth - it.getDepth());
            octomap::OcTreeKey minKey=it.getIndexKey();
            for(int dx = 0; dx < intSize; dx++) {
                int i = (minKey[0] + dx - m_paddedMinKey[0]) / m_multires2DScale;
                for(int dy = 0; dy < intSize; dy++){
                    auto idx = mapIdx(i, (minKey[1] + dy - m_paddedMinKey[1]) /
                                      m_multires2DScale);
                    if (occupied) {
                        m_gridmap.data[idx] = 100;
                    } else if (m_gridmap.data[idx] == -1) {
                        m_gridmap.data[idx] = 0;
                    }
                }
            }
        }
    }

    bool OctomapServer::isSpeckleNode(
        const octomap::OcTreeKey &nKey) const {
        octomap::OcTreeKey key;
        bool neighborFound = false;
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

    void OctomapServer::adjustMapData(
        nav_msgs::msg::OccupancyGrid &map,
        const nav_msgs::msg::MapMetaData &oldMapInfo) const {
        if (map.info.resolution != oldMapInfo.resolution) {
            RCLCPP_ERROR(this->get_logger(),
                "Resolution of map changed, cannot be adjusted");
            return;
        }

        int i_off = int(
            (oldMapInfo.origin.position.x - map.info.origin.position.x) /
            map.info.resolution + 0.5);
        int j_off = int(
            (oldMapInfo.origin.position.y - map.info.origin.position.y) /
            map.info.resolution + 0.5);

        if (i_off < 0 || j_off < 0
            || oldMapInfo.width  + i_off > map.info.width
            || oldMapInfo.height + j_off > map.info.height) {
            RCLCPP_ERROR(
                this->get_logger(),
                "New 2D map does not contain old map area, this case is not implemented");
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
            fromStart = oldMapData.begin() + j*oldMapInfo.width;
            fromEnd = fromStart + oldMapInfo.width;
            toStart = map.data.begin() + ((j+j_off)*m_gridmap.info.width + i_off);
            copy(fromStart, fromEnd, toStart);
        }
    }

    std_msgs::msg::ColorRGBA OctomapServer::heightMapColor(double h) {
        std_msgs::msg::ColorRGBA color;
        color.a = 1.0;
        // blend over HSV-values (more colors)

        double s = 1.0;
        double v = 1.0;

        h -= floor(h);
        h *= 6;
        int i;
        double m, n, f;

        i = floor(h);
        f = h - i;
        if (!(i & 1))
            f = 1 - f; // if i is even
        m = v * (1 - s);
        n = v * (1 - s * f);

        switch (i) {
        case 6:
        case 0:
            color.r = v; color.g = n; color.b = m;
            break;
        case 1:
            color.r = n; color.g = v; color.b = m;
            break;
        case 2:
            color.r = m; color.g = v; color.b = n;
            break;
        case 3:
            color.r = m; color.g = n; color.b = v;
            break;
        case 4:
            color.r = n; color.g = m; color.b = v;
            break;
        case 5:
            color.r = v; color.g = m; color.b = n;
            break;
        default:
            color.r = 1; color.g = 0.5; color.b = 0.5;
            break;
        }
        return color;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::OctomapServer)
