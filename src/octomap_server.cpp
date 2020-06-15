
#include <octomap_server2/octomap_server.hpp>

namespace octomap_server {
    OctomapServer::OctomapServer(
        const rclcpp::NodeOptions &options,
        const std::string node_name):
        Node(node_name, options),
        // m_pointCloudSub(NULL),
        // m_tfPointCloudSub(NULL),
        // m_reconfigureServer(m_config_mutex, private_nh_),
        m_octree(NULL),
        m_maxRange(-1.0),
        m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
        m_useHeightMap(true),
        m_useColoredMap(false),
        m_colorFactor(0.8),
        m_latchedTopics(true),
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
        m_minSizeX(0.0), m_minSizeY(0.0),
        m_filterSpeckles(false), m_filterGroundPlane(false),
        m_groundFilterDistance(0.04),
        m_groundFilterAngle(0.15),
        m_groundFilterPlaneDistance(0.07),
        m_compressMap(true),
        m_incrementalUpdate(false),
        m_initConfig(true) {
        
        
    }

    OctomapServer::~OctomapServer() {
        /*
        if (m_tfPointCloudSub){
            delete m_tfPointCloudSub;
            m_tfPointCloudSub = NULL;
        }

        if (m_pointCloudSub){
            delete m_pointCloudSub;
            m_pointCloudSub = NULL;
        }
        */

        if (m_octree){
            delete m_octree;
            m_octree = NULL;
        }
    }

    bool OctomapServer::openFile(const std::string& filename){
        if (filename.length() <= 3)
            return false;

        std::string suffix = filename.substr(filename.length()-3, 3);
        if (suffix== ".bt"){
            if (!m_octree->readBinary(filename)){
                return false;
            }
        } else if (suffix == ".ot"){
            octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(filename);
            if (!tree){
                return false;
            }
            if (m_octree){
                delete m_octree;
                m_octree = NULL;
            }
            m_octree = dynamic_cast<OcTreeT*>(tree);
            if (!m_octree){
                std::string msg = "Could not read OcTree in file";
                RCLCPP_ERROR(this->get_logger(), msg.c_str());
                return false;
            }
        } else{
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

        //!!!!! publishAll();
        return true;
    }

    void OctomapServer::insertCloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr &cloud){
        // ros::WallTime startTime = ros::WallTime::now();
        
        //
        // ground filtering in base frame
        //
        PCLPointCloud pc; // input cloud for filtering and ground-detection
        pcl::fromROSMsg(*cloud, pc);

        /*
        tf::StampedTransform sensorToWorldTf;
        try {
            m_tfListener.lookupTransform(
                m_worldFrameId, cloud->header.frame_id,
                cloud->header.stamp, sensorToWorldTf);
        } catch(tf::TransformException& ex){
            ROS_ERROR_STREAM("Transform error of sensor data: " <<
                             ex.what() << ", quitting callback");
            return;
        }
        */
        
        Eigen::Matrix4f sensorToWorld;
        /*
        pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
        */

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

        if (m_filterGroundPlane){
            /*
            tf::StampedTransform sensorToBaseTf, baseToWorldTf;
            try{
                m_tfListener.waitForTransform(
                    m_baseFrameId, cloud->header.frame_id,
                    cloud->header.stamp, ros::Duration(0.2));
                m_tfListener.lookupTransform(
                    m_baseFrameId, cloud->header.frame_id,
                    cloud->header.stamp, sensorToBaseTf);
                m_tfListener.lookupTransform(
                    m_worldFrameId, m_baseFrameId,
                    cloud->header.stamp, baseToWorldTf);
            } catch(tf::TransformException& ex){
                RCLCPP_ERROR(this->get_logger(),
                             "Transform error for ground plane filter: "
                             << ex.what() << ", quitting callback.\n"
                             "You need to set the base_frame_id or disable filter_ground.");
            }
            */

            Eigen::Matrix4f sensorToBase, baseToWorld;
            /*
            pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
            pcl_ros::transformAsMatrix(baseToWorldTf, baseToWorld);
            */
            

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

        //!!!! insertScan(sensorToWorldTf.getOrigin(), pc_ground, pc_nonground);

        /*
        double total_elapsed = 0;  //(ros::WallTime::now() - startTime).toSec();
        RCLCPP_INFO(this->get_logger(),
                    "Pointcloud insertion in OctomapServer done (%zu+%zu pts (ground/nonground), %f sec)",
                    pc_ground.size(), pc_nonground.size(),
        total_elapsed);
        */

        //!!!!!!!!! publishAll(cloud->header.stamp);
    }

    void OctomapServer::insertScan(
        const geometry_msgs::msg::Point &sensorOriginTf,
        const PCLPointCloud& ground,
        const PCLPointCloud& nonground){
        octomap::point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);
        
        if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
            || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax)) {
            // ROS_ERROR_STREAM("Could not generate Key for origin "<< sensorOrigin);
        }

#ifdef COLOR_OCTOMAP_SERVER
        unsigned char* colors = new unsigned char[3];
#endif
        
        // instead of direct scan insertion, compute update to filter ground:
        KeySet free_cells, occupied_cells;
        // insert ground points only as free:
        for (PCLPointCloud::const_iterator it = ground.begin();
             it != ground.end(); ++it){
            octomap::point3d point(it->x, it->y, it->z);
            // maxrange check
            if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange) ) {
                point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            }

            // only clear space (ground points)
            if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }

            octomap::OcTreeKey endKey;
            if (m_octree->coordToKeyChecked(point, endKey)){
                updateMinKey(endKey, m_updateBBXMin);
                updateMaxKey(endKey, m_updateBBXMax);
            } else{
                // ROS_ERROR_STREAM("Could not generate Key for endpoint "<<point);
            }
        }

        // all other points: free on ray, occupied on endpoint:
        for (PCLPointCloud::const_iterator it = nonground.begin();
             it != nonground.end(); ++it){
            octomap::point3d point(it->x, it->y, it->z);
            // maxrange check
            if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {
                
                // free cells
                if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                }
                // occupied endpoint
                OcTreeKey key;
                if (m_octree->coordToKeyChecked(point, key)){
                    occupied_cells.insert(key);

                    updateMinKey(key, m_updateBBXMin);
                    updateMaxKey(key, m_updateBBXMax);

#ifdef COLOR_OCTOMAP_SERVER // NB: Only read and interpret color if it's an occupied node
                    m_octree->averageNodeColor(
                        it->x,
                        it->y,
                        it->z,
                        it->r,
                        it->g,
                        it->b);
#endif
                }
            } else {
                // ray longer than maxrange:;
                octomap::point3d new_end = sensorOrigin +
                    (point - sensorOrigin).normalized() * m_maxRange;
                if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());

                    octomap::OcTreeKey endKey;
                    if (m_octree->coordToKeyChecked(new_end, endKey)){
                        free_cells.insert(endKey);
                        updateMinKey(endKey, m_updateBBXMin);
                        updateMaxKey(endKey, m_updateBBXMax);
                    } else{
                        // ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
                    }
                }
            }
        }

        // mark free cells only if not seen occupied in this cloud
        for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
            if (occupied_cells.find(*it) == occupied_cells.end()){
                m_octree->updateNode(*it, false);
            }
        }

        // now mark all occupied cells:
        for (KeySet::iterator it = occupied_cells.begin(),
                 end=occupied_cells.end(); it!= end; it++) {
            m_octree->updateNode(*it, true);
        }

        // TODO: eval lazy+updateInner vs. proper insertion
        // non-lazy by default (updateInnerOccupancy() too slow for large maps)
        //m_octree->updateInnerOccupancy();
        octomap::point3d minPt, maxPt;
        /*
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
        /*
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
}

RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::OctomapServer)
