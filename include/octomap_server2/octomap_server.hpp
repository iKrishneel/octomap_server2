
#pragma once
#ifndef _OCTOMAP_SERVER_HPP_
#define _OCTOMAP_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <octomap_server2/transforms.hpp>
#include <octomap_server2/conversions.h>

#ifdef COLOR_OCTOMAP_SERVER
#include <octomap/ColorOcTree.h>
#endif

#ifdef COLOR_OCTOMAP_SERVER
using PCLPoint = pcl::PointXYZRGB;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTreeT = octomap::ColorOcTree;
#else
using PCLPoint = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTreeT = octomap::OcTree;
#endif

using OctomapSrv =  octomap_msgs::srv::GetOctomap;
using BBXSrv =  octomap_msgs::srv::BoundingBoxQuery;

namespace octomap_server {
    class OctomapServer: public rclcpp::Node {
    protected:

        static std_msgs::msg::ColorRGBA heightMapColor(double h);
        /*
        ros::NodeHandle m_nh;
        ros::NodeHandle m_nh_private;
        ros::Publisher  m_markerPub,
            m_binaryMapPub,
            m_fullMapPub,
            m_pointCloudPub,
            m_collisionObjectPub,
            m_mapPub,
            m_cmapPub,
            m_fmapPub,
            m_fmarkerPub;

        message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
        tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;
        ros::ServiceServer m_octomapBinaryService,
            m_octomapFullService,
            m_clearBBXService,
            m_resetService;
        tf::TransformListener m_tfListener;
        boost::recursive_mutex m_config_mutex;
        dynamic_reconfigure::Server<OctomapServerConfig>
        m_reconfigureServer;
        */

        OcTreeT* m_octree;
        octomap::KeyRay m_keyRay;  // temp storage for ray casting
        octomap::OcTreeKey m_updateBBXMin;
        octomap::OcTreeKey m_updateBBXMax;

        double m_maxRange;
        std::string m_worldFrameId; // the map frame
        std::string m_baseFrameId; // base of the robot for ground plane filtering
        bool m_useHeightMap;
        std_msgs::msg::ColorRGBA m_color;
        std_msgs::msg::ColorRGBA m_colorFree;
        double m_colorFactor;

        bool m_latchedTopics;
        bool m_publishFreeSpace;

        double m_res;
        unsigned m_treeDepth;
        unsigned m_maxTreeDepth;

        double m_pointcloudMinX;
        double m_pointcloudMaxX;
        double m_pointcloudMinY;
        double m_pointcloudMaxY;
        double m_pointcloudMinZ;
        double m_pointcloudMaxZ;
        double m_occupancyMinZ;
        double m_occupancyMaxZ;
        double m_minSizeX;
        double m_minSizeY;
        bool m_filterSpeckles;

        bool m_filterGroundPlane;
        double m_groundFilterDistance;
        double m_groundFilterAngle;
        double m_groundFilterPlaneDistance;

        bool m_compressMap;

        bool m_initConfig;

        // downprojected 2D map:
        bool m_incrementalUpdate;
        nav_msgs::msg::OccupancyGrid m_gridmap;
        bool m_publish2DMap;
        bool m_mapOriginChanged;
        octomap::OcTreeKey m_paddedMinKey;
        unsigned m_multires2DScale;
        bool m_projectCompleteMap;
        bool m_useColoredMap;
        
        inline static void updateMinKey(const octomap::OcTreeKey& in,
                                        octomap::OcTreeKey& min) {
            for (unsigned i = 0; i < 3; ++i)
                min[i] = std::min(in[i], min[i]);
        };

        inline static void updateMaxKey(const octomap::OcTreeKey& in,
                                        octomap::OcTreeKey& max) {
            for (unsigned i = 0; i < 3; ++i)
                max[i] = std::max(in[i], max[i]);
        };
        
        /// Test if key is within update area of map (2D, ignores height)
        inline bool isInUpdateBBX(const OcTreeT::iterator& it) const {
            // 2^(tree_depth-depth) voxels wide:
            unsigned voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
            octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
            return (key[0] + voxelWidth >= m_updateBBXMin[0]
                    && key[1] + voxelWidth >= m_updateBBXMin[1]
                    && key[0] <= m_updateBBXMax[0]
                    && key[1] <= m_updateBBXMax[1]);
        }
        
        inline unsigned mapIdx(int i, int j) const {
            return m_gridmap.info.width * j + i;
        }
        
        inline unsigned mapIdx(const octomap::OcTreeKey& key) const {
            return mapIdx((key[0] - m_paddedMinKey[0]) / m_multires2DScale,
                          (key[1] - m_paddedMinKey[1]) / m_multires2DScale);

        }

        inline bool mapChanged(const nav_msgs::msg::MapMetaData& oldMapInfo,
                               const nav_msgs::msg::MapMetaData& newMapInfo) {
            return (oldMapInfo.height != newMapInfo.height
                    || oldMapInfo.width != newMapInfo.width
                    || oldMapInfo.origin.position.x != newMapInfo.origin.position.x
                    || oldMapInfo.origin.position.y != newMapInfo.origin.position.y);
        }

        /*
        void publishBinaryOctoMap(const ros::Time& rostime = ros::Time::now()) const;
        void publishFullOctoMap(const ros::Time& rostime = ros::Time::now()) const;
        virtual void publishAll(const ros::Time& rostime =
        ros::Time::now());

        */
        
        virtual void insertScan(
            const geometry_msgs::msg::Point& sensorOrigin,
            const PCLPointCloud& ground,
            const PCLPointCloud& nonground);


        void filterGroundPlane(const PCLPointCloud& pc,
                               PCLPointCloud& ground,
                               PCLPointCloud& nonground) const;

        bool isSpeckleNode(const octomap::OcTreeKey& key) const;

        // virtual void handlePreNodeTraversal(const ros::Time& rostime);
        // virtual void handlePostNodeTraversal(const ros::Time& rostime);
        virtual void handleNode(const OcTreeT::iterator& it) {};
        virtual void handleNodeInBBX(const OcTreeT::iterator& it) {};
        virtual void handleOccupiedNode(const OcTreeT::iterator& it);
        virtual void handleOccupiedNodeInBBX(const OcTreeT::iterator& it);
        virtual void handleFreeNode(const OcTreeT::iterator& it);
        virtual void handleFreeNodeInBBX(const OcTreeT::iterator& it);

        virtual void update2DMap(const OcTreeT::iterator& it, bool
        occupied);
        
        void adjustMapData(nav_msgs::msg::OccupancyGrid& map,
                           const nav_msgs::msg::MapMetaData& oldMapInfo) const;
        
    public:        
        explicit OctomapServer(
            const rclcpp::NodeOptions &,
            const std::string = "octomap_server");
        virtual ~OctomapServer();        
        virtual bool octomapBinarySrv(OctomapSrv::Request &req,
                                      OctomapSrv::GetOctomap::Response &res);
        virtual bool octomapFullSrv(OctomapSrv::Request &req,
                                    OctomapSrv::GetOctomap::Response &res);
        bool clearBBXSrv(BBXSrv::Request &req,
                         BBXSrv::Response &resp);
        bool resetSrv(std_srvs::srv::Empty::Request &req,
                      std_srvs::srv::Empty::Response &resp);

        virtual void insertCloudCallback(
            const sensor_msgs::msg::PointCloud2::SharedPtr &);
        virtual bool openFile(const std::string& filename);

    };    
} // octomap_server

#endif  // _OCTOMAP_SERVER_HPP_
