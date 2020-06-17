/**
 * OctoMap ROS integration
 *
 * @author A. Hornung, University of Freiburg, Copyright (C) 2011-2012.
 * @see http://www.ros.org/wiki/octomap_ros
 * License: BSD
 */

/*
 * Copyright (c) 2011, A. Hornung, University of Freiburg
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

#ifndef OCTOMAP_ROS_CONVERSIONS_H
#define OCTOMAP_ROS_CONVERSIONS_H

#include <octomap/octomap.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace octomap {
    /**
     * @brief Conversion from octomap::point3d_list
     (e.g. all occupied nodes from getOccupied()) to
     * sensor_msgs::msg::PointCloud2
     *
     * @param points
     * @param cloud
     */
    void pointsOctomapToPointCloud2(const point3d_list& points,
                                    sensor_msgs::msg::PointCloud2& cloud);

    /**
     * @brief Conversion from a sensor_msgs::msg::PointCLoud2 to
     octomap::Pointcloud, used internally in OctoMap     
     *
     * @param cloud
     * @param octomapCloud
     */

    void pointCloud2ToOctomap(const sensor_msgs::msg::PointCloud2& cloud,
                              Pointcloud& octomapCloud);

    /// Conversion from octomap::point3d to geometry_msgs::msg::Point
    static inline geometry_msgs::msg::Point pointOctomapToMsg(
        const point3d& octomapPt){
        geometry_msgs::msg::Point pt;
        pt.x = octomapPt.x();
        pt.y = octomapPt.y();
        pt.z = octomapPt.z();

        return pt;
    }

    /// Conversion from geometry_msgs::msg::Point to octomap::point3d
    static inline octomap::point3d pointMsgToOctomap(
        const geometry_msgs::msg::Point& ptMsg){
        return octomap::point3d(ptMsg.x, ptMsg.y, ptMsg.z);
    }

    /// Conversion from octomap::point3d to tf2::Point
    static inline geometry_msgs::msg::Point pointOctomapToTf(
        const point3d &octomapPt) {
        geometry_msgs::msg::Point pt;
        pt.x = octomapPt.x();
        pt.y = octomapPt.y();
        pt.z = octomapPt.z();
        return pt;        
    }

    /// Conversion from tf2::Point to octomap::point3d
    static inline octomap::point3d pointTfToOctomap(
        const geometry_msgs::msg::Point& ptTf){
        return point3d(ptTf.x, ptTf.y, ptTf.z);
    }    
    
    static inline octomap::point3d pointTfToOctomap(
        const geometry_msgs::msg::Vector3& ptTf){
        return point3d(ptTf.x, ptTf.y, ptTf.z);
    }
    
    /// Conversion from octomap Quaternion to tf2::Quaternion
    static inline tf2::Quaternion quaternionOctomapToTf(
        const octomath::Quaternion& octomapQ){
        return tf2::Quaternion(
            octomapQ.x(), octomapQ.y(), octomapQ.z(), octomapQ.u());
    }
    
    /// Conversion from tf2::Quaternion to octomap Quaternion
    static inline octomath::Quaternion quaternionTfToOctomap(
        const tf2::Quaternion& qTf){
        return octomath::Quaternion(qTf.w(), qTf.x(), qTf.y(), qTf.z());
    }

    static inline octomath::Quaternion quaternionTfToOctomap(
        const geometry_msgs::msg::Quaternion &qTf){
        return octomath::Quaternion(qTf.w, qTf.x, qTf.y, qTf.z);
    }
    
    /// Conversion from octomap::pose6f to tf2::Pose
    static inline geometry_msgs::msg::Pose poseOctomapToTf(
        const octomap::pose6d& octomapPose){
        auto r = quaternionOctomapToTf(octomapPose.rot());
        geometry_msgs::msg::Quaternion orientation;
        orientation.x = r.x();
        orientation.y = r.y();
        orientation.z = r.z();
        orientation.w = r.w();
        
        geometry_msgs::msg::Pose pose;
        pose.position = pointOctomapToTf(octomapPose.trans());        
        pose.orientation = orientation;        
        return pose;     
    }
    
    /// Conversion from tf2::Pose to octomap::pose6d
    static inline octomap::pose6d poseTfToOctomap(
        const geometry_msgs::msg::Pose& poseTf) {
        return octomap::pose6d(pointTfToOctomap(poseTf.position),
                               quaternionTfToOctomap(poseTf.orientation));
    }
}

#endif
