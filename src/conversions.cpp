// $Id:  $

/**
 * OctoMap ROS integration
 *
 * @author A. Hornung, University of Freiburg, Copyright (C) 2011.
 * @see http://www.ros.org/wiki/octomap_ros
 * License: BSD
 */

/*
 * Copyright (c) 2010, A. Hornung, University of Freiburg
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

#include <octomap_server2/conversions.h>

namespace octomap {

    /**
     * @brief Conversion from octomap::point3d_list
     (e.g. all occupied nodes from getOccupied()) to
     * sensor_msgs::PointCloud2
     *
     * @param points
     * @param cloud
     */

    void pointsOctomapToPointCloud2(const point3d_list& points,
                                    sensor_msgs::msg::PointCloud2& cloud) {
        // make sure the channel is valid

        /*
        std::vector<sensor_msgs::PointField>::const_iterator field_iter =
            cloud.fields.begin(),
            field_end = cloud.fields.end();
        */

        auto field_iter = cloud.fields.begin();
        auto field_end = cloud.fields.end();
        
        bool has_x, has_y, has_z;
        has_x = has_y = has_z = false;
        while (field_iter != field_end) {
            if ((field_iter->name == "x") || (field_iter->name == "X"))
                has_x = true;
            if ((field_iter->name == "y") || (field_iter->name == "Y"))
                has_y = true;
            if ((field_iter->name == "z") || (field_iter->name == "Z"))
                has_z = true;
            ++field_iter;
        }

        if ((!has_x) || (!has_y) || (!has_z))
            throw std::runtime_error("One of the fields xyz does not exist");

        /*
        sensor_msgs::msg::PointCloud2Modifier pcd_modifier(cloud);
        pcd_modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (point3d_list::const_iterator it = points.begin(); it != points.end();
             ++it, ++iter_x, ++iter_y, ++iter_z) {
            *iter_x = it->x();
            *iter_y = it->y();
            *iter_z = it->z();
        }
        */
    }

    /**
     * @brief Conversion from a sensor_msgs::PointCLoud2 to
     * octomap::Pointcloud,
     used internally in OctoMap
     *
     * @param cloud
     * @param octomapCloud
     */

    void pointCloud2ToOctomap(const sensor_msgs::msg::PointCloud2 &cloud,
                              Pointcloud &octomapCloud){
        octomapCloud.reserve(cloud.data.size() / cloud.point_step);

        /*
          sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
          sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
          sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

          for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
          // Check if the point is invalid
          if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
          octomapCloud.push_back(*iter_x, *iter_y, *iter_z);
          }
        */
    }
}


