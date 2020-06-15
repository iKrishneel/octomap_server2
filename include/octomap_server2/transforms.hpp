
#pragma once
#ifndef pcl_ROS_TRANSFORMS_H_
#define pcl_ROS_TRANSFORMS_H_

#include <eigen3/Eigen/Eigen>

#include <tf2/buffer_core.h>
#include <tf2/transform_datatypes.h>


namespace pcl_ros {
    
  /** \brief Obtain the transformation matrix from TF into an Eigen form
    * \param bt the TF transformation
    * \param out_mat the Eigen transformation
    */
  void transformAsMatrix(const tf2::Transform&, Eigen::Matrix4f &);
}

#endif // PCL_ROS_TRANSFORMS_H_

