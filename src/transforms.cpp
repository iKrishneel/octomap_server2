
#include <octomap_server2/transforms.hpp>

namespace pcl_ros {

    void transformAsMatrix(const tf2::Transform &bt,
                           Eigen::Matrix4f &out_mat) {
        double mv[12];
        bt.getBasis ().getOpenGLSubMatrix (mv);

        tf2::Vector3 origin = bt.getOrigin ();

        out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
        out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
        out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];
                                                                     
        out_mat (3, 0) = out_mat(3, 1) = out_mat(3, 2) = 0;
        out_mat(3, 3) = 1;
        out_mat (0, 3) = origin.x();
        out_mat (1, 3) = origin.y();
        out_mat (2, 3) = origin.z();
    }
} // namespace pcl_ros
