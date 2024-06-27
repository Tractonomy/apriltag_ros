#include "pose_estimation.hpp"
#include <Eigen/Geometry>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/homography.h>
#include <opencv2/calib3d.hpp>
#include <tf2/convert.h>


geometry_msgs::msg::Transform
homography(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize, bool z_up)
{
    apriltag_detection_info_t info = {detection, tagsize, intr[0], intr[1], intr[2], intr[3]};

    apriltag_pose_t pose;
    estimate_pose_for_tag_homography(&info, &pose);

    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(pose.R->data);

    if (z_up) {
        // rotate by half rotation about x-axis to have z-axis
        // point upwards orthogonal to the tag plane
        R.col(1) *= -1;
        R.col(2) *= -1;
    }

    return tf2::toMsg<apriltag_pose_t, geometry_msgs::msg::Transform>(const_cast<const apriltag_pose_t&>(pose));
}

geometry_msgs::msg::Transform
pnp(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize, bool z_up)
{
    const std::vector<cv::Point3d> objectPoints{
        {-tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, +tagsize / 2, 0},
        {-tagsize / 2, +tagsize / 2, 0},
    };

    const std::vector<cv::Point2d> imagePoints{
        {detection->p[0][0], detection->p[0][1]},
        {detection->p[1][0], detection->p[1][1]},
        {detection->p[2][0], detection->p[2][1]},
        {detection->p[3][0], detection->p[3][1]},
    };

    cv::Matx33d cameraMatrix;
    cameraMatrix(0, 0) = intr[0];// fx
    cameraMatrix(1, 1) = intr[1];// fy
    cameraMatrix(0, 2) = intr[2];// cx
    cameraMatrix(1, 2) = intr[3];// cy

    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, {}, rvec, tvec);

    if (!z_up) {
        cv::Mat rvecR;
        cv::Rodrigues(rvec, rvecR);
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(reinterpret_cast<double*>(rvecR.data));
        // rotate by half rotation about x-axis to have z-axis
        // point upwards orthogonal to the tag plane
        rvecR.col(1) *= -1;
        rvecR.col(2) *= -1;
        cv::Rodrigues(rvecR, rvec);
    }

    // rvec = cv::Mat(3,1, cv::DataType<double>::type);
    return tf2::toMsg<std::pair<cv::Mat_<double>, cv::Mat_<double>>, geometry_msgs::msg::Transform>(std::make_pair(tvec, rvec));
}

const std::unordered_map<std::string, pose_estimation_f> pose_estimation_methods{
    {"homography", homography},
    {"pnp", pnp},
};
