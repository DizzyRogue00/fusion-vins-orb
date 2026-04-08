#ifndef COSTFUNCTIONFACTORY_H
#define COSTFUNCTIONFACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "Camera.h"

namespace ceres
{
    class CostFunction;
}

namespace cameraModelCal
{

enum
{
    CAMERA_INTRINSICS =         1 << 0,
    CAMERA_POSE =               1 << 1,
    POINT_3D =                  1 << 2,
    ODOMETRY_INTRINSICS =       1 << 3,
    ODOMETRY_3D_POSE =          1 << 4,
    ODOMETRY_6D_POSE =          1 << 5,
    CAMERA_ODOMETRY_TRANSFORM = 1 << 6
};

class CostFunctionFactory
{ 
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CostFunctionFactory();

    static boost::shared_ptr<CostFunctionFactory> instance(void);

    // 标准重投影误差
    ceres::CostFunction* generateCostFunction(
        const CameraConstPtr& camera,      // 相机模型
        const Eigen::Vector3d& observed_P, // 观测到的3D点
        const Eigen::Vector2d& observed_p, // 观测到的2D点
        int flags) const;                  // 优化标志

    // 带权重的重投影误差
    ceres::CostFunction* generateCostFunction(
        const CameraConstPtr& camera,
        const Eigen::Vector3d& observed_P,
        const Eigen::Vector2d& observed_p,
        const Eigen::Matrix2d& sqrtPrecisionMat,  // 精度矩阵的平方根
        int flags) const;

    // 仅从图像点生成
    ceres::CostFunction* generateCostFunction(
        const CameraConstPtr& camera,
        const Eigen::Vector2d& observed_p,  // 仅观测到的2D点
        int flags, 
        bool optimize_cam_odo_z = true) const;

    ceres::CostFunction* generateCostFunction(
        const CameraConstPtr& camera,
        const Eigen::Vector2d& observed_p,
        const Eigen::Matrix2d& sqrtPrecisionMat,
        int flags, bool optimize_cam_odo_z = true) const;
                                
    // 里程计-视觉
    ceres::CostFunction* generateCostFunction(
        const CameraConstPtr& camera,
        const Eigen::Vector3d& odo_pos,     // 里程计位置
        const Eigen::Vector3d& odo_att,     // 里程计姿态（欧拉角）
        const Eigen::Vector2d& observed_p,
        int flags, 
        bool optimize_cam_odo_z = true) const;

    //相机-里程计外参和里程计位姿
    ceres::CostFunction* generateCostFunction(
        const CameraConstPtr& camera,
        const Eigen::Quaterniond& cam_odo_q,  // 相机-里程计旋转
        const Eigen::Vector3d& cam_odo_t,     // 相机-里程计平移
        const Eigen::Vector3d& odo_pos,
        const Eigen::Vector3d& odo_att,
        const Eigen::Vector2d& observed_p,
        int flags) const;
    
    // 双目视觉
    ceres::CostFunction* generateCostFunction(
        const CameraConstPtr& cameraLeft,   // 左相机
        const CameraConstPtr& cameraRight,  // 右相机
        const Eigen::Vector3d& observed_P,
        const Eigen::Vector2d& observed_p_left,
        const Eigen::Vector2d& observed_p_right) const;
private:
        static boost::shared_ptr<CostFunctionFactory> m_instance;
};

}


#endif