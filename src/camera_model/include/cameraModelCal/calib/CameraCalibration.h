#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include "cameraModelCal/camera_models/Camera.h"

namespace cameraModelCal
{

class CameraCalibration
{
public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    CameraCalibration();
    CameraCalibration(
        Camera::ModelType modelType,
        const std::string& cameraName,
        const cv::Size& imageSize,
        const cv::Size& boardSize, //棋盘格内角点数
        float squareSize);  // 棋盘格方块边长（mm）
    
    void clear(void);

    void addChessboardData(const std::vector<cv::Point2f>& corners);

    bool calibrate(void);

    int sampleCount(void) const;  // 返回已添加的样本数量

    // 图像点（像素坐标）
    std::vector<std::vector<cv::Point2f>>& imagePoints(void);
    const std::vector<std::vector<cv::Point2f>>& imagePoints(void) const;

    // 场景点（世界坐标）
    std::vector<std::vector<cv::Point3f>>& scenePoints(void);
    const std::vector<std::vector<cv::Point3f>>& scenePoints(void) const;

    CameraPtr& camera(void);
    const CameraConstPtr& camera(void) const;

    // 测量协方差矩阵（用于加权优化）
    Eigen::Matrix2d& measurementCovariance(void);
    const Eigen::Matrix2d& measurementCovariance(void) const;

    // 相机位姿（每帧的 rvec, tvec）
    cv::Mat& cameraPoses(void);
    const cv::Mat& cameraPoses(void) const;

    // 可视化标定结果
    void drawResults(std::vector<cv::Mat>& images) const;

    // 保存标定参数到文件
    void writeParams(const std::string& filename) const;

    // 保存/加载棋盘格角点数据（避免重复检测）
    bool writeChessboardData(const std::string& filename) const;
    bool readChessboardData(const std::string& filename);

    // 设置是否输出详细信息
    void setVerbose(bool verbose);

private:
    // 标定辅助函数
    bool calibrateHelper(CameraPtr& camera,
                     std::vector<cv::Mat>& rvecs, 
                     std::vector<cv::Mat>& tvecs) const;

    // 非线性优化
    void optimize(CameraPtr& camera,
              std::vector<cv::Mat>& rvecs, 
              std::vector<cv::Mat>& tvecs) const;

    // 模板读写函数（用于序列化）
    template<typename T>
    void readData(std::ifstream& ifs, T& data) const;

    template<typename T>
    void writeData(std::ofstream& ofs, T data) const;

    cv::Size m_boardSize;      // 棋盘格内角点数 (如 9x6)
    float m_squareSize;        // 方格边长 (mm)

    CameraPtr m_camera;        // 相机对象（包含内参和畸变）
    cv::Mat m_cameraPoses;     // 每帧图像的相机位姿

    std::vector<std::vector<cv::Point2f>> m_imagePoints;  // 图像点集
    std::vector<std::vector<cv::Point3f>> m_scenePoints;  // 世界点集

    Eigen::Matrix2d m_measurementCovariance;  // 测量协方差
    bool m_verbose;  // 是否输出详细信息
};

}


#endif