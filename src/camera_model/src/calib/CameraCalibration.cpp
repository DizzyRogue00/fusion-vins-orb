#include "cameraModelCal/calib/CameraCalibration.h"

#include "camodocal/calib/CameraCalibration.h"

#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "cameraModelCal/camera_models/CameraFactory.h"
#include "cameraModelCal/sparse_graph/Transform.h"
#include "ceres/manifold.h" //EigenQuaternionManifold
#include "cameraModelCal/gpl/EigenUtils.h"
#include "cameraModelCal/camera_models/CostFunctionFactory.h"

#include "ceres/ceres.h"

namespace cameraModelCal
{
CameraCalibration::CameraCalibration()
:m_boardSize(cv::Size(0, 0)),m_squareSize(0.0f),m_verbose(false)
{ 
}

CameraCalibration::CameraCalibration(
        Camera::ModelType modelType,
        const std::string& cameraName,
        const cv::Size& imageSize,
        const cv::Size& boardSize, //棋盘格内角点数
        float squareSize)  // 棋盘格方块边长（mm）
:m_boardSize(boardSize)
,m_squareSize(squareSize)
,m_verbose(false)
{
    m_camera = CameraFactory::instance()->generateCamera(modelType, cameraName, imageSize);
}
    
void CameraCalibration::clear(void)
{
    m_imagePoints.clear();
    m_scenePoints.clear();
}

void CameraCalibration::addChessboardData(const std::vector<cv::Point2f>& corners)
{
    m_imagePoints.push_back(corners);

    std::vector<cv::Point3f> scenePointsInView;

    for(int i = 0; i < m_boardSize.height; ++i)
    {
        for(int j=0;j<m_boardSize.width; ++j)
        {
            scenePointsInView.push_back(cv::Point3f(j*m_squareSize, i*m_squareSize, 0.0));
        }
    }
    m_scenePoints.push_back(scenePointsInView);
}

bool CameraCalibration::calibrate(void)
{
    int imageCount = m_imagePoints.size();

    // 计算相机内参和外参
    std::vector<cv::Mat> rvecs, tvecs;
    bool ret=calibrateHelper(m_camera,rvecs,tvecs);

    for(int i=0;i<imageCount;++i)
    {
        m_cameraPoses.at<double>(i,0)=rvecs.at(i).at<double>(0);
        m_cameraPoses.at<double>(i,1)=rvecs.at(i).at<double>(1);
        m_cameraPoses.at<double>(i,2)=rvecs.at(i).at<double>(2);

        m_cameraPoses.at<double>(i,3)=tvecs.at(i).at<double>(0);
        m_cameraPoses.at<double>(i,4)=tvecs.at(i).at<double>(1);
        m_cameraPoses.at<double>(i,5)=tvecs.at(i).at<double>(2);
    }

    // 测量协方差
    std::vector<std::vector<cv::Point2f>> errVec(m_imagePoints.size());
    Eigen::Vector2d errSum = Eigen::Vector2d::Zero();
    size_t errCount = 0;
    for (size_t i = 0; i < m_imagePoints.size(); ++i)
    {
        std::vector<cv::Point2f> estImagePoints;
        m_camera->projectPoints(m_scenePoints.at(i), rvecs.at(i), tvecs.at(i),
                                estImagePoints);

        for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j)
        {
            cv::Point2f pObs = m_imagePoints.at(i).at(j);
            cv::Point2f pEst = estImagePoints.at(j);

            cv::Point2f err = pObs - pEst;

            errVec.at(i).push_back(err);

            errSum += Eigen::Vector2d(err.x, err.y);
        }

        errCount += m_imagePoints.at(i).size();
    }

    Eigen::Vector2d errMean = errSum / static_cast<double>(errCount);

    Eigen::Matrix2d measurementCovariance = Eigen::Matrix2d::Zero();
    for (size_t i = 0; i < errVec.size(); ++i)
    {
        for (size_t j = 0; j < errVec.at(i).size(); ++j)
        {
            cv::Point2f err = errVec.at(i).at(j);
            double d0 = err.x - errMean(0);
            double d1 = err.y - errMean(1);

            measurementCovariance(0,0) += d0 * d0;
            measurementCovariance(0,1) += d0 * d1;
            measurementCovariance(1,1) += d1 * d1;
        }
    }
    measurementCovariance /= static_cast<double>(errCount);
    measurementCovariance(1,0) = measurementCovariance(0,1);

    m_measurementCovariance = measurementCovariance;

    return ret;
}

int CameraCalibration::sampleCount(void) const  // 返回已添加的样本数量
{
    return m_imagePoints.size();
}

// 图像点（像素坐标）
std::vector<std::vector<cv::Point2f>>& CameraCalibration::imagePoints(void)
{
    return m_imagePoints;
}

const std::vector<std::vector<cv::Point2f>>& CameraCalibration::imagePoints(void) const
{
    return m_imagePoints;
}

// 场景点（世界坐标）
std::vector<std::vector<cv::Point3f>>& CameraCalibration::scenePoints(void)
{
    return m_scenePoints;
}

const std::vector<std::vector<cv::Point3f>>& CameraCalibration::scenePoints(void) const
{
    return m_scenePoints;
}

CameraPtr& CameraCalibration::camera(void)
{
    return m_camera;
}

const CameraConstPtr& CameraCalibration::camera(void) const
{
    return m_camera;
}

// 测量协方差矩阵（用于加权优化）
Eigen::Matrix2d& CameraCalibration::measurementCovariance(void)
{
    return m_measurementCovariance;
}

const Eigen::Matrix2d& CameraCalibration::measurementCovariance(void) const
{
    return m_measurementCovariance;
}

// 相机位姿（每帧的 rvec, tvec）
cv::Mat& CameraCalibration::cameraPoses(void)
{
    return m_cameraPoses;
}
    
const cv::Mat& CameraCalibration::cameraPoses(void) const
{
    return m_cameraPoses;
}

// 可视化标定结果
void CameraCalibration::drawResults(std::vector<cv::Mat>& images) const
{
    std::vector<cv::Mat> rvecs, tvecs;

    for (size_t i = 0; i < images.size(); ++i)
    {
        cv::Mat rvec(3, 1, CV_64F);
        rvec.at<double>(0) = m_cameraPoses.at<double>(i,0);
        rvec.at<double>(1) = m_cameraPoses.at<double>(i,1);
        rvec.at<double>(2) = m_cameraPoses.at<double>(i,2);

        cv::Mat tvec(3, 1, CV_64F);
        tvec.at<double>(0) = m_cameraPoses.at<double>(i,3);
        tvec.at<double>(1) = m_cameraPoses.at<double>(i,4);
        tvec.at<double>(2) = m_cameraPoses.at<double>(i,5);

        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
    }

    int drawShiftBits = 4;
    int drawMultiplier = 1 << drawShiftBits;

    cv::Scalar green(0, 255, 0);
    cv::Scalar red(0, 0, 255);

    for (size_t i = 0; i < images.size(); ++i)
    {
        cv::Mat& image = images.at(i);
        if (image.channels() == 1)
        {
            cv::cvtColor(image, image, CV_GRAY2RGB);
        }

        std::vector<cv::Point2f> estImagePoints;
        m_camera->projectPoints(m_scenePoints.at(i), rvecs.at(i), tvecs.at(i),
                                estImagePoints);

        float errorSum = 0.0f;
        float errorMax = std::numeric_limits<float>::min();

        for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j)
        {
            cv::Point2f pObs = m_imagePoints.at(i).at(j);
            cv::Point2f pEst = estImagePoints.at(j);

            cv::circle(image,
                       cv::Point(cvRound(pObs.x * drawMultiplier),
                                 cvRound(pObs.y * drawMultiplier)),
                       5, green, 2, CV_AA, drawShiftBits);

            cv::circle(image,
                       cv::Point(cvRound(pEst.x * drawMultiplier),
                                 cvRound(pEst.y * drawMultiplier)),
                       5, red, 2, CV_AA, drawShiftBits);

            float error = cv::norm(pObs - pEst);

            errorSum += error;
            if (error > errorMax)
            {
                errorMax = error;
            }
        }

        std::ostringstream oss;
        oss << "Reprojection error: avg = " << errorSum / m_imagePoints.at(i).size()
            << "   max = " << errorMax;

        cv::putText(image, oss.str(), cv::Point(10, image.rows - 10),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                    1, CV_AA);
    }
}

// 保存标定参数到文件
void CameraCalibration::writeParams(const std::string& filename) const
{
    m_camera->writeParametersToYamlFile(filename);
}

// 保存/加载棋盘格角点数据（避免重复检测）
bool CameraCalibration::writeChessboardData(const std::string& filename) const
{
    std::ofstream ofs(filename.c_str(), std::ios::out | std::ios::binary);
    if (!ofs.is_open())
    {
        return false;
    }

    writeData(ofs, m_boardSize.width);
    writeData(ofs, m_boardSize.height);
    writeData(ofs, m_squareSize);

    writeData(ofs, m_measurementCovariance(0,0));
    writeData(ofs, m_measurementCovariance(0,1));
    writeData(ofs, m_measurementCovariance(1,0));
    writeData(ofs, m_measurementCovariance(1,1));

    writeData(ofs, m_cameraPoses.rows);
    writeData(ofs, m_cameraPoses.cols);
    writeData(ofs, m_cameraPoses.type());
    for (int i = 0; i < m_cameraPoses.rows; ++i)
    {
        for (int j = 0; j < m_cameraPoses.cols; ++j)
        {
            writeData(ofs, m_cameraPoses.at<double>(i,j));
        }
    }

    writeData(ofs, m_imagePoints.size());
    for (size_t i = 0; i < m_imagePoints.size(); ++i)
    {
        writeData(ofs, m_imagePoints.at(i).size());
        for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j)
        {
            const cv::Point2f& ipt = m_imagePoints.at(i).at(j);
            writeData(ofs, ipt.x);
            writeData(ofs, ipt.y);
        } 
    }


    writeData(ofs, m_scenePoints.size());
    for (size_t i = 0; i < m_scenePoints.size(); ++i)
    {
        writeData(ofs, m_scenePoints.at(i).size());
        for (size_t j = 0; j < m_scenePoints.at(i).size(); ++j)
        {
            const cv::Point3f& spt = m_scenePoints.at(i).at(j);

            writeData(ofs, spt.x);
            writeData(ofs, spt.y);
            writeData(ofs, spt.z);
        }
    }   

    return true;
}


bool CameraCalibration::readChessboardData(const std::string& filename)
{
    std::ifstream ifs(filename.c_str(), std::ios::in | std::ios::binary);
    if (!ifs.is_open())
    {
        return false;
    }

    readData(ifs, m_boardSize.width);
    readData(ifs, m_boardSize.height);
    readData(ifs, m_squareSize);

    readData(ifs, m_measurementCovariance(0,0));
    readData(ifs, m_measurementCovariance(0,1));
    readData(ifs, m_measurementCovariance(1,0));
    readData(ifs, m_measurementCovariance(1,1));

    int rows, cols, type;
    readData(ifs, rows);
    readData(ifs, cols);
    readData(ifs, type);
    m_cameraPoses = cv::Mat(rows, cols, type);

    for (int i = 0; i < m_cameraPoses.rows; ++i)
    {
        for (int j = 0; j < m_cameraPoses.cols; ++j)
        {
            readData(ifs, m_cameraPoses.at<double>(i,j));
        }
    }

    size_t nImagePointSets;
    readData(ifs, nImagePointSets);

    m_imagePoints.clear();
    m_imagePoints.resize(nImagePointSets);
    for(size_t i = 0; i < m_imagePoints.size(); ++i)
    {
        size_t nImagePoints;
        readData(ifs, nImagePoints);
        m_imagePoints.at(i).resize(nImagePoints);
        for(size_t j = 0; j < m_imagePoints.at(i).size(); ++j)
        {
            cv::Point2f& ipt = m_imagePoints.at(i).at(j);
            readData(ifs, ipt.x);
            readData(ifs, ipt.y);
        }
    }

    size_t nScenePointSets;
    readData(ifs, nScenePointSets);

    m_scenePoints.clear();
    m_scenePoints.resize(nScenePointSets);
    for (size_t i = 0; i < m_scenePoints.size(); ++i)
    {
        size_t nScenePoints;
        readData(ifs, nScenePoints);
        m_scenePoints.at(i).resize(nScenePoints);

        for (size_t j = 0; j < m_scenePoints.at(i).size(); ++j)
        {
            cv::Point3f& spt = m_scenePoints.at(i).at(j);
            readData(ifs, spt.x);
            readData(ifs, spt.y);
            readData(ifs, spt.z);
        }
    }

    return true;
}

// 设置是否输出详细信息
void CameraCalibration::setVerbose(bool verbose)
{
    m_verbose= verbose;
}

// 标定辅助函数
bool CameraCalibration::calibrateHelper(CameraPtr& camera,
                     std::vector<cv::Mat>& rvecs, 
                     std::vector<cv::Mat>& tvecs) const
{
    rvecs.assign(m_scenePoints.size(), cv::Mat());
    tvecs.assign(m_scenePoints.size(), cv::Mat());

    // 内参估计
    camera->estimateIntrinsic(m_boardSize, m_scenePoints, m_imagePoints);

    // 外参估计
    for (size_t i = 0; i < m_scenePoints.size(); ++i)
    {
        camera->estimateExtrinsic(m_scenePoints.at(i), m_imagePoints.at(i),
                                  rvecs.at(i), tvecs.at(i));
    }

    if(m_verbose)
    {
        std::cout << "[" << camera->cameraName() << "] "
                << "# INFO: " << "Initial reprojection error: "
                << std::fixed << std::setprecision(3)
                << camera->reprojectionError(m_scenePoints, m_imagePoints, rvecs, tvecs)
                << " pixels" << std::endl;

    }

// 优化
    optimize(camera, rvecs, tvecs);
    if(m_verbose)
    {
        double err=camera->reprojectionError(m_scenePoints, m_imagePoints, rvecs, tvecs);
        std::cout << "[" << camera->cameraName() << "] " << "# INFO: Final reprojection error: "
                  << err << " pixels" << std::endl;
        std::cout << "[" << camera->cameraName() << "] " << "# INFO: "
                  << camera->parametersToString() << std::endl;
    }

    return true;
}

// 非线性优化
void CameraCalibration::optimize(CameraPtr& camera,
              std::vector<cv::Mat>& rvecs, 
              std::vector<cv::Mat>& tvecs) const
{
    ceres::Problem problem;

    std::vector<Transform,Eigen::aligned_allocator<Transform>> transformVec(rvecs.size());
    std::vector<Transform, Eigen::aligned_allocator<Transform>> transformVec(rvecs.size());
    for (size_t i = 0; i < rvecs.size(); ++i)   
    {
        // 将 OpenCV 的旋转向量转换为 Eigen 向量
        Eigen::Vector3d rvec;
        cv::cv2eigen(rvecs.at(i), rvec);
    
        // 旋转向量 → 四元数（Transform 内部存储）
        transformVec.at(i).rotation() = Eigen::AngleAxisd(rvec.norm(), rvec.normalized());
    
        // 平移向量
        transformVec.at(i).translation() << tvecs[i].at<double>(0),
                                        tvecs[i].at<double>(1),
                                        tvecs[i].at<double>(2);
    }

    std::vector<double> intrinsicCameraParams;
    m_camera->writeParameters(intrinsicCameraParams);

    for (size_t i = 0; i < m_imagePoints.size(); ++i)           // 遍历每张图像
    {
        for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j) // 遍历每个角点
        {
            // 获取 3D 场景点（世界坐标）
            const cv::Point3f& spt = m_scenePoints.at(i).at(j);
            // 获取 2D 图像点（观测值）
            const cv::Point2f& ipt = m_imagePoints.at(i).at(j);
        
            // 创建代价函数
            ceres::CostFunction* costFunction =
                CostFunctionFactory::instance()->generateCostFunction(
                    camera,                                    // 相机模型
                    Eigen::Vector3d(spt.x, spt.y, spt.z),     // 3D点
                    Eigen::Vector2d(ipt.x, ipt.y),            // 2D观测
                    CAMERA_INTRINSICS | CAMERA_POSE);          // 优化标志
        
            // 创建损失函数（鲁棒核函数）
            ceres::LossFunction* lossFunction = new ceres::CauchyLoss(1.0);
        
            // 添加残差块
            problem.AddResidualBlock(costFunction, lossFunction,
                                 intrinsicCameraParams.data(),      // 相机内参
                                 transformVec.at(i).rotationData(), // 旋转（四元数）
                                 transformVec.at(i).translationData()); // 平移
            }
    }
}

// 模板读写函数（用于序列化）
template<typename T>
void CameraCalibration::readData(std::ifstream& ifs, T& data) const
{
    char* buffer = new char[sizeof(T)];

    ifs.read(buffer, sizeof(T));

    data = *(reinterpret_cast<T*>(buffer));

    delete buffer;
}

template<typename T>
void CameraCalibration::writeData(std::ofstream& ofs, T data) const
{
    char* pData = reinterpret_cast<char*>(&data);
    ofs.write(pData, sizeof(T));
}

}