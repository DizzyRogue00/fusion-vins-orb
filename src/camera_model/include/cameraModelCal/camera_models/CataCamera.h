//折反射相机模型
#ifndef CATACAMERA_H
#define CATACAMERA_H 

#include <opencv2/core/core.hpp>
#include <string>

#include "ceres/rotation.h"
#include "Camera.h"

namespace cameraModelCal
{ 
/**
 * C. Mei, and P. Rives, Single View Point Omnidirectional Camera Calibration
 * from Planar Grids, ICRA 2007
 */
class CataCamera : public Camera
{ 
public:
    class Parameters: public Camera::Parameters
    { 
    public:
        Parameters();
        Parameters(const std::string& cameraName,
                   int w, int h,
                   double xi, 
                   double k1,double k2, double p1,double p2,
                   double gamma1, double gamma2,
                   double u0, double v0);
        double& xi(void); // 镜面参数
        double& k1(void); // 径向畸变
        double& k2(void);   
        double& p1(void);
        double& p2(void); // 切向畸变
        double& gamma1(void); // 焦距参数，等效 fx，fy
        double& gamma2(void);
        double& u0(void); // 光心坐标
        double& v0(void);

        double xi(void) const; // 镜面参数
        double k1(void) const; // 径向畸变
        double k2(void) const;   
        double p1(void) const;
        double p2(void) const; // 切向畸变
        double gamma1(void) const; // 焦距参数，等效 fx，fy
        double gamma2(void) const;
        double u0(void) const; // 光心坐标
        double v0(void) const;

        bool readFromYamlFile(const std::string& filename);
        void writeToYamlFile(const std::string& filename) const; 

        Parameters& operator=(const Parameters& other);
        friend std::ostream& operator<< (std::ostream& out, const Parameters& params);
    
    private:
        double m_xi;
        double m_k1;
        double m_k2;
        double m_p1;
        double m_p2;
        double m_gamma1;
        double m_gamma2;
        double m_u0;
        double m_v0;   
    };

    CataCamera();

    /**
    * \brief Constructor from the projection model parameters
    */
    CataCamera(const std::string& cameraName,
               int imageWidth, int imageHeight,
               double xi, double k1, double k2, double p1, double p2,
               double gamma1, double gamma2, double u0, double v0);
    /**
    * \brief Constructor from the projection model parameters
    */
    CataCamera(const Parameters& params);

    Camera::ModelType modelType(void) const;
    const std::string& cameraName(void) const;
    int imageWidth(void) const;
    int imageHeight(void) const;

    void estimateIntrinsic(const cv::Size& boardSize,
                                   const std::vector<std::vector<cv::Point3f>>& objectPoints,
                                   const std::vector<std::vector<cv::Point2f>>& imagePoints)=0;
    
    // 像素坐标到相机坐标，输出为P
    void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;

    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;

    // 空间点坐标到像素坐标
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const;
    // 空间点坐标到像素坐标，输出为p和雅可比矩阵J
    void spaceToPlane(const Eigen::Vector3d& P,Eigen::Vector2d& p,
                      Eigen::Matrix<double,2,3>& J) const;

    void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const;
    //%output p

    template<typename T>
    satic void spaceToPlane(const T* const params,
                            const T* const q,
                            const T* const t,
                            const Eigen::Matrix<T,3,1>& P,
                            Eigen::Matrix<T,2,1>& p);

    void distortion(const Eigen::Vector2d& p_u,Eigen::Vector2d& d_u) const;
    void distortion(const Eigen::Vector2d& p_u,Eigen::Vector2d& d_u,
                    Eigen::Matrix2d& J) const;
    
    void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0) const;
    cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                    float fx = -1.0f, float fy = -1.0f,
                                    cv::Size imageSize = cv::Size(0, 0),
                                    float cx = -1.0f, float cy = -1.0f,
                                    cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const;

    int parameterCount(void) const;

    const Parameters& getParameters(void) const;
    void setParameters(const Parameters& parameters);

    void readParameters(const std::vector<double>& parameterVec);
    void writeParameters(std::vector<double>& parameterVec) const;

    void writeParametersToYamlFile(const std::string& filename) const;

    std::string parametersToString(void) const;

private:
    Parameters mParameters;

    double m_inv_K11,m_inv_K13,m_inv_K22,m_inv_K23;
    bool m_noDistortion;
};  

typedef boost::shared_ptr<CataCamera> CataCameraPtr;
typedef boost::shared_ptr<const CataCamera> CataCameraConstPtr;

template <typename T>
void
CataCamera::spaceToPlane(const T* const params,
                         const T* const q, const T* const t,
                         const Eigen::Matrix<T, 3, 1>& P,
                         Eigen::Matrix<T, 2, 1>& p)
{
    T P_w[3];
    P_w[0]=T(P(0));
    P_w[1]=T(P(1));
    P_w[2]=T(P(2));

    // Eigen(x,y,z,w) -> ceres (w,x,y,z)

    T q_ceres[4]={q[3],q[0],q[1],q[2]};

    T P_c[3] //相机坐标
    ceres::QuaternionRotatePoint(q_ceres,P_w,P_c);

    P_c[0]+=t[0];
    P_c[1]+=t[1];
    P_c[2]+=t[2];

    T xi = params[0];
    T k1 = params[1];
    T k2 = params[2];
    T p1 = params[3];
    T p2 = params[4];
    T gamma1 = params[5];
    T gamma2 = params[6];
    T alpha = T(0); //cameraParams.alpha();
    T u0 = params[7];
    T v0 = params[8];

    // 球面归一化
    T len=sqrt(P_c[0]*P_c[0]+P_c[1]*P_c[1]+P_c[2]*P_c[2]);
    P_c[0]/=len;
    P_c[1]/=len;
    P_c[2]/=len;

    // 投影到像素平面
    T u=P_c[0]/(P_c[2]+xi);
    T v=P_c[1]/(P_c[2]+xi);

    T rho_sqr=u*u+v*v;
    T L=T(1.0)+k1*rho_sqr+k2*rho_sqr*rho_sqr;
    T du=T(2.0)*p1*u*v+p2*(rho_sqr+T(2.0)*u*u);
    T dv=p1*(rho_sqr+T(2.0)*v*v)+T(2.0)*p2*u*v;

    u=L*u+du;
    v=L*v+dv;
    p(0)=gamma1*(u+alpha*v)+u0;
    p(1)=gamma2*v+v0;
}

}

#endif