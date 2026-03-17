#ifndef GPL_H
#define GPL_H

#include <algorithm>
#include <cmath>
#include <opencv2/core.core.hpp>

namespace cameraModelCal
{

// 将数值v限制在区间[a,b]内，像素坐标边界检查、角度限制、数值稳定性处理
template<class T>
const T clamp(const T& v,const T& a, const T& b)
{
    return std::min(b,std::max(a,v))
}

// 三维空间距离
double hypot3(double x,double y,double z);
float hypot3f(float x,float y,float z);

// 角度规范化，处理角度差、旋转矩阵计算、避免角度跳变
template<class T>
const T normalizeTheta(const T& theta)
{
    T normTheta=theta;
    while(normTheta<-M_PI){
        normTheta+=2.0*M_PI;
    }
    while(normTheta>M_PI)
    {
        normTheta-=2.0*M_PI;
    }
    return normTheta
}

// 角度弧度转化
double d2r(double deg);
float d2r(float deg);
double r2d(double rad);
float r2d(double rad);

// sinc函数 sin(theta)/theta
double sinc(double theta);

// 平方、立方
template<class T>
const T square(const T& x){return x*x;}

template<class T>
const T cube(const T& x){return x*x*x;}

// 随机数生成，[a,b]间均匀分布
template<class T>
const T random(const T& a,const T& b)
{
    return static_cast<double>(rand())/RAND_MAX*(b-a)+a;
}

//生成均值为0,标准差为sigma的正态分布随机数
template<class T>
const T randomNormal(const T& sigma)
{
    //Box-Muller 正态分布
    T,x1,x2,w;
    do{
        x1=2.0*random(0.0,1.0)-1.0;
        x2=2.0*random(0.0,1.0)-1.0;
        w=x1*x1+x2*x2;
    }while(w>=1.0||w==0.0)

    w=sqrt((-2.0*log(w))/w);
    return x1*w*sigma;
}

// 时间处理
unsigned long long timeInMicroseconds(void);
double timeInSeconds(void);

long int timestampDiff(uint64_t t1,uint64_t t2);//计算时间差

// 图像处理，将单通道深度图转换为彩色深度图
void colorDepthImage(cv::Mat& imgDepth,
                    cv::Mat& imgColoredDepth,
                    float minRange,floar maxRange
);

// 由颜色名称和id获取r,g,b值
bool colormap(const std::string& name,unsigned char idx,
float& r,float& g, float& b);
}

// 计算直线或者圆周上的所有像素,bresenham算法
std::vector<cv::Point2i> bresLine(int x0,int y0,int x1,int y1);
std::vector<cv::Point2i> bresCircle(int x0,int y0,int r);

// 圆拟合
void fitCircle(const std::vector<cv::point2d>& points,
                double& centerX,double& centerY,double& radius);

// 两个圆的交点
void std::vector<cv::point2d> intersectCircles(
    double x1,double y1, double r1,
    double x2,double y2, double r2
);

// 坐标转化 LL与UTM
void LLtoUTM(
    double latitude,double longitude,
    double& utmNorthing, double& utmEasting,
    std::string& utmZone
);

void UTMtoLL(
    double utmNorthing, double utmEasting,
    const std::string& utmZone,
    double& latitude,double& longitude
);
#endif