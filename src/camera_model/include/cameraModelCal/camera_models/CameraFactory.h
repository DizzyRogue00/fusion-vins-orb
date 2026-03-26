#ifndef CAMERAFACTORY_H
#define CAMERAFACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "cameraModelCal/camera_models/Camera.h"

namespace cameraModelCal
{
class CameraFactory
{
public: 
    CameraFactory();

    static boost::shared_ptr<CameraFactory> instance(void);

    CameraPtr generateCamera(Camera::ModelType modelType,
                          const std::string& cameraName,
                          const cv::Size& imageSize) const;

    CameraPtr generateCameraFromYamlFile(const std::string& filename);

private:
    static boost::shared_ptr<CameraFactory> m_instance;
};
}

#endif