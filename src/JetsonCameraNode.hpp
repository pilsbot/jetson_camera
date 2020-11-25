#pragma once

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>

namespace jetson_camera
{

class JetsonCameraNode : public rclcpp::Node
{
  public:
    JetsonCameraNode(const rclcpp::NodeOptions& options);
    ~JetsonCameraNode();

  private:
    std::string createGStreamerPipeline();
    void rescaleCameraInformation();
    void captureFunc();
    void declareParameters();
    void loadParameters();
  	
    rclcpp::Node::SharedPtr subNode;
    image_transport::CameraPublisher cameraPublisher;	
    int captureWidth;
    int	captureHeight;
    int width;
    int height;
    int framerate;
    int flipMethod;
    std::string frameId;
    double delay;
    std::string topicName;
    std::string url;
    sensor_msgs::msg::CameraInfo cameraInfo;

    std::thread captureThread;
    std::atomic<bool> isRunning;

};

}
