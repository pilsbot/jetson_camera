#include "JetsonCameraNode.hpp"

#include <opencv2/opencv.hpp>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <thread>
#include <functional>
#include <atomic>

namespace jetson_camera
{

JetsonCameraNode::JetsonCameraNode(const rclcpp::NodeOptions& options)
    : Node("JetsonCameraNode", options)
    {
        subNode = create_sub_node("camera_sub");

	image_transport::ImageTransport imageTransport(subNode);

	declareParameters();
	loadParameters();

	camera_info_manager::CameraInfoManager cameraInfoManager(this, "camera", url);

	cameraInfo = cameraInfoManager.getCameraInfo();
	cameraInfo.header.frame_id = frameId;

	// Rescale camera information

	double widthCoeff = static_cast<double>(width) / cameraInfo.width;
	double heightCoeff = static_cast<double>(height) / cameraInfo.height;

	cameraInfo.width = width;
	cameraInfo.height = height;

	cameraInfo.k[0] *= widthCoeff;
	cameraInfo.k[2] *= widthCoeff;
	cameraInfo.k[4] *= heightCoeff;
	cameraInfo.k[5] *= heightCoeff;

	cameraInfo.p[0] *= widthCoeff;
	cameraInfo.p[2] *= widthCoeff;
	cameraInfo.p[5] *= heightCoeff;
	cameraInfo.p[6] *= heightCoeff;

	RCLCPP_INFO(get_logger(), "jetson_camera: Starting node with the following parameters:\n"\
		             "capture width: %d, capture height: %d\n"\
		             "published width: %d, published height: %d\n"\
		             "requested framerate: %d, flip method: %d\n",
		             captureWidth, captureHeight, width, height, framerate, flipMethod);

	cameraPublisher = imageTransport.advertiseCamera(topicName, 1);
	
	isRunning = true;
	captureThread = std::thread(std::bind(&JetsonCameraNode::captureFunc, this));
    }
	
    JetsonCameraNode::~JetsonCameraNode()
    {
	isRunning = false;
	captureThread.join();
    }

    void JetsonCameraNode::declareParameters()
    {
	declare_parameter<std::string>("topic_name", "image_raw");
	declare_parameter<int>("cap_width", 1280);
	declare_parameter<int>("cap_height", 720);		
	declare_parameter<int>("width", 640);
	declare_parameter<int>("height", 480);
	declare_parameter<int>("framerate", 60);
	declare_parameter<int>("flip_method", 0);
	declare_parameter<std::string>("frame_id", "main_camera_optical");
	declare_parameter<double>("capture_delay", 0.0);
	declare_parameter<std::string>("url", "");
    }

    void JetsonCameraNode::loadParameters()
    {
	get_parameter("cap_width", captureWidth);
	get_parameter("cap_height", captureHeight);
	get_parameter("width", width);
	get_parameter("height", height);
	get_parameter("framerate", framerate);
	get_parameter("flip_method", flipMethod);
	get_parameter("frame_id", frameId);
	get_parameter("delay", delay);
	get_parameter("topic_name", topicName);
	get_parameter("url", url);
    }

    std::string JetsonCameraNode::createGStreamerPipeline()
    {
        return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(captureWidth) + ", height=(int)" +
                std::to_string(captureHeight) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
                "/1 ! nvvidconv flip-method=" + std::to_string(flipMethod) + " ! video/x-raw, width=(int)" + std::to_string(width) + ", height=(int)" +
                std::to_string(height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
        }


    void JetsonCameraNode::captureFunc()
    {
	std::string pipeline = createGStreamerPipeline();
	RCLCPP_INFO(get_logger(), "Pipeline: %s", pipeline.c_str());
	cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);	

	cv_bridge::CvImage img;
	img.encoding = sensor_msgs::image_encodings::BGR8;
	img.header.frame_id = frameId;

	while(isRunning)
	{
            if(cap.read(img.image))
	    {
	        img.header.stamp = now() - rclcpp::Duration(delay);	
		cameraInfo.header.stamp = img.header.stamp;
		cameraPublisher.publish(*img.toImageMsg(), cameraInfo);	
	    }
	}
    }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(jetson_camera::JetsonCameraNode)
