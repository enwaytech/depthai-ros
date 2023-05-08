#include <cstdio>
#include <iostream>

#include "ros/ros.h"

/ Inludes common necessary includes for development using depthai library
include <depthai_bridge/BridgePublisher.hpp>
include <depthai_bridge/ImageConverter.hpp>

#include "depthai/depthai.hpp"

dai::Pipeline createPipeline(bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, std::string resolution) {
    dai::Pipeline pipeline;
    //nn
    //std::string nnPath("/home/sebastian/nn/yolop_320x320.xml");
    //pipeline.setOpenVINOVersion(dai::OpenVINO::Version::VERSION_2021_4);
    //Color
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("video");

    //Sereo
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();

    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    xoutDepth->setStreamName("depth");
    //Edge
    auto edgeDetectorRgb = pipeline.create<dai::node::EdgeDetector>();
    std::vector<std::vector<int>> sobelHorizontalKernel = {{1, 0, -1}, {2, 0, -2}, {1, 0, -1}};
    std::vector<std::vector<int>> sobelVerticalKernel = {{1, 2, 1}, {0, 0, 0}, {-1, -2, -1}};
    //edgeDetectorRgb->setSobelFilterKernels(sobelHorizontalKernel, sobelVerticalKernel);

    auto xoutEdgeRgb = pipeline.create<dai::node::XLinkOut>();

    xoutEdgeRgb->setStreamName("edge_rgb");

    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    if(resolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
    } else if(resolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
    } else if(resolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
    } else if(resolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
    } else {
        ROS_ERROR("Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    //Properties
    // MonoCamera
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
    // stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

    //Edge
    edgeDetectorRgb->setMaxOutputFrameSize(colorCam->getVideoWidth() * colorCam->getVideoHeight());

    // Color camers steream setup -------->
    colorCam->setBoardSocket(dai::CameraBoardSocket::RGB);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);

    //Linking
    // Color Link plugins CAM -> XLINK
    colorCam->video.link(xlinkOut->input);
    
    // Stereo Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->depth.link(xoutDepth->input);

    //Edge Link plugins CAM -> EDGE -> XLINK
    colorCam->video.link(edgeDetectorRgb->inputImage);
    edgeDetectorRgb->outputImage.link(xoutEdgeRgb->input);

    return pipeline;
}

template <typename T>
static inline void getParamWithWarning(ros::NodeHandle& pnh, const char* key, T val) {
    bool gotParam = pnh.getParam(key, val);
    if(!gotParam) {
        std::stringstream ss;
        ss << val;
        ROS_WARN("Could not find param '%s' on node '%s'. Defaulting to '%s'", key, pnh.getNamespace().c_str(), ss.str().c_str());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rgb_stereo_node");
    ros::NodeHandle pnh("~");

    std::string tfPrefix = "dai";
    std::string camera_param_uri;
    std::string monoResolution = "720p";

    bool lrcheck, extended, subpixel;
    int confidence = 200;
    int LRchecktresh = 5;
    int badParams = 0;

    badParams += !pnh.getParam("camera_param_uri", camera_param_uri);
    badParams += !pnh.getParam("tf_prefix", tfPrefix);
    badParams += !pnh.getParam("lrcheck", lrcheck);
    badParams += !pnh.getParam("extended", extended);
    badParams += !pnh.getParam("subpixel", subpixel);
    badParams += !pnh.getParam("confidence", confidence);
    badParams += !pnh.getParam("LRchecktresh", LRchecktresh);

    if(badParams > 0) {
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find %d of the parameters");
    }

    //dai::DeviceInfo device_info("169.254.1.222");
    dai::Pipeline pipeline = createPipeline(lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution);
    std::cout<< "uploading pipeline...\n";
    dai::Device device(pipeline);
    std::cout<< "...pipeline upload !\n";
    auto calibrationHandler = device.readCalibration();
    auto stereoQueue    = device.getOutputQueue("depth", 30, false);
    auto previewQueue   = device.getOutputQueue("video", 30, false);
    //auto edgeLeftQueue  = device.getOutputQueue("edge_left", 30, false);
    //auto edgeRightQueue = device.getOutputQueue("edge_right", 30, false); 
    auto edgeRgbQueue   = device.getOutputQueue("edge_rgb", 30, false);   
                          
    bool latched_cam_info = true;
    std::string stereo_uri = camera_param_uri + "/" + "right.yaml";
    std::string color_uri = camera_param_uri + "/" + "color.yaml";


    //auto edgeRgb = edgeRgbQueue->get<dai::ImgFrame>();
    //cv::Mat edgeRgbFrame = edgeRgb->getFrame();
    //cv::imshow("edge_rgb", edgeRgbFrame);

    //Depth
    dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_depth_camera_optical_frame", true);
    auto rgbCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, 1280, 720);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(stereoQueue,
                                                                                    pnh,
                                                                                    std::string("stereo/depth"),
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                              &depthConverter,  // since the converter has the same frame name
                                                                                                                // and image type is also same we can reuse it
                                                                                              std::placeholders::_1,
                                                                                              std::placeholders::_2),
                                                                                    30,
                                                                                    rgbCameraInfo,
                                                                                    "stereo");

    //Color
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(previewQueue,
                                                                                  pnh,
                                                                                  std::string("color/image"),
                                                                                  std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                            &rgbConverter,  // since the converter has the same frame name
                                                                                                            // and image type is also same we can reuse it
                                                                                            std::placeholders::_1,
                                                                                            std::placeholders::_2),
                                                                                  30,
                                                                                  rgbCameraInfo,
                                                                                  "color");
    //Edge
    dai::rosBridge::ImageConverter edgeConverter(tfPrefix + "_edge_camera_optical_frame", true);
    //auto MonoLeftCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, 1280, 720);
    //auto MonoRightCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, 1280, 720);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> edgePublish(edgeRgbQueue,
                                                                                    pnh,
                                                                                    std::string("edge/image"),
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                              &edgeConverter,  // since the converter has the same frame name
                                                                                                                // and image type is also same we can reuse it
                                                                                              std::placeholders::_1,
                                                                                              std::placeholders::_2),
                                                                                    30,
                                                                                    rgbCameraInfo,
                                                                                    "edge");

    depthPublish.addPublisherCallback();  // addPublisherCallback works only when the dataqueue is non blocking.
    rgbPublish.addPublisherCallback();
    edgePublish.addPublisherCallback();

    // We can add the rectified frames also similar to these publishers.
    // Left them out so that users can play with it by adding and removing

    ros::spin();

    return 0;
}
