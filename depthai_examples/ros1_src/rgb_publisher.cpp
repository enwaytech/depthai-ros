
#include <cstdio>
#include <iostream>

#include "ros/ros.h"
// #include "utility.hpp"
#include <camera_info_manager/camera_info_manager.h>

#include "sensor_msgs/Image.h"

// Inludes common necessary includes for development using depthai library
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

#include "depthai/depthai.hpp"

dai::Pipeline createPipeline() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("video");

    auto controlIn = pipeline.create<dai::node::XLinkIn>();
    controlIn->setStreamName("control");

    // prepoerties
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    // THE_4_K: image_height: 2160 image_width: 4096
    // THE_1080_P: image_height: 1080 image_width: 1920
    colorCam->setInterleaved(false);
    colorCam->setFps(60);

    std::cout << "res: " << colorCam->getResolutionWidth() << " " << colorCam->getResolutionHeight() << std::endl;
    std::cout << "fps: " << colorCam->getFps() << std::endl;

    // Link plugins CAM -> XLINK
    colorCam->video.link(xlinkOut->input);
    controlIn->out.link(colorCam->inputControl);
    return pipeline;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rgb_node");
    ros::NodeHandle pnh("~");

    std::string tfPrefix;
    std::string camera_param_uri;
    int badParams = 0;

    badParams += !pnh.getParam("tf_prefix", tfPrefix);
    badParams += !pnh.getParam("camera_param_uri", camera_param_uri);

    if(badParams > 0) {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    dai::Pipeline pipeline = createPipeline();
    dai::Device device(pipeline);
    std::shared_ptr<dai::DataOutputQueue> imgQueue = device.getOutputQueue("video", 30, false);
    auto controlQueue = device.getInputQueue("control");

    std::string color_uri = camera_param_uri + "/" + "color.yaml";

    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(imgQueue,
                                                                                  pnh,
                                                                                  std::string("color/image"),
                                                                                  std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                            &rgbConverter,  // since the converter has the same frame name
                                                                                                            // and image type is also same we can reuse it
                                                                                            std::placeholders::_1,
                                                                                            std::placeholders::_2),
                                                                                  30,
                                                                                  color_uri,
                                                                                  "color");

    rgbPublish.addPublisherCallback();

    // modify camera control based on
    // https://docs.luxonis.com/projects/api/en/latest/samples/ColorCamera/rgb_camera_control/#rgb-camera-control

    std::string message =
        "This example shows usage of Camera Control message as well as ColorCamera configInput to change crop x and y"
        "\n* Uses 'WASD' controls to move the crop window, 'C' to capture a still image, 'T' to trigger autofocus, 'IOKL,.[]'"
        "\n* for manual exposure/focus/white-balance:"
        "\n*   Control:      key[dec/inc]  min..max"
        "\n*     exposure time:     I   O      1..33000 [us]"
        "\n*     sensitivity iso:   K   L    100..1600"
        "\n*     ---- focus:             ,   .      0..255 [far..near] ----"
        "\n*     white balance:     [   ]   1000..12000 (light color temperature K)"
        "\n*     brightness:   Y   U    -10..10"
        "\n*     contrast:   H   J    -10..10"
        "\n*     saturation:   N   M    -10..10"
        "\n*     sharpness:   R   T    0..4"
        "\n*     auto-exposure compensation:   F   G    -9..9"
        "\n* To go back to auto controls:"
        "\n*   'E' - autoexposure"
        "\n*    ---- 'F' - autofocus (continuous) ----"
        "\n*   'B' - auto white-balance";

    // defaults
    int expTime = 100;
    int expMin = 1;
    int expMax = 33000;

    int sensIso = 800;
    int sensMin = 100;
    int sensMax = 1600;

    int wbManual = 4000;
    int wbMin = 1000;
    int wbMax = 12000;

    int brightness = 0;
    int brightnessMin = -10;
    int brightnessMax = 10;
    int contrast = 0;
    int contrastMin = -10;
    int contrastMax = 10;
    int saturation = 0;
    int saturationMin = -10;
    int saturationMax = 10;
    int sharpness = 0;
    int sharpnessMin = 0;
    int sharpnessMax = 4;

    int autoExpComp = 0;
    int autoExpCompMin = -9;
    int autoExpCompMax = 9;

    static constexpr int EXP_STEP = 20;  // us
    static constexpr int ISO_STEP = 50;
    static constexpr int WB_STEP = 200;
    static constexpr int GENERAL_STEP = 1;

    bool interactive = true;

    std::cout << "interactive: " << interactive << std::endl;

    // {
    //     std::cout << "set exposure 1000 iso 800" << std::endl;
    //     dai::CameraControl ctrl;
    //     ctrl.setManualExposure(1000, sensIso);
    //     controlQueue->send(ctrl);
    // }

    printf("Autoexposure enable\n");
    ROS_INFO_STREAM("Autoexposure enable");
    dai::CameraControl ctrl;
    ctrl.setAutoExposureEnable();
    controlQueue->send(ctrl);

    int default_saturation = 7;
    printf("Setting manual saturation: %d \n", default_saturation);
    ROS_INFO_STREAM("Setting manual saturation: " << default_saturation);
    ctrl.setSaturation(default_saturation);
    controlQueue->send(ctrl);

    // printf("Auto white-balance enable\n");
    // dai::CameraControl ctrl;
    // ctrl.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
    // controlQueue->send(ctrl);

    // ros::spin();

    ros::AsyncSpinner spinner(3);
    spinner.start();

    int counter = 0;

    while(ros::ok()) {
        ros::spinOnce();
        if(counter % 10 == 0) {
            counter = 0;
            std::cout << message << std::endl;
        }
        counter++;

        if(interactive) {
            // int key = cv::waitKey(1);   // requires an openCV window, doesn't work on input to console
            int key = getchar();
            // std::cout << "key: " << key << std::endl;

            if(key == 'e') {
                printf("Autoexposure enable\n");
                ROS_INFO_STREAM("Autoexposure enable\n");
                dai::CameraControl ctrl;
                ctrl.setAutoExposureEnable();
                controlQueue->send(ctrl);
            } else if(key == 'b') {
                printf("Auto white-balance enable\n");
                ROS_INFO_STREAM("Auto white-balance enable\n");
                dai::CameraControl ctrl;
                ctrl.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
                controlQueue->send(ctrl);
            } else if(key == 'i' || key == 'o' || key == 'k' || key == 'l') {
                if(key == 'i') expTime -= EXP_STEP;
                if(key == 'o') expTime += EXP_STEP;
                if(key == 'k') sensIso -= ISO_STEP;
                if(key == 'l') sensIso += ISO_STEP;
                expTime = std::max(expMin, std::min(expTime, expMax));
                sensIso = std::max(sensMin, std::min(sensIso, sensMax));
                printf("Setting manual exposure, time: %d, iso: %d\n", expTime, sensIso);
                ROS_INFO_STREAM("Setting manual exposure, time: " << expTime << " iso: " << sensIso << "\n");
                dai::CameraControl ctrl;
                ctrl.setManualExposure(expTime, sensIso);
                controlQueue->send(ctrl);
            } else if(key == '[' || key == ']') {
                if(key == '[') wbManual -= WB_STEP;
                if(key == ']') wbManual += WB_STEP;
                wbManual = std::max(wbMin, std::min(wbManual, wbMax));
                printf("Setting manual white balance, temperature: %d K\n", wbManual);
                ROS_INFO_STREAM("Setting manual white balance, temperature: " << wbManual << " K\n");
                dai::CameraControl ctrl;
                ctrl.setManualWhiteBalance(wbManual);
                controlQueue->send(ctrl);
            } else if(key == 'y' || key == 'u') {
                if(key == 'y') brightness -= GENERAL_STEP;
                if(key == 'u') brightness += GENERAL_STEP;
                brightness = std::max(brightnessMin, std::min(brightness, brightnessMax));
                printf("Setting manual brightness: %d \n", brightness);
                ROS_INFO_STREAM("Setting manual brightness: " << brightness << " \n");
                dai::CameraControl ctrl;
                ctrl.setBrightness(brightness);
                controlQueue->send(ctrl);
            } else if(key == 'h' || key == 'j') {
                if(key == 'h') contrast -= GENERAL_STEP;
                if(key == 'j') contrast += GENERAL_STEP;
                contrast = std::max(contrastMin, std::min(contrast, contrastMax));
                printf("Setting manual contrast: %d \n", contrast);
                ROS_INFO_STREAM("Setting manual contrast: " << contrast << " \n");
                dai::CameraControl ctrl;
                ctrl.setContrast(contrast);
                controlQueue->send(ctrl);
            } else if(key == 'n' || key == 'm') {
                if(key == 'n') saturation -= GENERAL_STEP;
                if(key == 'm') saturation += GENERAL_STEP;
                saturation = std::max(saturationMin, std::min(saturation, saturationMax));
                printf("Setting manual saturation: %d \n", saturation);
                ROS_INFO_STREAM("Setting manual saturation: " << saturation << " \n");
                dai::CameraControl ctrl;
                ctrl.setSaturation(saturation);
                controlQueue->send(ctrl);
            } else if(key == 'r' || key == 't') {
                if(key == 'r') sharpness -= GENERAL_STEP;
                if(key == 't') sharpness += GENERAL_STEP;
                sharpness = std::max(saturationMin, std::min(sharpness, saturationMax));
                printf("Setting manual sharpness: %d \n", sharpness);
                ROS_INFO_STREAM("Setting manual sharpness: " << sharpness << " \n");
                dai::CameraControl ctrl;
                ctrl.setSharpness(sharpness);
                controlQueue->send(ctrl);
            } else if(key == 'f' || key == 'g') {
                if(key == 'f') autoExpComp -= GENERAL_STEP;
                if(key == 'g') autoExpComp += GENERAL_STEP;
                autoExpComp = std::max(saturationMin, std::min(autoExpComp, saturationMax));
                printf("Setting manual autoExpComp: %d \n", autoExpComp);
                ROS_INFO_STREAM("Setting manual autoExpComp: " << autoExpComp << " \n");
                dai::CameraControl ctrl;
                ctrl.setAutoExposureCompensation(autoExpComp);
                controlQueue->send(ctrl);
            }
        }
    }
    return 0;
}
