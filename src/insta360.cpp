#include "../include/insta360/insta360Camera.h"
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "insta360");
    ros::NodeHandle n("~");

    ////READ LAUNCH PARAMS
    int cameraStreamFlag = n.param("cameraStreamFlag", 25);
    std::string rightCameraTopic = n.param<std::string>("rightCameraTopic", "/insta360/right/image_raw");
    std::string leftCameraTopic = n.param<std::string>("leftCameraTopic", "/insta360/left/image_raw");
    std::string rightCameraInfoTopic = n.param<std::string>("rightCameraInfoTopic", "/insta360/right/camera_info");
    std::string leftCameraInfoTopic = n.param<std::string>("leftCameraInfoTopic", "/insta360/left/camera_info");
    int rightCameraWidth = n.param("rightCameraWidth", 1152);
    int rightCameraHeight = n.param("rightCameraHeight", 560);
    int leftCameraWidth = n.param("leftCameraWidth", 1152);
    int leftCameraHeight = n.param("leftCameraHeight", 560);
    std::vector<double>  rightCameraK = { n.param("rightCameraK_au", 900.0),
                                       n.param("rightCameraK_av", 900.0),
                                       n.param("rightCameraK_u0", 550.0),
                                       n.param("rightCameraK_v0", 550.0)};
    std::vector<double>  leftCameraK = { n.param("leftCameraK_au", 900.0),
                                              n.param("leftCameraK_av", 900.0),
                                              n.param("leftCameraK_u0", 550.0),
                                              n.param("leftCameraK_v0", 550.0)};
    std::vector<double>  rightCameraD = { n.param("rightCameraD_r1", 0.0),
                                               n.param("rightCameraD_r2", 0.0),
                                               n.param("rightCameraD_p1", 0.0),
                                               n.param("rightCameraD_p2", 0.0),
                                               n.param("rightCameraD_xi", 1.5)};
    std::vector<double>  leftCameraD = { n.param("leftCameraD_r1", 0.0),
                                              n.param("leftCameraD_r2", 0.0),
                                              n.param("leftCameraD_p1", 0.0),
                                              n.param("leftCameraD_p2", 0.0),
                                              n.param("leftCameraD_xi", 1.5)};

    ////PUBLISHERS
    ros::Publisher rightImgPub = n.advertise<sensor_msgs::Image>(rightCameraTopic, 30);
    ros::Publisher leftImgPub = n.advertise<sensor_msgs::Image>(leftCameraTopic, 30);
    ros::Publisher rightCameraInfoPub = n.advertise<sensor_msgs::CameraInfo>(rightCameraInfoTopic, 30);
    ros::Publisher leftCameraInfoPub = n.advertise<sensor_msgs::CameraInfo>(leftCameraInfoTopic, 30);
    ros::Rate loop_rate(30);

    ///CAMERA DRIVER
    insta360Camera camera((ins_camera::VideoResolution)cameraStreamFlag); // BEST FLAG SO FAR...

    ////BUILD ROS MESSAGES
    cv::Mat I, R, L;
    sensor_msgs::ImagePtr Rmsg, Lmsg;
    sensor_msgs::CameraInfo RInfomsg, LInfomsg;

    RInfomsg.width = rightCameraWidth;
    RInfomsg.height = rightCameraHeight;
    RInfomsg.K = {rightCameraK[0], 0, rightCameraK[2], 0, rightCameraK[1], rightCameraK[3], 0 ,0 ,1 };
    RInfomsg.D = {rightCameraD[0],   rightCameraD[1],   rightCameraD[2],   rightCameraD[3], rightCameraD[4]};
    RInfomsg.distortion_model = "r1r2p1p2xi";
    RInfomsg.header.frame_id = "/camera_right_optical_frame";

    LInfomsg.width = leftCameraWidth;
    LInfomsg.height = leftCameraHeight;
    LInfomsg.K = {leftCameraK[0], 0, leftCameraK[2], 0, leftCameraK[1], leftCameraK[3], 0 ,0 ,1 };
    LInfomsg.D = {leftCameraD[0],   leftCameraD[1],   leftCameraD[2],   leftCameraD[3], leftCameraD[4]};
    LInfomsg.distortion_model = "r1r2p1p2xi";
    LInfomsg.header.frame_id = "/camera_left_optical_frame";

    ////SEND ROS MESSAGES
    while (ros::ok()) {
        I = camera.getRLImage();
        if(!I.empty()) {
            camera.extractLeftAndRightImages(I,L,R,true);
            ros::Time nowTime = ros::Time::now();
            RInfomsg.header.stamp = nowTime;
            LInfomsg.header.stamp = nowTime;

            Rmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", R).toImageMsg();
            Rmsg->header.stamp = nowTime;
            Rmsg->header.frame_id = "/camera_right_optical_frame";

            Lmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", L).toImageMsg();
            Lmsg->header.stamp = nowTime;
            Lmsg->header.frame_id = "/camera_left_optical_frame";

            rightCameraInfoPub.publish(RInfomsg);
            leftCameraInfoPub.publish(LInfomsg);
            rightImgPub.publish(Rmsg);
            leftImgPub.publish(Lmsg);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}