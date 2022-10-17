#include "../include/insta360/insta360Camera.h"
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "insta360");
    ros::NodeHandle n;
    ros::Publisher rightImgPub = n.advertise<sensor_msgs::Image>("/insta360/right/image_raw", 30);
    ros::Publisher leftImgPub = n.advertise<sensor_msgs::Image>("/insta360/left/image_raw", 30);
    ros::Publisher rightCameraInfoPub = n.advertise<sensor_msgs::CameraInfo>("/insta360/right/camera_info", 30);
    ros::Publisher leftCameraInfoPub = n.advertise<sensor_msgs::CameraInfo>("/insta360/left/camera_info", 30);
    ros::Rate loop_rate(30);

    insta360Camera camera((ins_camera::VideoResolution)25);
    cv::Mat I, R, L;
    sensor_msgs::ImagePtr Rmsg, Lmsg;
    sensor_msgs::CameraInfo RInfomsg, LInfomsg;

    ////TODO: read from params file
    RInfomsg.width = 1152;
    RInfomsg.height = 1152;
    RInfomsg.K = {587.24019, 0, 575.36742, 0, 589.09648, 580.39315, 0 ,0 ,1 };
    RInfomsg.D = {-0.23127,   0.03318,   -0.00079,   -0.00058,  0.72458};
    RInfomsg.distortion_model = "r1r2p1p2xi";
    RInfomsg.header.frame_id = "/camera_right_optical_frame";

    LInfomsg.width = 1152;
    LInfomsg.height = 1152;
    LInfomsg.K = {589.44525 , 0, 567.75937, 0, 591.69373,  576.11049, 0 ,0 ,1 };
    LInfomsg.D = {-0.22796,   0.03118,   0.00034,   0.00057,  0.74222};
    LInfomsg.distortion_model = "r1r2p1p2xi";
    LInfomsg.header.frame_id = "/camera_left_optical_frame";

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