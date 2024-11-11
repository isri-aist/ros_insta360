#include "../include/insta360/insta360Camera.h"
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <visp/vpHomogeneousMatrix.h>

float imageScale;

int main(int argc, char **argv){
    ros::init(argc, argv, "insta360");
    ros::NodeHandle n("~");

    ////READ LAUNCH PARAMS
    int cameraStreamFlag = n.param("cameraStreamFlag", 25);
    std::string dualCameraTopic = n.param<std::string>("dualCameraTopic", "/insta360/dual/image_raw");
    imageScale = n.param<float>("imageScale", 1.0);
    std::string rightCameraTopic = n.param<std::string>("rightCameraTopic", "/insta360/right/image_raw");
    std::string leftCameraTopic = n.param<std::string>("leftCameraTopic", "/insta360/left/image_raw");
    std::string rightCameraInfoTopic = n.param<std::string>("rightCameraInfoTopic", "/insta360/right/camera_info");
    std::string leftCameraInfoTopic = n.param<std::string>("leftCameraInfoTopic", "/insta360/left/camera_info");
    XmlRpc::XmlRpcValue rightCamCalib, leftCamCalib;
    n.getParam("/insta360/rightCameraCalib", rightCamCalib);
    n.getParam("/insta360/leftCameraCalib", leftCamCalib);
    int rightCameraWidth = static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["size"]["width"]);
    int rightCameraHeight = static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["size"]["height"]);
    int leftCameraWidth = static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["size"]["width"]);
    int leftCameraHeight = static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["size"]["height"]);
    std::vector<double>  rightCameraK = { static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["k"]["au"]),
                                          static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["k"]["av"]),
                                          static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["k"]["u0"]),
                                          static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["k"]["v0"])};
    std::vector<double>  leftCameraK = { static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["k"]["au"]),
                                         static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["k"]["av"]),
                                         static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["k"]["u0"]),
                                         static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["k"]["v0"])};
    std::vector<double>  rightCameraD = { static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["d"]["r1"]),
                                          static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["d"]["r2"]),
                                          static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["d"]["p1"]),
                                          static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["d"]["p2"]),
                                          static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["d"]["xi"])};
    std::vector<double>  leftCameraD = { static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["d"]["r1"]),
                                         static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["d"]["r2"]),
                                         static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["d"]["p1"]),
                                         static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["d"]["p2"]),
                                         static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["d"]["xi"])};
    vpHomogeneousMatrix crMref(vpTranslationVector(static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["pose"]["position"]["x"]),
                                                      static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["pose"]["position"]["y"]),
                                                      static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["pose"]["position"]["z"])),
                                vpQuaternionVector(static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["pose"]["orientation"]["x"]),
                                                      static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["pose"]["orientation"]["y"]),
                                                      static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["pose"]["orientation"]["z"]),
                                                      static_cast<XmlRpc::XmlRpcValue>(rightCamCalib["pose"]["orientation"]["w"])));
    vpHomogeneousMatrix clMref(vpTranslationVector(static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["pose"]["position"]["x"]),
                                                   static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["pose"]["position"]["y"]),
                                                   static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["pose"]["position"]["z"])),
                               vpQuaternionVector(static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["pose"]["orientation"]["x"]),
                                                  static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["pose"]["orientation"]["y"]),
                                                  static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["pose"]["orientation"]["z"]),
                                                  static_cast<XmlRpc::XmlRpcValue>(leftCamCalib["pose"]["orientation"]["w"])));


    ////PUBLISHERS
    ros::Publisher dualImgPub = n.advertise<sensor_msgs::Image>(dualCameraTopic, 30);
    ros::Publisher rightImgPub = n.advertise<sensor_msgs::Image>(rightCameraTopic, 30);
    ros::Publisher leftImgPub = n.advertise<sensor_msgs::Image>(leftCameraTopic, 30);
    ros::Publisher rightCameraInfoPub = n.advertise<sensor_msgs::CameraInfo>(rightCameraInfoTopic, 30);
    ros::Publisher leftCameraInfoPub = n.advertise<sensor_msgs::CameraInfo>(leftCameraInfoTopic, 30);
    ros::Rate loop_rate(100);

    ///CAMERA DRIVER
    insta360Camera camera((ins_camera::VideoResolution)cameraStreamFlag); // BEST FLAG SO FAR...

    ////BUILD ROS MESSAGES
    cv::Mat I, R, L, Io;
    cv::Mat II, LL, RR;
    sensor_msgs::ImagePtr Dmsg, Rmsg, Lmsg;
    sensor_msgs::CameraInfo RInfomsg, LInfomsg;

    RInfomsg.width = rightCameraWidth*imageScale;
    RInfomsg.height = rightCameraHeight*imageScale;
    RInfomsg.K = {rightCameraK[0]*imageScale, 0, rightCameraK[2]*imageScale, 0, rightCameraK[1]*imageScale, rightCameraK[3]*imageScale, 0 ,0 ,1 };
    RInfomsg.D = {rightCameraD[0],   rightCameraD[1],   rightCameraD[2],   rightCameraD[3], rightCameraD[4]};
    RInfomsg.distortion_model = "r1r2p1p2xi";
    RInfomsg.header.frame_id = "/camera_right_optical_frame";
    LInfomsg.width = leftCameraWidth*imageScale;
    LInfomsg.height = leftCameraHeight*imageScale;
    LInfomsg.K = {leftCameraK[0]*imageScale, 0, leftCameraK[2]*imageScale, 0, leftCameraK[1]*imageScale, leftCameraK[3]*imageScale, 0 ,0 ,1 };
    LInfomsg.D = {leftCameraD[0],   leftCameraD[1],   leftCameraD[2],   leftCameraD[3], leftCameraD[4]};
    LInfomsg.distortion_model = "r1r2p1p2xi";
    LInfomsg.header.frame_id = "/camera_left_optical_frame";

    int ij=0;
    for(int j=0;j<3;j++){
        for(int i=0;i<4;i++,ij++){
            RInfomsg.P[ij] = crMref[j][i];
            LInfomsg.P[ij] = clMref[j][i];
        }
    }


    ////SEND ROS MESSAGES
    while (ros::ok()) {
        II = camera.getRLImage();
        if(!II.empty()) 
        {
            //std::cout << "imageScale: " << imageScale << std::endl;
            //std::cout << II.cols << " " << II.rows << std::endl;
            cv::resize(II, I, cv::Size(), imageScale, imageScale);
            I.copyTo(Io);
        
            camera.extractLeftAndRightImages(I,L,R,true);
            ros::Time nowTime = ros::Time::now();
            RInfomsg.header.stamp = nowTime;
            LInfomsg.header.stamp = nowTime;
            
            /*
            cv::resize(LL, L, cv::Size(), imageScale, imageScale);
            cv::resize(RR, R, cv::Size(), imageScale, imageScale);
            cv::resize(Io, Io, cv::Size(), imageScale, imageScale);
            */
            //std::cout << L.cols << " " << R.cols << std::endl;
            //std::cout << L.rows << " " << R.rows << std::endl;

            L.copyTo(Io(cv::Rect(0,0,L.cols, L.rows)));
            R.copyTo(Io(cv::Rect(L.cols,0, R.cols, R.rows)));

            Dmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Io).toImageMsg();
            Dmsg->header.stamp = nowTime;
            Dmsg->header.frame_id = "/camera_dual_optical_frame";

            Rmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", R).toImageMsg();
            Rmsg->header.stamp = nowTime;
            Rmsg->header.frame_id = "/camera_right_optical_frame";

            Lmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", L).toImageMsg();
            Lmsg->header.stamp = nowTime;
            Lmsg->header.frame_id = "/camera_left_optical_frame";

            dualImgPub.publish(Dmsg);
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