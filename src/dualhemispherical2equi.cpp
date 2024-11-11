//
// Created by Nathan Crombez on 06/11/2023.
//

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include <per/prcommon.h>
#include <per/prOmni.h>
#include <per/prEquirectangular.h>
#include <per/prStereoModel.h>
#include <per/prStereoModelXML.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>
#include <visp_bridge/image.h>

#define INTERPTYPE prInterpType::IMAGEPLANE_NEARESTNEIGH

sensor_msgs::Image Emsg;
prEquirectangular equirectCamera;
ros::Publisher equiImagePub, equiCameraInfoPub;
vpImage<unsigned char> Mask, Maskk;

prOmni rightCamera, leftCamera;
prStereoModel dualCamera(2);
sensor_msgs::CameraInfo rightCameraInfo, leftCameraInfo, equiCameraInfo;
vpHomogeneousMatrix crMref, clMref;

double *coordMapping[4] = {NULL, NULL, NULL, NULL};

void camerasImageCallback(const sensor_msgs::Image::ConstPtr &Imsg);

float maskScale;

int main(int argc, char **argv) {
    ros::init(argc, argv, "dualhemispherical2equi");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(100);

    std::string dualCameraTopic = n.param<std::string>("dualCameraTopic", "/insta360/dual/image_raw");
    std::string rightCameraInfoTopic = n.param<std::string>("rightCameraInfoTopic", "/insta360/right/camera_info");
    std::string leftCameraInfoTopic = n.param<std::string>("leftCameraInfoTopic", "/insta360/left/camera_info");
    std::string equiImageTopic = n.param<std::string>("equiImageTopic", "/insta360/equi/image_raw");
    std::string equiCameraInfoTopic = n.param<std::string>("equiCameraInfoTopic", "/insta360/equi/camera_info");
    std::string pathToDualHemisphericalMask = n.param<std::string>("pathToDualHemisphericalMask",
                                                                   "/home/nathan/ROS/Noetic/catkin_ws/src/insta360/config/mask_insta360ONEX2_CIAD.png");
    maskScale = n.param<float>("maskScale", 1.0);

    rightCameraInfo = (*ros::topic::waitForMessage<sensor_msgs::CameraInfo>(rightCameraInfoTopic, n));
    leftCameraInfo = (*ros::topic::waitForMessage<sensor_msgs::CameraInfo>(leftCameraInfoTopic, n));
    vpImageIo::read(Maskk, pathToDualHemisphericalMask);
    Mask.resize(Maskk.getHeight()*maskScale, Maskk.getWidth()*maskScale, false);
    vpImageTools::resize(Maskk, Mask, vpImageTools::INTERPOLATION_AREA);
    //vpImageIo::write(Mask, pathToDualHemisphericalMask);

    std::cout << rightCameraInfo.K[0] << " " << rightCameraInfo.K[4] << " " << rightCameraInfo.K[2] << " " << rightCameraInfo.K[5] << std::endl;

    rightCamera.init(rightCameraInfo.K[0], rightCameraInfo.K[4], rightCameraInfo.K[2], rightCameraInfo.K[5],
                     rightCameraInfo.D[4], rightCameraInfo.D[0], rightCameraInfo.D[1], rightCameraInfo.D[2],
                     rightCameraInfo.D[3]);
    leftCamera.init(leftCameraInfo.K[0], leftCameraInfo.K[4], leftCameraInfo.K[2], leftCameraInfo.K[5],
                    leftCameraInfo.D[4], leftCameraInfo.D[0], leftCameraInfo.D[1], leftCameraInfo.D[2],
                    leftCameraInfo.D[3]);
    rightCamera.setDistorsions(false);
    leftCamera.setDistorsions(false);
    equirectCamera.setDistorsions(false);

    dualCamera.setSensor(0, &rightCamera);
    dualCamera.setSensor(1, &leftCamera);
    int ij = 0;
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 4; i++, ij++) {
            crMref[j][i] = rightCameraInfo.P[ij];
            clMref[j][i] = leftCameraInfo.P[ij];
        }
    }

    dualCamera.setsjMr(0, crMref);
    dualCamera.setsjMr(1, clMref);

    prPointFeature P;
    double Xs, Ys, Zs, u, v, dv, du, unmdv, unmdu;
    unsigned int icam, imWidth, imHeight;
    unsigned int i, j;
    imWidth = rightCameraInfo.width * 2;
    imHeight = rightCameraInfo.height;
    equirectCamera.init(imWidth * 0.5 / M_PI, imHeight * 0.5 / (M_PI * 0.5), imWidth * 0.5, imHeight * 0.5);

    for (unsigned int c = 0; c < 4; c++)
        coordMapping[c] = new double[imWidth * imHeight];

    // Compute the mapping
    double *pt_uer = coordMapping[0], *pt_ver = coordMapping[1], *pt_udf = coordMapping[2], *pt_vdf = coordMapping[3];

    for (unsigned int v_er = 0; v_er < imHeight; v_er++) {
        for (unsigned int u_er = 0; u_er < imWidth; u_er++, pt_uer++, pt_ver++, pt_udf++, pt_vdf++) {
            P.set_u(u_er);
            P.set_v(v_er);
            equirectCamera.pixelMeterConversion(P);
            equirectCamera.projectImageSphere(P, Xs, Ys, Zs);
            P.set_X(Xs);
            P.set_Y(Ys);
            P.set_Z(Zs);

            if (P.get_Z() > 0) {
                icam = 0;
            } else {
                icam = 1;
                P.sX = P.sX.changeFrame(clMref);
            }

            ((prOmni *) (dualCamera.sen[icam]))->project3DImage(P);
            ((prOmni *) (dualCamera.sen[icam]))->meterPixelConversion(P);

            u = P.get_u();
            v = P.get_v();

            if (icam == 0) {
                u += (imWidth / 2.0);
            }

            *pt_uer = u_er;
            *pt_ver = v_er;
            *pt_udf = u;
            *pt_vdf = v;
        }
    }

    ////PUBLISHERS
    equiImagePub = n.advertise<sensor_msgs::Image>(equiImageTopic, 30);
    equiCameraInfoPub = n.advertise<sensor_msgs::CameraInfo>(equiCameraInfoTopic, 30);

    ////SUBSCRIBER
    ros::Subscriber dualCameraSub;
    dualCameraSub = n.subscribe(dualCameraTopic, 1, &camerasImageCallback);

    ros::spin();

    return 0;
}


void camerasImageCallback(const sensor_msgs::Image::ConstPtr &Imsg) {
    vpImage<vpRGBa> I(visp_bridge::toVispImageRGBa(*Imsg)), E;
    double u, v, dv, du, unmdv, unmdu;
    unsigned int imWidth, imHeight;
    unsigned int i, j;
    vpRGBa curPix, *pt_bitmap_er;

    imWidth = I.getWidth();
    imHeight = I.getHeight();

    //std::cout << imWidth << " " << imHeight << std::endl;

    E.resize(imHeight, imWidth);
    equirectCamera.init(imWidth * 0.5 / M_PI, imHeight * 0.5 / (M_PI * 0.5), imWidth * 0.5, imHeight * 0.5);

    ros::Time nowTime = ros::Time::now();
    equiCameraInfo.header.stamp = nowTime;
    equiCameraInfo.width = imWidth;
    equiCameraInfo.height = imHeight;
    equiCameraInfo.header.frame_id = "/camera_right_optical_frame";
    equiCameraInfo.distortion_model = "equirectangular";

    //std::cout << imWidth << " " << imHeight << std::endl;

    //Dual hemispherical to equirectangular
    pt_bitmap_er = E.bitmap;
    double *pt_vdf = coordMapping[3];
    double *pt_udf = coordMapping[2];

    for (unsigned int pixIndex = 0; pixIndex < imWidth * imHeight; pixIndex++, pt_bitmap_er++, pt_udf++, pt_vdf++) {
        u = *(pt_udf);
        v = *(pt_vdf);
//std::cout << u << " " << v << std::endl;
        if ((u >= 0) && (v >= 0) && (u < (imWidth - 1)) && (v < (imHeight - 1))) {
            switch (INTERPTYPE) {
                case IMAGEPLANE_BILINEAR:
                    curPix.R = curPix.G = curPix.B = 0;

                    i = (int) v;
                    dv = v - i;
                    unmdv = 1.0 - dv;
                    j = (int) u;
                    du = u - j;
                    unmdu = 1.0 - du;


                    if (Mask[i][j] != 0) {
                        curPix = curPix + I[i][j] * unmdv * unmdu;
                    }

                    if (Mask[i + 1][j] != 0) {
                        curPix = curPix + I[i + 1][j] * dv * unmdu;
                    }

                    if (Mask[i][j + 1] != 0) {
                        curPix = curPix + I[i][j + 1] * unmdv * du;
                    }

                    if (Mask[i + 1][j + 1] != 0) {
                        curPix = curPix + I[i + 1][j + 1] * dv * du;
                    }

                    (*pt_bitmap_er).R = curPix.B;
                    (*pt_bitmap_er).G = curPix.G;
                    (*pt_bitmap_er).B = curPix.R;
                    (*pt_bitmap_er).A = curPix.A;

                    break;
                case IMAGEPLANE_NEARESTNEIGH:
                default:

                    i = vpMath::round(v);
                    j = vpMath::round(u);

                    if (Mask[i][j] != 0) 
                    {
                        (*pt_bitmap_er).R = I[i][j].B;
                        (*pt_bitmap_er).G = I[i][j].G;
                        (*pt_bitmap_er).B = I[i][j].R;
                        (*pt_bitmap_er).A = I[i][j].A;
                    }

                    break;

            }
        }
    }

    Emsg = visp_bridge::toSensorMsgsImage(E);
    Emsg.header = Imsg->header;
    Emsg.header.frame_id = "/camera_equi_optical_frame";

    equiImagePub.publish(Emsg);
    equiCameraInfoPub.publish(equiCameraInfo);
}



//Callback avec le calcul du mapping
/*
void camerasImageCallback(const sensor_msgs::Image::ConstPtr& Imsg){
    vpImage<vpRGBa> I(visp_bridge::toVispImageRGBa(*Imsg)), E;
    prPointFeature P;
    double Xs, Ys, Zs, u, v, dv, du, unmdv, unmdu;
    unsigned int icam, imWidth, imHeight;
    unsigned int i, j;
    vpRGBa curPix, *pt_bitmap_er;

    vpHomogeneousMatrix cMc0(0,0,0,0,0,0);

    imWidth = I.getWidth();
    imHeight = I.getHeight();
    E.resize(imHeight, imWidth);
    equirectCamera.init(imWidth * 0.5 / M_PI, imHeight * 0.5 / (M_PI * 0.5), imWidth * 0.5, imHeight * 0.5);

    //Dual hemispherical to equirectangular
    pt_bitmap_er = E.bitmap;
    for(unsigned int v_er = 0 ; v_er < imHeight ; v_er++){
        for(unsigned int u_er = 0 ; u_er < imWidth ; u_er++, pt_bitmap_er++){
            P.set_u(u_er);
            P.set_v(v_er);
            equirectCamera.pixelMeterConversion(P);
            equirectCamera.projectImageSphere(P, Xs, Ys, Zs);
            P.set_X(Xs);
            P.set_Y(Ys);
            P.set_Z(Zs);
            if(P.get_Z() > 0){
                icam = 0;
            }
            else{
                icam = 1;
                P.sX = P.sX.changeFrame(clMref);
            }

            ((prOmni *)(dualCamera.sen[icam]))->project3DImage(P);
            ((prOmni *)(dualCamera.sen[icam]))->meterPixelConversion(P);

            u = P.get_u();
            v = P.get_v();

            if(icam == 1){
                u+=(imWidth/2);
            }

            if((u >= 0) && (v >= 0) && (u < (imWidth-1)) && (v < (imHeight-1))){
                switch(INTERPTYPE){
                    case IMAGEPLANE_BILINEAR:
                        curPix.R = curPix.G = curPix.B = 0;

                        i = (int)v; dv = v-i; unmdv = 1.0-dv;
                        j = (int)u; du = u-j; unmdu = 1.0-du;


                        if (Mask[i][j] != 0){
                            curPix = curPix + I[i][j]*unmdv*unmdu;
                        }

                        if (Mask[i+1][j] != 0){
                            curPix = curPix + I[i+1][j]*dv*unmdu;
                        }

                        if (Mask[i][j+1] != 0){
                            curPix = curPix + I[i][j+1]*unmdv*du;
                        }

                        if (Mask[i+1][j+1] != 0){
                            curPix = curPix + I[i+1][j+1]*dv*du;
                        }

                        (*pt_bitmap_er).R = curPix.B;
                        (*pt_bitmap_er).G = curPix.G;
                        (*pt_bitmap_er).B = curPix.R;
                        (*pt_bitmap_er).A = curPix.A;

                        break;
                    case IMAGEPLANE_NEARESTNEIGH:
                    default:

                        i = vpMath::round(v);
                        j = vpMath::round(u);

                        if(Mask[i][j] != 0){
                            (*pt_bitmap_er).R = I[i][j].B;
                            (*pt_bitmap_er).G = I[i][j].G;
                            (*pt_bitmap_er).B = I[i][j].R;
                            (*pt_bitmap_er).A = I[i][j].A;
                        }

                        break;

                }
            }
        }
    }

    Emsg = visp_bridge::toSensorMsgsImage(E);
    Emsg.header = Imsg->header;
    Emsg.header.frame_id = "/camera_equi_optical_frame";

    equiImagePub.publish(Emsg);
}
 */
