//
// Created by Nathan Crombez on 24/05/2022.
//

#include "insta360Camera.h"

insta360Camera::insta360Camera(ins_camera::VideoResolution res) {
    std::cout << "begin open camera" << std::endl;

    auto list = discovery.GetAvailableDevices();
    if (list.size() <= 0) {
        std::cerr << "no device found." << std::endl;
        return;
    }

    cam = std::make_shared<ins_camera::Camera>(list[0].info);
    if (!cam->Open()) {
        std::cerr << "failed to open camera" << std::endl;
        return;
    }

    cam->StopLiveStreaming();

    delegate = std::make_shared<MyStreamDelegate>();
    cam->SetStreamDelegate(delegate);
    discovery.FreeDeviceDescriptors(list);
    std::cout << "Succeed to open camera..." << std::endl;

    ins_camera::LiveStreamParam param;
    param.video_resolution = res;
    param.lrv_video_resulution = res;
    param.using_lrv = true;

    if(cam->StartLiveStreaming(param)) {
        std::cout << "successfully started live stream" << std::endl ;
    }


}


insta360Camera::~insta360Camera() {
    if(cam->StopLiveStreaming()) {
        std::cout << "successfully stopped live stream" << std::endl ;
    } else {
        std::cerr << "failed to stop live." << std::endl;
    }
    cam->Close();
}

void insta360Camera::displayInfo(){
    auto list = discovery.GetAvailableDevices();
    for(int i = 0;i < list.size(); ++i) {
        auto desc = list[i];
        std::cout << "serial:" << desc.serial_number << "\t"
                  << "camera type:" << int(desc.camera_type) << "\t"
                  << "lens type:" << int(desc.lens_type) << std::endl;
    }
    std::cout << "http base url:" << cam->GetHttpBaseUrl() << std::endl;
}


cv::Mat insta360Camera::getRLImage(){
    return ((MyStreamDelegate*)delegate.get())->getImage();
}

void insta360Camera::extractLeftAndRightImages(cv::Mat I, cv::Mat &L, cv::Mat &R, bool reoriented){

    if(reoriented){
        cv::rotate(I(cv::Rect(0, 0,static_cast<int>(I.size().width / 2), I.size().height)) , R,0);
        cv::rotate( I(cv::Rect(static_cast<int>(I.size().width / 2), 0,static_cast<int>(I.size().width / 2), I.size().height)), L , 2);
    }else{
        I(cv::Rect(0, 0,static_cast<int>(I.size().width / 2), I.size().height));
        I(cv::Rect(static_cast<int>(I.size().width / 2), 0,static_cast<int>(I.size().width / 2), I.size().height));
    }

}


