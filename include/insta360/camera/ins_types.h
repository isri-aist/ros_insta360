#pragma once
#include <string>
#include <vector>

#define CAMERASDK_API
#if WIN32
	#ifdef USE_EXPORTDLL
		#define CAMERASDK_API _declspec(dllexport)
	#else
		#define CAMERASDK_API _declspec(dllimport)
	#endif
#endif

namespace ins_camera {
enum class CameraType {
    Unknown,
    Insta360OneX,
    Insta360OneR,
	Insta360OneX2
};

enum class CameraLensType {
    PanoDefault,
    Wide577,
    Pano577,
    WIDE283
};

enum class ConnectionType {
    USB,
    Wifi,
    Bluetooth
};

enum class VideoEncodeType {
	H264,
	H265
};

struct DeviceConnectionInfo {
    ConnectionType connection_type;
    void* native_connection_info;
};


struct DeviceDescriptor {
    CameraType camera_type;
    CameraLensType lens_type;
    std::string serial_number;
    DeviceConnectionInfo info;
};
    
class CAMERASDK_API MediaUrl {
public:
    MediaUrl(const std::vector<std::string>& uris,const std::vector<std::string>& lrv_uris = std::vector<std::string>());
    bool Empty() const;
    bool IsSingleOrigin() const;
    bool IsSingleLRV() const;
    std::string GetSingleOrigin() const;
    std::string GetSingleLRV() const;
    const std::vector<std::string>& OriginUrls() const;
    const std::vector<std::string>& LRVUrls() const;
private:
    std::vector<std::string> uris_;
    std::vector<std::string> lrv_uris_;
};

}
