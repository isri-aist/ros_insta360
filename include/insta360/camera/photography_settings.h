#pragma once
#include <memory>
#include <unordered_map>
#include <vector>
#include "ins_types.h"

namespace ins_camera {

    struct EnumClassHash {
        template<typename T>
        std::size_t operator()(T const& v) const noexcept {
            return static_cast<std::size_t>(v);
        }
    };

    enum CameraFunctionMode {
		FUNCTION_MODE_NORMAL = 0,
		FUNCTION_MODE_LIVE_STREAM = 1,
        FUNCTION_MODE_NORMAL_IMAGE = 6,
        FUNCTION_MODE_NORMAL_VIDEO = 7
    };

	enum CaptureStatus  {
		NOT_CAPTURE = 0,
		NORMAL_CAPTURE = 1,
		TIMELAPSE_CAPTURE = 2,
		INTERVAL_SHOOTING_CAPTURE = 3,
		SINGLE_SHOOTING = 4,
		HDR_SHOOTING = 5,
		SELF_TIMER_SHOOTING = 6,
		BULLET_TIME_CAPTURE = 7,
		SETTINGS_NEW_VALUE = 8,
		HDR_CAPTURE = 9,
		BURST_SHOOTING = 10,
		STATIC_TIMELAPSE_SHOOTING = 11,
		INTERVAL_VIDEO_CAPTURE = 12,
		TIMESHIFT_CAPTURE = 13,
		AEB_NIGHT_SHOOTING = 14,
		SINGLE_POWER_PANO_SHOOTING = 15,
		HDR_POWER_PANO_SHOOTING = 16,
		SUPER_NORMAL_CAPTURE = 17,
		LOOP_RECORDING_CAPTURE = 18,
		STARLAPSE_SHOOTING = 19,
	};

    enum VideoResolution {
        RES_3840_1920P30 = 0,	//YES
        RES_2560_1280P30 = 1,	//YES
        RES_1920_960P30 = 2,	//YES
        RES_2560_1280P60 = 3,
        RES_2048_512P120 = 4,
        RES_3328_832P60 = 5,
        RES_3072_1536P30 = 6,
        RES_2240_1120P30 = 7,
        RES_2240_1120P24 = 8,
        RES_1440_720P30 = 9,	//YES
        RES_2880_2880P30 = 10,
        RES_3840_1920P60 = 11,
        RES_3840_1920P50 = 12,
        RES_3008_1504P100 = 13,
        RES_960_480P30 = 14,
        RES_3040_1520P30 = 15,
        RES_2176_1088P30 = 16,
        RES_720_360P30 = 17,	//NO
        RES_480_240P30 = 18,
        RES_2880_2880P25 = 19,
        RES_2880_2880P24 = 20,
        RES_3840_1920P20 = 21,
        RES_1920_960P20 = 22,
        RES_3840_2160p60 = 23,
        RES_3840_2160p30 = 24,
        RES_2720_1530p100 = 25,
        RES_1920_1080p200 = 26,
        RES_1920_1080p240 = 27,
        RES_1920_1080p120 = 28,
        RES_1920_1080p30 = 29,
        RES_5472_3078p30 = 30,
        RES_4000_3000p30 = 31,
        RES_854_640P30 = 32,
        RES_720_406P30 = 33,
        RES_424_240P15 = 34,	//NO
        RES_1024_512P30 = 35,
        RES_640_320P30 = 36,	//NO
        RES_5312_2988P30 = 37,
        RES_2720_1530P60 = 38,
        RES_2720_1530P30 = 39,
        RES_1920_1080P60 = 40,
        RES_2720_2040P30 = 41,
        RES_1920_1440P30 = 42,
        RES_1280_720P30 = 43,
        RES_1280_960P30 = 44,
        RES_1152_768P30 = 45,
        RES_5312_2988P25 = 46,
        RES_5312_2988P24 = 47,
        RES_3840_2160P25 = 48,
        RES_3840_2160P24 = 49,
        RES_2720_1530P25 = 50,
        RES_2720_1530P24 = 51,
        RES_1920_1080P25 = 52,
        RES_1920_1080P24 = 53,
        RES_4000_3000P25 = 54,
        RES_4000_3000P24 = 55,
        RES_2720_2040P25 = 56,
        RES_2720_2040P24 = 57,
        RES_1920_1440P25 = 58,
        RES_1920_1440P24 = 59
    };

 
    enum PhotographyOptions_ExposureMode {
        PhotographyOptions_ExposureOptions_Program_AUTO = 0,
        PhotographyOptions_ExposureOptions_Program_ISO_PRIORITY = 1,
        PhotographyOptions_ExposureOptions_Program_SHUTTER_PRIORITY = 2,
        PhotographyOptions_ExposureOptions_Program_MANUAL = 3
    };

    enum PhotographyOptions_WhiteBalance {
        PhotographyOptions_WhiteBalance_WB_UNKNOWN = -1,
        PhotographyOptions_WhiteBalance_WB_AUTO = 0,
        PhotographyOptions_WhiteBalance_WB_2700K = 1,
        PhotographyOptions_WhiteBalance_WB_4000K = 2,
        PhotographyOptions_WhiteBalance_WB_5000K = 3,
        PhotographyOptions_WhiteBalance_WB_6500K = 4,
        PhotographyOptions_WhiteBalance_WB_7500K = 5
    };

    class ExposureSettingsPrivate;
    class CAMERASDK_API ExposureSettings {
    public:
        friend class Camera;
        ExposureSettings();
        /**
         * \brief set iso value,this value will only take effect in 
         *   manual and PhotographyOptions_ExposureMode::PhotographyOptions_ExposureOptions_Program_ISO_PRIORITY mode.
         * \param value int value representing iso. available values are like 100,400,800,1600,etc.
         */
        void SetIso(int32_t value);
        /**
         * \brief set shutter,this value will only take effect in manual 
         *  and PhotographyOptions_ExposureMode::PhotographyOptions_ExposureOptions_Program_SHUTTER_PRIORITY mode.
         * \param value double value representing shutter speed in second. available values are like 1/30,1/60,1/120,etc.
         */
        void SetShutterSpeed(double speed);
        void SetExposureMode(PhotographyOptions_ExposureMode mode);
        void SetEVBias(int32_t value);

        int32_t Iso() const;
        double ShutterSpeed() const;
        PhotographyOptions_ExposureMode ExposureMode() const;
        int32_t EVBias() const;

    private:
        std::shared_ptr<ExposureSettingsPrivate> private_impl_;
    };
    /**
     * \class CaptureSettings
     * \brief a class wrapping capture settings. CaptureSettings holds the temperary settings value
     * you want to apply to the camera. The exactly settings to be applied are implied by a list of
     * SettingType,which you may get by calling GetSettingTypes(). The types will be automatically recorded
     * when you call any SetXXX() function. However, you can also ResetSettingTypes() or UpdateSettingTypes() mannually.
     * 
     * ## available settings and value range:
     * - Contrast: 0~256, default 64
     * - Saturation:0~256, default 64
     * - Brightness:-256~256, default 0
     * - WhiteBalance: see #PhotographyOptions_WhiteBalance
     * - Sharpness: 0~6, default 3
     */
    class CAMERASDK_API CaptureSettings {
    public:
        
        CaptureSettings() = default;
        CaptureSettings(const CaptureSettings& other);
        enum SettingsType {
            CaptureSettings_Contrast = 0,
            CaptureSettings_Saturation,
            CaptureSettings_Brightness,
            CaptureSettings_Sharpness,
            CaptureSettings_WhiteBalance
        };
        

        /**
         * \brief get a list of settings types you have set. Any settings not included in this list 
         *   will not be applied to the camera.
         * \return a list of SettingsType that will be applied
         */
        std::vector<SettingsType> GetSettingTypes() const;

        /**
         * \brief set setting types to be applied mannually.
         */
        void UpdateSettingTypes(std::vector<SettingsType>& types);

        /**
         * \brief reset setting types, any types recorded when you call SetXXX() 
         * previously will not be applied until you call SetXXX next time.
         * however, the value already be applied will be kept
         */
        void ResetSettingTypes();
         
        void SetValue(SettingsType type, int32_t value,bool apply_to_camera = true);
        void SetWhiteBalance(PhotographyOptions_WhiteBalance wb,bool apply_to_camera = true);
        int32_t GetIntValue(SettingsType type) const;

        PhotographyOptions_WhiteBalance WhiteBalance();

    private:
        std::unordered_map<SettingsType, int32_t,EnumClassHash> int_values_;
        std::vector<SettingsType> types_;
    };

    struct RecordParams {
        VideoResolution resolution;
        int32_t bitrate;
    };

    struct LiveStreamParam {
        bool enable_audio = false;
        bool enable_video = true;
        uint32_t audio_samplerate = 48000;
        uint32_t audio_bitrate = 128000;
        uint32_t video_bitrate = 1024 * 1024 * 10;
        VideoResolution video_resolution;
        uint32_t lrv_video_bitrate = 1024 * 1024 * 1;
        VideoResolution lrv_video_resulution;
        bool enable_gyro = false;
        bool using_lrv = false;
    };
}


