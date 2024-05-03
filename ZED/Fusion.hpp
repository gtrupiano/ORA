/*
 * SOFTWARE LICENSE
 * BY USING YOUR CAMERA YOU AGREE TO THIS SOFTWARE LICENSE. BEFORE SETTING IT UP,
 * PLEASE READ THIS SOFTWARE LICENSE CAREFULLY. IF YOU DO NOT ACCEPT THIS
 * SOFTWARE LICENSE, DO NOT USE YOUR CAMERA. RETURN IT TO UNUSED TO STEREOLABS
 * FOR A REFUND. Contact STEREOLABS at support@stereolabs.com
 *
 * 1. Definitions
 *
 * "Authorized Accessory" means a STEREOLABS branded ZED, ZED 2, ZED 2i, ZED Mini, ZED X, ZED X Mini, and a STEREOLABS
 * licensed, third party branded, ZED hardware accessory whose packaging bears the official
 * "Licensed for ZED" logo. The ZED, ZED 2, ZED 2i, ZED Mini, ZED X, ZED X Mini camera are Authorized Accessories
 * solely for purpose of this Software license.
 * "Software" means the Software Development Kit, available on the stereolabs.com website, and including any updates STEREOLABS may make available from
 * time to time.
 * "Unauthorized Accessories" means all hardware accessories other than an Authorized Accessory.
 * "Unauthorized Software" means any software not distributed by STEREOLABS.
 * "You" means the user of a ZED, ZED 2, ZED 2i, ZED Mini, ZED X, ZED X Mini camera.
 *
 * 2. License
 *
 * a. The Software is licensed to You, not sold. You are licensed to use the
 * Software only as downloaded from the stereolabs.com website, and updated by
 * STEREOLABS from time to time. You may not copy or reverse engineer the Software.
 *
 * b. As conditions to this Software license, You agree that:
 *   i. You will use Your Software with ZED, ZED 2, ZED 2i, ZED Mini, ZED X, ZED X Mini camera only and not with any
 *      other device (including). You will not use Unauthorized Accessories. They may
 *      not work or may stop working permanently after a Software update.
 *   ii. You will not use or install any Unauthorized Software with an Authorized Accessory. If You do, Your ZED, ZED 2,
 *      ZED Mini, ZED X camera may stop working permanently at that time or after a later
 *       Software update.
 *   iii. You will not attempt to defeat or circumvent any Software technical limitation,
 *        security, or anti-piracy system. If You do, Your ZED, ZED 2, ZED 2i, ZED Mini, ZED X, ZED X Mini camera may stop
 *        working permanently at that time or after a later Software update.
 *   iv. STEREOLABS may use technical measures, including Software updates, to limit use
 *       of the Software to the ZED, ZED 2, ZED 2i, ZED Mini, ZED X, ZED X Mini camera, to prevent use of Unauthorized
 *       Accessories, and to protect the technical limitations, security and anti-piracy
 *       systems in the ZED, ZED 2, ZED 2i, ZED Mini, ZED X, ZED X Mini camera.
 *   v. STEREOLABS may update the Software from time to time without further notice to You,
 *      for example, to update any technical limitation, security, or anti-piracy system.
 *
 * 3. Warranty
 *
 * The Software is covered by the Limited Warranty for Your ZED, ZED 2, ZED 2i, ZED Mini, ZED X, ZED X Mini camera, and
 * STEREOLABS gives no other guarantee, warranty, or condition for the Software. No one
 * else may give any guarantee, warranty, or condition on STEREOLABS's behalf.
 *
 * 4. EXCLUSION OF CERTAIN DAMAGES
 *
 * STEREOLABS IS NOT RESPONSIBLE FOR ANY INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL
 * DAMAGES; ANY LOSS OF DATA, PRIVACY, CONFIDENTIALITY, OR PROFITS; OR ANY INABILITY TO
 * USE THE SOFTWARE. THESE EXCLUSIONS APPLY EVEN IF STEREOLABS HAS BEEN ADVISED OF THE
 * POSSIBILITY OF THESE DAMAGES, AND EVEN IF ANY REMEDY FAILS OF ITS ESSENTIAL PURPOSE.
 *
 * 5. Choice of Law
 *
 * French law governs the interpretation of this Software license and any claim that
 * STEREOLABS has breached it, regardless of conflict of law principles.
 *
 */

#ifndef __FUSION_HPP__
#define __FUSION_HPP__

#include <sl/Camera.hpp>

// Stereolabs namespace
namespace sl {

    /**
      \class FusionConfiguration
      \ingroup Fusion_group
      \brief Useful struct to store the Fusion configuration, can be read from /write to a JSON file.
     */
    struct FusionConfiguration {
        /**
        \brief The serial number of the used ZED camera.
         */
        int serial_number;

        /**
        \brief The communication parameters to connect this camera to the Fusion.
         */
        CommunicationParameters communication_parameters;

        /**
        \brief The WORLD Pose of the camera for Fusion.
         */
        Transform pose;

        /**
        \brief The input type for the current camera.
         */
        InputType input_type;
    };

    /**
    \ingroup Fusion_group
    \brief Read a configuration JSON file to configure a fusion process.
    \param json_config_filename : The name of the JSON file containing the configuration.
    \param serial_number : The serial number of the ZED Camera you want to retrieve.
    \param coord_sys : The COORDINATE_SYSTEM in which you want the World Pose to be in.
    \param unit : The UNIT in which you want the World Pose to be in.

    \return A \ref FusionConfiguration for the requested camera.
    \note Empty if no data were found for the requested camera.
     */
    FusionConfiguration /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ readFusionConfigurationFile(std::string json_config_filename, int serial_number, sl::COORDINATE_SYSTEM coord_sys, sl::UNIT unit);

    /**
    \ingroup Fusion_group
    \brief Read a Configuration JSON file to configure a fusion process.
    \param json_config_filename : The name of the JSON file containing the configuration.
    \param coord_sys : The COORDINATE_SYSTEM in which you want the World Pose to be in.
    \param unit : The UNIT in which you want the World Pose to be in.

    \return A vector of \ref FusionConfiguration for all the camera present in the file.
    \note Empty if no data were found for the requested camera.
     */
    std::vector<FusionConfiguration> /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ readFusionConfigurationFile(std::string json_config_filename, sl::COORDINATE_SYSTEM coord_sys, sl::UNIT unit);

    /**
    \ingroup Fusion_group
    \brief Write a Configuration JSON file to configure a fusion process.
    \param json_config_filename : The name of the JSON that will contain the information.
    \param conf: A vector of \ref FusionConfiguration listing all the camera configurations.
    \param coord_sys : The COORDINATE_SYSTEM in which the World Pose is.
    \param unit : The UNIT in which the World Pose is.
     */
    void /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ writeConfigurationFile(std::string json_config_filename, std::vector<FusionConfiguration> &configuration, sl::COORDINATE_SYSTEM coord_sys, sl::UNIT unit);

    /**
    \class InitFusionParameters
    \ingroup Fusion_group
    \brief Holds the options used to initialize the \ref Fusion object.
     */
    struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ InitFusionParameters {
        /**
         * @brief This parameter allows you to select the unit to be used for all metric values of the SDK (depth, point cloud, tracking, mesh, and others).
         * 
         * Default : \ref UNIT "UNIT::MILLIMETER"
         */
        UNIT coordinate_units;

        /**
         * @brief Positional tracking, point clouds and many other features require a given \ref COORDINATE_SYSTEM to be used as reference.
         * This parameter allows you to select the \ref COORDINATE_SYSTEM used by the \ref Camera to return its measures.
         * 
         * This defines the order and the direction of the axis of the coordinate system.
         * \n Default : \ref COORDINATE_SYSTEM "COORDINATE_SYSTEM::IMAGE"
         */
        COORDINATE_SYSTEM coordinate_system;

        /**
         * @brief It allows users to extract some stats of the Fusion API like drop frame of each camera, latency, etc...
         */
        bool output_performance_metrics;

        /**
         * @brief Enable the verbosity mode of the SDK.
         */
        int verbose;

        /**
         * @brief If specified change the number of period necessary for a source to go in timeout without data. For example, if you set this to 5
         * then, if any source do not receive data during 5 period, these sources will go to timeout and will be ignored.
         */
        unsigned timeout_period_number;


        /**
         * @brief Default constructor.
         */
        InitFusionParameters(
                UNIT coordinate_units_ = UNIT::MILLIMETER,
                COORDINATE_SYSTEM coordinate_system_ = COORDINATE_SYSTEM::IMAGE,
                bool output_performance_metrics = true,
                int verbose_ = 0,
                unsigned timeout_period_number = 20);
    };

    /**
    \class CameraIdentifier
    \ingroup Fusion_group
    \brief Used to identify a specific camera in the Fusion API.
     */
    struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ CameraIdentifier {

        CameraIdentifier() : sn(0) {
        }

        CameraIdentifier(uint64_t sn_) : sn(sn_) {
        }
        uint64_t sn;
    };

    inline bool operator==(const CameraIdentifier & a, const CameraIdentifier & b) {
        return a.sn == b.sn;
    }

    /*@cond SHOWHIDDEN*/
    inline bool operator<(const sl::CameraIdentifier &a, const sl::CameraIdentifier &b) {
        return a.sn < b.sn;
    }
    /*@endcond*/

    /**
    \enum FUSION_ERROR_CODE
    \ingroup Fusion_group
    \brief Lists the types of error that can be raised by the Fusion.
     */
    enum class FUSION_ERROR_CODE {
        GNSS_DATA_NEED_FIX = -9, /*< GNSS Data need fix status in order to run fusion. */
        GNSS_DATA_COVARIANCE_MUST_VARY = -8, /*< Ingested covariance data must vary between ingest */
        BODY_FORMAT_MISMATCH = -7, /*< Senders are using different body formats. Please use the same body format. */
        MODULE_NOT_ENABLED = -6, /*< The following module is not enabled. Please enable it to proceed. */
        SOURCE_MISMATCH = -5, /**< Some sources are provided by SVO and others by LIVE stream. */
        CONNECTION_TIMED_OUT = -4, /*< Connection timed out. Unable to reach the sender. Verify the sender's IP address and port. */
        MEMORY_ALREADY_USED = -3, /*< Intra-process shared memory allocation issue. Multiple connections to the same data. Check memory usage. */
        INVALID_IP_ADDRESS = -2, /*< The provided IP address format is incorrect. Please provide a valid IP address in the format 'a.b.c.d'. */
        FAILURE = -1, /*< Standard code indicating unsuccessful behavior. */
        SUCCESS = 0, /*< Standard code indicating successful behavior. */
        FUSION_INCONSISTENT_FPS = 1, /*< Significant differences observed between sender's FPS. Fusion quality will be affected. */
        FUSION_FPS_TOO_LOW = 2, /*< Fusion FPS is too low because at least one sender has an FPS lower than 10 FPS. Fusion quality will be affected. */
        INVALID_TIMESTAMP = 3, /*< Detected problem with ingested timestamp. Ignoring sample data with invalid timestamps. */
        INVALID_COVARIANCE = 4, /*< Detected problem with ingested covariance. Ignoring sample data with invalid covariances. */
        NO_NEW_DATA_AVAILABLE = 5 /**< All data from all sources has been consumed. No new data is available for processing. */
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const FUSION_ERROR_CODE &state);

    inline std::ostream &operator<<(std::ostream &os, const FUSION_ERROR_CODE &state) {
        return os << toString(state);
    }
    ///@endcond

    /**
    \enum SENDER_ERROR_CODE
    \ingroup Fusion_group
    \brief Lists the types of error that can be raised during the Fusion by senders.
     */
    enum class SENDER_ERROR_CODE {
        DISCONNECTED = -1, /*< The sender has been disconnected. */
        SUCCESS = 0, /*< Standard code indicating successful behavior. */
        GRAB_ERROR = 1, /*< The sender encountered a grab error. Check sender's hardware and connection. */
        INCONSISTENT_FPS = 2, /*< The sender does not run with a constant frame rate. */
        FPS_TOO_LOW = 3 /**< The frame rate of the sender is lower than 10 FPS. Check sender's settings and performance. */
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const SENDER_ERROR_CODE &state);

    inline std::ostream &operator<<(std::ostream &os, const SENDER_ERROR_CODE &state) {
        return os << toString(state);
    }
    ///@endcond

    /**
    \class BodyTrackingFusionParameters
    \ingroup Fusion_group
    \brief Holds the options used to initialize the body tracking module of the \ref Fusion.
     */
    struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ BodyTrackingFusionParameters {
        /**
        \brief Defines if the object detection will track objects across images flow.
         */
        bool enable_tracking = true;

        /**
        \brief Defines if the body fitting will be applied.
        \note If you enable it and the camera provides data as BODY_18 the fused body format will be BODY_34.
         */
        bool enable_body_fitting = false;
    };

    /**
    \class BodyTrackingFusionRuntimeParameters
    \ingroup Fusion_group
    \brief Holds the options used to change the behavior of the body tracking module at runtime.
     */
    struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ BodyTrackingFusionRuntimeParameters {
        /**
         * @brief If the fused skeleton has less than skeleton_minimum_allowed_keypoints keypoints, it will be discarded.
         * 
         */
        int skeleton_minimum_allowed_keypoints = -1;

        /**
         * @brief If a skeleton was detected in less than skeleton_minimum_allowed_camera cameras, it will be discarded.
         *
         */
        int skeleton_minimum_allowed_camera = -1;

        /**
         * @brief This value controls the smoothing of the tracked or fitted fused skeleton.
         * 
         * It is ranged from 0 (low smoothing) and 1 (high smoothing).
         */
        float skeleton_smoothing = 0.f;
    };

    /**
    \class ObjectDetectionFusionParameters
    \ingroup Fusion_group
    \brief Holds the options used to initialize the object detection module of the \ref Fusion.
     */
    struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ ObjectDetectionFusionParameters {
        /**
        \brief Defines if the object detection will track objects across images flow.
         */
        bool enable_tracking = true;
    };

    /**
     \class GNSSCalibrationParameters
     \ingroup Fusion_group
     \brief Holds the options used for calibrating GNSS / VIO.
     */
    struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ GNSSCalibrationParameters {
        /**
         * @brief This parameter defines the target yaw uncertainty at which the calibration process between GNSS and VIO concludes.
         * The unit of this parameter is in radian.
         * 
         * Default: 0.1 radians
         */
        float target_yaw_uncertainty = 0.1;
        /**
         * @brief When this parameter is enabled (set to true), the calibration process between GNSS and VIO accounts for the uncertainty in the determined translation, thereby facilitating the calibration termination.
         * The maximum allowable uncertainty is controlled by the 'target_translation_uncertainty' parameter.
         */
        bool enable_translation_uncertainty_target = false;
        /**
         * @brief This parameter defines the target translation uncertainty at which the calibration process between GNSS and VIO concludes.
         * 
         * Default: 10e-2 (10 centimeters)
         */
        float target_translation_uncertainty = 10e-2;
        /**
         * @brief This parameter determines whether reinitialization should be performed between GNSS and VIO fusion when a significant disparity is detected between GNSS data and the current fusion data.
         * It becomes particularly crucial during prolonged GNSS signal loss scenarios.
         */
        bool enable_reinitialization = true;
        /**
         * @brief This parameter determines the threshold for GNSS/VIO reinitialization.
         * If the fused position deviates beyond out of the region defined by the product of the GNSS covariance and the gnss_vio_reinit_threshold, a reinitialization will be triggered.
         */
        float gnss_vio_reinit_threshold = 5;
        /**
         * @brief If this parameter is set to true, the fusion algorithm will used a rough VIO / GNSS calibration at first and then refine it.
         * This allow you to quickly get a fused position.
         */
        bool enable_rolling_calibration = true;
        /**
         * @brief Define a transform between the GNSS antenna and the camera system for the VIO / GNSS calibration.
         * Default value is [0,0,0], this position can be refined by the calibration if enabled
         */
        sl::float3 gnss_antenna_position;
    };

    /**
     \class PositionalTrackingFusionParameters
     \ingroup Fusion_group
     \brief Holds the options used for initializing the positional tracking fusion module.
     */
    struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ PositionalTrackingFusionParameters {
        /**
         * @brief This attribute is responsible for enabling or not GNSS positional tracking fusion.
         */
        bool enable_GNSS_fusion = false;
        /**
         * @brief Control the VIO / GNSS calibration process.  
         */
        GNSSCalibrationParameters gnss_calibration_parameters;
    };

    /**
    \enum POSITION_TYPE
    \ingroup Fusion_group
    \brief Lists the types of possible position outputs.
     */
    enum class POSITION_TYPE {
        RAW = 0, /**< The output position will be the raw position data.*/
        FUSION = 1, /**< The output position will be the fused position projected into the requested camera repository.*/
        ///@cond SHOWHIDDEN 
        LAST
        ///@endcond
    };

    String /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ toString(const POSITION_TYPE& position_type);

    inline ::std::ostream& operator<<(::std::ostream& os, const POSITION_TYPE& position_type) {
        return os << toString(position_type);
    }

    /**
    \class SpatialMappingFusionParameters
    \ingroup Fusion_group
    \brief Sets the spatial mapping parameters.

    Instantiating with the default constructor will set all parameters to their default values.
    \n You can customize these values to fit your application, and then save them to a preset to be loaded in future executions.

    \note Users can adjust these parameters as they see fit.
     */
    struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ SpatialMappingFusionParameters {
        /**
        \brief Sets the resolution corresponding to the given \ref MAPPING_RESOLUTION preset.
        \param mapping_resolution: The desired \ref MAPPING_RESOLUTION.  Default: \ref MAPPING_RESOLUTION::MEDIUM.
         */
        void set(SpatialMappingParameters::MAPPING_RESOLUTION mapping_resolution = SpatialMappingParameters::MAPPING_RESOLUTION::MEDIUM);

        /**
        \brief Sets the maximum value of the depth corresponding to the given \ref MAPPING_RANGE preset.
        \param mapping_range: The desired \ref MAPPING_RANGE. Default: \ref MAPPING_RANGE::MEDIUM.
         */
        void set(SpatialMappingParameters::MAPPING_RANGE mapping_range = SpatialMappingParameters::MAPPING_RANGE::MEDIUM);

        /**
        \brief Spatial mapping resolution in meters. Should fit \ref allowed_resolution.
         */
        float resolution_meter = 0.05f;

        /**
        \brief Depth range in meters.
        Can be different from the value set by \ref sl::InitParameters::depth_maximum_distance.
        \n Set to 0 by default. In this case, the range is computed from resolution_meter
        and from the current internal parameters to fit your application.
         */
        float range_meter = 0.f;

        /**
        \brief Set to false if you want to ensure consistency between the mesh and its inner chunk data.

        \note Updating the mesh is time-consuming. Setting this to true results in better performance.
         */
        bool use_chunk_only = false;

        /**
        \brief The maximum CPU memory (in MB) allocated for the meshing process.
         */
        int max_memory_usage = 2048;

        /**
        \brief Control the integration rate of the current depth into the mapping process.
        This parameter controls how many times a stable 3D points should be seen before it is integrated into the spatial mapping.
        Default value is 0, this will define the stability counter based on the mesh resolution, the higher the resolution, the higher the stability counter.
         */
        int stability_counter = 0;

        /**
        \brief The type of spatial map to be created. This dictates the format that will be used for the mapping(e.g. mesh, point cloud). See \ref SPATIAL_MAP_TYPE
         */
        SpatialMappingParameters::SPATIAL_MAP_TYPE map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;
    };

    /**
    \class CameraMetrics
    \ingroup Fusion_group
    \brief Holds the metrics of a sender in the fusion process.
     */
    struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ CameraMetrics {
        /**
         * @brief Default constructor.
         */
        CameraMetrics();

        /**
         * @brief FPS of the received data.
         */
        float received_fps;

        /**
         * @brief Latency (in seconds) of the received data.
         * Timestamp difference between the time when the data are sent and the time they are received (mostly introduced when using the local network workflow).
         */
        float received_latency;

        /**
         * @brief Latency (in seconds) after Fusion synchronization.
         * Difference between the timestamp of the data received and the timestamp at the end of the Fusion synchronization.
         */
        float synced_latency;

        /**
         * @brief Is set to false if no data in this batch of metrics.
         */
        bool is_present;

        /**
         * @brief Skeleton detection percent during the last second.
         * Number of frames with at least one detection / number of frames, over the last second.
         * A low value means few detections occured lately for this sender.
         */
        float ratio_detection;

        /**
         * @brief Average data acquisition timestamp difference.
         * Average standard deviation of sender's period since the start.
         */
        float delta_ts;
    };

    /**
    \class FusionMetrics
    \ingroup Fusion_group
    \brief Holds the metrics of the fusion process.
     */
    struct /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ FusionMetrics {
        /**
         * @brief Default constructor.
         */
        FusionMetrics();

        /**
         * @brief Reset the current metrics.
         */
        void reset();

        /**
         * @brief Mean number of camera that provides data during the past second.
         */
        float mean_camera_fused;

        /** 
         * @brief Standard deviation of the data timestamp fused, the lower the better.
         */
        float mean_stdev_between_camera;

        /**
         * @brief Sender metrics.
         */
        std::map<CameraIdentifier, CameraMetrics> camera_individual_stats;
    };

    /**
    \class ECEF
    \ingroup Fusion_group
    \brief Represents a world position in ECEF format.
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ ECEF {
    public:
        /**
         * @brief x coordinate of ECEF.
         * 
         */
        double x;
        /**
         * @brief y coordinate of ECEF.
         * 
         */
        double y;
        /**
         * @brief z coordinate of ECEF.
         * 
         */
        double z;
    };

    /**
    \class LatLng
    \ingroup Fusion_group
    \brief Represents a world position in LatLng format.
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ LatLng {
    public:
        /**
         * @brief Get the coordinates in radians (default) or in degrees.
         * 
         * @param latitude Latitude coordinate.
         * @param longitude Longitude coordinate.
         * @param altitude  Altitude coordinate.
         * @param in_radian Should the output be expressed in radians or degrees.
         */
        void getCoordinates(double & latitude, double & longitude, double & altitude, bool in_radian = true) const;
        /**
         * @brief Set the coordinates in radians (default) or in degrees.
         * 
         * @param latitude Latitude coordinate.
         * @param longitude Longitude coordinate.
         * @param altitude Altitude coordinate.
         * @param in_radian Is input are in radians or in degrees.
         */
        void setCoordinates(double latitude, double longitude, double altitude, bool in_radian = true);
        /**
         * @brief Get the latitude coordinate.
         * 
         * @param in_radian Is the output should be in radian or degree.
         * @return Latitude in radian or in degree depending \ref in_radian parameter.
         */
        double getLatitude(bool in_radian = true) const;
        /**
         * @brief Get the longitude coordinate.
         * 
         * @param in_radian is the output should be in radian or degree.
         * @return Longitude in radian or in degree depending \ref in_radian parameter.
         */
        double getLongitude(bool in_radian = true) const;
        /**
         * @brief Get the altitude coordinate.
         * 
         * @return Altitude coordinate in meters.
         */
        double getAltitude() const;
    protected:
        /**
         * @brief Latitude coordinate.
         * 
         */
        double latitude;
        /**
         * @brief Longitude coordinate.
         * 
         */
        double longitude;
        /**
         * @brief Altitude coordinate.
         * 
         */
        double altitude;
    };

    /** 
    \class UTM
    \ingroup Fusion_group
    \brief Represents a world position in UTM format.
     */
    class UTM {
    public:
        /**
         * @brief Northing coordinate.
         * 
         */
        double northing;
        /**
         * @brief Easting coordinate.
         * 
         */
        double easting;
        /**
         * @brief Gamma coordinate.
         * 
         */
        double gamma;
        /**
         * @brief UTMZone of the coordinate.
         * 
         */
        std::string UTMZone;
    };

    /**
    \class GeoConverter
    \ingroup Fusion_group
    \brief Purely static class for Geo functions.
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ GeoConverter {
    public:
        /**
         * @brief Convert ECEF coordinates to Lat/Long coordinates.
         */
        static void ecef2latlng(sl::ECEF &in, sl::LatLng &out);

        /**
         * @brief Convert ECEF coordinates to UTM coordinates.
         */
        static void ecef2utm(sl::ECEF &in, sl::UTM &out);

        /**
         * @brief Convert Lat/Long coordinates to ECEF coordinates.
         */
        static void latlng2ecef(sl::LatLng &in, sl::ECEF &out);

        /**
         * @brief Convert Lat/Long coordinates to UTM coordinates.
         */
        static void latlng2utm(sl::LatLng &in, sl::UTM &out);

        /**
         * @brief Convert UTM coordinates to ECEF coordinates.
         */
        static void utm2ecef(sl::UTM &in, sl::ECEF &out);

        /**
         * @brief Convert UTM coordinates to Lat/Long coordinates.
         */
        static void utm2latlng(sl::UTM &in, sl::LatLng &out);
    };

    /**
     * \class GeoPose
     * \ingroup Fusion_group
     * \brief Holds geographic reference position information.
     *
     * This class represents a geographic pose, including position, orientation, and accuracy information.
     * It is used for storing and manipulating geographic data, such as latitude, longitude, altitude,
     * pose matrices, covariances, and timestamps.
     *
     * The pose data is defined in the East-North-Up (ENU) reference frame. The ENU frame is a local
     * Cartesian coordinate system commonly used in geodetic applications. In this frame, the X-axis
     * points towards the East, the Y-axis points towards the North, and the Z-axis points upwards.
     */
    class /*@cond SHOWHIDDEN*/SL_CORE_EXPORT/*@endcond*/ GeoPose {
    public:
        /**
         * @brief Default constructor.
         */
        GeoPose();

        /**
         * @brief Copy constructor.
         *
         * @param geopose The GeoPose object to copy from.
         */
        GeoPose(const GeoPose& geopose);

        /**
         * @brief The 4x4 matrix defining the pose in the East-North-Up (ENU) coordinate system.
         */
        sl::Transform pose_data;

        /**
         * @brief The pose covariance matrix in ENU.
         */
        float pose_covariance[36];

        /**
         * @brief The horizontal accuracy of the pose in meters.
         */
        double horizontal_accuracy;

        /**
         * @brief The vertical accuracy of the pose in meters.
         */
        double vertical_accuracy;

        /**
         * @brief The latitude, longitude, and altitude coordinates of the pose.
         */
        sl::LatLng latlng_coordinates;

        /**
         * @brief The heading (orientation) of the pose in radians. It indicates the direction in which the object or observer is facing, with 0 degrees corresponding to North and increasing in a clockwise direction.
         */
        double heading;

        /**
         * @brief The timestamp associated with the GeoPose.
         */
        sl::Timestamp timestamp;
    };

    /**
     \class GNSS_STATUS
     \ingroup Sensors_group
     \brief Class representing the status of the of GNSS signal.
     */
    enum class GNSS_STATUS {
        UNKNOWN = 0, /**< No GNSS fix data is available. */
        SINGLE = 1, /**< Single Point Positioning */
        DGNSS = 2, /**< Differential GNSS */
        PPS = 5,  /**< Precise Positioning Service */
        RTK_FLOAT = 4, /**< Real Time Kinematic Float */
        RTK_FIX = 3, /**< Real Time Kinematic Fixed */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const GNSS_STATUS &state);

    inline std::ostream &operator<<(std::ostream &os, const GNSS_STATUS &state) {
        return os << toString(state);
    }
    ///@endcond

    /**
     \class GNSS_MODE
     \ingroup Sensors_group
     \brief Class representing the mode of GNSS signal.
     */
    enum class GNSS_MODE {
        UNKNOWN = 0, /**< No GNSS fix data is available. */
        NO_FIX = 1, /**< No GNSS fix is available. */
        FIX_2D = 2, /**< 2D GNSS fix, providing latitude and longitude coordinates but without altitude information. */
        FIX_3D = 3, /**< 3D GNSS fix, providing latitude, longitude, and altitude coordinates. */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const GNSS_MODE &state);

    inline std::ostream &operator<<(std::ostream &os, const GNSS_MODE &state) {
        return os << toString(state);
    }
    ///@endcond

    /**
    \class GNSSData
    \ingroup Sensors_group
    \brief Class containing GNSS data to be used for positional tracking as prior.
     */
    class SL_CORE_EXPORT GNSSData {
    public:
        /**
        \brief Sets the sl::LatLng coordinates of sl::GNSSData.
        
        The sl::LatLng coordinates could be expressed in degrees or radians.
        \param latitude Latitude coordinate.
        \param longitude Longitude coordinate.
        \param altitude Altitude coordinate.
        \param is_radian If the inputs are expressed in radians or in degrees.
         */
        void setCoordinates(double latitude, double longitude, double altitude, bool is_radian = true);
        /**
        \brief Gets the sl::LatLng coordinates of the sl::GNSSData.
        
        The sl::LatLng coordinates could be expressed in degrees or radians.
        \param latitude Latitude coordinate.
        \param longitude Longitude coordinate.
        \param altitude Altitude coordinate.
        \param is_radian Should the output be expressed in radians or degrees.
         */
        void getCoordinates(double &latitude, double &longitude, double &altitude, bool in_radian = true);
        /**
        \brief Timestamp of the GNSS position (must be aligned with the camera time reference).
         */
        sl::Timestamp ts;
        /**
        \brief Covariance of the position in meter (must be expressed in the ENU coordinate system).

        For eph, epv GNSS sensors, set it as follow: ```{eph*eph, 0, 0, 0, eph*eph, 0, 0, 0, epv*epv}```.
         */
        std::array<double, 9> position_covariance = {0.1 * 0.1, 0, 0, 0, 0.1 * 0.1, 0, 0, 0, 0.1 * 0.1};
        /**
        \brief Longitude standard deviation.
         */
        double longitude_std;
        /**
        \brief Latitude standard deviation.
         */
        double latitude_std;
        /**
        \brief Altitude standard deviation.
         */
        double altitude_std;

        /**
        @brief  Represents the current status of GNSS.
         */
        GNSS_STATUS gnss_status = sl::GNSS_STATUS::UNKNOWN;
        
        /**
        @brief  Represents the current mode of GNSS.
         */
        GNSS_MODE gnss_mode = sl::GNSS_MODE::UNKNOWN;

    protected:
        /**
        \brief Longitude coordinate.
         */
        double longitude;
        /**
        \brief Latitude coordinate.
         */
        double latitude;
        /**
        \brief Altitude coordinate.
         */
        double altitude;
    };

    /**
     \class GNSS_FUSION_STATUS
     \ingroup Sensors_group
     \brief Class containing the current GNSS fusion status.
     */
    enum class GNSS_FUSION_STATUS {
        OK = 0, /**< The GNSS fusion module is calibrated and working successfully. */
        OFF = 1, /**< The GNSS fusion module is not enabled. */
        CALIBRATION_IN_PROGRESS = 2, /**< Calibration of the GNSS/VIO fusion module is in progress. */
        RECALIBRATION_IN_PROGRESS = 3, /**< Re-alignment of GNSS/VIO data is in progress, leading to potentially inaccurate global position. */
        LAST
    };

    ///@cond SHOWHIDDEN
    String SL_CORE_EXPORT toString(const GNSS_FUSION_STATUS &state);

    inline std::ostream &operator<<(std::ostream &os, const GNSS_FUSION_STATUS &state) {
        return os << toString(state);
    }
    ///@endcond

    /**
     \class FusedPositionalTrackingStatus
     \ingroup Sensors_group
     \brief Class containing the overall position fusion status
     */
    class FusedPositionalTrackingStatus {
    public:
        /**
        @brief  Represents the current state of Visual-Inertial Odometry (VIO) tracking between the previous frame and the current frame.
         */
        ODOMETRY_STATUS odometry_status;

        /**
        @brief  Represents the current state of camera tracking in the global map.
         */
        SPATIAL_MEMORY_STATUS spatial_memory_status;

        /**
        @brief  Represents the current status of GNSS.
         */
        GNSS_STATUS gnss_status = sl::GNSS_STATUS::UNKNOWN;
        
        /**
        @brief  Represents the current mode of GNSS.
         */
        GNSS_MODE gnss_mode = sl::GNSS_MODE::UNKNOWN;
        
        /**
        @brief  Represents the current state of GNSS fusion for global localization.
         */
        GNSS_FUSION_STATUS gnss_fusion_status;

        /**
        @brief  Represents the current state of positional tracking fusion.
         */
        POSITIONAL_TRACKING_FUSION_STATUS tracking_fusion_status;
    };

    /*@cond SHOWHIDDEN*/
    class FusionHandlerImp;
    /*@endcond*/

    /**
    \class Fusion
    \ingroup Fusion_group
    \brief Holds Fusion process data and functions
     */
    class /*@cond SHOWHIDDEN*/ SL_CORE_EXPORT /*@endcond*/ Fusion {
    public:
        /**
         * @brief Default constructor.
         */
        Fusion();

        /**
         * @brief Default destructor.
         */
        ~Fusion();

        /**
         * @brief Initialize the fusion module with the requested parameters.
         * \param init_parameters Initialization parameters.
         * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
         */
        FUSION_ERROR_CODE init(sl::InitFusionParameters init_parameters = InitFusionParameters());

        /**
         * @brief Will deactivate all the fusion modules and free internal data.
         */
        void close();

        /**
         * @brief Set the specified camera as a data provider.
         * \param uuid: The requested camera identifier.
         * \param param: The communication parameters to connect to the camera.
         * \param pose: The World position of the camera, regarding the other camera of the setup.
         * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
         */
        FUSION_ERROR_CODE subscribe(CameraIdentifier uuid, CommunicationParameters param = CommunicationParameters(), sl::Transform pose = sl::Transform());

        /**
         * @brief Remove the specified camera from data provider.
         * \param uuid: The requested camera identifier..
         * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
         */
        FUSION_ERROR_CODE unsubscribe(CameraIdentifier uuid);

        /**
         * @brief Updates the specified camera position inside fusion WORLD.
         * \param uuid: The requested camera identifier.
         * \param pose: The World position of the camera, regarding the other camera of the setup.
         * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
         */
        FUSION_ERROR_CODE updatePose(CameraIdentifier uuid, sl::Transform pose);

        /**
         * @brief Get the metrics of the Fusion process, for the fused data as well as individual camera provider data.
         * \param metrics The process metrics.
         * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
         */
        FUSION_ERROR_CODE getProcessMetrics(FusionMetrics &metrics);

        /**
         * @brief Returns the state of each connected data senders.
         * \return The individual state of each connected senders.
         */
        std::map<CameraIdentifier, SENDER_ERROR_CODE> getSenderState();

        /**
         * @brief Runs the main function of the Fusion, this trigger the retrieve and synchronization of all connected senders and updates the enabled modules.
         * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
         */
        FUSION_ERROR_CODE process();

        /**
         * @brief Returns the current sl::VIEW::LEFT of the specified camera, the data is synchronized.
         * \param mat: the CPU BGRA image of the requested camera.
         * \param uuid: the requested camera identifier.
         * \param resolution: the requested resolution of the output image, can be lower or equal (default) to the original image resolution.
         * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
         * \note Only the Left BGRA image is available.
         * \warning This feature is not available for network cameras. 
         */
        FUSION_ERROR_CODE retrieveImage(sl::Mat &mat, CameraIdentifier uuid, sl::Resolution resolution = sl::Resolution(0, 0));

        /**
         * @brief Returns the current measure of the specified camera, the data is synchronized.
         * \param mat: the CPU data of the requested camera.
         * \param uuid: the requested camera identifier.
         * \param measure: the requested measure type, by default DEPTH (F32_C1)
         * \param resolution: the requested resolution of the output image, can be lower or equal (default) to the original image resolution.
         * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
         * \note Only MEASURE: DEPTH, XYZ, XYZRGBA, XYZBGRA, XYZARGB, XYZABGR, DEPTH_U16_MM are available.
         * \warning This feature is not available for network cameras. 
         */
        FUSION_ERROR_CODE retrieveMeasure(sl::Mat &mat, CameraIdentifier uuid, sl::MEASURE measure = sl::MEASURE::DEPTH, sl::Resolution resolution = sl::Resolution(0, 0));

        /**
         * @brief Enables the body tracking fusion module.
         * \param params: Structure containing all specific parameters for body tracking fusion.
         * \n For more information, see the \ref BodyTrackingFusionParameters documentation.
         * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
         */
        FUSION_ERROR_CODE enableBodyTracking(BodyTrackingFusionParameters params = BodyTrackingFusionParameters());

        /**
         * @brief Retrieves the body data, can be the fused data (default), or the raw data provided by a specific sender.
         * \param objs: The fused bodies will be saved into this objects.
         * \param parameters: Body detection runtime settings, can be changed at each detection.
         * \param uuid: If set to a sender serial number (different from 0), this will retrieve the raw data provided by this sender.
         * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
         */
        FUSION_ERROR_CODE retrieveBodies(sl::Bodies &objs, BodyTrackingFusionRuntimeParameters parameters = BodyTrackingFusionRuntimeParameters(), CameraIdentifier uuid = CameraIdentifier());

        /**
         * @brief Disable the body fusion tracking module.
         */
        void disableBodyTracking();

        /**
         * @brief Enables positional tracking fusion module.
         * \param parameters A structure containing all the \ref PositionalTrackingFusionParameters that define positional tracking fusion module.
         * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
         */
        FUSION_ERROR_CODE enablePositionalTracking(PositionalTrackingFusionParameters parameters = PositionalTrackingFusionParameters());

        /**
         * @brief Get the Fused Position referenced to the first camera subscribed. If \ref uuid is specified then project position on the referenced camera.
         * \param camera_pose Will contain the fused position referenced by default in world (world is given by the calibration of the cameras system).
         * \param reference_frame Defines the reference from which you want the pose to be expressed. Default : \ref REFERENCE_FRAME "REFERENCE_FRAME::WORLD".
         * \param uuid If set to a sender serial number (different from 0), this will retrieve position projected on the requested camera if \ref position_type is equal to \ref POSITION_TYPE "POSITION_TYPE::FUSION" or raw sender position if \ref position_type is equal to \ref POSITION_TYPE "POSITION_TYPE::RAW".
         * \param position_type: Select if the position should the fused position re-projected in the camera with uuid or if the position should be the raw position (without fusion) of camera with uui.
         * \return POSITIONAL_TRACKING_STATE is the current state of the tracking process.
         */
        POSITIONAL_TRACKING_STATE getPosition(Pose &camera_pose, sl::REFERENCE_FRAME reference_frame = REFERENCE_FRAME::WORLD, CameraIdentifier uuid = CameraIdentifier(), POSITION_TYPE position_type = POSITION_TYPE::FUSION);

        /**
         * @brief Get the current status of fused position.
         * \return FusedPositionalTrackingStatus is the current status of the tracking process.
         */
        FusedPositionalTrackingStatus getFusedPositionalTrackingStatus();

        /**
         * @brief Ingest GNSS data from an external sensor into the fusion module.
         * \param gnss_data: The current GNSS data to combine with the current positional tracking data.
         * \return \ref FUSION_ERROR_CODE "SUCCESS" if it goes as it should, otherwise it returns an FUSION_ERROR_CODE.
         */
        FUSION_ERROR_CODE ingestGNSSData(sl::GNSSData gnss_data);

        /**
         * @brief Returns the last synchronized gnss data.
         * \param out Last synchronized gnss data.
         * \return POSITIONAL_TRACKING_STATE is the current state of the tracking process.
         */
        POSITIONAL_TRACKING_STATE getCurrentGNSSData(sl::GNSSData &out);

        /**
         * @brief Returns the current GeoPose.
         * \param pose The current GeoPose.
         * \return GNSS_FUSION_STATUS is the current state of the tracking process.
         */
        GNSS_FUSION_STATUS getGeoPose(sl::GeoPose &pose);

        /**
         * @brief Convert latitude / longitude into position in sl::Fusion coordinate system.
         * \param in The latitude / longitude to be converted in sl::Fusion coordinate system.
         * \param out Converted position in sl::Fusion coordinate system.
         * \return GNSS_FUSION_STATUS is the current state of the tracking process.
         */
        GNSS_FUSION_STATUS Geo2Camera(sl::LatLng &in, sl::Pose &out);

        /**
         * @brief Convert a position in sl::Fusion coordinate system in global world coordinate. 
         * \param pose Position to convert in global world coordinate. 
         * \param out Converted position in global world coordinate. 
         * \return GNSS_FUSION_STATUS is the current state of the tracking process.
         */
        GNSS_FUSION_STATUS Camera2Geo(sl::Pose const &in, sl::GeoPose &out);

        /**
         * @brief Disable the fusion positional tracking module.
         */
        void disablePositionalTracking();

        /**
         * @brief Return the current fusion timestamp, aligned with the synchronized GNSS and camera data.
         * 
         * @return sl::Timestamp current fusion timestamp.
         */
        sl::Timestamp getCurrentTimeStamp();

        /**
         * @brief Get the current calibration uncertainty obtained during calibration process.
         * 
         * @param yaw_std Output yaw uncertainty in radian.
         * @param position_std Output position uncertainty in meter.
         * \return sl::GNSS_FUSION_STATUS representing current initialisation status.
         */
        GNSS_FUSION_STATUS getCurrentGNSSCalibrationSTD(float & yaw_std, sl::float3 & position_std);

        /**
         * @brief Get the calibration found between VIO and GNSS.
         * 
         * \return sl::Transform is the calibration found between VIO and GNSS during calibration process.
         */
        sl::Transform getGeoTrackingCalibration();

        /**
        \brief Initializes and starts the spatial mapping processes.

        The spatial mapping will create a geometric representation of the scene based on both tracking data and 3D point clouds.
        \n The resulting output can be a \ref Mesh or a \ref FusedPointCloud. It can be be obtained by calling \ref retrieveSpatialMapAsync().
        Note that \ref retrieveSpatialMapAsync() should be called after \ref requestSpatialMapAsync().

        \param spatial_mapping_parameters The structure containing all the specific parameters for the spatial mapping.
        \n Default: a balanced parameter preset between geometric fidelity and output file size. For more information, see the \ref SpatialMappingParameters documentation.
        \return \ref ERROR_CODE "SUCCESS" if everything went fine, \ref ERROR_CODE "ERROR_CODE::FAILURE" otherwise.

        \note The tracking (\ref enablePositionalTracking()) needs to be enabled to use the spatial mapping.
        \note Lower SpatialMappingParameters.range_meter and SpatialMappingParameters.resolution_meter for higher performance.
        \warning This fuction is only available for INTRA_PROCESS communication type.
         */
        FUSION_ERROR_CODE enableSpatialMapping(SpatialMappingFusionParameters parameters = SpatialMappingFusionParameters());

        // -----------------------------------------------------------------
        // Async functions of spatial map generation ( *Async())
        // -----------------------------------------------------------------
        /**
        \brief Starts the spatial map generation process in a non blocking thread from the spatial mapping process.

        The spatial map generation can take a long time depending on the mapping resolution and covered area. This function will trigger the generation of a mesh without blocking the program.
        You can get info about the current generation using \ref getSpatialMapRequestStatusAsync(), and retrieve the mesh using \ref retrieveSpatialMapAsync(...) .

        \note Only one mesh can be generated at a time. If the previous mesh generation is not over, new calls of the function will be ignored.
         */
        void requestSpatialMapAsync();

        /**
        \brief Returns the spatial map generation status. This status allows to know if the mesh can be retrieved by calling \ref retrieveSpatialMapAsync.
        \return \ref ERROR_CODE "SUCCESS" if the mesh is ready and not yet retrieved, otherwise \ref ERROR_CODE "ERROR_CODE::FAILURE".

        \n See \ref requestSpatialMapAsync() for an example.
         */
        FUSION_ERROR_CODE getSpatialMapRequestStatusAsync();

        /**
        \brief Retrieves the current generated spatial map only if \ref SpatialMappingParameters::map_type was set as SPATIAL_MAP_TYPE::MESH.

        After calling \ref requestSpatialMapAsync , this function allows you to retrieve the generated mesh. The mesh will only be available when \ref getMeshRequestStatusAsync() returns \ref ERROR_CODE "SUCCESS".

        \param mesh : \b  The mesh to be filled with the generated spatial map.
        \return \ref ERROR_CODE "SUCCESS" if the mesh is retrieved, otherwise \ref ERROR_CODE "ERROR_CODE::FAILURE".

        \note This function only updates the necessary chunks and adds the new ones in order to improve update speed.
        \warning You should not modify the mesh between two calls of this function, otherwise it can lead to corrupted mesh.
        \warning If the SpatialMappingParameters::map_type has not been setup as SPATIAL_MAP_TYPE::MESH, the object will be empty.
        \n See \ref requestSpatialMapAsync() for an example.
         */
        FUSION_ERROR_CODE retrieveSpatialMapAsync(Mesh& mesh);

        /**
        \brief Retrieves the current generated spatial map only if \ref SpatialMappingParameters::map_type was set as SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD.
        After calling \ref requestSpatialMapAsync , this function allows you to retrieve the generated fused point cloud. The fused point cloud will only be available when \ref getMeshRequestStatusAsync() returns \ref ERROR_CODE "SUCCESS"

        \param fpc : \b The fused point cloud to be filled with the generated spatial map.
        \return \ref ERROR_CODE "SUCCESS" if the fused point cloud is retrieved, otherwise \ref ERROR_CODE "ERROR_CODE::FAILURE".

        \note This function only updates the necessary chunks and adds the new ones in order to improve update speed.
        \warning You should not modify the fused point cloud between two calls of this function, otherwise it can lead to a corrupted fused point cloud.
        \warning If the SpatialMappingParameters::map_type has not been setup as SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD, the object will be empty.

        \n See \ref requestSpatialMapAsync() for an example.
         */
        FUSION_ERROR_CODE retrieveSpatialMapAsync(FusedPointCloud& fpc);

        /**
        \brief Disables the spatial mapping process.

        The spatial mapping is immediately stopped.
        \n If the mapping has been enabled, this function will automatically be called by \ref close().

        \note This function frees the memory allocated for the spatial mapping, consequently, the spatial map cannot be retrieved after this call.
         */
        void disableSpatialMapping();


    private:
        std::unique_ptr<FusionHandlerImp> pimpl;
    };
}

#endif /* __FUSION_HPP__ */
