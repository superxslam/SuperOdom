//
// Created by shibo zhao on 2020-09-27.
//
#include "super_odometry/config/parameter.h"

// Define color escape codes for ~beautification~
#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

#define BOLD "\033[1m"
#define UNDERLINE "\033[4m"
#define ITALIC "\033[3m"

std::string IMU_TOPIC;
std::string LASER_TOPIC;
std::string ODOM_TOPIC;
std::string DepthUP_TOPIC;
std::string DepthDown_TOPIC;
std::string ProjectName;

std::string WORLD_FRAME;
std::string WORLD_FRAME_ROT;
std::string SENSOR_FRAME;
std::string SENSOR_FRAME_ROT;
SensorType sensor;

int PROVIDE_IMU_LASER_EXTRINSIC;

Eigen::Matrix3d imu_laser_R;

Eigen::Vector3d imu_laser_T;

Eigen::Vector3d imu_laser_offset;

Eigen::Matrix3d cam_laser_R;

Eigen::Vector3d cam_laser_T;

Eigen::Matrix3d imu_camera_R;

Eigen::Vector3d imu_camera_T;

Transformd Tcam_lidar;

Transformd T_i_c;

Transformd T_i_l;

Transformd T_l_i;

Transformd T_ouster_sensor;

Eigen::Matrix3d ouster_sensor_R;

Eigen::Vector3d ouster_sensor_T;

float lidar_imu_offset_roll;

float up_realsense_roll;

float up_realsense_pitch;

float up_realsense_yaw;

float up_realsense_x;

float up_realsense_y;

float up_realsense_z;

float down_realsense_roll;

float down_realsense_pitch;

float down_realsense_yaw;

float down_realsense_x;

float down_realsense_y;

float down_realsense_z;

float yaw_ratio;

float IMU_ACC_X_LIMIT;

float IMU_ACC_Y_LIMIT;

float IMU_ACC_Z_LIMIT;

bool USE_IMU_ROLL_PITCH;

bool SAVE_PLY;

std::string SENSOR;


template <typename T>
T readParam(rclcpp::Node::SharedPtr node, std::string name)
{
    T ans;
    // node->declare_parameter<T>(name);
    if (node->get_parameter(name, ans)) {
        RCLCPP_INFO(node->get_logger(),  "Loaded %s: ", name.c_str());
    }
    else {
        RCLCPP_ERROR(node->get_logger(), "Failed to load %s", name.c_str());
        rclcpp::shutdown();
    }
    return ans;
}

bool readCalibration(rclcpp::Node::SharedPtr node)
{
    RCLCPP_INFO(node->get_logger(), "[super_odometry] read parameter");
    std::string calib_file;
    // calib_file = readParam<std::string>(node, "calib_file");
    calib_file = node->declare_parameter("calibration_file", std::string(""));
    RCLCPP_INFO(node->get_logger(), "[super_odometry] calib_file: %s", calib_file.c_str());
    cv::FileStorage fsSettings(calib_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        return false;
    }
    PROVIDE_IMU_LASER_EXTRINSIC = node->declare_parameter("provide_imu_laser_extrinsic", true);
    RCLCPP_INFO(node->get_logger(), "PROVIDE_IMU_LASER_EXTRINSIC: %d", PROVIDE_IMU_LASER_EXTRINSIC);

    // Defaults to 0 if no entry
    up_realsense_roll = fsSettings["up_realsense_roll"];
    up_realsense_pitch = fsSettings["up_realsense_pitch"];
    up_realsense_yaw = fsSettings["up_realsense_yaw"];
    up_realsense_x = fsSettings["up_realsense_x"];
    up_realsense_y = fsSettings["up_realsense_y"];
    up_realsense_z = fsSettings["up_realsense_z"];

    down_realsense_roll = fsSettings["down_realsense_roll"];
    down_realsense_pitch = fsSettings["down_realsense_pitch"];
    down_realsense_yaw = fsSettings["down_realsense_yaw"];
    down_realsense_x = fsSettings["down_realsense_x"];
    down_realsense_y = fsSettings["down_realsense_y"];
    down_realsense_z = fsSettings["down_realsense_z"];
    
    yaw_ratio=fsSettings["yaw_ratio"];

    RCLCPP_INFO(node->get_logger(), "up realsense extrinsic to velodyne (RPYXYZ): %f, %f, %f, %f, %f, %f",
                up_realsense_roll,
                up_realsense_pitch,
                up_realsense_yaw,
                up_realsense_x,
                up_realsense_y,
                up_realsense_z);

    RCLCPP_INFO(node->get_logger(), "down realsense extrinsic to velodyne (RPYXYZ): %f, %f, %f, %f, %f, %f",
                down_realsense_roll,
                down_realsense_pitch,
                down_realsense_yaw,
                down_realsense_x,
                down_realsense_y,
                down_realsense_z);

    RCLCPP_INFO(node->get_logger(), "yaw ratio: %f", yaw_ratio);
    
    if (PROVIDE_IMU_LASER_EXTRINSIC)
    {
        cv::Mat cv_R, cv_T;

        cv::Mat imu_laser_rotation_offset;
        fsSettings["imu_laser_rotation_offset"] >> imu_laser_rotation_offset;
        fsSettings["extrinsicRotation_imu_laser"] >> cv_R;
        fsSettings["extrinsicTranslation_imu_laser"] >> cv_T;
        cv::cv2eigen(cv_R, imu_laser_R);
        cv::cv2eigen(cv_T, imu_laser_T);
        cv::cv2eigen(imu_laser_rotation_offset, imu_laser_offset);
        // RCLCPP_INFO(node->get_logger(),  "\n imu_laser_R: \n"
        //           << imu_laser_R;
        // RCLCPP_INFO(node->get_logger(),  "\n imu_laser_T: \n"
        //           << imu_laser_T.transpose();
        
        // RCLCPP_INFO(node->get_logger(),  "\n imu_laser_rotation_offset: \n" << imu_laser_offset.transpose();
        
        // RCLCPP_INFO(node->get_logger(), BLUE <<"\n Before Apply offset on  imu_laser_R : \n"<<RESET
        //           << imu_laser_R;
        // RCLCPP_INFO(node->get_logger(), "\n Before Apply offset on  imu_laser_R : \n"
        //           << imu_laser_R;

        //previous rotation matrix
        T_i_l = Transformd(imu_laser_R, imu_laser_T);
        T_l_i = T_i_l.inverse();   
        
        //previous roll, pitch, yaw
        double roll, pitch, yaw;
        tf2::Quaternion orientation_pre(T_i_l.rot.x(), T_i_l.rot.y(), T_i_l.rot.z(), T_i_l.rot.w());
        tf2::Matrix3x3(orientation_pre).getRPY(roll, pitch, yaw);
        RCLCPP_INFO(node->get_logger(), BLUE"\n previous roll: %f previous pitch: %f previous yaw: %f" RESET, roll *180/M_PI, pitch *180/M_PI, yaw *180/M_PI); 

        //add offset rotation matrix
        tf2::Quaternion IMU_LASER_R_offset;
        IMU_LASER_R_offset.setRPY(imu_laser_offset[0]* M_PI / 180, imu_laser_offset[1] * M_PI / 180, 
                            imu_laser_offset[2]* M_PI / 180);

        tf2::Quaternion IMU_LASER_R(T_i_l.rot.x(), T_i_l.rot.y(), T_i_l.rot.z(),
                                                        T_i_l.rot.w());
        tf2::Quaternion IMU_LASER = IMU_LASER_R_offset * IMU_LASER_R;
        Eigen::Quaterniond imu_laser_rot;             
        imu_laser_rot = Eigen::Quaterniond(IMU_LASER.w(), IMU_LASER.x(), IMU_LASER.y(),
                                                      IMU_LASER.z());
         
        T_i_l.rot=imu_laser_rot;
        T_l_i = T_i_l.inverse(); 
        imu_laser_R=T_i_l.rot.toRotationMatrix();
        
        RCLCPP_INFO_STREAM(node->get_logger(),  GREEN BOLD "T_i_l Extrinsic : \n" << T_i_l.matrix());
        RCLCPP_INFO_STREAM(node->get_logger(),  GREEN BOLD "T_l_i Extrinsic : \n" << T_l_i.matrix()); 

        //lasted roll pitch yaw
        double updated_roll, updated_pitch, updated_yaw;
        tf2::Quaternion orientation_curr(IMU_LASER.x(), IMU_LASER.y(), IMU_LASER.z(), IMU_LASER.w());
        tf2::Matrix3x3(orientation_curr).getRPY(updated_roll, updated_pitch, updated_yaw);
        
        RCLCPP_INFO(node->get_logger(), GREEN BOLD"\n updated roll: %f updated pitch: %f updated yaw: %f" RESET, updated_roll*180/M_PI, updated_pitch *180/M_PI, updated_yaw*180/M_PI); 

        // RCLCPP_INFO(node->get_logger(), "\n After Apply offset on  imu_laser_R : \n"
        //           << imu_laser_R; 
        // RCLCPP_INFO(node->get_logger(), "\n Apply offset on  T_i_l : \n"
        //           << T_i_l; 
    }
    else
    {
        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation_camera_laser"] >> cv_R;
        fsSettings["extrinsicTranslation_camera_laser"] >> cv_T;
        cv::cv2eigen(cv_R, cam_laser_R);
        cv::cv2eigen(cv_T, cam_laser_T);

        Tcam_lidar = Transformd(cam_laser_R, cam_laser_T);

        // RCLCPP_INFO(node->get_logger(),  "\n cam_laser_R: \n"
        //           << cam_laser_R;
        // RCLCPP_INFO(node->get_logger(),  "\n cam_laser_T: \n"
        //           << cam_laser_T.transpose();
        // RCLCPP_INFO(node->get_logger(),  "\n T_cam_lidar: \n"
        //           << Tcam_lidar;

        fsSettings["extrinsicRotation_imu_camera"] >> cv_R;
        fsSettings["extrinsicTranslation_imu_camera"] >> cv_T;

        cv::cv2eigen(cv_R, imu_camera_R);
        cv::cv2eigen(cv_T, imu_camera_T);
        Eigen::Quaterniond Q(imu_camera_R);
        imu_camera_R = Q.normalized();

        T_i_c = Transformd(imu_camera_R, imu_camera_T);

        T_i_l = T_i_c * Tcam_lidar;
        T_l_i = T_i_l.inverse();

        // RCLCPP_INFO(node->get_logger(),  "imu_camera_R: \n"
        //           << imu_camera_R;
        // RCLCPP_INFO(node->get_logger(),  "Timu_camera_T : \n"
        //           << imu_camera_T;

        RCLCPP_INFO_STREAM(node->get_logger(),  GREEN BOLD "T_i_l Extrinsic : \n" << T_i_l.matrix());
        RCLCPP_INFO_STREAM(node->get_logger(),  GREEN BOLD "T_l_i Extrinsic : \n" << T_l_i.matrix());
    }

    ouster_sensor_R << -1, 0,  0,
                    0, -1, 0,
                    0,  0,  1;

    ouster_sensor_T << 0, 0, 0.036180;

    T_ouster_sensor = Transformd(ouster_sensor_R, ouster_sensor_T);

    return true;
}

bool readGlobalparam(rclcpp::Node::SharedPtr node)
{
    node->declare_parameter<std::string>("imu_topic","imu/data");
    node->declare_parameter<std::string>("laser_topic","velodyne_points");
    node->declare_parameter<std::string>("odom_topic","integrated_to_init");
    node->declare_parameter<std::string>("depthup_topic","/rs_up/depth/cloud_filtered");
    node->declare_parameter<std::string>("depthdown_topic","/rs_down/depth/cloud_filtered");
    node->declare_parameter<std::string>("world_frame", "sensor_init");
    node->declare_parameter<std::string>("world_frame_rot", "sensor_init_rot");
    node->declare_parameter<std::string>("sensor_frame", "sensor");
    node->declare_parameter<std::string>("sensor_frame_rot", "sensor_rot");
    node->declare_parameter<std::string>("PROJECT_NAME", "");
    node->declare_parameter<std::string>("sensor", "livox");
    node->declare_parameter<double>("imu_acc_x_limit", 0.5);
    node->declare_parameter<double>("imu_acc_y_limit", 0.2);
    node->declare_parameter<double>("imu_acc_z_limit", 0.4);
    node->declare_parameter<bool>("save_ply", false);
    // node->declare_parameter<bool>("use_imu_roll_pitch", false);

    
    LASER_TOPIC = node->get_parameter("laser_topic").as_string();
    IMU_TOPIC = node->get_parameter("imu_topic").as_string();
    ODOM_TOPIC = node->get_parameter("odom_topic").as_string();
    DepthUP_TOPIC = node->get_parameter("depthup_topic").as_string();
    DepthDown_TOPIC = node->get_parameter("depthdown_topic").as_string();
    WORLD_FRAME = node->get_parameter("world_frame").as_string();
    WORLD_FRAME_ROT = node->get_parameter("world_frame_rot").as_string();
    SENSOR_FRAME = node->get_parameter("sensor_frame").as_string();
    SENSOR_FRAME_ROT = node->get_parameter("sensor_frame_rot").as_string();
    ProjectName = node->get_parameter("PROJECT_NAME").as_string();
    SENSOR = node->get_parameter("sensor").as_string();
    // USE_IMU_ROLL_PITCH = node->get_parameter("use_imu_roll_pitch").as_bool();
    SAVE_PLY = node->get_parameter("save_ply").as_bool();
    IMU_ACC_X_LIMIT = node->get_parameter("imu_acc_x_limit").as_double();
    IMU_ACC_Y_LIMIT = node->get_parameter("imu_acc_y_limit").as_double();
    IMU_ACC_Z_LIMIT = node->get_parameter("imu_acc_z_limit").as_double();
    //check whether sensor is support 
    const std::unordered_map<std::string, SensorType> sensorTypeMap = {
        {"velodyne", SensorType::VELODYNE},
        {"ouster", SensorType::OUSTER},
        {"livox", SensorType::LIVOX}
    };

    if (sensorTypeMap.find(SENSOR) == sensorTypeMap.end()) {
        RCLCPP_ERROR(node->get_logger(), "Unsupported sensor type: %s", SENSOR.c_str());
        return false;
    }
    
    RCLCPP_INFO(node->get_logger(), "LASER_TOPIC %s", LASER_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "IMU_TOPIC %s", IMU_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "ODOM_TOPIC %s", ODOM_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "DepthUP_TOPIC %s", DepthUP_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "DepthDown_TOPIC %s", DepthDown_TOPIC.c_str());
    RCLCPP_INFO(node->get_logger(), "WORLD_FRAME %s", WORLD_FRAME.c_str());
    RCLCPP_INFO(node->get_logger(), "WORLD_FRAME_ROT %s", WORLD_FRAME_ROT.c_str());
    RCLCPP_INFO(node->get_logger(), "SENSOR_FRAME %s", SENSOR_FRAME.c_str());
    RCLCPP_INFO(node->get_logger(), "SENSOR_FRAME_ROT %s", SENSOR_FRAME_ROT.c_str());
    RCLCPP_INFO(node->get_logger(), "ProjectName %s", ProjectName.c_str());
    RCLCPP_INFO(node->get_logger(), "SENSOR %s", SENSOR.c_str());
    RCLCPP_INFO(node->get_logger(), "SAVE_PLY %d", SAVE_PLY);
    RCLCPP_INFO(node->get_logger(), "SAVE_PLY %d", SAVE_PLY);

    return true;
}