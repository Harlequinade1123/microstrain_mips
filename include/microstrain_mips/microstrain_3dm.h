#ifndef _MICROSTRAIN_3DM_H
#define _MICROSTRAIN_3DM_H

extern "C"
{
  #include "mip_sdk.h"
  #include "byteswap_utilities.h"
  #include "mip_gx4_imu.h"
  #include "mip_gx4_45.h"
  #include "mip_gx4_25.h"
  #include "mip_sdk_3dm.h"
  #include "GX4-45_Test.h"
}

#include <cstdio>
#include <unistd.h>
#include <time.h>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
// #include "microstrain_mips/msg/status_msg.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace microstrain
{
  class Microstrain : public rclcpp::Node
  {
  public:
    Microstrain();
    ~Microstrain();

    void run();

    void filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    void gps_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);

    void device_status_callback();

    u16 mip_3dm_cmd_hw_specific_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer);

    bool get_model_gps()
    {
      if (Microstrain::GX5_45 || Microstrain::GX5_35)
        return true;
      else
        return false;
    }
  
  private:
    // KF reset service
    void reset_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Empty::Request> req,
      std::shared_ptr<std_srvs::srv::Empty::Response> resp);

    void print_packet_stats();

    //The primary device interface structure
    mip_interface device_interface_;
    base_device_info_field device_info;
    u8  temp_string[20];
    //Packet Counters (valid, timeout, and checksum errors)
    u32 filter_valid_packet_count_;
    u32 ahrs_valid_packet_count_;
    u32 gps_valid_packet_count_;    u32 filter_timeout_packet_count_;
    u32 ahrs_timeout_packet_count_;
    u32 gps_timeout_packet_count_;    u32 filter_checksum_error_packet_count_;
    u32 ahrs_checksum_error_packet_count_;
    u32 gps_checksum_error_packet_count_;
    //Data field storage
    //AHRS
    mip_ahrs_scaled_gyro  curr_ahrs_gyro_;
    mip_ahrs_scaled_accel curr_ahrs_accel_;
    mip_ahrs_scaled_mag   curr_ahrs_mag_;
    mip_ahrs_quaternion  curr_ahrs_quaternion_;
    //GPS
    mip_gps_llh_pos curr_llh_pos_;
    mip_gps_ned_vel curr_ned_vel_;
    mip_gps_time    curr_gps_time_;
    //FILTER
    mip_filter_llh_pos               curr_filter_pos_;
    mip_filter_ned_velocity          curr_filter_vel_;
    mip_filter_attitude_euler_angles curr_filter_angles_;
    mip_filter_attitude_quaternion   curr_filter_quaternion_;
    mip_filter_compensated_angular_rate curr_filter_angular_rate_;
    mip_filter_compensated_acceleration curr_filter_accel_comp_;
    mip_filter_linear_acceleration curr_filter_linear_accel_;
    mip_filter_llh_pos_uncertainty   curr_filter_pos_uncertainty_;
    mip_filter_ned_vel_uncertainty   curr_filter_vel_uncertainty_;
    mip_filter_euler_attitude_uncertainty curr_filter_att_uncertainty_;
    mip_filter_status curr_filter_status_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr filtered_imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr nav_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr nav_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr bias_pub_;
    // rclcpp::Publisher<microstrain_mips::msg::StatusMsg>::SharedPtr device_status_pub_;

    // Example messages
    sensor_msgs::msg::NavSatFix gps_msg_;
    sensor_msgs::msg::Imu imu_msg_;

    // tixiao
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_correct_pub_;
    sensor_msgs::msg::Imu imu_correct_msg_;
    double fixed_roll, fixed_pitch, fixed_yaw;
    tf2::Quaternion orientation;
    geometry_msgs::msg::Quaternion geoQuat;
    tf2::Matrix3x3 beforeMatrix, betweenMatrix, afterMatrix;
    sensor_msgs::msg::Imu filtered_imu_msg_;
    nav_msgs::msg::Odometry nav_msg_;
    std_msgs::msg::Int16MultiArray nav_status_msg_;
    geometry_msgs::msg::Vector3 bias_msg_;

    std::string gps_frame_id_;
    std::string imu_frame_id_;
    std::string odom_frame_id_;
    std::string odom_child_frame_id_;
    // microstrain_mips::msg::StatusMsg device_status_msg_;
    bool publish_gps_;
    bool publish_imu_;
    bool publish_odom_;
    bool publish_bias_;
    bool publish_filtered_imu_;
    bool remove_imu_gravity_;
    bool frame_based_enu_;
    std::vector<double> imu_linear_cov_;
    std::vector<double> imu_angular_cov_;
    std::vector<double> imu_orientation_cov_;    //Device Flags
    bool GX5_15;
    bool GX5_25;
    bool GX5_35;
    bool GX5_45;
    bool GQX_45;
    bool RQX_45;
    bool CXX_45;
    bool CVX_10;
    bool CVX_15;
    bool CVX_25;    // Update rates
    int nav_rate_;
    int imu_rate_;
    int gps_rate_;    clock_t start;
    float field_data[3];
    float soft_iron[9];
    float soft_iron_readback[9];
    float angles[3];
    float heading_angle;
    float readback_angles[3];
    float noise[3];
    float beta[3];
    float readback_beta[3];
    float readback_noise[3];
    float offset[3];
    float readback_offset[3];
    u8  com_mode;
    u16 duration;
    u8 reference_position_enable_command;
    u8 reference_position_enable_readback;
    double reference_position_command[3];
    double reference_position_readback[3];
    u8 enable_flag;
    u16 estimation_control;
    u16 estimation_control_readback;
    u8 dynamics_mode;
    u8 readback_dynamics_mode;
    gx4_25_basic_status_field basic_field;
    gx4_25_diagnostic_device_status_field diagnostic_field;
    gx4_45_basic_status_field basic_field_45;
    gx4_45_diagnostic_device_status_field diagnostic_field_45;
    mip_complementary_filter_settings comp_filter_command, comp_filter_readback;
    mip_filter_accel_magnitude_error_adaptive_measurement_command accel_magnitude_error_command, accel_magnitude_error_readback;
    mip_filter_magnetometer_magnitude_error_adaptive_measurement_command mag_magnitude_error_command, mag_magnitude_error_readback;
    mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command mag_dip_angle_error_command, mag_dip_angle_error_readback;
    mip_filter_zero_update_command zero_update_control, zero_update_readback;

  };
#ifdef __cplusplus
  extern "C"
#endif
  {    /**
     * Callback for KF estimate packets from sensor.
     */
    void filter_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    /**
     * Callback for AHRS packets from sensor.
     */
    void ahrs_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    /**
     * Callback for GPS packets from sensor.
     */
    void gps_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
#ifdef __cplusplus
  }
#endif
} // namespace Microstrain

#endif  // _MICROSTRAIN_3DM_GX5_45_H