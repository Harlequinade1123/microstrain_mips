/*

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the microstrain_3dm_gx5_45 package.

microstrain_3dm_gx5_45 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

microstrain_3dm_gx5_45 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <tf2/LinearMath/Transform.h>
#include <string>
#include <algorithm>
#include <time.h>

// #include "microstrain_mips/msg/status_msg.hpp"
// #include "microstrain_diagnostic_updater.h"
#include "microstrain_3dm.h"
#include <vector>

#define MIP_SDK_GX4_45_IMU_STANDARD_MODE  0x01
#define MIP_SDK_GX4_45_IMU_DIRECT_MODE  0x02
#define NUM_COMMAND_LINE_ARGUMENTS 3
#define DEFAULT_PACKET_TIMEOUT_MS  1000 //milliseconds//macro to cause Sleep call to behave as it does for windows
#define Sleep(x) usleep(x*1000.0)
#define GX5_45_DEVICE "3DM-GX5-45"
#define GX5_35_DEVICE "3DM-GX5-35"
#define GX5_25_DEVICE "3DM-GX5-25"
#define GX5_15_DEVICE "3DM-GX5-15"

namespace microstrain
{
  Microstrain::Microstrain()
    : rclcpp::Node("microstrain"),
    // Initialization list
    filter_valid_packet_count_(0),
    ahrs_valid_packet_count_(0),
    gps_valid_packet_count_(0),
    filter_timeout_packet_count_(0),
    ahrs_timeout_packet_count_(0),
    gps_timeout_packet_count_(0),
    filter_checksum_error_packet_count_(0),
    ahrs_checksum_error_packet_count_(0),
    gps_checksum_error_packet_count_(0),
    gps_frame_id_("gps_frame"),
    imu_frame_id_("imu_frame"),
    odom_frame_id_("odom_frame"),
    odom_child_frame_id_("odom_frame"),
    publish_gps_(true),
    publish_imu_(true),
    publish_odom_(true),
    publish_filtered_imu_(false),
    remove_imu_gravity_(false),
    frame_based_enu_(false),
    imu_linear_cov_(std::vector<double>(9, 0.0)),
    imu_angular_cov_(std::vector<double>(9, 0.0)),
    imu_orientation_cov_(std::vector<double>(9, 0.0))
  {
    // pass
  }
  Microstrain::~Microstrain()
  {
    // pass
  }

  void Microstrain::run()
  {
    // Variables for device configuration, ROS parameters, etc.
    u32 com_port, baudrate;
    bool device_setup = false;
    bool readback_settings = true;
    bool save_settings = true;
    bool auto_init = true;
    u8 auto_init_u8 = 1;
    u8 readback_headingsource = 0;
    u8 readback_auto_init = 0;
    int declination_source;
    u8 declination_source_u8;
    u8 readback_declination_source;
    double declination;

    // Variables
    base_device_info_field device_info;
    u8  enable = 1;
    u8  data_stream_format_descriptors[10];
    u16 data_stream_format_decimation[10];
    u8  data_stream_format_num_entries = 0;
    u8  readback_data_stream_format_descriptors[10] = {0};
    u16 readback_data_stream_format_decimation[10]  = {0};
    u8  readback_data_stream_format_num_entries     =  0;
    u16 base_rate = 0;
    u16 device_descriptors[128]  = {0};
    u16 device_descriptors_size  = 128*2;
    u8  gps_source     = 0;
    u8  heading_source = 0x1;
    mip_low_pass_filter_settings filter_settings;
    u16 duration = 0;
    mip_filter_external_gps_update_command external_gps_update;
    mip_filter_external_heading_update_command external_heading_update;
    mip_filter_external_heading_with_time_command external_heading_with_time;

    com_mode = 0;

    // Device model flags
    GX5_15 = false;
    GX5_25 = false;
    GX5_35 = false;
    GX5_45 = false;

    // ROS Parameters
    // Comms Parameters
    std::string port;
    int baud, pdyn_mode;
    port    = this->declare_parameter<std::string>("port", "/dev/ttyACM1");
    baud    = this->declare_parameter<int>("baudrate", 115200);
    baudrate = (u32)baud;
    // Configuration Parameters
    device_setup     = this->declare_parameter<bool>("device_setup", false);
    readback_settings= this->declare_parameter<bool>("readback_settings", true);
    save_settings    = this->declare_parameter<bool>("save_settings", true);

    auto_init        = this->declare_parameter<bool>("auto_init", true);
    gps_rate_        = this->declare_parameter<int>("gps_rate", 1);
    imu_rate_        = this->declare_parameter<int>("imu_rate", 10);
    nav_rate_        = this->declare_parameter<int>("nav_rate", 10);
    pdyn_mode        = this->declare_parameter<int>("dynamics_mode", 1);

    dynamics_mode = (u8)pdyn_mode;
    if (dynamics_mode < 1 || dynamics_mode > 3)
    {
      RCLCPP_WARN(this->get_logger(),"dynamics_mode can't be %#04X, must be 1, 2 or 3.  Setting to 1.", dynamics_mode);
      dynamics_mode = 1;
    }

    declination_source = this->declare_parameter<int>("declination_source", 2);
    if (declination_source < 1 || declination_source > 3)
    {
      RCLCPP_WARN(this->get_logger(),"declination_source can't be %#04X, must be 1, 2 or 3.  Setting to 2.", declination_source);
      declination_source = 2;
    }
    declination_source_u8 = (u8)declination_source;

    // declination_source_command=(u8)declination_source;
    declination       = this->declare_parameter<double>("declination", 0.23);
    gps_frame_id_     = this->declare_parameter<std::string>("gps_frame_id", "wgs84");
    imu_frame_id_     = this->declare_parameter<std::string>("imu_frame_id", "base_link");
    odom_frame_id_    = this->declare_parameter<std::string>("odom_frame_id", "wgs84");
    odom_child_frame_id_= this->declare_parameter<std::string>("odom_child_frame_id", "base_link");

    publish_imu_          = this->declare_parameter<bool>("publish_imu", true);
    publish_bias_         = this->declare_parameter<bool>("publish_bias", true);
    publish_filtered_imu_ = this->declare_parameter<bool>("publish_filtered_imu", false);
    remove_imu_gravity_   = this->declare_parameter<bool>("remove_imu_gravity", false);
    frame_based_enu_      = this->declare_parameter<bool>("frame_based_enu", false);

    // Covariance parameters to set the sensor_msg/IMU covariance values
    std::vector<double> default_cov(9, 0.0);
    imu_orientation_cov_ = this->declare_parameter<std::vector<double>>("imu_orientation_cov", default_cov);
    imu_linear_cov_      = this->declare_parameter<std::vector<double>>("imu_linear_cov", default_cov);
    imu_angular_cov_     = this->declare_parameter<std::vector<double>>("imu_angular_cov", default_cov);

    // ROS publishers and subscribers
    if (publish_imu_)
    {
      imu_pub_        = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", 100);
      imu_correct_pub_= this->create_publisher<sensor_msgs::msg::Imu>("imu_correct", 100); // tixiao
    }
    if (publish_filtered_imu_)
      filtered_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("filtered/imu/data", 100);

    // Publishes device status
    // device_status_pub_ = this->create_publisher<microstrain_mips::msg::StatusMsg>("device/status", rclcpp::QoS(100));

    // Initialize the serial interface to the device
    RCLCPP_INFO(this->get_logger(), "Attempting to open serial port <%s> at <%d> \n", port.c_str(), baudrate);
    if (mip_interface_init(port.c_str(), baudrate,
        &device_interface_, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK)
    {
      RCLCPP_FATAL(this->get_logger(),"Couldn't open serial port!  Is it plugged in?");
    }

    // We want to get the default device info even if we don't setup the device
    // Get device info
    start = clock();
    while (mip_base_cmd_get_device_info(&device_interface_, &device_info) != MIP_INTERFACE_OK)
    {
      if (clock() - start > 5000)
      {
        RCLCPP_INFO(this->get_logger(), "mip_base_cmd_get_device_info function timed out.");
        rclcpp::shutdown();
        break;
      }
    }

    // Get device model name
    memset(temp_string, 0, 20*sizeof(char));
    memcpy(temp_string, device_info.model_name, BASE_DEVICE_INFO_PARAM_LENGTH*2);
    RCLCPP_INFO(this->get_logger(), "Model Name  => %s\n", temp_string);
    std::string model_name;

    for (int i = 6; i < 20; i++)
    {
      model_name += temp_string[i];
    }

    // Set device model flag
    model_name = model_name.c_str();
    if (model_name == GX5_45_DEVICE)
    {
      GX5_45 = true;
    }
    if (model_name == GX5_35_DEVICE)
    {
      GX5_35 = true;
    }
    if (model_name == GX5_25_DEVICE)
    {
      // GX5_25 = true;
    }
    if (model_name == GX5_15_DEVICE)
    {
      GX5_15 = true;
    }

    ////////////////////////////////////////
    // Device setup
    float dT = 0.5;  // common sleep time after setup communications
    if (device_setup)
    {
      // Put device into standard mode - we never really use "direct mode"
      RCLCPP_INFO(this->get_logger(), "Putting device communications into 'standard mode'");
      device_descriptors_size  = 128*2;
      com_mode = MIP_SDK_GX4_45_IMU_STANDARD_MODE;
      start = clock();
      while (mip_system_com_mode(&device_interface_,
          MIP_FUNCTION_SELECTOR_WRITE, &com_mode) != MIP_INTERFACE_OK)
      {
        if (clock() - start > 5000)
        {
          RCLCPP_INFO(this->get_logger(), "mip_system_com_mode function timed out.");
          break;
        }
      }

      // Verify device mode setting
      RCLCPP_INFO(this->get_logger(), "Verify comm's mode");
      start = clock();
      while (mip_system_com_mode(&device_interface_,
          MIP_FUNCTION_SELECTOR_READ, &com_mode) != MIP_INTERFACE_OK)
      {
        if (clock() - start > 5000)
        {
          RCLCPP_INFO(this->get_logger(), "mip_system_com_mode function timed out.");
          break;
        }
      }

      RCLCPP_INFO(this->get_logger(), "Sleep for a second...");
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
      RCLCPP_INFO(this->get_logger(), "Right mode?");
      if (com_mode != MIP_SDK_GX4_45_IMU_STANDARD_MODE)
      {
        RCLCPP_ERROR(this->get_logger(), "Appears we didn't get into standard mode!");
      }

      // Set GPS publishing to true if IMU model has GPS
      if (GX5_45 || GX5_35)
      {
        publish_gps_  = this->declare_parameter<bool>("publish_gps", true);
        publish_odom_ = this->declare_parameter<bool>("publish_odom", true);
      }
      else
      {
        publish_gps_  = this->declare_parameter<bool>("publish_gps", false);
        publish_odom_ = this->declare_parameter<bool>("publish_odom", false);
      }
      
      if (publish_gps_)
      {
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 100);
      }
      
      if (publish_odom_)
      {
        nav_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("nav/odom", 100);
      }
      
      // This is the EKF filter status, not just navigation/odom status
      if (publish_odom_ || publish_filtered_imu_)
      {
        nav_status_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("nav/status", 100);
      }

      // Setup device callbacks
      if (mip_interface_add_descriptor_set_callback(&device_interface_, MIP_FILTER_DATA_SET,
          this, &filter_packet_callback_wrapper) != MIP_INTERFACE_OK)
      {
        RCLCPP_FATAL(this->get_logger(),"Can't setup filter callback!");
        return;
      }

      if (mip_interface_add_descriptor_set_callback(&device_interface_, MIP_AHRS_DATA_SET,
          this, &ahrs_packet_callback_wrapper) != MIP_INTERFACE_OK)
      {
        RCLCPP_FATAL(this->get_logger(),"Can't setup ahrs callbacks!");
        return;
      }

      if (mip_interface_add_descriptor_set_callback(&device_interface_, MIP_GPS_DATA_SET,
          this, &gps_packet_callback_wrapper) != MIP_INTERFACE_OK)
      {
        RCLCPP_FATAL(this->get_logger(),"Can't setup gpscallbacks!");
        return;
      }

      // Put into idle mode
      RCLCPP_INFO(this->get_logger(), "Idling Device: Stopping data streams and/or waking from sleep");
      start = clock();
      while (mip_base_cmd_idle(&device_interface_) != MIP_INTERFACE_OK)
      {
        if (clock() - start > 5000)
        {
          RCLCPP_INFO(this->get_logger(), "mip_base_cmd_idle function timed out.");
          break;
        }
      }
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));

      // AHRS Setup
      // Get base rate
      if (publish_imu_ || publish_filtered_imu_)
      {
        start = clock();
        while (mip_3dm_cmd_get_ahrs_base_rate(&device_interface_, &base_rate) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_get_ahrs_base_rate function timed out.");
            rclcpp::shutdown();
            break;
          }
        }

        RCLCPP_INFO(this->get_logger(), "AHRS Base Rate => %d Hz", base_rate);
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        // Deterimine decimation to get close to goal rate (We use the highest of the imu rates)
        int rate = (imu_rate_ > nav_rate_) ? imu_rate_ : nav_rate_;
        u8 imu_decimation = (u8)(static_cast<float>(base_rate)/ static_cast<float>(rate));
        RCLCPP_INFO(this->get_logger(), "AHRS decimation set to %#04X", imu_decimation);

        // AHRS Message Format
        // Set message format
        RCLCPP_INFO(this->get_logger(), "Setting the AHRS message format");
        data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED;
        data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED;
        data_stream_format_descriptors[2] = MIP_AHRS_DATA_QUATERNION;
        data_stream_format_decimation[0]  = imu_decimation;  // 0x32;
        data_stream_format_decimation[1]  = imu_decimation;  // 0x32;
        data_stream_format_decimation[2]  = imu_decimation;  // 0x32;
        data_stream_format_num_entries = 3;

        start = clock();
        while (mip_3dm_cmd_ahrs_message_format(&device_interface_,
            MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries,
            data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_ahrs_message_format function timed out.");
            break;
          }
        }

        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        // Poll to verify
        RCLCPP_INFO(this->get_logger(), "Poll AHRS data to verify");
        start = clock();
        while (mip_3dm_cmd_poll_ahrs(&device_interface_,
            MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries,
            data_stream_format_descriptors) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_poll_ahrs function timed out.");
            break;
          }
        }

        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        // Save
        if (save_settings)
        {
          RCLCPP_INFO(this->get_logger(), "Saving AHRS data settings");
          start = clock();
          while (mip_3dm_cmd_ahrs_message_format(&device_interface_,
              MIP_FUNCTION_SELECTOR_STORE_EEPROM, 0, NULL, NULL) != MIP_INTERFACE_OK)
          {
            if (clock() - start > 5000)
            {
              RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_ahrs_message_format function timed out.");
              break;
            }
          }
          rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        }

        // Declination Source
        // Set declination
        RCLCPP_INFO(this->get_logger(), "Setting declination source to %#04X", declination_source_u8);
        start = clock();
        while (mip_filter_declination_source(&device_interface_,
            MIP_FUNCTION_SELECTOR_WRITE, &declination_source_u8) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_filter_declination_source function timed out.");
            break;
          }
        }

        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        // Read back the declination source
        RCLCPP_INFO(this->get_logger(), "Reading back declination source");
        start = clock();
        while (mip_filter_declination_source(&device_interface_,
            MIP_FUNCTION_SELECTOR_READ, &readback_declination_source) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_filter_declination_source function timed out.");
            break;
          }
        }

        if (declination_source_u8 == readback_declination_source)
        {
          RCLCPP_INFO(this->get_logger(), "Success: Declination source set to %#04X", declination_source_u8);
        }
        else
        {
          RCLCPP_WARN(this->get_logger(),"Failed to set the declination source to %#04X!", declination_source_u8);
        }

        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        if (save_settings)
        {
          RCLCPP_INFO(this->get_logger(), "Saving declination source settings to EEPROM");
          start = clock();
          while (mip_filter_declination_source(&device_interface_,
              MIP_FUNCTION_SELECTOR_STORE_EEPROM, NULL) != MIP_INTERFACE_OK)
          {
            if (clock() - start > 5000)
            {
              RCLCPP_INFO(this->get_logger(), "mip_filter_declination_source function timed out.");
              break;
            }
          }

          rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        }
      }  // end of AHRS setup


      // GPS Setup
      if (publish_gps_)
      {
        start = clock();
        while (mip_3dm_cmd_get_gps_base_rate(&device_interface_, &base_rate) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_get_gps_base_rate function timed out.");
            break;
          }
        }

        RCLCPP_INFO(this->get_logger(), "GPS Base Rate => %d Hz", base_rate);
        u8 gps_decimation = (u8)(static_cast<float>(base_rate)/ static_cast<float>(gps_rate_));
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));

        ////////// GPS Message Format
        // Set
        RCLCPP_INFO(this->get_logger(), "Setting GPS stream format");
        data_stream_format_descriptors[0] = MIP_GPS_DATA_LLH_POS;
        data_stream_format_descriptors[1] = MIP_GPS_DATA_NED_VELOCITY;
        data_stream_format_descriptors[2] = MIP_GPS_DATA_GPS_TIME;
        data_stream_format_decimation[0]  = gps_decimation;  // 0x01; // 0x04;
        data_stream_format_decimation[1]  = gps_decimation;  // 0x01; // 0x04;
        data_stream_format_decimation[2]  = gps_decimation;  // 0x01; // 0x04;
        data_stream_format_num_entries = 3;
        start = clock();

        while (mip_3dm_cmd_gps_message_format(&device_interface_,
            MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries,
            data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_gps_message_format function timed out.");
            break;
          }
        }

        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        // Save
        if (save_settings)
        {
          RCLCPP_INFO(this->get_logger(), "Saving GPS data settings");
          start = clock();
          while (mip_3dm_cmd_gps_message_format(&device_interface_,
              MIP_FUNCTION_SELECTOR_STORE_EEPROM, 0, NULL, NULL) != MIP_INTERFACE_OK)
          {
            if (clock() - start > 5000)
            {
              RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_gps_message_format function timed out.");
              break;
            }
          }
          rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        }
      }  // end of GPS setup

      // Filter setup
      if (publish_odom_ || publish_filtered_imu_)
      {
        start = clock();
        while (mip_3dm_cmd_get_filter_base_rate(&device_interface_, &base_rate) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_get_filter_base_rate function timed out.");
            break;
          }
        }

        RCLCPP_INFO(this->get_logger(), "FILTER Base Rate => %d Hz", base_rate);
        // If we have made it this far in this statement, we know we want to publish filtered data
        // from the IMU. We need to make sure to set the data rate to the correct speed dependent
        // upon which filtered field we are after. Thus make sure we get the fastest data rate.
        int rate = nav_rate_;
        if(publish_filtered_imu_)
        {
          // Set filter rate based on max of filter topic rates
          rate = (imu_rate_ > nav_rate_) ? imu_rate_ : nav_rate_;
        }

        u8 nav_decimation = (u8)(static_cast<float>(base_rate)/ static_cast<float>(rate));
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));

        ////////// Filter Message Format
        // Set
        RCLCPP_INFO(this->get_logger(), "Setting Filter stream format");

        // Order doesn't matter since we parse them with a case statement below.
        // First start by loading the common values.
        data_stream_format_descriptors[0] = MIP_FILTER_DATA_ATT_QUATERNION;
        data_stream_format_descriptors[1] = MIP_FILTER_DATA_ATT_UNCERTAINTY_EULER;
        data_stream_format_descriptors[2] = MIP_FILTER_DATA_COMPENSATED_ANGULAR_RATE;
        data_stream_format_descriptors[3] = MIP_FILTER_DATA_FILTER_STATUS;
        data_stream_format_decimation[0]  = nav_decimation;  // 0x32;
        data_stream_format_decimation[1]  = nav_decimation;  // 0x32;
        data_stream_format_decimation[2]  = nav_decimation;  // 0x32;
        data_stream_format_decimation[3]  = nav_decimation;  // 0x32;

        // If we want the odometry add that data
        if (publish_odom_ && publish_filtered_imu_)
        {
          // Size is up to 10 elements
          data_stream_format_descriptors[4] = MIP_FILTER_DATA_LLH_POS;
          data_stream_format_descriptors[5] = MIP_FILTER_DATA_NED_VEL;
          // data_stream_format_descriptors[2] = MIP_FILTER_DATA_ATT_EULER_ANGLES;
          data_stream_format_descriptors[6] = MIP_FILTER_DATA_POS_UNCERTAINTY;
          data_stream_format_descriptors[7] = MIP_FILTER_DATA_VEL_UNCERTAINTY;

          // The filter has one message that removes gravity and one that does not
          if (remove_imu_gravity_)
          {
            data_stream_format_descriptors[8] = MIP_FILTER_DATA_LINEAR_ACCELERATION;
          }
          else
          {
            data_stream_format_descriptors[8] = MIP_FILTER_DATA_COMPENSATED_ACCELERATION;
          }

          data_stream_format_decimation[4]  = nav_decimation;  // 0x32;
          data_stream_format_decimation[5]  = nav_decimation;  // 0x32;
          data_stream_format_decimation[6]  = nav_decimation;  // 0x32;
          data_stream_format_decimation[7]  = nav_decimation;  // 0x32;
          data_stream_format_decimation[8]  = nav_decimation;  // 0x32;
          data_stream_format_num_entries = 9;
        }
        else if (publish_odom_ && !publish_filtered_imu_)
        {
          data_stream_format_descriptors[4] = MIP_FILTER_DATA_LLH_POS;
          data_stream_format_descriptors[5] = MIP_FILTER_DATA_NED_VEL;
          // data_stream_format_descriptors[2] = MIP_FILTER_DATA_ATT_EULER_ANGLES;
          data_stream_format_descriptors[6] = MIP_FILTER_DATA_POS_UNCERTAINTY;
          data_stream_format_descriptors[7] = MIP_FILTER_DATA_VEL_UNCERTAINTY;

          data_stream_format_decimation[4]  = nav_decimation;  // 0x32;
          data_stream_format_decimation[5]  = nav_decimation;  // 0x32;
          data_stream_format_decimation[6]  = nav_decimation;  // 0x32;
          data_stream_format_decimation[7]  = nav_decimation;  // 0x32;
          data_stream_format_num_entries = 8;
        }
        else
        {
          // The filter has one message that removes gravity and one that does not
          if (remove_imu_gravity_)
          {
            data_stream_format_descriptors[4] = MIP_FILTER_DATA_LINEAR_ACCELERATION;
          }
          else
          {
            data_stream_format_descriptors[4] = MIP_FILTER_DATA_COMPENSATED_ACCELERATION;
          }
          data_stream_format_decimation[4]  = nav_decimation;  // 0x32;
          data_stream_format_num_entries = 5;
        }

        start = clock();
        while (mip_3dm_cmd_filter_message_format(&device_interface_,
            MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries,
            data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_filter_message_format function timed out.");
            break;
          }
        }

        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        // Poll to verify
        RCLCPP_INFO(this->get_logger(), "Poll filter data to test stream");
        start = clock();
        while (mip_3dm_cmd_poll_filter(&device_interface_,
            MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries,
            data_stream_format_descriptors) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_poll_filter function timed out.");
            break;
          }
        }

        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        // Save
        if (save_settings)
        {
          RCLCPP_INFO(this->get_logger(), "Saving Filter data settings");
          start = clock();
          while (mip_3dm_cmd_filter_message_format(&device_interface_,
              MIP_FUNCTION_SELECTOR_STORE_EEPROM, 0, NULL, NULL) != MIP_INTERFACE_OK)
          {
            if (clock() - start > 5000)
            {
              RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_filter_message_format function timed out.");
              break;
            }
          }
          rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        }


        // GX5_25 doesn't appear to suport this feature thus GX5_15 probably won't either
        if (GX5_35 == true || GX5_45 == true)
        {
          // Dynamics Mode
          // Set dynamics mode
          RCLCPP_INFO(this->get_logger(), "Setting dynamics mode to %#04X", dynamics_mode);
          start = clock();
          while (mip_filter_vehicle_dynamics_mode(&device_interface_,
              MIP_FUNCTION_SELECTOR_WRITE, &dynamics_mode) != MIP_INTERFACE_OK)
          {
            if (clock() - start > 5000)
            {
              RCLCPP_INFO(this->get_logger(), "mip_filter_vehicle_dynamics_mode function timed out.");
              break;
            }
          }

          rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
          // Readback dynamics mode
          if (readback_settings)
          {
            // Read the settings back
            RCLCPP_INFO(this->get_logger(), "Reading back dynamics mode setting");
            start = clock();
            while (mip_filter_vehicle_dynamics_mode(&device_interface_,
                MIP_FUNCTION_SELECTOR_READ, &readback_dynamics_mode) != MIP_INTERFACE_OK)
            {
              if (clock() - start > 5000)
              {
                RCLCPP_INFO(this->get_logger(), "mip_filter_vehicle_dynamics_mode function timed out.");
                break;
              }
            }

            rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
            if (dynamics_mode == readback_dynamics_mode)
              RCLCPP_INFO(this->get_logger(), "Success: Dynamics mode setting is: %#04X", readback_dynamics_mode);
            else
              RCLCPP_ERROR(this->get_logger(), "Failure: Dynamics mode set to be %#04X, but reads as %#04X",
                  dynamics_mode, readback_dynamics_mode);
          }

          if (save_settings)
          {
            RCLCPP_INFO(this->get_logger(), "Saving dynamics mode settings to EEPROM");
            start = clock();
            while (mip_filter_vehicle_dynamics_mode(&device_interface_,
                MIP_FUNCTION_SELECTOR_STORE_EEPROM, NULL) != MIP_INTERFACE_OK)
            {
              if (clock() - start > 5000)
              {
                RCLCPP_INFO(this->get_logger(), "mip_filter_vehicle_dynamics_mode function timed out.");
                break;
              }
            }
            rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
          }
        }

        // Set heading Source
        RCLCPP_INFO(this->get_logger(), "Set heading source to internal mag.");
        heading_source = 0x1;
        RCLCPP_INFO(this->get_logger(), "Setting heading source to %#04X", heading_source);
        start = clock();
        while (mip_filter_heading_source(&device_interface_,
            MIP_FUNCTION_SELECTOR_WRITE, &heading_source) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_filter_heading_source function timed out.");
            break;
          }
        }
        // Read back heading source
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        RCLCPP_INFO(this->get_logger(), "Read back heading source...");
        start = clock();
        while (mip_filter_heading_source(&device_interface_,
            MIP_FUNCTION_SELECTOR_READ, &readback_headingsource)!= MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_filter_heading_source function timed out.");
            break;
          }
        }

        RCLCPP_INFO(this->get_logger(), "Heading source = %#04X", readback_headingsource);
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));

        if (save_settings)
        {
          RCLCPP_INFO(this->get_logger(), "Saving heading source to EEPROM");
          start = clock();
          while (mip_filter_heading_source(&device_interface_,
              MIP_FUNCTION_SELECTOR_STORE_EEPROM, NULL)!= MIP_INTERFACE_OK)
          {
            if (clock() - start > 5000)
            {
              RCLCPP_INFO(this->get_logger(), "mip_filter_heading_source function timed out.");
              break;
            }
          }
          rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        }
      }  // end of Filter setup

      // Set auto-initialization based on ROS parameter
      RCLCPP_INFO(this->get_logger(), "Setting auto-initinitalization to: %#04X", auto_init);
      auto_init_u8 = auto_init;  // convert bool to u8
      start = clock();
      while (mip_filter_auto_initialization(&device_interface_,
          MIP_FUNCTION_SELECTOR_WRITE, &auto_init_u8) != MIP_INTERFACE_OK)
      {
        if (clock() - start > 5000)
        {
          RCLCPP_INFO(this->get_logger(), "mip_filter_auto_initialization function timed out.");
          break;
        }
      }
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));

      if (readback_settings)
      {
        // Read the settings back
        RCLCPP_INFO(this->get_logger(), "Reading back auto-initialization value");
        start = clock();
        while (mip_filter_auto_initialization(&device_interface_,
            MIP_FUNCTION_SELECTOR_READ, &readback_auto_init)!= MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_filter_auto_initialization function timed out.");
            break;
          }
        }

        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
        if (auto_init == readback_auto_init)
          RCLCPP_INFO(this->get_logger(), "Success: Auto init. setting is: %#04X",
              readback_auto_init);
        else
          RCLCPP_ERROR(this->get_logger(), "Failure: Auto init. setting set to be %#04X, but reads as %#04X",
              auto_init, readback_auto_init);
      }

      if (save_settings)
      {
        RCLCPP_INFO(this->get_logger(), "Saving auto init. settings to EEPROM");
        start = clock();
        while (mip_filter_auto_initialization(&device_interface_,
            MIP_FUNCTION_SELECTOR_STORE_EEPROM, NULL) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_filter_auto_initialization function timed out.");
            break;
          }
        }
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
      }

      // Enable Data streams
      dT = 0.25;
      if (publish_imu_ || publish_filtered_imu_)
      {
        RCLCPP_INFO(this->get_logger(), "Enabling AHRS stream");
        enable = 0x01;
        start = clock();
        while (mip_3dm_cmd_continuous_data_stream(&device_interface_,
            MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_AHRS_DATASTREAM, &enable) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_continuous_data_stream function timed out.");
            break;
          }
        }
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
      }

      if (publish_odom_)
      {
        RCLCPP_INFO(this->get_logger(), "Enabling Filter stream");
        enable = 0x01;
        start = clock();
        while (mip_3dm_cmd_continuous_data_stream(&device_interface_,
            MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_INS_DATASTREAM, &enable) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_continuous_data_stream function timed out.");
            break;
          }
        }
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
      }

      if (publish_gps_)
      {
        RCLCPP_INFO(this->get_logger(), "Enabling GPS stream");
        enable = 0x01;
        start = clock();
        while (mip_3dm_cmd_continuous_data_stream(&device_interface_,
            MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_GPS_DATASTREAM, &enable) != MIP_INTERFACE_OK)
        {
          if (clock() - start > 5000)
          {
            RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_continuous_data_stream function timed out.");
            break;
          }
        }
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));
      }

      RCLCPP_INFO(this->get_logger(), "End of device setup - starting streaming");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Skipping device setup and listing for existing streams");
    }  // end of device_setup

    // Reset filter - should be for either the KF or CF
    RCLCPP_INFO(this->get_logger(), "Reset filter");
    start = clock();
    while (mip_filter_reset_filter(&device_interface_) != MIP_INTERFACE_OK)
    {
      if (clock() - start > 5000)
      {
        RCLCPP_INFO(this->get_logger(), "mip_filter_reset_filter function timed out.");
        break;
      }
    }
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(dT*1000)));

    // Loop
    // Determine loop rate as 2*(max update rate), but abs. max of 1kHz
    int max_rate = 1;
    if (publish_imu_)
    {
      max_rate = std::max(max_rate, imu_rate_);
    }
    if (publish_filtered_imu_)
    {
      max_rate = std::max(std::max(max_rate, nav_rate_),imu_rate_);
    }
    if (publish_gps_)
    {
      max_rate = std::max(max_rate, gps_rate_);
    }
    if (publish_odom_)
    {
      max_rate = std::max(max_rate, nav_rate_);
    }
    int spin_rate = std::min(3*max_rate, 1000);
    RCLCPP_INFO(this->get_logger(), "Setting spin rate to <%d>", spin_rate);
    rclcpp::Rate r(spin_rate);  // Rate in Hz

    // microstrain_mips::RosDiagnosticUpdater ros_diagnostic_updater(this);

    while (rclcpp::ok())
    {
      // Update the parser (this function reads the port and parses the bytes
      mip_interface_update(&device_interface_);

      if (GX5_25)
      {
        device_status_callback();
      }

      // ros::spinOnce();  // take care of service requests.
      rclcpp::spin_some(this->get_node_base_interface());
      r.sleep();
    }  // end loop

    // close serial port
    mip_sdk_port_close(device_interface_.port_handle);
  }  // End of ::run()

  // Get basic or diagnostic status of device. Called by basic and diagnostic services.
  u16 Microstrain::mip_3dm_cmd_hw_specific_device_status(mip_interface *device_interface,
      u16 model_number, u8 status_selector, u8 *response_buffer)
  {
    int total_size = 0;
    if (GX5_25)
    {
      gx4_25_basic_status_field *basic_ptr;
      gx4_25_diagnostic_device_status_field *diagnostic_ptr;
      u16 response_size = MIP_FIELD_HEADER_SIZE;

      // Set response size based on device model and whether basic or diagnostic status is chosen
      if (status_selector == GX4_25_BASIC_STATUS_SEL)
      {
        response_size += sizeof(gx4_25_basic_status_field);
      }
      else if (status_selector == GX4_25_DIAGNOSTICS_STATUS_SEL)
      {
        response_size += sizeof(gx4_25_diagnostic_device_status_field);
      }

      while (mip_3dm_cmd_device_status(device_interface, model_number,
          status_selector, response_buffer, &response_size) != MIP_INTERFACE_OK) {}

      if (status_selector == GX4_25_BASIC_STATUS_SEL)
      {
        if (response_size != sizeof(gx4_25_basic_status_field))
        {
          return MIP_INTERFACE_ERROR;
        }
        else if (MIP_SDK_CONFIG_BYTESWAP)
        {
           // Perform byteswapping
           byteswap_inplace(&response_buffer[0], sizeof(basic_field.device_model));
           byteswap_inplace(&response_buffer[2], sizeof(basic_field.status_selector));
           byteswap_inplace(&response_buffer[3], sizeof(basic_field.status_flags));
           byteswap_inplace(&response_buffer[7], sizeof(basic_field.system_state));
           byteswap_inplace(&response_buffer[9], sizeof(basic_field.system_timer_ms));
        }

        void * struct_pointer;
        struct_pointer = &basic_field;

        // Copy response from response buffer to basic status struct
        memcpy(struct_pointer, response_buffer, sizeof(basic_field.device_model));
        memcpy((struct_pointer+2), &(response_buffer[2]), sizeof(basic_field.status_selector));
        memcpy((struct_pointer+3), &(response_buffer[3]), sizeof(basic_field.status_flags));
        memcpy((struct_pointer+7), &(response_buffer[7]), sizeof(basic_field.system_state));
        memcpy((struct_pointer+9), &(response_buffer[9]), sizeof(basic_field.system_timer_ms));
      }
      else if (status_selector == GX4_25_DIAGNOSTICS_STATUS_SEL)
      {
        if (response_size != sizeof(gx4_25_diagnostic_device_status_field))
        {
          return MIP_INTERFACE_ERROR;
        }
        else if (MIP_SDK_CONFIG_BYTESWAP)
        {
          // byteswap and copy response to diagnostic status struct
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.device_model));
          total_size += sizeof(diagnostic_field.device_model);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.status_selector));
          total_size += sizeof(diagnostic_field.status_selector);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.status_flags));
          total_size += sizeof(diagnostic_field.status_flags);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.system_state));
          total_size += sizeof(diagnostic_field.system_state);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.system_timer_ms));
          total_size += sizeof(diagnostic_field.system_timer_ms);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.imu_stream_enabled));
          total_size += sizeof(diagnostic_field.imu_stream_enabled);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.filter_stream_enabled));
          total_size += sizeof(diagnostic_field.filter_stream_enabled);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.imu_dropped_packets));
          total_size += sizeof(diagnostic_field.imu_dropped_packets);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.filter_dropped_packets));
          total_size += sizeof(diagnostic_field.filter_dropped_packets);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.com1_port_bytes_written));
          total_size += sizeof(diagnostic_field.com1_port_bytes_written);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.com1_port_bytes_read));
          total_size += sizeof(diagnostic_field.com1_port_bytes_read);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.com1_port_write_overruns));
          total_size += sizeof(diagnostic_field.com1_port_write_overruns);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.com1_port_read_overruns));
          total_size += sizeof(diagnostic_field.com1_port_read_overruns);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.imu_parser_errors));
          total_size += sizeof(diagnostic_field.imu_parser_errors);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.imu_message_count));
          total_size += sizeof(diagnostic_field.imu_message_count);
          byteswap_inplace(&response_buffer[total_size],
              sizeof(diagnostic_field.imu_last_message_ms));
        }

        void * struct_pointer;
        struct_pointer = &diagnostic_field;
        total_size = 0;

        memcpy(struct_pointer, response_buffer,
            sizeof(diagnostic_field.device_model));
        total_size += sizeof(diagnostic_field.device_model);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.status_selector));
        total_size += sizeof(diagnostic_field.status_selector);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.status_flags));
        total_size += sizeof(diagnostic_field.status_flags);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.system_state));
        total_size += sizeof(diagnostic_field.system_state);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.system_timer_ms));
        total_size += sizeof(diagnostic_field.system_timer_ms);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.imu_stream_enabled));
        total_size += sizeof(diagnostic_field.imu_stream_enabled);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.filter_stream_enabled));
        total_size += sizeof(diagnostic_field.filter_stream_enabled);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.imu_dropped_packets));
        total_size += sizeof(diagnostic_field.imu_dropped_packets);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.filter_dropped_packets));
        total_size += sizeof(diagnostic_field.filter_dropped_packets);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.com1_port_bytes_written));
        total_size += sizeof(diagnostic_field.com1_port_bytes_written);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.com1_port_bytes_read));
        total_size += sizeof(diagnostic_field.com1_port_bytes_read);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.com1_port_write_overruns));
        total_size += sizeof(diagnostic_field.com1_port_write_overruns);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.com1_port_read_overruns));
        total_size += sizeof(diagnostic_field.com1_port_read_overruns);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.imu_parser_errors));
        total_size += sizeof(diagnostic_field.imu_parser_errors);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.imu_message_count));
        total_size += sizeof(diagnostic_field.imu_message_count);
        memcpy((struct_pointer + total_size), &(response_buffer[total_size]),
            sizeof(diagnostic_field.imu_last_message_ms));
        total_size += sizeof(diagnostic_field.imu_last_message_ms);
      }
      else
        return MIP_INTERFACE_ERROR;

      return MIP_INTERFACE_OK;
    }
  }

  // Start callbacks for data packets

  ////////////////////////////////////////////////////////////////////////////////
  //
  // Filter Packet Callback
  //
  ////////////////////////////////////////////////////////////////////////////////
  void Microstrain::filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;

    // If we aren't publishing, then return
    if (!publish_odom_ && !publish_filtered_imu_)
      return;

    // RCLCPP_INFO(this->get_logger(), "Filter callback");
    // The packet callback can have several types, process them all
    switch (callback_type)
    {
      ///
      // Handle valid packets
      ///
      case MIP_INTERFACE_CALLBACK_VALID_PACKET:
      {
        filter_valid_packet_count_++;

        ///
        // Loop through all of the data fields
        ///

        while (mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
        {
          ///
          // Decode the field
          ///

          switch (field_header->descriptor)
          {
            ///
            // Estimated LLH Position
            ///

            case MIP_FILTER_DATA_LLH_POS:
            {
              memcpy(&curr_filter_pos_, field_data, sizeof(mip_filter_llh_pos));

              // For little-endian targets, byteswap the data field
              mip_filter_llh_pos_byteswap(&curr_filter_pos_);

              // nav_msg_.header.seq = filter_valid_packet_count_;
              nav_msg_.header.stamp = this->now();
              nav_msg_.header.frame_id = odom_frame_id_;
              nav_msg_.child_frame_id = odom_child_frame_id_;
              nav_msg_.pose.pose.position.y = curr_filter_pos_.latitude;
              nav_msg_.pose.pose.position.x = curr_filter_pos_.longitude;
              nav_msg_.pose.pose.position.z = curr_filter_pos_.ellipsoid_height;
            }
            break;

            ///
            // Estimated NED Velocity
            ///

            case MIP_FILTER_DATA_NED_VEL:
            {
              memcpy(&curr_filter_vel_, field_data, sizeof(mip_filter_ned_velocity));

              // For little-endian targets, byteswap the data field
              mip_filter_ned_velocity_byteswap(&curr_filter_vel_);

              // rotate velocities from NED to sensor coordinates
              // Constructor takes x, y, z , w
              tf2::Quaternion nav_quat(curr_filter_quaternion_.q[2],
                     curr_filter_quaternion_.q[1],
                     -1.0*curr_filter_quaternion_.q[3],
                     curr_filter_quaternion_.q[0]);

              tf2::Vector3 vel_enu(curr_filter_vel_.east,
                 curr_filter_vel_.north,
                 -1.0*curr_filter_vel_.down);
              tf2::Vector3 vel_in_sensor_frame = tf2::quatRotate(nav_quat.inverse(), vel_enu);

              nav_msg_.twist.twist.linear.x = vel_in_sensor_frame[0];  // curr_filter_vel_.east;
              nav_msg_.twist.twist.linear.y =  vel_in_sensor_frame[1];  // curr_filter_vel_.north;
              nav_msg_.twist.twist.linear.z =  vel_in_sensor_frame[2];  // -1*curr_filter_vel_.down;
            }
            break;

            ///
            // Estimated Attitude, Euler Angles
            ///

            case MIP_FILTER_DATA_ATT_EULER_ANGLES:
            {
              memcpy(&curr_filter_angles_, field_data, sizeof(mip_filter_attitude_euler_angles));

              // For little-endian targets, byteswap the data field
              mip_filter_attitude_euler_angles_byteswap(&curr_filter_angles_);
            }
            break;

            // Quaternion
            case MIP_FILTER_DATA_ATT_QUATERNION:
            {
              memcpy(&curr_filter_quaternion_, field_data, sizeof(mip_filter_attitude_quaternion));

              // For little-endian targets, byteswap the data field
              mip_filter_attitude_quaternion_byteswap(&curr_filter_quaternion_);

              // If we want the orientation to be based on the reference on the imu
              tf2::Quaternion q(curr_filter_quaternion_.q[1],curr_filter_quaternion_.q[2],
                                curr_filter_quaternion_.q[3],curr_filter_quaternion_.q[0]);
              geometry_msgs::msg::Quaternion quat_msg;

              if(frame_based_enu_)
              {
                // Create a rotation from NED -> ENU
                tf2::Quaternion q_rotate;
                q_rotate.setRPY(M_PI,0.0,M_PI/2);
                // Apply the NED to ENU rotation such that the coordinate frame matches
                q = q_rotate*q;
                quat_msg = tf2::toMsg(q);
              }
              else
              {
                // put into ENU - swap X/Y, invert Z
                quat_msg.x = q[1];
                quat_msg.y = q[0];
                quat_msg.z = -1.0*q[2];
                quat_msg.w = q[3];
              }

              nav_msg_.pose.pose.orientation = quat_msg;

              if (publish_filtered_imu_)
              {
                // Header
                // filtered_imu_msg_.header.seq = filter_valid_packet_count_;
                filtered_imu_msg_.header.stamp = this->now();
                filtered_imu_msg_.header.frame_id = imu_frame_id_;
                filtered_imu_msg_.orientation = nav_msg_.pose.pose.orientation;
              }
            }
            break;

            // Angular Rates
            case MIP_FILTER_DATA_COMPENSATED_ANGULAR_RATE:
            {
              memcpy(&curr_filter_angular_rate_, field_data, sizeof(mip_filter_compensated_angular_rate));

              // For little-endian targets, byteswap the data field
              mip_filter_compensated_angular_rate_byteswap(&curr_filter_angular_rate_);

              nav_msg_.twist.twist.angular.x = curr_filter_angular_rate_.x;
              nav_msg_.twist.twist.angular.y = curr_filter_angular_rate_.y;
              nav_msg_.twist.twist.angular.z = curr_filter_angular_rate_.z;

              if (publish_filtered_imu_)
              {
                filtered_imu_msg_.angular_velocity.x = curr_filter_angular_rate_.x;
                filtered_imu_msg_.angular_velocity.y = curr_filter_angular_rate_.y;
                filtered_imu_msg_.angular_velocity.z = curr_filter_angular_rate_.z;
              }
            }
            break;

            // Position Uncertainty
            case MIP_FILTER_DATA_POS_UNCERTAINTY:
            {
              memcpy(&curr_filter_pos_uncertainty_, field_data, sizeof(mip_filter_llh_pos_uncertainty));

              // For little-endian targets, byteswap the data field
              mip_filter_llh_pos_uncertainty_byteswap(&curr_filter_pos_uncertainty_);

              // x-axis
              nav_msg_.pose.covariance[0] = curr_filter_pos_uncertainty_.east*curr_filter_pos_uncertainty_.east;
              nav_msg_.pose.covariance[7] = curr_filter_pos_uncertainty_.north*curr_filter_pos_uncertainty_.north;
              nav_msg_.pose.covariance[14] = curr_filter_pos_uncertainty_.down*curr_filter_pos_uncertainty_.down;
            }
            break;

            // Velocity Uncertainty
            case MIP_FILTER_DATA_VEL_UNCERTAINTY:
            {
              memcpy(&curr_filter_vel_uncertainty_, field_data, sizeof(mip_filter_ned_vel_uncertainty));

              // For little-endian targets, byteswap the data field
              mip_filter_ned_vel_uncertainty_byteswap(&curr_filter_vel_uncertainty_);

              nav_msg_.twist.covariance[0] = curr_filter_vel_uncertainty_.east*curr_filter_vel_uncertainty_.east;
              nav_msg_.twist.covariance[7] = curr_filter_vel_uncertainty_.north*curr_filter_vel_uncertainty_.north;
              nav_msg_.twist.covariance[14] = curr_filter_vel_uncertainty_.down*curr_filter_vel_uncertainty_.down;
            }
            break;

            // Attitude Uncertainty
            case MIP_FILTER_DATA_ATT_UNCERTAINTY_EULER:
            {
              memcpy(&curr_filter_att_uncertainty_, field_data, sizeof(mip_filter_euler_attitude_uncertainty));

              // For little-endian targets, byteswap the data field
              mip_filter_euler_attitude_uncertainty_byteswap(&curr_filter_att_uncertainty_);
              nav_msg_.pose.covariance[21] = curr_filter_att_uncertainty_.roll*curr_filter_att_uncertainty_.roll;
              nav_msg_.pose.covariance[28] = curr_filter_att_uncertainty_.pitch*curr_filter_att_uncertainty_.pitch;
              nav_msg_.pose.covariance[35] = curr_filter_att_uncertainty_.yaw*curr_filter_att_uncertainty_.yaw;

              if (publish_filtered_imu_)
              {
                filtered_imu_msg_.orientation_covariance[0] =
                    curr_filter_att_uncertainty_.roll*curr_filter_att_uncertainty_.roll;
                filtered_imu_msg_.orientation_covariance[4] =
                    curr_filter_att_uncertainty_.pitch*curr_filter_att_uncertainty_.pitch;
                filtered_imu_msg_.orientation_covariance[8] =
                    curr_filter_att_uncertainty_.yaw*curr_filter_att_uncertainty_.yaw;
              }
            }
            break;

            // Filter Status
            case MIP_FILTER_DATA_FILTER_STATUS:
            {
              memcpy(&curr_filter_status_, field_data, sizeof(mip_filter_status));

              // For little-endian targets, byteswap the data field
              mip_filter_status_byteswap(&curr_filter_status_);

              nav_status_msg_.data.clear();
              // RCLCPP_DEBUG_THROTTLE(this->get_logger(), std::chrono::seconds(1),
              //         "Filter Status: %#06X, Dyn. Mode: %#06X, Filter State: %#06X",
              //         curr_filter_status_.filter_status,
              //         curr_filter_status_.dynamic_mode,
              //         curr_filter_status_.filter_state);
              nav_status_msg_.data.push_back(curr_filter_status_.filter_state);
              nav_status_msg_.data.push_back(curr_filter_status_.dynamics_mode);
              nav_status_msg_.data.push_back(curr_filter_status_.status_flags);
              nav_status_pub_->publish(nav_status_msg_);
            }
            break;

            ///
            // Scaled Accelerometer
            ///

            case MIP_FILTER_DATA_LINEAR_ACCELERATION:
            {
              memcpy(&curr_filter_linear_accel_, field_data, sizeof(mip_filter_linear_acceleration));

              // For little-endian targets, byteswap the data field
              mip_filter_linear_acceleration_byteswap(&curr_filter_linear_accel_);

              // If we want gravity removed, use this as acceleration
              if (remove_imu_gravity_)
              {
                // Stuff into ROS message - acceleration already in m/s^2
                filtered_imu_msg_.linear_acceleration.x = curr_filter_linear_accel_.x;
                filtered_imu_msg_.linear_acceleration.y = curr_filter_linear_accel_.y;
                filtered_imu_msg_.linear_acceleration.z = curr_filter_linear_accel_.z;
              }
              // Otherwise, do nothing with this packet
            }
            break;

            case MIP_FILTER_DATA_COMPENSATED_ACCELERATION:
            {
              memcpy(&curr_filter_accel_comp_, field_data, sizeof(mip_filter_compensated_acceleration));

              // For little-endian targets, byteswap the data field
              mip_filter_compensated_acceleration_byteswap(&curr_filter_accel_comp_);

              // If we do not want to have gravity removed, use this as acceleration
              if (!remove_imu_gravity_)
              {
                // Stuff into ROS message - acceleration already in m/s^2
                filtered_imu_msg_.linear_acceleration.x = curr_filter_accel_comp_.x;
                filtered_imu_msg_.linear_acceleration.y = curr_filter_accel_comp_.y;
                filtered_imu_msg_.linear_acceleration.z = curr_filter_accel_comp_.z;
              }
              // Otherwise, do nothing with this packet
            }
            break;

            default: break;
          }
        }

        // Publish
        if (publish_odom_)
        {
          nav_pub_->publish(nav_msg_);
        }

        if (publish_filtered_imu_)
        {
          // Does it make sense to get the angular velocity bias and acceleration bias to populate these?
          // Since the sensor does not produce a covariance for linear acceleration, set it based
          // on our pulled in parameters.
          std::copy(imu_linear_cov_.begin(), imu_linear_cov_.end(),
              filtered_imu_msg_.linear_acceleration_covariance.begin());
          // Since the sensor does not produce a covariance for angular velocity, set it based
          // on our pulled in parameters.
          std::copy(imu_angular_cov_.begin(), imu_angular_cov_.end(),
              filtered_imu_msg_.angular_velocity_covariance.begin());
          filtered_imu_pub_->publish(filtered_imu_msg_);
        }
      }
      break;

      ///
      // Handle checksum error packets
      ///

      case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
      {
        filter_checksum_error_packet_count_++;
      }
      break;

      ///
      // Handle timeout packets
      ///

      case MIP_INTERFACE_CALLBACK_TIMEOUT:
      {
        filter_timeout_packet_count_++;
      }
      break;

      default: break;
    }

    print_packet_stats();
  }  // filter_packet_callback



  // Send diagnostic information to device status topic and diagnostic aggregator
  void Microstrain::device_status_callback()
  {
    if (GX5_25)
    {
      u8 response_buffer[sizeof(gx4_25_diagnostic_device_status_field)];
      start = clock();
      while (mip_3dm_cmd_hw_specific_device_status(&device_interface_,
                GX4_25_MODEL_NUMBER, GX4_25_DIAGNOSTICS_STATUS_SEL,
                response_buffer) != MIP_INTERFACE_OK)
      {
        if (clock() - start > 5000)
        {
          RCLCPP_INFO(this->get_logger(), "mip_3dm_cmd_hw_specific_device_status function timed out.");
          break;
        }
      }

      // RCLCPP_INFO(this->get_logger(), "Adding device diagnostics to status msg");
      // RCLCPP_INFO(this->get_logger(), "adding data to message");

      // device_status_msg_.device_model = diagnostic_field.device_model;
      // device_status_msg_.status_selector =  diagnostic_field.status_selector;
      // device_status_msg_.status_flags = diagnostic_field.status_flags;
      // device_status_msg_.system_state = diagnostic_field.system_state;
      // device_status_msg_.system_timer_ms = diagnostic_field.system_timer_ms;
      // device_status_msg_.imu_stream_enabled = diagnostic_field.imu_stream_enabled;
      // device_status_msg_.filter_stream_enabled =  diagnostic_field.filter_stream_enabled;
      // device_status_msg_.imu_dropped_packets = diagnostic_field.imu_dropped_packets;
      // device_status_msg_.filter_dropped_packets = diagnostic_field.filter_dropped_packets;
      // device_status_msg_.com1_port_bytes_written = diagnostic_field.com1_port_bytes_written;
      // device_status_msg_.com1_port_bytes_read = diagnostic_field.com1_port_bytes_read;
      // device_status_msg_.com1_port_write_overruns = diagnostic_field.com1_port_write_overruns;
      // device_status_msg_.com1_port_read_overruns = diagnostic_field.com1_port_read_overruns;
      // device_status_msg_.imu_parser_errors =  diagnostic_field.imu_parser_errors;
      // device_status_msg_.imu_message_count = diagnostic_field.imu_message_count;
      // device_status_msg_.imu_last_message_ms = diagnostic_field.imu_last_message_ms;

      // device_status_pub_->publish(device_status_msg_);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Device status messages not configured for this model");
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  //
  // AHRS Packet Callback
  //
  ////////////////////////////////////////////////////////////////////////////////

  void Microstrain::ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;
    // If we aren't publishing, then return
    if (!publish_imu_)
      return;
    // The packet callback can have several types, process them all
    switch (callback_type)
    {
      ///
      // Handle valid packets
      ///

      case MIP_INTERFACE_CALLBACK_VALID_PACKET:
      {
        ahrs_valid_packet_count_++;

        ///
        // Loop through all of the data fields
        ///

        while (mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
        {
          ///
          // Decode the field
          ///

          switch (field_header->descriptor)
          {
            ///
            // Scaled Accelerometer
            ///

            case MIP_AHRS_DATA_ACCEL_SCALED:
            {
              memcpy(&curr_ahrs_accel_, field_data, sizeof(mip_ahrs_scaled_accel));

              // For little-endian targets, byteswap the data field
              mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel_);

              // Stuff into ROS message - acceleration in m/s^2
              // Header
              // imu_msg_.header.seq = ahrs_valid_packet_count_;
              imu_msg_.header.stamp = this->now();
              imu_msg_.header.frame_id = imu_frame_id_;
              imu_msg_.linear_acceleration.x = 9.81*curr_ahrs_accel_.scaled_accel[0];
              imu_msg_.linear_acceleration.y = 9.81*curr_ahrs_accel_.scaled_accel[1];
              imu_msg_.linear_acceleration.z = 9.81*curr_ahrs_accel_.scaled_accel[2];
              // Since the sensor does not produce a covariance for linear acceleration,
              // set it based on our pulled in parameters.
              std::copy(imu_linear_cov_.begin(), imu_linear_cov_.end(),
                  imu_msg_.linear_acceleration_covariance.begin());
            }
            break;

            ///
            // Scaled Gyro
            ///

            case MIP_AHRS_DATA_GYRO_SCALED:
            {
              memcpy(&curr_ahrs_gyro_, field_data, sizeof(mip_ahrs_scaled_gyro));

              // For little-endian targets, byteswap the data field
              mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro_);

              imu_msg_.angular_velocity.x = curr_ahrs_gyro_.scaled_gyro[0];
              imu_msg_.angular_velocity.y = curr_ahrs_gyro_.scaled_gyro[1];
              imu_msg_.angular_velocity.z = curr_ahrs_gyro_.scaled_gyro[2];
              // Since the sensor does not produce a covariance for angular velocity, set it based
              // on our pulled in parameters.
              std::copy(imu_angular_cov_.begin(), imu_angular_cov_.end(),
                  imu_msg_.angular_velocity_covariance.begin());
            }
            break;

            ///
            // Scaled Magnetometer
            ///

            case MIP_AHRS_DATA_MAG_SCALED:
            {
              memcpy(&curr_ahrs_mag_, field_data, sizeof(mip_ahrs_scaled_mag));

              // For little-endian targets, byteswap the data field
              mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag_);
            }
            break;

            // Quaternion
            case MIP_AHRS_DATA_QUATERNION:
            {
              memcpy(&curr_ahrs_quaternion_, field_data, sizeof(mip_ahrs_quaternion));

              // For little-endian targets, byteswap the data field
              mip_ahrs_quaternion_byteswap(&curr_ahrs_quaternion_);

              // If we want the orientation to be based on the reference on the imu
              tf2::Quaternion q(curr_ahrs_quaternion_.q[1],curr_ahrs_quaternion_.q[2],
                                curr_ahrs_quaternion_.q[3],curr_ahrs_quaternion_.q[0]);
              geometry_msgs::msg::Quaternion quat_msg;

              if(frame_based_enu_)
              {
                // Create a rotation from NED -> ENU
                tf2::Quaternion q_rotate;
                q_rotate.setRPY(M_PI,0.0,M_PI/2);
                // Apply the NED to ENU rotation such that the coordinate frame matches
                q = q_rotate*q;
                quat_msg = tf2::toMsg(q);
              }
              else
              {
                // put into ENU - swap X/Y, invert Z
                quat_msg.x = q[1];
                quat_msg.y = q[0];
                quat_msg.z = -1.0*q[2];
                quat_msg.w = q[3];
              }

              imu_msg_.orientation = quat_msg;



              // Since the MIP_AHRS data does not contain uncertainty values
              // we have to set them based on the parameter values.
              std::copy(imu_orientation_cov_.begin(), imu_orientation_cov_.end(),
                  imu_msg_.orientation_covariance.begin());
            }
            break;

            default: break;
          }
        }

        // Publish
        imu_pub_->publish(imu_msg_);

        // tixiao
        // Roboat IMU
        // IMU placement   ----     ROS
        // y ---  z (*)               x
        //        |                   |
        //        |                   |
        //        x             y ----z (x)
        imu_correct_msg_ = imu_msg_;
        imu_correct_msg_.header.frame_id = "base_link";
        // transform angular velocity
        imu_correct_msg_.angular_velocity.x = - imu_msg_.angular_velocity.x;
        imu_correct_msg_.angular_velocity.y =   imu_msg_.angular_velocity.y;
        imu_correct_msg_.angular_velocity.z = - imu_msg_.angular_velocity.z;
        // transform acceleration
        imu_correct_msg_.linear_acceleration.x = - imu_msg_.linear_acceleration.x;
        imu_correct_msg_.linear_acceleration.y =   imu_msg_.linear_acceleration.y;
        imu_correct_msg_.linear_acceleration.z = - imu_msg_.linear_acceleration.z;
        // Obtain orientation
        // tf::quaternionMsgToTF(imu_msg_.orientation, orientation);
        tf2::fromMsg(imu_msg_.orientation, orientation);
        beforeMatrix = tf2::Matrix3x3(orientation);
        betweenMatrix.setRPY(0, 0, -M_PI/2.0);
        afterMatrix = beforeMatrix * betweenMatrix;
        afterMatrix.getRPY(fixed_roll, fixed_pitch, fixed_yaw);
        // tf2のQuaternionに変換
        tf2::Quaternion q;
        q.setRPY(fixed_roll, fixed_pitch, fixed_yaw);
        // geometry_msgs::msg::Quaternion に変換
        geoQuat = tf2::toMsg(q);
        // geoQuat = tf::createQuaternionMsgFromRollPitchYaw(fixed_roll, fixed_pitch, fixed_yaw);
        imu_correct_msg_.orientation.x = geoQuat.x;
        imu_correct_msg_.orientation.y = geoQuat.y;
        imu_correct_msg_.orientation.z = geoQuat.z;
        imu_correct_msg_.orientation.w = geoQuat.w;
        
        imu_correct_pub_->publish(imu_correct_msg_);
      }
      break;

      ///
      // Handle checksum error packets
      ///

      case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
      {
        ahrs_checksum_error_packet_count_++;
      }
      break;

      ///
      // Handle timeout packets
      ///

      case MIP_INTERFACE_CALLBACK_TIMEOUT:
      {
        ahrs_timeout_packet_count_++;
      }
      break;

       default: break;
    }
    print_packet_stats();
  }  // ahrs_packet_callback


  ////////////////////////////////////////////////////////////////////////////////
  //
  // GPS Packet Callback
  //
  ////////////////////////////////////////////////////////////////////////////////
  void Microstrain::gps_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;
    u8 msgvalid = 1;  // keep track of message validity

    // If we aren't publishing, then return
    if (!publish_gps_)
      return;
    // The packet callback can have several types, process them all
    switch (callback_type)
    {
      ///
      // Handle valid packets
      ///

      case MIP_INTERFACE_CALLBACK_VALID_PACKET:
      {
        gps_valid_packet_count_++;

        ///
        // Loop through all of the data fields
        ///

        while (mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
        {
          ///
          // Decode the field
          ///

          switch (field_header->descriptor)
          {
            ///
            // LLH Position
            ///

            case MIP_GPS_DATA_LLH_POS:
            {
              memcpy(&curr_llh_pos_, field_data, sizeof(mip_gps_llh_pos));

              // For little-endian targets, byteswap the data field
              mip_gps_llh_pos_byteswap(&curr_llh_pos_);

              // push into ROS message
              gps_msg_.latitude = curr_llh_pos_.latitude;
              gps_msg_.longitude = curr_llh_pos_.longitude;
              gps_msg_.altitude = curr_llh_pos_.ellipsoid_height;
              gps_msg_.position_covariance_type = 2;  // diagnals known
              gps_msg_.position_covariance[0] = curr_llh_pos_.horizontal_accuracy*curr_llh_pos_.horizontal_accuracy;
              gps_msg_.position_covariance[4] = curr_llh_pos_.horizontal_accuracy*curr_llh_pos_.horizontal_accuracy;
              gps_msg_.position_covariance[8] = curr_llh_pos_.vertical_accuracy*curr_llh_pos_.vertical_accuracy;
              gps_msg_.status.status = curr_llh_pos_.valid_flags - 1;
              gps_msg_.status.service = 1;  // assumed
              // Header
              // gps_msg_.header.seq = gps_valid_packet_count_;
              gps_msg_.header.stamp = this->now();
              gps_msg_.header.frame_id = gps_frame_id_;
            }
            break;

            ///
            // NED Velocity
            ///

            case MIP_GPS_DATA_NED_VELOCITY:
            {
              memcpy(&curr_ned_vel_, field_data, sizeof(mip_gps_ned_vel));

              // For little-endian targets, byteswap the data field
              mip_gps_ned_vel_byteswap(&curr_ned_vel_);
            }
            break;

            ///
            // GPS Time
            ///

            case MIP_GPS_DATA_GPS_TIME:
            {
              memcpy(&curr_gps_time_, field_data, sizeof(mip_gps_time));

              // For little-endian targets, byteswap the data field
              mip_gps_time_byteswap(&curr_gps_time_);
            }
            break;

            default: break;
          }
        }
      }
      break;

      ///
      // Handle checksum error packets
      ///

      case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
      {
        msgvalid = 0;
        gps_checksum_error_packet_count_++;
      }
      break;

      ///
      // Handle timeout packets
      ///

      case MIP_INTERFACE_CALLBACK_TIMEOUT:
      {
        msgvalid = 0;
        gps_timeout_packet_count_++;
      }
      break;

      default: break;
    }

    if (msgvalid)
    {
      // Publish the message
        gps_pub_->publish(gps_msg_);
    }

    print_packet_stats();
  }  // gps_packet_callback

  void Microstrain::print_packet_stats()
  {
      auto logger = this->get_logger();

      RCLCPP_DEBUG_STREAM_THROTTLE(
          logger, *this->get_clock(), 1000,  // 1000 ms = 1秒
          filter_valid_packet_count_ << " FILTER (" 
          << (filter_timeout_packet_count_ + filter_checksum_error_packet_count_) << " errors)    "
          << ahrs_valid_packet_count_ << " AHRS (" 
          << (ahrs_timeout_packet_count_ + ahrs_checksum_error_packet_count_) << " errors)    "
          << gps_valid_packet_count_ << " GPS (" 
          << (gps_timeout_packet_count_ + gps_checksum_error_packet_count_) << " errors) Packets"
      );
  }




  // Wrapper callbacks
  void filter_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    Microstrain* ustrain = reinterpret_cast<Microstrain*>(user_ptr);
    ustrain->filter_packet_callback(user_ptr, packet, packet_size, callback_type);
  }

  void ahrs_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    Microstrain* ustrain = reinterpret_cast<Microstrain*>(user_ptr);
    ustrain->ahrs_packet_callback(user_ptr, packet, packet_size, callback_type);
  }
  // Wrapper callbacks
  void gps_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    Microstrain* ustrain = reinterpret_cast<Microstrain*>(user_ptr);
    ustrain->gps_packet_callback(user_ptr, packet, packet_size, callback_type);
  }

}  // namespace Microstrain
