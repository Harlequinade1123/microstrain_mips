#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/update_functions.hpp"
#include "microstrain_mips/msg/status_msg.hpp"
#include "microstrain_3dm.h"

#include <string>


namespace microstrain_mips
{
  class RosDiagnosticUpdater : public rclcpp::Node
  {
    public:
    RosDiagnosticUpdater(Microstrain::Microstrain *device)
    : Node("ros_diagnostic_updater"), device_(device), updater_(this)
    {
      updater_.setHardwareID("unknown");
      updater_.add("general", this, &RosDiagnosticUpdater::generalDiagnostics);
      updater_.add("packet", this, &RosDiagnosticUpdater::packetDiagnostics);
      updater_.add("port", this, &RosDiagnosticUpdater::portDiagnostics);
      updater_.add("imu", this, &RosDiagnosticUpdater::imuDiagnostics);

      status_sub_ = this->create_subscription<microstrain_mips::msg::StatusMsg>(
        "device/status", 5,
        std::bind(&RosDiagnosticUpdater::statusCallback, this, std::placeholders::_1));
    }

    void generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void packetDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void portDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void imuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    private:
    void statusCallback(const microstrain_mips::msg::StatusMsg::SharedPtr status)
    {
      last_status_ = *status;
      updater_.update();
    }

    Microstrain::Microstrain *device_;
    diagnostic_updater::Updater updater_;
    rclcpp::Subscription<microstrain_mips::msg::StatusMsg>::SharedPtr status_sub_;
    microstrain_mips::msg::StatusMsg last_status_;
  };
}