#include "kaixuan_can_hardware/kaixuan_can_system.hpp"
#include "kaixuan_can_hardware/dc_reference_profile.hpp"
#include "kaixuan_can_hardware/dc_clock_bridge.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <exception>
#include <functional>
#include <filesystem>
#include <iomanip>
#include <locale>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kaixuan_can_hardware
{

namespace
{
constexpr std::chrono::milliseconds kActivationWaitStep{20};
// The seven drives on each master are configured sequentially. Initial DC
// convergence can add several 5 s waits, so six seconds is not sufficient.
constexpr int kActivationWaitIterations = 3000;
constexpr double kPi = 3.14159265358979323846;
constexpr std::uint64_t kO6DisableCycles = 20;
constexpr std::uint64_t kO6EnableBaselineCycles = 5;
// All drives and the gateway use the same 1 ms application/DC Sync0 period.
constexpr std::int64_t kO6Sync0CycleNs = 1000000;
constexpr std::uint8_t kO6PressureRequestBit = 0x04;
constexpr std::uint8_t kO6AllFingersValidMask = 0x1F;
constexpr std::uint8_t kO6PressureCompleteState = 2;
constexpr std::uint16_t kO6PressureMaxAgeMs = 500;
constexpr std::uint16_t kO6ImuReadyMask = 0x000F;
constexpr double kO6AccelLsbPerG = 5460.0;
constexpr double kO6GyroLsbPerDps = 65.6;
constexpr double kStandardGravity = 9.80665;

// KGU CSP mapping required by the vendor specification.
// RxPDO 0x1601: ControlWord, TargetPosition, ModesOfOperation.
const ec_pdo_entry_info_t kRxPdoEntries[] = {
  {0x6040, 0x00, 16},
  {0x607A, 0x00, 32},
  {0x6060, 0x00, 8},
};

// TxPDO 0x1A01: StatusWord, PositionActual, VelocityActual,
// TorqueActual, ErrorCode.
const ec_pdo_entry_info_t kTxPdoEntries[] = {
  {0x6041, 0x00, 16},
  {0x6064, 0x00, 32},
  {0x606C, 0x00, 32},
  {0x6077, 0x00, 16},
  {0x603F, 0x00, 16},
};

const ec_pdo_info_t kRxPdos[] = {
  {0x1601, static_cast<unsigned int>(std::size(kRxPdoEntries)), kRxPdoEntries},
};

const ec_pdo_info_t kTxPdos[] = {
  {0x1A01, static_cast<unsigned int>(std::size(kTxPdoEntries)), kTxPdoEntries},
};

const ec_sync_info_t kSyncManagers[] = {
  {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
  {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
  {2, EC_DIR_OUTPUT, 1, kRxPdos, EC_WD_ENABLE},
  {3, EC_DIR_INPUT, 1, kTxPdos, EC_WD_DISABLE},
  {0xFF, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT},
};

// CST layout verified on KDE revision 0x00050012. Keep the dormant position
// target initialized, and read actual mode separately from the mode request.
const ec_pdo_entry_info_t kCstRxEntries[] = {
  {0x6040, 0, 16}, {0x607A, 0, 32}, {0x6060, 0, 8}, {0x6071, 0, 16}};
const ec_pdo_entry_info_t kCstTxEntries[] = {
  {0x6041, 0, 16}, {0x6064, 0, 32}, {0x606C, 0, 32},
  {0x6077, 0, 16}, {0x603F, 0, 16}, {0x6061, 0, 8}};
const ec_pdo_info_t kCstRxPdos[] = {{0x1601, 4, kCstRxEntries}};
const ec_pdo_info_t kCstTxPdos[] = {{0x1A01, 6, kCstTxEntries}};
const ec_sync_info_t kCstSyncManagers[] = {
  {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
  {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
  {2, EC_DIR_OUTPUT, 1, kCstRxPdos, EC_WD_ENABLE},
  {3, EC_DIR_INPUT, 1, kCstTxPdos, EC_WD_DISABLE},
  {0xFF, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT}};

}  // namespace

KaixuanCanSystem::~KaixuanCanSystem()
{
  stop_diagnostics();
  stop_o6_ros_interfaces();
  close_ethercat();
}

hardware_interface::CallbackReturn KaixuanCanSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kNumJoints)
  {
    RCLCPP_ERROR(
      logger_, "Expected %zu joints in ros2_control description, got %zu",
      kNumJoints, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  masters_[0].master_id = 0;
  masters_[0].affinity = 6;
  masters_[1].master_id = 1;
  masters_[1].affinity = 7;

  hw_positions_.assign(kNumJoints, 0.0);
  hw_velocities_.assign(kNumJoints, 0.0);
  hw_efforts_.assign(kNumJoints, 0.0);
  hw_commands_.assign(kNumJoints, 0.0);
  joint_offset_deg_.assign(kNumJoints, 0.0);
  joint_direction_.assign(kNumJoints, 1.0);
  joint_gear_ratio_.assign(kNumJoints, 1.0);
  commanded_counts_.assign(kNumJoints, 0);
  position_initialized_.assign(kNumJoints, false);

  try
  {
    const auto mode = info_.hardware_parameters.count("control_mode") ?
      info_.hardware_parameters.at("control_mode") : "csp";
    if (mode != "csp" && mode != "cst") {throw std::invalid_argument("control_mode must be csp or cst");}
    cst_mode_ = mode == "cst";
    const auto stop_word = info_.hardware_parameters.count("cst_disabled_control_word") ?
      info_.hardware_parameters.at("cst_disabled_control_word") : "6";
    if (stop_word != "0" && stop_word != "6") {
      throw std::invalid_argument("cst_disabled_control_word must be 0 (disable voltage) or 6 (shutdown)");
    }
    cst_disabled_control_word_ = stop_word == "0" ? 0 : 6;
    stop_cst_group_on_fault_ = info_.hardware_parameters.count("stop_cst_group_on_fault") &&
      info_.hardware_parameters.at("stop_cst_group_on_fault") == "true";
    if (info_.hardware_parameters.count("effort_write_timeout_ms")) {
      const auto timeout = std::stoll(info_.hardware_parameters.at("effort_write_timeout_ms"));
      if (timeout < 20 || timeout > 1000) {throw std::invalid_argument("effort_write_timeout_ms must be in [20, 1000]");}
      effort_write_timeout_ns_ = static_cast<std::uint64_t>(timeout) * 1000000;
    }
    if (info_.hardware_parameters.count("active_arm"))
    {
      active_arm_ = info_.hardware_parameters.at("active_arm");
    }
    if (info_.hardware_parameters.count("bus_diagnostics")) {
      const auto & value = info_.hardware_parameters.at("bus_diagnostics");
      bus_diagnostics_ = value == "true" || value == "True" || value == "1";
    }
    if (info_.hardware_parameters.count("dc_reference_time")) {
      const auto & value = info_.hardware_parameters.at("dc_reference_time");
      if (value != "cycle_start" && value != "before_sync") {
        throw std::invalid_argument("dc_reference_time must be cycle_start or before_sync");
      }
      fresh_reference_time_ = value == "before_sync";
    }
    if (info_.hardware_parameters.count("dc_clock_mode")) {
      const auto & value = info_.hardware_parameters.at("dc_clock_mode");
      if (value != "host" && value != "reference") {
        throw std::invalid_argument("dc_clock_mode must be host or reference");
      }
      follow_reference_clock_ = value == "reference";
    }
    if (info_.hardware_parameters.count("dc_reference_sync_cycles")) {
      const auto & text = info_.hardware_parameters.at("dc_reference_sync_cycles");
      std::size_t parsed = 0;
      const auto value = std::stoll(text, &parsed);
      if (parsed != text.size() || value < 1 || value > 100) {
        throw std::invalid_argument("dc_reference_sync_cycles must be an integer in [1, 100]");
      }
      reference_sync_cycles_ = static_cast<std::uint32_t>(value);
    }
    if (info_.hardware_parameters.count("dc_reset_after_configuration")) {
      const auto & value = info_.hardware_parameters.at("dc_reset_after_configuration");
      if (value != "true" && value != "false" && value != "1" && value != "0") {
        throw std::invalid_argument("dc_reset_after_configuration must be true or false");
      }
      dc_reset_after_configuration_ = value == "true" || value == "1";
    }
    if (info_.hardware_parameters.count("cycle_time_ns"))
    {
      cycle_time_ns_ = std::stoll(info_.hardware_parameters.at("cycle_time_ns"));
    }
    if (info_.hardware_parameters.count("shift_time_ns"))
    {
      shift_time_ns_ = std::stoll(info_.hardware_parameters.at("shift_time_ns"));
    }
    if (info_.hardware_parameters.count("priority"))
    {
      priority_ = std::stoi(info_.hardware_parameters.at("priority"));
    }
    if (info_.hardware_parameters.count("master0_affinity"))
    {
      masters_[0].affinity =
        std::stoi(info_.hardware_parameters.at("master0_affinity"));
    }
    if (info_.hardware_parameters.count("master1_affinity"))
    {
      masters_[1].affinity =
        std::stoi(info_.hardware_parameters.at("master1_affinity"));
    }
    if (info_.hardware_parameters.count("o6_command_timeout_ms"))
    {
      o6_command_timeout_ns_ =
        std::stoll(info_.hardware_parameters.at("o6_command_timeout_ms")) * 1000000LL;
    }
    if (info_.hardware_parameters.count("o6_max_step"))
    {
      const auto max_step = std::stoi(info_.hardware_parameters.at("o6_max_step"));
      if (max_step < 1 || max_step > 255)
      {
        throw std::out_of_range("o6_max_step must be in [1, 255]");
      }
      o6_max_step_ = static_cast<std::uint8_t>(max_step);
    }
    if (info_.hardware_parameters.count("o6_imu_only"))
    {
      const auto & value = info_.hardware_parameters.at("o6_imu_only");
      o6_imu_only_ = value == "true" || value == "True" || value == "1";
    }

    for (std::size_t joint = 0; joint < kJointsPerArm; ++joint)
    {
      const auto legacy_key = "comp" + std::to_string(joint);
      const auto right_key = "right_comp" + std::to_string(joint);
      const auto left_key = "left_comp" + std::to_string(joint);
      const auto right_direction_key =
        "right_direction" + std::to_string(joint + 1);
      const auto left_direction_key =
        "left_direction" + std::to_string(joint + 1);
      const auto right_gear_ratio_key =
        "right_gear_ratio" + std::to_string(joint + 1);
      const auto left_gear_ratio_key =
        "left_gear_ratio" + std::to_string(joint + 1);

      if (info_.hardware_parameters.count(legacy_key))
      {
        joint_offset_deg_[joint] =
          std::stod(info_.hardware_parameters.at(legacy_key));
      }
      if (info_.hardware_parameters.count(right_key))
      {
        joint_offset_deg_[joint] =
          std::stod(info_.hardware_parameters.at(right_key));
      }
      if (info_.hardware_parameters.count(left_key))
      {
        joint_offset_deg_[kJointsPerArm + joint] =
          std::stod(info_.hardware_parameters.at(left_key));
      }
      if (info_.hardware_parameters.count(right_direction_key))
      {
        joint_direction_[joint] =
          std::stod(info_.hardware_parameters.at(right_direction_key));
      }
      if (info_.hardware_parameters.count(left_direction_key))
      {
        joint_direction_[kJointsPerArm + joint] =
          std::stod(info_.hardware_parameters.at(left_direction_key));
      }
      if (info_.hardware_parameters.count(right_gear_ratio_key))
      {
        joint_gear_ratio_[joint] =
          std::stod(info_.hardware_parameters.at(right_gear_ratio_key));
      }
      if (info_.hardware_parameters.count(left_gear_ratio_key))
      {
        joint_gear_ratio_[kJointsPerArm + joint] =
          std::stod(info_.hardware_parameters.at(left_gear_ratio_key));
      }
    }

    cst_selected_.fill(false);
    if (cst_mode_) {
      std::istringstream names(info_.hardware_parameters.at("cst_joints"));
      std::string name;
      while (std::getline(names, name, ',')) {
        const auto found = std::find_if(info_.joints.begin(), info_.joints.end(),
          [&name](const auto & joint) {return joint.name == name;});
        if (found == info_.joints.end()) {throw std::invalid_argument("Unknown cst_joints entry: " + name);}
        const auto index = static_cast<std::size_t>(found - info_.joints.begin());
        if (cst_selected_[index] || !joint_is_active(index)) {
          throw std::invalid_argument("Duplicate or inactive CST joint: " + name);
        }
        cst_selected_[index] = true;
      }
      if (std::none_of(cst_selected_.begin(), cst_selected_.end(), [](bool v) {return v;})) {
        throw std::invalid_argument("CST requires an explicit cst_joints selection");
      }
    }
    joint_command_names_.resize(kNumJoints);
    for (std::size_t joint = 0; joint < kNumJoints; ++joint) {
      const auto expected = std::string(joint < 7 ? "right_joint" : "left_joint") + std::to_string(joint % 7 + 1);
      if (info_.joints[joint].name != expected) {throw std::invalid_argument("Unexpected joint order; expected " + expected);}
      auto & p = torque_parameters_[joint];
      const auto parameter = [&](const char * key, double fallback) {
        const auto & params = info_.joints[joint].parameters;
        return params.count(key) ? std::stod(params.at(key)) : fallback;
      };
      const double nominal = joint % 7 < 2 ? 68.0 : (joint % 7 < 4 ? 40.0 : 15.0);
      p.rated_nm = parameter("rated_torque_nm", nominal);
      p.command_units_per_nm = parameter("torque_command_units_per_nm", 1000.0 / p.rated_nm);
      p.limit_nm = parameter("effort_limit_nm", 0.05 * p.rated_nm);
      p.slew_nm_s = parameter("effort_slew_nm_s", 0.5 * p.rated_nm);
      p.release_slew_nm_s = parameter("effort_release_slew_nm_s", 0.0);
      p.max_velocity_rad_s = parameter("max_velocity_rad_s", degrees_to_radians(90.0));
      p.max_displacement_rad = parameter("max_displacement_rad", degrees_to_radians(120.0));
      p.min_position_rad = parameter("min_position_rad", -3.14);
      p.max_position_rad = parameter("max_position_rad", 3.14);
      const auto & params = info_.joints[joint].parameters;
      const auto normalization = params.count("normalize_position") ? params.at("normalize_position") : "false";
      if (normalization != "true" && normalization != "True" && normalization != "1" &&
        normalization != "false" && normalization != "False" && normalization != "0")
      {throw std::invalid_argument("normalize_position must be true or false for " + expected);}
      p.normalize_position = normalization == "true" || normalization == "True" || normalization == "1";
      if (!p.valid()) {throw std::invalid_argument("Invalid torque limits/scales for " + expected);}
      const auto & interfaces = info_.joints[joint].command_interfaces;
      const auto interface_name = cst_mode_ ? "effort" : "position";
      const std::size_t wanted = !cst_mode_ || cst_selected_[joint] ? 1 : 0;
      if (interfaces.size() != wanted || (wanted && interfaces[0].name != interface_name)) {
        throw std::invalid_argument("Command interfaces do not match control_mode/cst_joints for " + expected);
      }
      joint_command_names_[joint] = expected + "/" + interface_name;
    }
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(logger_, "Invalid hardware parameter: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t joint = 0; joint < kNumJoints; ++joint)
  {
    if (joint_direction_[joint] != -1.0 && joint_direction_[joint] != 1.0)
    {
      RCLCPP_ERROR(
        logger_, "Joint %zu direction must be -1 or 1, got %.3f",
        joint, joint_direction_[joint]);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!std::isfinite(joint_gear_ratio_[joint]) || joint_gear_ratio_[joint] <= 0.0)
    {
      RCLCPP_ERROR(
        logger_, "Joint %zu gear ratio must be positive, got %.6f",
        joint, joint_gear_ratio_[joint]);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint_gear_ratio_[joint] == 1.0)
    {
      RCLCPP_WARN(
        logger_, "Joint %zu KGU gear ratio is 1.0; set the motor-to-output reduction ratio before motion",
        joint);
    }
  }

  if (cycle_time_ns_ < 100000 || cycle_time_ns_ > 5000000)
  {
    RCLCPP_ERROR(
      logger_, "cycle_time_ns must be between 100000 and 5000000, got %ld",
      static_cast<long>(cycle_time_ns_));
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (priority_ < 1 || priority_ > 95)
  {
    RCLCPP_ERROR(logger_, "priority must be in [1, 95], got %d", priority_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (active_arm_ != "both" && active_arm_ != "right" && active_arm_ != "left")
  {
    RCLCPP_ERROR(
      logger_, "active_arm must be one of: both, right, left; got '%s'",
      active_arm_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (o6_command_timeout_ns_ != 0 &&
    (o6_command_timeout_ns_ < 100000000LL || o6_command_timeout_ns_ > 60000000000LL))
  {
    RCLCPP_ERROR(
      logger_, "o6_command_timeout_ms must be 0 or in [100, 60000], got %ld",
      static_cast<long>(o6_command_timeout_ns_ / 1000000LL));
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (follow_reference_clock_ &&
    (cycle_time_ns_ != 1000000 || dc_reset_after_configuration_))
  {
    RCLCPP_ERROR(logger_, "Reference-clock following requires 1 ms and dc_reset_after_configuration=false; each active master follows its own reference");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const long cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
  for (std::size_t i = 0; i < masters_.size(); ++i)
  {
    const auto & context = masters_[i];
    if (!master_is_active(i))
    {
      continue;
    }
    if (context.affinity < 0 || context.affinity >= cpu_count)
    {
      RCLCPP_ERROR(
        logger_, "Master%u CPU affinity %d is outside online CPUs [0, %ld)",
        context.master_id, context.affinity, cpu_count);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (active_arm_ == "both" && masters_[0].affinity == masters_[1].affinity)
  {
    RCLCPP_ERROR(logger_, "The two EtherCAT realtime threads must use different CPUs");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger_,
    "Direct IgH EtherCAT: active_arm=%s master0_cpu=%d master1_cpu=%d cycle=%ldns "
    "shift=%ldns priority=%d o6_imu_only=%s dc_reference_time=%s dc_reference_sync_cycles=%u dc_reset_after_configuration=%s dc_clock_mode=%s",
    active_arm_.c_str(),
    masters_[0].affinity, masters_[1].affinity,
    static_cast<long>(cycle_time_ns_), static_cast<long>(shift_time_ns_),
    priority_, o6_imu_only_ ? "true" : "false",
    fresh_reference_time_ ? "before_sync" : "cycle_start", reference_sync_cycles_,
    dc_reset_after_configuration_ ? "true" : "false", follow_reference_clock_ ? "reference" : "host");
  RCLCPP_INFO(logger_, "Control mode: %s; effort interface unit=N.m; write timeout=%lums",
    cst_mode_ ? "CST (controller claim and fresh zero command required)" : "CSP",
    static_cast<unsigned long>(effort_write_timeout_ns_ / 1000000));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KaixuanCanSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  stop_diagnostics();
  stop_o6_ros_interfaces();
  close_ethercat();
  diagnostic_snapshots_ = {};
  logged_torque_fault_.fill(0);
  for (auto & capture : cycle_captures_) {capture.reset();}
  for (auto & capture : bus_captures_) {capture.reset();}
  deactivate_requested_ns_ = 0;
  reset_runtime_state();
  start_diagnostics();

  if (!open_ethercat())
  {
    RCLCPP_ERROR(logger_, "Failed to initialize EtherCAT for active_arm=%s", active_arm_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const bool has_o6 = std::any_of(
    masters_.begin(), masters_.end(),
    [](const MasterContext & context) {return context.o6.configured;});
  if (has_o6 && !start_o6_ros_interfaces())
  {
    RCLCPP_ERROR(logger_, "Failed to start LinkerHand O6 ROS interfaces");
    close_ethercat();
    return hardware_interface::CallbackReturn::ERROR;
  }

  configured_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KaixuanCanSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!configured_)
  {
    RCLCPP_ERROR(logger_, "Cannot activate EtherCAT hardware before configure");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (int i = 0; i < kActivationWaitIterations; ++i)
  {
    {
      std::unique_lock<std::mutex> lock(io_mutex_);
      bool dc_ready = true;
      for (std::size_t master = 0; master < kMasterCount; ++master) {
        if (!master_is_active(master)) {continue;}
        const auto & snapshot = diagnostic_snapshots_[master];
        if (masters_[master].thread_failed.load() || snapshot.dc_reset_failed || snapshot.dc_follower_failed) {
          RCLCPP_ERROR(logger_, "Master%zu startup failed: DC reset phase=%u error=%u follower_error=%u schedule_failed=%d; drives remain disabled",
            master, snapshot.dc_reset_phase, snapshot.dc_reset_error,
            static_cast<unsigned int>(snapshot.dc_follower.failure), snapshot.follower_schedule_failed);
          const auto & f = snapshot.dc_follower;
          RCLCPP_ERROR(logger_,
            "Master%zu follower detail: calibrated=%d locked=%d initial=%ldns residual=%ldns filtered=%.1fns "
            "frequency=%.3fns/cycle acquire=%lluns last_valid_age=%lluns accepted=%llu rejected=%llu "
            "last_reject=%u last_reject_residual=%ldns getter_errors=%llu WKC=%u state=%u",
            master, f.calibrated, f.locked, static_cast<long>(f.initial_residual_ns),
            static_cast<long>(f.residual_ns), f.filtered_error_ns, f.frequency_ns_per_cycle,
            static_cast<unsigned long long>(f.acquisition_elapsed_ns),
            static_cast<unsigned long long>(f.last_valid_age_ns),
            static_cast<unsigned long long>(f.accepted_samples), static_cast<unsigned long long>(f.rejected_samples),
            snapshot.follower_reject_reason, static_cast<long>(snapshot.follower_reject_residual_ns),
            static_cast<unsigned long long>(snapshot.reference_getter_errors), snapshot.working_counter, snapshot.wc_state);
          // controller_manager may abort immediately after ERROR. Join/release
          // here so the frozen trace and disabled shutdown survive that abort.
          lock.unlock();
          close_ethercat();
          return hardware_interface::CallbackReturn::ERROR;
        }
        dc_ready = dc_ready && snapshot.dc_reset_ready && snapshot.dc_follower_ready;
      }
      if (all_joint_positions_ready() && dc_ready)
      {
        std::copy(measured_positions_.begin(), measured_positions_.end(), hw_positions_.begin());
        std::copy(measured_velocities_.begin(), measured_velocities_.end(), hw_velocities_.begin());
        activated_ = true;
        if (cst_mode_) {
          std::fill(hw_commands_.begin(), hw_commands_.end(), std::numeric_limits<double>::quiet_NaN());
          requested_efforts_.fill(std::numeric_limits<double>::quiet_NaN());
          effort_claimed_.fill(false);
          effort_write_ns_ = 0;
          for (auto & guard : torque_safety_) {guard.reset();}
        } else {hw_commands_ = hw_positions_;}
        for (std::size_t joint = 0; joint < kNumJoints; ++joint)
        {
          if (!joint_is_active(joint))
          {
            continue;
          }
          commanded_counts_[joint] =
            radians_to_joint_counts(hw_positions_[joint], joint);
        }
        RCLCPP_INFO(
          logger_, "EtherCAT hardware activated for %s arm(s)", active_arm_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
      }
    }
    std::this_thread::sleep_for(kActivationWaitStep);
  }

  std::string not_ready;
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    for (std::size_t joint = 0; joint < kNumJoints; ++joint)
    {
      if (joint_is_active(joint) && !position_initialized_[joint])
      {
        if (!not_ready.empty())
        {
          not_ready += ", ";
        }
        not_ready += info_.joints[joint].name;
      }
    }
  }
  RCLCPP_ERROR(
    logger_, "Timed out waiting for EtherCAT feedback/DC initialization. Missing feedback: %s",
    not_ready.c_str());
  close_ethercat();
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn KaixuanCanSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  deactivate_requested_ns_ = steady_now_ns();
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    activated_ = false;
    effort_claimed_.fill(false);
    requested_efforts_.fill(std::numeric_limits<double>::quiet_NaN());
    effort_write_ns_ = 0;
  }
  {
    std::lock_guard<std::mutex> lock(o6_mutex_);
    for (auto & command : o6_commands_)
    {
      command.enable_requested = false;
      command.update_time_ns = steady_now_ns();
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KaixuanCanSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(kNumJoints * 3);
  for (std::size_t i = 0; i < kNumJoints; ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KaixuanCanSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(kNumJoints);
  for (std::size_t i = 0; i < kNumJoints; ++i)
  {
    if (cst_mode_ && !cst_selected_[i]) {continue;}
    command_interfaces.emplace_back(
      info_.joints[i].name, cst_mode_ ? hardware_interface::HW_IF_EFFORT : hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type KaixuanCanSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> &)
{
  for (const auto & key : start) {
    for (std::size_t joint = 0; joint < kNumJoints; ++joint) {
      if (key.rfind(info_.joints[joint].name + "/", 0) != 0) {continue;}
      if (key != joint_command_names_[joint] ||
        (cst_mode_ && (!cst_selected_[joint] || !joint_is_active(joint))))
      {return hardware_interface::return_type::ERROR;}
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KaixuanCanSystem::perform_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  if (!cst_mode_) {return hardware_interface::return_type::OK;}
  std::lock_guard<std::mutex> lock(io_mutex_);
  for (std::size_t joint = 0; joint < kNumJoints; ++joint) {
    const auto & key = joint_command_names_[joint];
    const bool stopping = std::find(stop.begin(), stop.end(), key) != stop.end();
    const bool starting = std::find(start.begin(), start.end(), key) != start.end();
    if (!stopping && !starting) {continue;}
    effort_claimed_[joint] = starting && cst_selected_[joint];
    requested_efforts_[joint] = hw_commands_[joint] = std::numeric_limits<double>::quiet_NaN();
    torque_safety_[joint].reset();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KaixuanCanSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(io_mutex_);
  std::copy(measured_positions_.begin(), measured_positions_.end(), hw_positions_.begin());
  std::copy(measured_velocities_.begin(), measured_velocities_.end(), hw_velocities_.begin());
  std::copy(measured_efforts_.begin(), measured_efforts_.end(), hw_efforts_.begin());
  return configured_ ? hardware_interface::return_type::OK
                     : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type KaixuanCanSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(io_mutex_);
  if (!activated_)
  {
    return hardware_interface::return_type::OK;
  }

  if (cst_mode_) {
    effort_write_ns_ = steady_now_ns();
    for (std::size_t joint = 0; joint < kNumJoints; ++joint) {
      requested_efforts_[joint] = effort_claimed_[joint] ? hw_commands_[joint] :
        std::numeric_limits<double>::quiet_NaN();
    }
    return hardware_interface::return_type::OK;
  }

  for (std::size_t joint = 0; joint < kNumJoints; ++joint)
  {
    if (position_initialized_[joint] && std::isfinite(hw_commands_[joint]))
    {
      commanded_counts_[joint] =
        radians_to_joint_counts(hw_commands_[joint], joint);
    }
  }
  return hardware_interface::return_type::OK;
}

bool KaixuanCanSystem::open_ethercat()
{
  initialize_runtime();
  for (auto & context : masters_)
  {
    if (!master_is_active(context.master_id))
    {
      continue;
    }
    if (!configure_master(context))
    {
      close_ethercat();
      return false;
    }
  }
  return true;
}

void KaixuanCanSystem::close_ethercat()
{
  const auto close_ns = steady_now_ns();
  struct ClosingMaster
  {
    bool present{false};
    std::uint64_t joined_ns{0}, deactivate_ns{0}, deactivated_ns{0}, released_ns{0};
    std::uint64_t receive_ns{0}, send_ns{0}, loop_exit_ns{0}, cycle{0};
    int deactivate_result{0}, last_send_result{0};
    bool was_activated{false}, o6_disable_cycles{false}, final_pdo_available{false};
    MasterDiagnostic before{};
    std::array<std::array<std::int64_t, 6>, kJointsPerArm> final_pdo{};
  };
  std::array<ClosingMaster, kMasterCount> closing{};
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    for (std::size_t i = 0; i < kMasterCount; ++i) {
      closing[i].present = masters_[i].master != nullptr;
      closing[i].before = diagnostic_snapshots_[i];
    }
    activated_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(o6_mutex_);
    for (auto & command : o6_commands_)
    {
      command.enable_requested = false;
      command.update_time_ns = steady_now_ns();
    }
  }
  // Keep PDO traffic alive until each running arm has sent shutdown controlwords
  // and received non-enabled feedback. Allow at least 20 communication cycles.
  const auto disable_deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(500);
  bool disable_complete = false;
  while (std::chrono::steady_clock::now() < disable_deadline) {
    disable_complete = true;
    {
      std::lock_guard<std::mutex> lock(io_mutex_);
      for (std::size_t i = 0; i < kMasterCount; ++i) {
        if (!closing[i].present || !masters_[i].running.load()) {continue;}
        const auto & latest = diagnostic_snapshots_[i];
        if (latest.cycle_count < closing[i].before.cycle_count + 20) {
          disable_complete = false;
        }
        for (const auto & axis : latest.axes) {
          if (axis.control_word != disabled_control_word() || (axis.status_word & 0x006f) == 0x0027) {
            disable_complete = false;
          }
        }
      }
    }
    if (disable_complete) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (!disable_complete) {
    RCLCPP_WARN(logger_, "Arm disable feedback not confirmed within 500 ms before release");
  }
  for (auto & context : masters_) {context.running.store(false);}
  for (auto & context : masters_)
  {
    if (context.cyclic_thread.joinable())
    {
      context.cyclic_thread.join();
    }
    closing[context.master_id].joined_ns = steady_now_ns();
  }
  for (auto & context : masters_)
  {
    if (context.master != nullptr)
    {
      auto & trace = closing[context.master_id];
      trace.was_activated = context.master_activated;
      trace.o6_disable_cycles = context.o6.configured && context.master_activated;
      trace.receive_ns = context.final_receive_ns;
      trace.send_ns = context.last_send_time_ns;
      trace.loop_exit_ns = context.loop_exit_ns;
      trace.cycle = context.cycle_count;
      trace.last_send_result = context.previous_send_result;
      if (context.master_activated && context.domain_data) {
        trace.final_pdo_available = true;
        for (std::size_t i = 0; i < kJointsPerArm; ++i) {
          const auto & axis = context.axes[i];
          const auto * data = context.domain_data;
          trace.final_pdo[i] = {EC_READ_S32(data + axis.position_actual_value),
            EC_READ_S32(data + axis.target_position), EC_READ_U16(data + axis.status_word),
            EC_READ_U16(data + axis.control_word), EC_READ_U16(data + axis.error_code),
            EC_READ_S16(data + axis.torque_actual_value)};
        }
      }
      if (context.master_activated)
      {
        trace.deactivate_ns = steady_now_ns();
        trace.deactivate_result = ecrt_master_deactivate(context.master);
        trace.deactivated_ns = steady_now_ns();
      }
      ecrt_release_master(context.master);
      trace.released_ns = steady_now_ns();
    }
    context.master = nullptr;
    context.domain = nullptr;
    context.domain_data = nullptr;
    context.slave_configs.fill(nullptr);
    context.probe_registers.fill(nullptr);
    context.probe_sdos.fill(nullptr);
    context.dc_reset_request = nullptr;
    context.o6 = O6Handle{};
    context.master_activated = false;
    context.thread_ready.store(false);
    context.thread_failed.store(false);
  }

  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    configured_ = false;
    activated_ = false;
  }
  // Independent of KAIXUAN_SHUTDOWN_LOG. Both cyclic threads have joined and
  // every master has been released before any trace filesystem operation.
  for (auto & context : masters_) {dump_dc_reference_trace(context);}
  // ROS/recorder may already have stopped. Only now, after both masters have
  // been released, serialize/fsync the shutdown evidence; never delay stopping
  // the cyclic threads on disk I/O. Arm disable traffic has already completed.
  const char * path = std::getenv("KAIXUAN_SHUTDOWN_LOG");
  if (!path || !*path || std::none_of(closing.begin(), closing.end(),
      [](const auto & trace) {return trace.present;})) {return;}
  std::ostringstream output;
  for (std::size_t master = 0; master < kMasterCount; ++master) {
    const auto & trace = closing[master];
    if (!trace.present) {continue;}
    output << "{\"kind\":\"hardware_shutdown\",\"schema\":1,\"master\":" << master
      << ",\"pid\":" << getpid()
      << ",\"write_system_ns\":" << std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count()
      << ",\"write_steady_ns\":" << steady_now_ns()
      << ",\"lifecycle_deactivate_requested_ns\":" << deactivate_requested_ns_
      << ",\"close_requested_ns\":" << close_ns
      << ",\"loop_exit_ns\":" << trace.loop_exit_ns
      << ",\"thread_joined_ns\":" << trace.joined_ns
      << ",\"master_deactivate_begin_ns\":" << trace.deactivate_ns
      << ",\"master_deactivate_end_ns\":" << trace.deactivated_ns
      << ",\"master_deactivate_result\":" << trace.deactivate_result
      << ",\"master_released_ns\":" << trace.released_ns
      << ",\"was_activated\":" << trace.was_activated
      << ",\"o6_disable_cycles_ran\":" << trace.o6_disable_cycles
      << ",\"last_receive_return_ns\":" << trace.receive_ns
      << ",\"last_send_return_ns\":" << trace.send_ns
      << ",\"last_send_result\":" << trace.last_send_result
      << ",\"last_normal_cycle\":" << trace.cycle
      << ",\"before_sample_steady_ns\":" << trace.before.sample_steady_ns
      << ",\"before_wkc\":" << trace.before.working_counter
      << ",\"before_wc_state\":" << trace.before.wc_state
      << ",\"axes\":[";
    for (std::size_t i = 0; i < kJointsPerArm; ++i) {
      const auto & before = trace.before.axes[i];
      const auto & final = trace.final_pdo[i];
      if (i) {output << ',';}
      output << "{\"slave\":" << i << ",\"before_actual_counts\":" << before.actual_counts
        << ",\"before_target_counts\":" << before.target_counts
        << ",\"before_status_word\":" << before.status_word
        << ",\"before_control_word\":" << before.control_word
        << ",\"before_error_code\":" << before.error_code
        << ",\"final_pdo_available\":" << trace.final_pdo_available
        << ",\"final_actual_counts\":" << final[0] << ",\"final_target_counts\":" << final[1]
        << ",\"final_status_word\":" << final[2] << ",\"final_control_word\":" << final[3]
        << ",\"final_error_code\":" << final[4] << ",\"final_torque_raw\":" << final[5] << '}';
    }
    output << "],\"note\":\"Final PDO image is cached, not proof of drive state after master release; no additional arm stop command was introduced\"}\n";
  }
  const auto text = output.str();
  const int fd = ::open(path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0600);
  bool ok = fd >= 0;
  std::size_t offset = 0;
  while (ok && offset < text.size()) {
    const auto written = ::write(fd, text.data() + offset, text.size() - offset);
    if (written < 0 && errno == EINTR) {continue;}
    if (written <= 0) {ok = false; break;}
    offset += written;
  }
  if (fd >= 0) {if (::fsync(fd) != 0) {ok = false;} ::close(fd);}
  const std::string filename(path);
  const auto slash = filename.find_last_of('/');
  const std::string directory = slash == std::string::npos ? "." : filename.substr(0, slash);
  const int dir_fd = ::open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (dir_fd >= 0) {if (::fsync(dir_fd) != 0) {ok = false;} ::close(dir_fd);}
  else {ok = false;}
  if (!ok) {RCLCPP_ERROR(logger_, "Failed to persist hardware shutdown evidence: %s", path);}
}

void KaixuanCanSystem::dump_dc_reference_trace(MasterContext & context)
{
  if (!context.dc_trace || context.dc_trace_dump_attempted) {return;}
  context.dc_trace_dump_attempted = true;
  int fd = -1;
  try {
    std::filesystem::create_directories(context.dc_trace_directory);
    const auto filename = (std::filesystem::path(context.dc_trace_directory) /
      ("dc-reference-master" + std::to_string(context.master_id) + ".csv")).string();
    // Never truncate another run's evidence, including through an existing symlink.
    fd = ::open(filename.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (fd < 0) {
      RCLCPP_ERROR(logger_, "Cannot exclusively create DC trace %s: %s", filename.c_str(), std::strerror(errno));
      return;
    }
    std::ostringstream output;
    output.imbue(std::locale::classic());
    const auto & config = context.dc_follower_config;
    output << "# follower_proportional_gain=" << config.proportional_gain << '\n'
           << "# follower_integral_gain=" << config.integral_gain << '\n'
           << "# follower_filter_alpha=" << config.filter_alpha << '\n'
           << "# follower_fine_entry_filtered_ns=" << config.fine_entry_filtered_ns << '\n'
           << "# follower_bounded_handoff=" << config.bumpless_fine_entry << '\n';
    output << "# follower_acquisition_rate_feedforward=" << config.acquisition_rate_feedforward << '\n'
           << "# follower_acquisition_rate_alpha=" << config.acquisition_rate_alpha << '\n'
           << "# follower_host_clock=CLOCK_MONOTONIC_RAW\n"
           << "# sleep_clock=CLOCK_MONOTONIC_rebased_from_RAW_each_cycle\n";
    context.dc_trace->write_csv(output, context.master_id);
    const auto text = output.str();
    bool ok = output.good();
    std::size_t offset = 0;
    while (ok && offset < text.size()) {
      const auto written = ::write(fd, text.data() + offset, text.size() - offset);
      if (written < 0 && errno == EINTR) {continue;}
      if (written <= 0) {ok = false; break;}
      offset += static_cast<std::size_t>(written);
    }
    if (::fsync(fd) != 0) {ok = false;}
    if (::close(fd) != 0) {ok = false;}
    fd = -1;
    const int directory_fd = ::open(context.dc_trace_directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (directory_fd >= 0) {
      if (::fsync(directory_fd) != 0) {ok = false;}
      ::close(directory_fd);
    } else {ok = false;}
    if (ok) {
      RCLCPP_INFO(logger_, "DC reference trace saved: %s (%zu rows; frozen=%d; first_failure=%lu)",
        filename.c_str(), context.dc_trace->size(), context.dc_trace->frozen(),
        static_cast<unsigned long>(context.dc_trace->first_failure_cycle()));
    } else {
      RCLCPP_ERROR(logger_, "DC reference trace write was incomplete: %s", filename.c_str());
    }
  } catch (const std::exception & exception) {
    if (fd >= 0) {::close(fd);}
    RCLCPP_ERROR(logger_, "DC reference trace export failed after master release: %s", exception.what());
  }
}

bool KaixuanCanSystem::configure_master(MasterContext & context)
{
  context.master = ecrt_request_master(context.master_id);
  if (context.master == nullptr)
  {
    RCLCPP_ERROR(
      logger_, "ecrt_request_master(%u) failed; check /dev/EtherCAT%u permissions",
      context.master_id, context.master_id);
    return false;
  }

  if (ecrt_master_set_send_interval(context.master, cycle_time_ns_ / 1000) != 0) {
    RCLCPP_ERROR(logger_, "Cannot set Master%u send interval", context.master_id);
    return false;
  }

  ec_master_info_t master_info{};
  if (ecrt_master(context.master, &master_info) != 0)
  {
    RCLCPP_ERROR(logger_, "Failed to query Master%u", context.master_id);
    return false;
  }
  const bool has_o6 = master_info.slave_count == kMaxSlavesPerMaster;
  if ((dc_reset_after_configuration_ || follow_reference_clock_) && has_o6) {
    RCLCPP_ERROR(logger_, "DC clock experiments require seven axes without O6");
    return false;
  }
  const bool slave_count_valid =
    master_info.slave_count == kJointsPerArm || has_o6;
  if (!slave_count_valid)
  {
    RCLCPP_ERROR(
      logger_, "Master%u detected %u slaves; expected 7 or 8 (with O6 at position 7)",
      context.master_id, master_info.slave_count);
    return false;
  }

  context.domain = ecrt_master_create_domain(context.master);
  if (context.domain == nullptr)
  {
    RCLCPP_ERROR(logger_, "Failed to create domain for Master%u", context.master_id);
    return false;
  }

  const std::size_t joint_base =
    context.master_id == 0 ? 0 : kJointsPerArm;
  for (std::uint16_t slave_pos = 0; slave_pos < kJointsPerArm; ++slave_pos)
  {
    if (!configure_axis(context, slave_pos, joint_base + slave_pos))
    {
      return false;
    }
  }
  if (has_o6 && !configure_o6(context))
  {
    return false;
  }

  if (ecrt_master_select_reference_clock(
      context.master, context.slave_configs[0]) != 0)
  {
    RCLCPP_ERROR(
      logger_, "Failed to select Master%u Slave0 as DC reference",
      context.master_id);
    return false;
  }

  context.dc_reset_request = nullptr;
  if (dc_reset_after_configuration_) {
    context.dc_reset_request = ecrt_slave_config_create_reg_request(context.slave_configs[0], 16);
    if (context.dc_reset_request == nullptr) {
      RCLCPP_ERROR(logger_, "Master%u cannot allocate the DC initialization request", context.master_id);
      return false;
    }
  }
  context.dc_reset = DcStartupReset{dc_reset_after_configuration_};
  context.dc_reset_status = context.dc_reset.status();
  context.dc_reset_action = {};
  context.dc_reset_queue_rejected = false;
  context.dc_follower_config = DcReferenceFollower::Config{};
  if (follow_reference_clock_) {
    // Faster bounded tracking of the observed reference-rate changes. Keep all
    // quality/timeout/output guards; smooth acquisition handoff avoids PI windup.
    context.dc_follower_config = reference_follow_config();
    RCLCPP_INFO(logger_, "Master%u reference follower: host_clock=CLOCK_MONOTONIC_RAW sleep_clock=CLOCK_MONOTONIC(rebased) Kp=%.6f Ki=%.9f alpha=%.8f fine_entry=%ldns bounded_handoff=%s acquisition_rate_feedforward=%s",
      context.master_id, context.dc_follower_config.proportional_gain,
      context.dc_follower_config.integral_gain, context.dc_follower_config.filter_alpha,
      context.dc_follower_config.fine_entry_filtered_ns,
      context.dc_follower_config.bumpless_fine_entry ? "true" : "false",
      context.dc_follower_config.acquisition_rate_feedforward ? "true" : "false");
  }
  context.dc_follower = DcReferenceFollower{context.dc_follower_config};
  context.dc_follower_status = context.dc_follower.status();
  context.reference_sample_pending = context.follower_schedule_failed = false;
  context.follower_getter_failed = false;
  context.follower_getter_consecutive_errors = 0;
  context.reference_sample_virtual_ns = context.reference_sample_mono_ns = 0;
  context.reference_sample_host_ns = 0;
  context.reference_sample_cycle = context.reference_getter_retry_cycle = 0;
  context.reference_getter_errors = context.follower_skipped_periods = 0;
  context.follower_last_ref32 = 0;
  context.follower_failure_cycle = context.follower_reject_cycle = 0;
  context.follower_failure_residual_ns = context.follower_reject_residual_ns = 0;
  context.follower_reject_reason = 0;
  context.follower_ref_received_cycle = context.follower_calibration_cycle = 0;
  context.follower_calibration_delta_ns = 0;
  context.dc_trace.reset();
  context.dc_trace_directory.clear();
  context.dc_trace_dump_attempted = false;
  context.reference_sample_raw = {};
  context.dc_trace_row = {};
  const char * dc_trace_directory = std::getenv("KAIXUAN_DC_TRACE_DIR");
  if (follow_reference_clock_ && dc_trace_directory && *dc_trace_directory) {
    try {
      context.dc_trace_directory = dc_trace_directory;
      context.dc_trace = std::make_unique<DcReferenceTraceRing>();
      RCLCPP_INFO(logger_, "Master%u reference trace allocated/touched: %zu rows, %zu bytes; active trace adds 3 bracket + 2 getter clock reads/cycle",
        context.master_id, DcReferenceTraceRing::kCapacity, sizeof(DcReferenceTraceRing));
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(logger_, "Cannot allocate requested DC reference trace: %s", exception.what());
      return false;
    }
  }
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    diagnostic_snapshots_[context.master_id].dc_reset_ready = context.dc_reset_status.ready;
    diagnostic_snapshots_[context.master_id].dc_follower_ready = !follow_reference_clock_;
    diagnostic_snapshots_[context.master_id].dc_follower_config = context.dc_follower_config;
  }

  if (ecrt_master_activate(context.master) != 0)
  {
    RCLCPP_ERROR(logger_, "Failed to activate Master%u", context.master_id);
    return false;
  }
  context.master_activated = true;
  context.domain_data = ecrt_domain_data(context.domain);
  if (context.domain_data == nullptr)
  {
    RCLCPP_ERROR(logger_, "Master%u returned null domain data", context.master_id);
    return false;
  }

  context.running.store(true);
  context.thread_ready.store(false);
  context.thread_failed.store(false);
  context.cycle_count = 0;
  context.queue_count = 0;
  context.send_count = 0;
  context.last_cycle_time_ns = 0;
  context.max_cycle_lateness_ns = 0;
  context.max_loop_interval_ns = 0;
  context.last_send_time_ns = 0;
  context.last_complete_send_time_ns = 0;
  context.previous_send_result = 0;
  context.previous_send_duration_ns = 0;
  context.probe.reset();
  context.max_probe_work_ns = 0;
  context.loop_exit_ns = 0;
  context.final_receive_ns = 0;
  context.startup_guard.reset();
  context.dc_monitor_pending = false;
  context.dc_sample_updated = false;
  context.dc_difference_ns = UINT32_MAX;
  context.timing_max_ns = {};
  context.timing_window_max_ns = {};
  context.timing_completed_max_ns = {};
  context.timing_window_start_ns = context.timing_window_cycles = 0;
  context.timing_completed_start_ns = context.timing_completed_end_ns = context.timing_completed_cycles = 0;
  context.timing_mutex_wait_ns = context.slow_send_cycles = context.last_slow_send_ns = 0;
  context.sync_errors = context.schedule_rebases = 0;
  context.reference_sync_attempts = context.reference_sync_queued = context.last_reference_sync_cycle = 0;
  context.timing_reference_sync_cycles = context.timing_completed_reference_sync_cycles = 0;
  context.dc_pending_queue_cycle = context.dc_sample_queue_cycle = 0;
  context.dc_pending_reference_age_cycles = context.dc_sample_reference_age_cycles = UINT64_MAX;
  context.cyclic_thread = std::thread(
    [this, &context]() {cyclic_loop(context);});

  for (int i = 0; i < 200; ++i)
  {
    if (context.thread_ready.load())
    {
      RCLCPP_INFO(
        logger_, "Master%u started: 7 axes%s, CPU%d, %.1f Hz",
        context.master_id, context.o6.configured ? " + LinkerHand O6" : "",
        context.affinity,
        1.0e9 / static_cast<double>(cycle_time_ns_));
      return true;
    }
    if (context.thread_failed.load())
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  RCLCPP_ERROR(
    logger_, "Master%u realtime thread failed to start", context.master_id);
  context.running.store(false);
  if (context.cyclic_thread.joinable())
  {
    context.cyclic_thread.join();
  }
  return false;
}

bool KaixuanCanSystem::configure_axis(
  MasterContext & context, std::uint16_t slave_pos, std::size_t joint_index)
{
  ec_slave_info_t slave_info{};
  if (ecrt_master_get_slave(context.master, slave_pos, &slave_info) != 0)
  {
    RCLCPP_ERROR(
      logger_, "Master%u cannot read Slave%u identity",
      context.master_id, slave_pos);
    return false;
  }
  if (slave_info.vendor_id != kVendorId ||
    slave_info.product_code != kProductCode ||
    slave_info.revision_number != kRevision)
  {
    RCLCPP_ERROR(
      logger_,
      "Master%u Slave%u identity mismatch: got %08x:%08x:%08x, expected %08x:%08x:%08x",
      context.master_id, slave_pos,
      slave_info.vendor_id, slave_info.product_code, slave_info.revision_number,
      kVendorId, kProductCode, kRevision);
    return false;
  }

  RCLCPP_INFO(logger_, "Drive identity master=%u slave=%u vendor=%u product=%u revision=%u serial=%u",
    context.master_id, slave_pos, slave_info.vendor_id, slave_info.product_code,
    slave_info.revision_number, slave_info.serial_number);
  auto & axis = context.axes[slave_pos];
  axis = AxisHandle{};
  axis.joint_index = joint_index;
  axis.slave_pos = slave_pos;
  const bool cst_axis = cst_mode_ && cst_selected_[joint_index];
  const auto read_parameter = [&](std::uint16_t index, std::uint16_t & result) {
    std::uint8_t bytes[2]{};
    std::size_t size = 0;
    std::uint32_t abort = 0;
    if (ecrt_master_sdo_upload(context.master, slave_pos, index, 0,
        bytes, sizeof(bytes), &size, &abort) != 0 || size != sizeof(bytes))
    {
      RCLCPP_ERROR(logger_, "Cannot read unit parameter master=%u slave=%u index=%04x abort=%08x",
        context.master_id, slave_pos, index, abort);
      return false;
    }
    result = EC_READ_U16(bytes);
    return true;
  };
  std::uint16_t pn083{}, pn085{}, pn088{};
  if (!read_parameter(0x2083, pn083) || !read_parameter(0x2085, pn085) ||
    !read_parameter(0x2088, pn088) || pn083 > 1 || pn088 > 1 || pn085 == 0)
  {
    RCLCPP_ERROR(logger_, "Unsupported effort/velocity scaling on master=%u slave=%u", context.master_id, slave_pos);
    return false;
  }
  axis.torque_feedback_unit = pn083;
  axis.rated_current_a = pn085 * 0.01;
  axis.velocity_counts_per_second = pn088 == 0;
  if (cst_axis) {
    std::uint8_t bytes[4]{};
    std::size_t size = 0;
    std::uint32_t abort = 0;
    if (ecrt_master_sdo_upload(context.master, slave_pos, 0x6502, 0,
        bytes, sizeof(bytes), &size, &abort) != 0 || size != sizeof(bytes) ||
        (EC_READ_U32(bytes) & (1U << 9)) == 0)
    {
      RCLCPP_ERROR(logger_, "CST support not confirmed on master=%u slave=%u", context.master_id, slave_pos);
      return false;
    }
    RCLCPP_INFO(logger_, "CST %s: command=%.6f raw/Nm, feedback Pn083=%u, rated_current=%.2fA, limit=%.3fNm",
      info_.joints[joint_index].name.c_str(), torque_parameters_[joint_index].command_units_per_nm,
      pn083, axis.rated_current_a, torque_parameters_[joint_index].limit_nm);
  }
  // Read-only configuration evidence before this master is activated. Object
  // indices and uint16/subindex 0 match src/normal/pid.bash. Never run in RT.
  for (const std::uint16_t index : {0x2033, 0x2034, 0x2035, 0x2063, 0x2064, 0x2065}) {
    std::uint8_t value[2]{};
    std::size_t size = 0;
    std::uint32_t abort_code = 0;
    const int result = ecrt_master_sdo_upload(
      context.master, slave_pos, index, 0, value, sizeof(value), &size, &abort_code);
    if (result != 0 || size != sizeof(value)) {
      RCLCPP_WARN(logger_, "Drive parameter snapshot incomplete master=%u slave=%u index=0x%04x result=%d abort=0x%08x size=%zu; skipping remaining reads",
        context.master_id, slave_pos, index, result, abort_code, size);
      break;
    }
    RCLCPP_INFO(logger_, "Drive parameter master=%u slave=%u index=0x%04x sub=0 value=%u",
      context.master_id, slave_pos, index, static_cast<unsigned int>(EC_READ_U16(value)));
  }

  auto * slave_config = ecrt_master_slave_config(
    context.master, 0, slave_pos, kVendorId, kProductCode);
  if (slave_config == nullptr)
  {
    RCLCPP_ERROR(
      logger_, "Cannot create config for Master%u Slave%u",
      context.master_id, slave_pos);
    return false;
  }
  context.slave_configs[slave_pos] = slave_config;
  if (bus_diagnostics_) {
    // Allocate before activation. Runtime uses only scheduled read requests,
    // never a blocking SDO upload or a second thread accessing this master.
    context.probe_registers[slave_pos] = ecrt_slave_config_create_reg_request(slave_config, 20);
    context.probe_sdos[slave_pos] = ecrt_slave_config_create_sdo_request(slave_config, 0x2070, 0, 64);
    if (context.probe_sdos[slave_pos]) {
      if (ecrt_sdo_request_timeout(context.probe_sdos[slave_pos], 300) != 0) {
        // The master still owns/frees this request. Do not use it without the
        // intended timeout; the missing handle will produce explicit failures.
        context.probe_sdos[slave_pos] = nullptr;
      }
    }
    if (!context.probe_registers[slave_pos] || !context.probe_sdos[slave_pos]) {
      RCLCPP_WARN(logger_, "Read-only bus diagnostics request allocation incomplete master=%u slave=%u",
        context.master_id, slave_pos);
    }
  }

  // KGU exposes configurable CoE PDOs. Select its documented CiA402 layout.
  if (ecrt_slave_config_pdos(slave_config, EC_END, cst_axis ? kCstSyncManagers : kSyncManagers) != 0)
  {
    RCLCPP_ERROR(
      logger_, "PDO assignment failed for Master%u Slave%u",
      context.master_id, slave_pos);
    return false;
  }
  if (ecrt_slave_config_dc(
      slave_config, kDcAssignActivate,
      static_cast<std::uint32_t>(cycle_time_ns_),
      static_cast<std::int32_t>(shift_time_ns_), 0, 0) != 0)
  {
    RCLCPP_ERROR(
      logger_, "DC configuration failed for Master%u Slave%u",
      context.master_id, slave_pos);
    return false;
  }

  if (cst_axis && (ecrt_slave_config_sdo16(slave_config, 0x6071, 0, 0) != 0 ||
      ecrt_slave_config_sdo8(slave_config, 0x6060, 0, 10) != 0))
  {
    RCLCPP_ERROR(logger_, "CST startup configuration failed");
    return false;
  }

  auto register_entry =
    [this, &context, slave_config, slave_pos](
      int & destination, std::uint16_t index, std::uint8_t subindex)
    {
      destination = ecrt_slave_config_reg_pdo_entry(
        slave_config, index, subindex, context.domain, nullptr);
      if (destination < 0)
      {
        RCLCPP_ERROR(
          logger_, "Master%u Slave%u PDO %04x:%02x is not mapped",
          context.master_id, slave_pos, index, subindex);
        return false;
      }
      return true;
    };

  return
    register_entry(axis.control_word, 0x6040, 0) &&
    register_entry(axis.target_position, 0x607A, 0) &&
    register_entry(axis.mode_of_operation, 0x6060, 0) &&
    register_entry(axis.status_word, 0x6041, 0) &&
    register_entry(axis.position_actual_value, 0x6064, 0) &&
    register_entry(axis.velocity_actual_value, 0x606C, 0) &&
    register_entry(axis.torque_actual_value, 0x6077, 0) &&
    register_entry(axis.error_code, 0x603F, 0) &&
    (!cst_axis || (register_entry(axis.target_torque, 0x6071, 0) &&
    register_entry(axis.mode_display, 0x6061, 0)));
}

bool KaixuanCanSystem::configure_o6(MasterContext & context)
{
  constexpr std::uint16_t slave_pos = 7;
  ec_slave_info_t slave_info{};
  if (ecrt_master_get_slave(context.master, slave_pos, &slave_info) != 0)
  {
    RCLCPP_ERROR(
      logger_, "Master%u cannot read O6 identity at Slave%u",
      context.master_id, slave_pos);
    return false;
  }
  if (slave_info.alias != 0 || slave_info.vendor_id != kO6VendorId ||
    slave_info.product_code != kO6ProductCode ||
    slave_info.revision_number != kO6Revision)
  {
    RCLCPP_ERROR(
      logger_, "Master%u Slave%u is not the expected O6: alias=%u id=%08x:%08x:%08x",
      context.master_id, slave_pos, slave_info.alias, slave_info.vendor_id, slave_info.product_code,
      slave_info.revision_number);
    return false;
  }

  if (cycle_time_ns_ != kO6Sync0CycleNs)
  {
    RCLCPP_ERROR(
      logger_,
      "LinkerHand O6 and the arm drives require a common 1000000 ns cycle; configured %ld ns",
      static_cast<long>(cycle_time_ns_));
    return false;
  }

  auto * slave_config = ecrt_master_slave_config(
    context.master, 0, slave_pos, kO6VendorId, kO6ProductCode);
  if (slave_config == nullptr)
  {
    RCLCPP_ERROR(
      logger_, "Cannot create config for Master%u LinkerHand O6 Slave%u",
      context.master_id, slave_pos);
    return false;
  }
  context.slave_configs[slave_pos] = slave_config;

  // The O6 PDO assignment and mapping are fixed in SII (PdoAssign=false,
  // PdoConfig=false). Use the mapping discovered from EEPROM and do not queue
  // CoE writes for 0x1C12/0x1C13 or 0x1600..0x1A02.
  if (ecrt_slave_config_watchdog(slave_config, 2498, 1000) != 0)
  {
    RCLCPP_ERROR(
      logger_, "Watchdog configuration failed for Master%u O6 Slave%u",
      context.master_id, slave_pos);
    return false;
  }
  if (ecrt_slave_config_dc(
      slave_config, kO6DcAssignActivate,
      static_cast<std::uint32_t>(kO6Sync0CycleNs),
      static_cast<std::int32_t>(shift_time_ns_), 0, 0) != 0)
  {
    RCLCPP_ERROR(
      logger_, "DC configuration failed for Master%u O6 Slave%u",
      context.master_id, slave_pos);
    return false;
  }

  auto & o6 = context.o6;
  o6 = O6Handle{};
  o6.slave_config = slave_config;
  o6.rx_base = ecrt_slave_config_reg_pdo_entry(
    slave_config, 0x7000, 0x01, context.domain, nullptr);
  const int rx_last = ecrt_slave_config_reg_pdo_entry(
    slave_config, 0x7040, 0x06, context.domain, nullptr);
  o6.tx_base = ecrt_slave_config_reg_pdo_entry(
    slave_config, 0x6000, 0x01, context.domain, nullptr);
  std::array<int, 10> imu_offsets{};
  for (std::uint8_t subindex = 1; subindex <= imu_offsets.size(); ++subindex)
  {
    imu_offsets[subindex - 1] = ecrt_slave_config_reg_pdo_entry(
      slave_config, 0x6200, subindex, context.domain, nullptr);
  }
  bool imu_layout_ok = o6.tx_base >= 0;
  for (std::size_t i = 0; i < imu_offsets.size(); ++i)
  {
    const int expected = o6.tx_base + 240 + static_cast<int>(2 * i);
    if (imu_offsets[i] != expected)
    {
      RCLCPP_ERROR(
        logger_, "O6 IMU PDO 0x6200:%02zx offset=%d, expected=%d",
        i + 1, imu_offsets[i], expected);
      imu_layout_ok = false;
    }
  }
  const int tx_last = imu_offsets.back();
  if (o6.rx_base < 0 || rx_last != o6.rx_base + 31 ||
    o6.tx_base < 0 || tx_last != o6.tx_base + 258 || !imu_layout_ok)
  {
    RCLCPP_ERROR(
      logger_, "O6 PDO layout mismatch: Rx base/last=%d/%d Tx base/last=%d/%d",
      o6.rx_base, rx_last, o6.tx_base, tx_last);
    return false;
  }

  o6.configured = true;
  RCLCPP_INFO(
    logger_,
    "Configured Master%u Slave%u %s O6+BMI088: Rx=32 B Tx=260 B "
    "DC Sync0=%ld ns master_cycle=%ld ns mode=%s",
    context.master_id, slave_pos, o6_hand_name(context.master_id),
    static_cast<long>(kO6Sync0CycleNs), static_cast<long>(cycle_time_ns_),
    o6_imu_only_ ? "IMU-only" : "O6+IMU");
  return true;
}

void KaixuanCanSystem::cyclic_loop(MasterContext & context)
{
  if (!configure_realtime_thread(context))
  {
    context.thread_failed.store(true);
    context.running.store(false);
    return;
  }

  timespec wake_time{};
  clock_gettime(CLOCK_MONOTONIC, &wake_time);
  add_ns(wake_time, cycle_time_ns_);
  context.thread_ready.store(true);
  std::uint64_t virtual_schedule_ns = 0;

  while (context.running.load())
  {
    int sleep_result;
    do
    {
      sleep_result = clock_nanosleep(
        CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, nullptr);
    } while (sleep_result == EINTR);

    if (sleep_result != 0)
    {
      context.thread_failed.store(true);
      break;
    }

    const auto cycle_time = steady_now_ns();
    std::uint64_t host_cycle_time = cycle_time;
    if (follow_reference_clock_) {
      timespec raw_now{};
      if (clock_gettime(CLOCK_MONOTONIC_RAW, &raw_now) != 0) {
        context.thread_failed.store(true);
        break;
      }
      host_cycle_time = timespec_to_ns(raw_now);
    }
    const auto scheduled_time = timespec_to_ns(wake_time);
    context.scheduled_time_ns = scheduled_time;
    context.current_lateness_ns = cycle_time > scheduled_time ? cycle_time - scheduled_time : 0;
    context.current_interval_ns = context.last_cycle_time_ns ? cycle_time - context.last_cycle_time_ns : 0;
    if (cycle_time > scheduled_time)
    {
      context.max_cycle_lateness_ns = std::max(
        context.max_cycle_lateness_ns, cycle_time - scheduled_time);
    }
    if (context.last_cycle_time_ns != 0)
    {
      context.max_loop_interval_ns = std::max(
        context.max_loop_interval_ns, cycle_time - context.last_cycle_time_ns);
    }
    context.last_cycle_time_ns = cycle_time;
    ++context.cycle_count;
    const bool trace_active = context.dc_trace && !context.dc_trace->frozen();
    if (trace_active) {
      auto & row = context.dc_trace_row;
      row = {};
      row.cycle = context.cycle_count;
      row.wake_mono_ns = cycle_time;
      row.wake_host_ns = host_cycle_time;
      row.scheduled_mono_ns = scheduled_time;
      row.previous_send_cycle = context.reference_sample_cycle;
      row.previous_send_mono_ns = context.reference_sample_mono_ns;
      row.previous_send_host_ns = context.reference_sample_host_ns;
      row.previous_send_virtual_ns = context.reference_sample_virtual_ns;
      row.previous_raw = context.reference_sample_raw;
      if (context.reference_sample_pending) {row.flags |= DcReferenceTraceRing::PreviousSendQueued;}
      if (row.previous_raw.valid) {row.flags |= DcReferenceTraceRing::PreviousRawValid;}
    }

    // Freeze B for the entire current cycle, even though the follower below
    // computes B for the NEXT deadline before PDO exchange.
    const auto cycle_base_ns = follow_reference_clock_ ? context.dc_follower_status.base_ns : 0;
    const auto application_time = static_cast<std::uint64_t>(
      static_cast<std::int64_t>(host_cycle_time) - cycle_base_ns);
    if (trace_active) {
      context.dc_trace_row.base_entry_ns = cycle_base_ns;
      context.dc_trace_row.application_ns = application_time;
    }
    if (follow_reference_clock_ && context.cycle_count == 1) {
      // IgH anchors SYNC0 to this first application time, not the earlier
      // planned wakeup. Keep the virtual grid on exactly that same anchor.
      virtual_schedule_ns = application_time;
    }
    ecrt_master_application_time(context.master, application_time);
    ecrt_master_receive(context.master);
    context.final_receive_ns = steady_now_ns();
    ecrt_domain_process(context.domain);
    ec_domain_state_t domain_state{};
    ecrt_domain_state(context.domain, &domain_state);
    context.dc_sample_updated = false;
    if (context.dc_monitor_pending) {
      context.dc_difference_ns = ecrt_master_sync_monitor_process(context.master);
      context.dc_sample_queue_cycle = context.dc_pending_queue_cycle;
      context.dc_sample_reference_age_cycles = context.dc_pending_reference_age_cycles;
      context.dc_sample_updated = true;
      context.dc_monitor_pending = false;
    }
    poll_dc_startup_reset(context, domain_state, cycle_time);
    poll_dc_reference_follower(context, domain_state, host_cycle_time);
    const auto receive_end_ns = steady_now_ns();
    exchange_process_data(context, domain_state);
    const auto exchange_end_ns = steady_now_ns();
    if (context.o6.configured)
    {
      ec_slave_config_state_t slave_state{};
      ecrt_slave_config_state(context.o6.slave_config, &slave_state);
      exchange_o6_process_data(
        context, domain_state, slave_state, cycle_time);
    }
    if (bus_diagnostics_) {
      const auto probe_start = steady_now_ns();
      poll_bus_diagnostics(context, probe_start);
      context.max_probe_work_ns = std::max(context.max_probe_work_ns, steady_now_ns() - probe_start);
    }
    // Keep application_time at the fixed cycle entry point for DC phase setup.
    // Refresh the separate reference-clock correction after variable cycle work.
    const auto sync_sample_ns = steady_now_ns();
    // Only the host-to-reference correction is decimated. PDO and slave-to-slave
    // FRMW synchronization below remain on every cycle (as in IgH dc_rtai).
    const bool reference_sync_due = !follow_reference_clock_ &&
      (context.cycle_count - 1) % reference_sync_cycles_ == 0;
    int sync_result = 0;
    if (reference_sync_due) {
      ++context.reference_sync_attempts;
      sync_result = fresh_reference_time_ ?
        ecrt_master_sync_reference_clock_to(context.master, sync_sample_ns) :
        ecrt_master_sync_reference_clock(context.master);
      context.sync_errors += sync_result != 0;
      if (sync_result == 0) {
        ++context.reference_sync_queued;
        ++context.timing_reference_sync_cycles;
        context.last_reference_sync_cycle = context.cycle_count;
      }
    }
    const auto sync_end_ns = steady_now_ns();
    const int slave_sync_result = ecrt_master_sync_slave_clocks(context.master);
    context.sync_errors += slave_sync_result != 0;
    const auto dc_period_cycles = context.startup_guard.latched ? 100U : 20U;
    if (context.cycle_count % dc_period_cycles == 0) {
      if (ecrt_master_sync_monitor_queue(context.master) == 0) {
        context.dc_monitor_pending = true;
        context.dc_pending_queue_cycle = context.cycle_count;
        context.dc_pending_reference_age_cycles = context.last_reference_sync_cycle == 0 ? UINT64_MAX :
          context.cycle_count - context.last_reference_sync_cycle;
      } else {
        context.dc_difference_ns = UINT32_MAX;
        context.startup_guard.invalidate_dc();
      }
    }
    ecrt_domain_queue(context.domain);
    ++context.queue_count;
    if (trace_active) {
      // Diagnostics only. Keep the original control timestamp below immediately
      // before ecrt_master_send; do not substitute the midpoint of this bracket.
      timespec mono_before{}, raw{}, mono_after{};
      const int before_result = clock_gettime(CLOCK_MONOTONIC, &mono_before);
      const int raw_result = clock_gettime(CLOCK_MONOTONIC_RAW, &raw);
      const int after_result = clock_gettime(CLOCK_MONOTONIC, &mono_after);
      auto & bracket = context.dc_trace_row.current_raw;
      bracket.mono_before_ns = timespec_to_ns(mono_before);
      bracket.raw_ns = timespec_to_ns(raw);
      bracket.mono_after_ns = timespec_to_ns(mono_after);
      bracket.valid = before_result == 0 && raw_result == 0 && after_result == 0 &&
        bracket.mono_after_ns >= bracket.mono_before_ns;
      if (bracket.valid) {context.dc_trace_row.flags |= DcReferenceTraceRing::CurrentRawValid;}
      else {context.dc_trace_row.flags |= DcReferenceTraceRing::ClockReadError;}
    }
    const auto send_start_ns = steady_now_ns();
    std::uint64_t host_send_ns = send_start_ns;
    if (follow_reference_clock_) {
      timespec raw_send{};
      if (clock_gettime(CLOCK_MONOTONIC_RAW, &raw_send) != 0) {
        context.thread_failed.store(true);
        break;
      }
      host_send_ns = timespec_to_ns(raw_send);
    }
    context.previous_send_result = ecrt_master_send(context.master);
    const auto send_end_ns = steady_now_ns();
    if (follow_reference_clock_) {
      context.reference_sample_pending = slave_sync_result == 0 && context.previous_send_result == 0;
      context.reference_sample_cycle = context.cycle_count;
      context.reference_sample_mono_ns = send_start_ns;
      context.reference_sample_host_ns = host_send_ns;
      context.reference_sample_virtual_ns = static_cast<std::uint64_t>(
        static_cast<std::int64_t>(host_send_ns) - cycle_base_ns);
      if (trace_active) {context.reference_sample_raw = context.dc_trace_row.current_raw;}
    }
    context.previous_send_duration_ns = send_end_ns - send_start_ns;
    ++context.send_count;
    context.last_send_time_ns = steady_now_ns();
    const std::array<std::uint64_t, 9> timing = {
      receive_end_ns - cycle_time, context.timing_mutex_wait_ns,
      exchange_end_ns - receive_end_ns,
      reference_sync_due ? sync_sample_ns - cycle_time : 0,
      reference_sync_due ? sync_end_ns - sync_sample_ns : 0,
      reference_sync_due && sync_result == 0 ?
      send_start_ns - (fresh_reference_time_ ? sync_sample_ns : cycle_time) : 0,
      send_start_ns - cycle_time, send_end_ns - send_start_ns, send_end_ns - cycle_time};
    if (context.timing_window_start_ns == 0) {context.timing_window_start_ns = cycle_time;}
    ++context.timing_window_cycles;
    for (std::size_t i = 0; i < timing.size(); ++i) {
      context.timing_max_ns[i] = std::max(context.timing_max_ns[i], timing[i]);
      context.timing_window_max_ns[i] = std::max(context.timing_window_max_ns[i], timing[i]);
    }
    if (timing[6] > 100000) {
      ++context.slow_send_cycles;
      context.last_slow_send_ns = cycle_time;
    }
    if (send_end_ns - context.timing_window_start_ns >= 1000000000ULL) {
      context.timing_completed_max_ns = context.timing_window_max_ns;
      context.timing_completed_start_ns = context.timing_window_start_ns;
      context.timing_completed_end_ns = send_end_ns;
      context.timing_completed_cycles = context.timing_window_cycles;
      context.timing_completed_reference_sync_cycles = context.timing_reference_sync_cycles;
      context.timing_reference_sync_cycles = 0;
      context.timing_window_max_ns = {};
      context.timing_window_start_ns = send_end_ns;
      context.timing_window_cycles = 0;
    }
    if (domain_state.wc_state == EC_WC_COMPLETE)
    {
      context.last_complete_send_time_ns = context.last_send_time_ns;
    }

    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (follow_reference_clock_) {
      virtual_schedule_ns += static_cast<std::uint64_t>(cycle_time_ns_);
      auto next_ns = static_cast<std::uint64_t>(static_cast<std::int64_t>(virtual_schedule_ns) +
        context.dc_follower_status.base_ns);
      // RAW drives both the application clock and follower. MONOTONIC is used
      // only by the supported sleep API; never carry its NTP slew into the PLL.
      timespec mono_before{}, raw_now{}, mono_after{};
      if (clock_gettime(CLOCK_MONOTONIC, &mono_before) != 0 ||
        clock_gettime(CLOCK_MONOTONIC_RAW, &raw_now) != 0 ||
        clock_gettime(CLOCK_MONOTONIC, &mono_after) != 0)
      {
        context.thread_failed.store(true);
        break;
      }
      const auto now_ns = timespec_to_ns(raw_now);
      if (next_ns <= now_ns) {
        context.follower_schedule_failed = true;
        if (context.follower_failure_cycle == 0) {context.follower_failure_cycle = context.cycle_count;}
        ++context.schedule_rebases;
        const auto skipped = (now_ns - next_ns) / static_cast<std::uint64_t>(cycle_time_ns_) + 1;
        context.follower_skipped_periods += skipped;
        const auto advance = skipped * static_cast<std::uint64_t>(cycle_time_ns_);
        virtual_schedule_ns += advance;
        next_ns += advance;
      }
      std::uint64_t sleep_deadline_ns = 0;
      if (!dc_raw_deadline_to_monotonic(next_ns, now_ns,
        timespec_to_ns(mono_before), timespec_to_ns(mono_after), sleep_deadline_ns))
      {
        context.thread_failed.store(true);
        break;
      }
      wake_time.tv_sec = static_cast<time_t>(sleep_deadline_ns / 1000000000ULL);
      wake_time.tv_nsec = static_cast<long>(sleep_deadline_ns % 1000000000ULL);
      if (trace_active) {context.dc_trace_row.next_host_deadline_ns = next_ns;}
    } else {
      add_ns(wake_time, cycle_time_ns_);
      if (timespec_to_ns(now) >
        timespec_to_ns(wake_time) + static_cast<std::uint64_t>(cycle_time_ns_))
      {
        ++context.schedule_rebases;
        wake_time = now;
        add_ns(wake_time, cycle_time_ns_);
      }
    }
    if (trace_active) {
      auto & row = context.dc_trace_row;
      const auto & follower = context.dc_follower_status;
      row.base_after_ns = follower.base_ns;
      row.measurement_ns = follower.measurement_error_ns;
      row.residual_ns = follower.residual_ns;
      row.filtered_ns = follower.filtered_error_ns;
      row.frequency_ns_per_cycle = follower.frequency_ns_per_cycle;
      row.correction_ns = follower.next_correction_ns;
      row.sample_age_ns = follower.sample_age_ns;
      row.last_valid_age_ns = follower.last_valid_age_ns;
      row.quality = static_cast<std::uint32_t>(follower.quality);
      row.failure = static_cast<std::uint32_t>(follower.failure);
      row.rejection = static_cast<std::uint32_t>(follower.rejected_because);
      row.bad_quality_samples = follower.bad_quality_samples;
      row.first_failure_cycle = context.follower_failure_cycle;
      row.working_counter = domain_state.working_counter;
      row.wc_state = domain_state.wc_state;
      row.receive_end_ns = receive_end_ns;
      row.exchange_end_ns = exchange_end_ns;
      row.mutex_wait_ns = context.timing_mutex_wait_ns;
      row.send_before_ns = send_start_ns;
      row.send_after_ns = send_end_ns;
      row.slave_sync_result = slave_sync_result;
      row.send_result = context.previous_send_result;
      row.next_deadline_ns = timespec_to_ns(wake_time);
      if (follower.sample_accepted) {row.flags |= DcReferenceTraceRing::SampleAccepted;}
      if (follower.residual_available) {row.flags |= DcReferenceTraceRing::ResidualAvailable;}
      if (follower.calibrated) {row.flags |= DcReferenceTraceRing::Calibrated;}
      if (follower.locked) {row.flags |= DcReferenceTraceRing::Locked;}
      if (follower.acquiring_phase) {row.flags |= DcReferenceTraceRing::Acquiring;}
      if (follower.failed) {row.flags |= DcReferenceTraceRing::FollowerFailed;}
      if (context.follower_getter_failed) {row.flags |= DcReferenceTraceRing::GetterFailed;}
      if (context.follower_schedule_failed) {row.flags |= DcReferenceTraceRing::ScheduleFailed;}
      if (context.startup_guard.latched) {row.flags |= DcReferenceTraceRing::StartupReady;}
      bool all_cw6 = true, all_cw15 = true, all_disabled = true, all_enabled = true;
      for (std::size_t i = 0; i < context.axes.size(); ++i) {
        const auto & axis = context.axes[i];
        const auto * data = context.domain_data;
        row.control[i] = EC_READ_U16(data + axis.control_word);
        row.status[i] = EC_READ_U16(data + axis.status_word);
        row.error[i] = EC_READ_U16(data + axis.error_code);
        const auto state = row.status[i] & 0x006f;
        all_cw6 = all_cw6 && row.control[i] == 6;
        all_cw15 = all_cw15 && row.control[i] == 15;
        all_disabled = all_disabled && (state == 0x21 || state == 0x40) && row.error[i] == 0;
        all_enabled = all_enabled && state == 0x27 && row.error[i] == 0;
      }
      if (all_cw6) {row.flags |= DcReferenceTraceRing::AllControl6;}
      if (all_cw15) {row.flags |= DcReferenceTraceRing::AllControl15;}
      if (all_disabled) {row.flags |= DcReferenceTraceRing::AllDisabledFeedback;}
      if (all_enabled) {row.flags |= DcReferenceTraceRing::AllEnabledFeedback;}
      // Include the first failing cycle, then preserve it throughout shutdown.
      context.dc_trace->push(row,
        follower.failed || context.follower_getter_failed || context.follower_schedule_failed);
    }
  }
  context.loop_exit_ns = steady_now_ns();
  if (context.o6.configured && context.master_activated)
  {
    send_o6_disable_cycles(context, wake_time);
  }
}

void KaixuanCanSystem::exchange_process_data(
  MasterContext & context, const ec_domain_state_t & domain_state)
{
  const auto mutex_start_ns = steady_now_ns();
  std::lock_guard<std::mutex> lock(io_mutex_);
  context.timing_mutex_wait_ns = steady_now_ns() - mutex_start_ns;
  auto & snapshot = diagnostic_snapshots_[context.master_id];
  snapshot.timing_max_ns = context.timing_max_ns;
  snapshot.timing_window_max_ns = context.timing_completed_max_ns;
  snapshot.timing_window_start_ns = context.timing_completed_start_ns;
  snapshot.timing_window_end_ns = context.timing_completed_end_ns;
  snapshot.timing_window_cycles = context.timing_completed_cycles;
  snapshot.slow_send_cycles = context.slow_send_cycles;
  snapshot.last_slow_send_ns = context.last_slow_send_ns;
  snapshot.sync_errors = context.sync_errors;
  snapshot.schedule_rebases = context.schedule_rebases;
  snapshot.reference_sync_attempts = context.reference_sync_attempts;
  snapshot.reference_sync_queued = context.reference_sync_queued;
  snapshot.last_reference_sync_cycle = context.last_reference_sync_cycle;
  snapshot.timing_reference_sync_cycles = context.timing_completed_reference_sync_cycles;
  snapshot.sample_steady_ns = context.last_cycle_time_ns;
  snapshot.cycle_count = context.cycle_count;
  snapshot.max_probe_work_ns = context.max_probe_work_ns;
  snapshot.working_counter = domain_state.working_counter;
  snapshot.wc_state = domain_state.wc_state;
  snapshot.bad_wc_cycles += domain_state.wc_state != EC_WC_COMPLETE;
  snapshot.max_cycle_lateness_ns = context.max_cycle_lateness_ns;
  snapshot.max_loop_interval_ns = context.max_loop_interval_ns;
  ec_master_state_t master_state{};
  const int master_state_result = ecrt_master_state(context.master, &master_state);
  context.probe_bus_ready = master_state_result == 0 && master_state.al_states == 8 &&
    master_state.link_up && domain_state.wc_state == EC_WC_COMPLETE;
  const bool follower_ready = !follow_reference_clock_ ||
    (context.dc_follower_status.locked && !context.dc_follower_status.failed &&
    !context.follower_schedule_failed && !context.follower_getter_failed);
  const bool enable_allowed = context.startup_guard.step(
    activated_ && context.dc_reset_status.ready && follower_ready, all_joint_positions_ready(),
    context.probe_bus_ready && master_state.slaves_responding ==
      (context.o6.configured ? kMaxSlavesPerMaster : kJointsPerArm),
    context.dc_sample_updated, context.dc_difference_ns);
  snapshot.startup_ready = context.startup_guard.latched;
  snapshot.dc_reset_ready = context.dc_reset_status.ready;
  snapshot.dc_reset_failed = context.dc_reset_status.failed;
  snapshot.dc_reset_phase = static_cast<unsigned int>(context.dc_reset_status.phase);
  snapshot.dc_reset_error = static_cast<unsigned int>(context.dc_reset_status.error);
  snapshot.dc_reset_original_bandwidth = context.dc_reset_status.speed_counter_start;
  snapshot.dc_reset_original_filter = context.dc_reset_status.speed_filter;
  snapshot.dc_reset_observation_start_ns = context.dc_reset_status.observation_start_ns;
  snapshot.dc_follower_config = context.dc_follower_config;
  snapshot.dc_follower = context.dc_follower_status;
  snapshot.dc_follower_ready = follower_ready;
  snapshot.dc_follower_failed = follow_reference_clock_ &&
    (context.dc_follower_status.failed || context.follower_schedule_failed || context.follower_getter_failed);
  snapshot.follower_schedule_failed = context.follower_schedule_failed;
  snapshot.follower_getter_failed = context.follower_getter_failed;
  snapshot.follower_getter_consecutive_errors = context.follower_getter_consecutive_errors;
  snapshot.reference_getter_errors = context.reference_getter_errors;
  snapshot.follower_skipped_periods = context.follower_skipped_periods;
  snapshot.reference_sample_cycle = context.reference_sample_cycle;
  snapshot.follower_last_ref32 = context.follower_last_ref32;
  snapshot.follower_failure_cycle = context.follower_failure_cycle;
  snapshot.follower_reject_cycle = context.follower_reject_cycle;
  snapshot.follower_failure_residual_ns = context.follower_failure_residual_ns;
  snapshot.follower_reject_residual_ns = context.follower_reject_residual_ns;
  snapshot.follower_reject_reason = context.follower_reject_reason;
  snapshot.follower_ref_received_cycle = context.follower_ref_received_cycle;
  snapshot.follower_calibration_cycle = context.follower_calibration_cycle;
  snapshot.follower_calibration_delta_ns = context.follower_calibration_delta_ns;
  snapshot.startup_bus_cycles = context.startup_guard.bus_cycles;
  snapshot.startup_dc_samples = context.startup_guard.dc_good_samples;
  snapshot.dc_difference_ns = context.dc_difference_ns;
  snapshot.dc_sample_queue_cycle = context.dc_sample_queue_cycle;
  snapshot.dc_sample_reference_age_cycles = context.dc_sample_reference_age_cycles;
  CycleFrame frame{};
  frame[0] = context.cycle_count;
  frame[1] = context.last_cycle_time_ns;
  frame[2] = context.scheduled_time_ns;
  frame[3] = context.current_lateness_ns;
  frame[4] = context.current_interval_ns;
  frame[5] = domain_state.working_counter;
  frame[6] = domain_state.wc_state;
  frame[7] = context.previous_send_result;
  frame[8] = context.previous_send_duration_ns;
  frame[9] = cycle_captures_[context.master_id].dropped();
  frame[10] = master_state.al_states;
  frame[11] = master_state.slaves_responding;
  frame[12] = master_state.link_up;
  frame[13] = master_state_result;
  for (auto & axis : context.axes)
  {
    const auto joint = axis.joint_index;
    const auto * data = context.domain_data;
    const auto raw_counts = EC_READ_S32(data + axis.position_actual_value);
    const auto raw_velocity = EC_READ_S32(data + axis.velocity_actual_value);
    const auto status_word = EC_READ_U16(data + axis.status_word);
    const auto error_code = EC_READ_U16(data + axis.error_code);
    const bool cst_axis = cst_mode_ && cst_selected_[joint];
    const int actual_mode = cst_axis ? EC_READ_S8(data + axis.mode_display) : 8;

    const bool feedback_ready = domain_state.wc_state == EC_WC_COMPLETE &&
      (status_word != 0 || raw_counts != 0);
    if (feedback_ready)
    {
      actual_counts_[joint] = raw_counts;
      const auto actual_position = joint_counts_to_radians(raw_counts, joint);
      measured_positions_[joint] = actual_position;
      measured_velocities_[joint] = axis.velocity_counts_per_second ?
        joint_direction_[joint] * raw_velocity * 2.0 * kPi /
        (kMotorCountsPerRevolution * joint_gear_ratio_[joint]) :
        motor_rpm_to_joint_radians_per_second(raw_velocity, joint);
      measured_efforts_[joint] = torque_parameters_[joint].feedback_nm(
        EC_READ_S16(data + axis.torque_actual_value), joint_direction_[joint],
        axis.torque_feedback_unit, axis.rated_current_a);
      if (!position_initialized_[joint])
      {
        position_initialized_[joint] = true;
        if (!cst_mode_) {hw_commands_[joint] = actual_position;}
        commanded_counts_[joint] = raw_counts;
      }
      if (cst_mode_) {commanded_counts_[joint] = raw_counts;}
    }

    EC_WRITE_S8(data + axis.mode_of_operation, cst_axis ? 10 : 8);
    if (position_initialized_[joint])
    {
      EC_WRITE_S32(data + axis.target_position, commanded_counts_[joint]);
    }
    bool axis_enable_allowed = enable_allowed && !cst_mode_;
    TorqueSafety::Output torque_output;
    if (cst_axis) {
      TorqueSafety::Input input;
      input.requested = activated_ && effort_claimed_[joint];
      input.startup_ready = enable_allowed;
      input.bus_ready = context.probe_bus_ready && follower_ready && feedback_ready;
      const auto now_ns = steady_now_ns();
      input.write_fresh = effort_write_ns_ != 0 && now_ns >= effort_write_ns_ &&
        now_ns - effort_write_ns_ <= effort_write_timeout_ns_;
      input.mode = actual_mode;
      input.status = status_word;
      input.error = error_code;
      input.command = requested_efforts_[joint];
      input.position = measured_positions_[joint];
      input.unwrapped_position = JointAngleMapping{
        joint_gear_ratio_[joint], joint_offset_deg_[joint], joint_direction_[joint]}.
        unwrapped_position(raw_counts);
      input.velocity = measured_velocities_[joint];
      input.period_s = cycle_time_ns_ * 1e-9;
      torque_output = torque_safety_[joint].step(input, torque_parameters_[joint]);
      axis_enable_allowed = torque_output.enable;
      EC_WRITE_S16(data + axis.target_torque, torque_parameters_[joint].to_raw(
        torque_output.effort_nm, joint_direction_[joint]));
    }
    std::uint16_t control_word = EC_READ_U16(data + axis.control_word);
    if (!axis_enable_allowed) {
      control_word = disabled_control_word();
      axis.enable_settle.reset();
    } else switch (decode_cia402_state(status_word))
    {
      case Cia402State::kNotReadyToSwitchOn:
      case Cia402State::kSwitchOnDisabled:
        control_word = 0x0006;
        axis.enable_settle.reset();
        break;
      case Cia402State::kReadyToSwitchOn:
        control_word = 0x0007;
        axis.enable_settle.reset();
        break;
      case Cia402State::kSwitchedOn:
        control_word = axis.enable_settle.switched_on_controlword();
        break;
      case Cia402State::kOperationEnabled:
        control_word = 0x000F;
        break;
      case Cia402State::kFault:
        control_word = 0x0080;
        axis.enable_settle.reset();
        break;
      case Cia402State::kQuickStopActive:
      case Cia402State::kFaultReactionActive:
      case Cia402State::kUnknown:
        break;
    }
    EC_WRITE_U16(data + axis.control_word, control_word);
    auto & diagnostic = snapshot.axes[joint % kJointsPerArm];
    const auto old_status = diagnostic.status_word;
    const auto old_control = diagnostic.control_word;
    const auto old_error = diagnostic.error_code;
    const bool old_following_error = diagnostic.following_error;
    diagnostic.actual_counts = raw_counts;
    diagnostic.target_counts = EC_READ_S32(data + axis.target_position);
    diagnostic.velocity_raw = raw_velocity;
    diagnostic.torque_raw = EC_READ_S16(data + axis.torque_actual_value);
    diagnostic.effort_nm = measured_efforts_[joint];
    diagnostic.commanded_effort_nm = torque_output.effort_nm;
    diagnostic.target_torque_raw = cst_axis ? EC_READ_S16(data + axis.target_torque) : 0;
    diagnostic.actual_mode = actual_mode;
    diagnostic.effort_claimed = effort_claimed_[joint];
    diagnostic.torque_armed = torque_safety_[joint].armed;
    diagnostic.torque_guard_fault = static_cast<unsigned>(torque_safety_[joint].fault);
    diagnostic.torque_fault_snapshot = torque_safety_[joint].fault_snapshot;
    diagnostic.status_word = status_word;
    diagnostic.control_word = control_word;
    diagnostic.error_code = error_code;
    if (diagnostic.error_code != 0) {
      diagnostic.last_error_code = diagnostic.error_code;
    }
    diagnostic.fault_cycles += (status_word & 0x0008U) != 0;
    diagnostic.position_rad = measured_positions_[joint];
    diagnostic.velocity_rad_s = measured_velocities_[joint];
    diagnostic.following_error = !cst_axis && domain_state.wc_state == EC_WC_COMPLETE &&
      (status_word & 0x006F) == 0x0027 &&
      std::abs(static_cast<std::int64_t>(diagnostic.target_counts) - raw_counts) >
      static_cast<double>(kMotorCountsPerRevolution) * joint_gear_ratio_[joint] / 360.0;
    const auto offset = kCaptureHeader + (joint % kJointsPerArm) * kCaptureAxisWidth;
    frame[offset] = diagnostic.actual_counts;
    frame[offset + 1] = diagnostic.target_counts;
    frame[offset + 2] = diagnostic.velocity_raw;
    frame[offset + 3] = diagnostic.torque_raw;
    frame[offset + 4] = diagnostic.status_word;
    frame[offset + 5] = diagnostic.control_word;
    frame[offset + 6] = diagnostic.error_code;
    frame[offset + 7] = capture_transition(old_status, diagnostic.status_word,
      old_control, diagnostic.control_word, old_error, diagnostic.error_code,
      diagnostic.following_error && !old_following_error);
    frame[offset + 8] = EC_READ_S8(data + axis.mode_of_operation);
  }
  if (cst_mode_ && stop_cst_group_on_fault_) {
    if (latch_torque_group(torque_safety_, effort_claimed_)) {
      for (auto & axis : context.axes) {
        const auto joint = axis.joint_index;
        if (!cst_selected_[joint] || !effort_claimed_[joint]) {continue;}
        auto * data = context.domain_data;
        auto & diagnostic = snapshot.axes[joint % kJointsPerArm];
        const auto offset = kCaptureHeader + (joint % kJointsPerArm) * kCaptureAxisWidth;
        frame[offset + 7] |= capture_transition(diagnostic.status_word, diagnostic.status_word,
          diagnostic.control_word, disabled_control_word(), diagnostic.error_code, diagnostic.error_code, false);
        EC_WRITE_S16(data + axis.target_torque, 0);
        EC_WRITE_U16(data + axis.control_word, disabled_control_word());
        axis.enable_settle.reset();
        diagnostic.commanded_effort_nm = 0;
        diagnostic.target_torque_raw = 0;
        diagnostic.control_word = disabled_control_word();
        diagnostic.torque_guard_fault = static_cast<unsigned>(torque_safety_[joint].fault);
        frame[offset + 5] = disabled_control_word();
      }
    }
  }
  cycle_captures_[context.master_id].push(frame);
}

void KaixuanCanSystem::start_diagnostics()
{
  try {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    diagnostic_node_ = std::make_shared<rclcpp::Node>("kaixuan_hardware_diagnostics", options);
    diagnostic_publisher_ = diagnostic_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/kaixuan/hardware_diagnostics", rclcpp::SensorDataQoS());
    cycle_publisher_ = diagnostic_node_->create_publisher<std_msgs::msg::Int64MultiArray>(
      "/kaixuan/cycle_capture", rclcpp::QoS(100).reliable());
    bus_publisher_ = diagnostic_node_->create_publisher<std_msgs::msg::Int64MultiArray>(
      "/kaixuan/bus_diagnostics", rclcpp::QoS(100).reliable());
    diagnostic_timer_ = diagnostic_node_->create_wall_timer(
      std::chrono::milliseconds(50), [this]() {
        publish_cycle_capture();
        publish_bus_capture();
        // Cheap 5 Hz normal snapshots; raw cycles are consumed separately.
        if (++diagnostic_divider_ % 4 == 0) {publish_diagnostics();}
      });
    diagnostic_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    diagnostic_executor_->add_node(diagnostic_node_);
    diagnostic_thread_ = std::thread([this]() {diagnostic_executor_->spin();});
  } catch (const std::exception & exception) {
    stop_diagnostics();
    RCLCPP_ERROR(logger_, "Hardware diagnostic publisher unavailable: %s", exception.what());
  }
}

void KaixuanCanSystem::stop_diagnostics()
{
  if (diagnostic_executor_) {diagnostic_executor_->cancel();}
  if (diagnostic_thread_.joinable()) {diagnostic_thread_.join();}
  diagnostic_timer_.reset();
  diagnostic_publisher_.reset();
  cycle_publisher_.reset();
  bus_publisher_.reset();
  diagnostic_executor_.reset();
  diagnostic_node_.reset();
}

void KaixuanCanSystem::publish_cycle_capture()
{
  for (std::size_t master = 0; master < kMasterCount; ++master) {
    if (!master_is_active(master)) {continue;}
    std_msgs::msg::Int64MultiArray message;
    message.data.reserve(3 + 256 * kCaptureWidth);
    message.data = {1, static_cast<std::int64_t>(master), static_cast<std::int64_t>(kCaptureWidth)};
    CycleFrame frame;
    for (unsigned int i = 0; i < 256 && cycle_captures_[master].pop(frame); ++i) {
      message.data.insert(message.data.end(), frame.begin(), frame.end());
    }
    if (message.data.size() > 3) {cycle_publisher_->publish(message);}
  }
}

void KaixuanCanSystem::poll_dc_reference_follower(
  MasterContext & context, const ec_domain_state_t & domain_state, std::uint64_t now_ns)
{
  if (!follow_reference_clock_) {return;}
  DcReferenceFollower::Input input;
  input.cycle_id = context.cycle_count;
  input.matched_send_cycle_id = context.reference_sample_cycle;
  input.host_virtual_send_ns = context.reference_sample_virtual_ns;
  input.host_mono_send_ns = context.reference_sample_host_ns;
  input.now_mono_ns = now_ns;
  const bool fresh_bus = domain_state.wc_state == EC_WC_COMPLETE && context.probe_bus_ready;
  input.drives_disabled = fresh_bus;
  for (const auto & axis : context.axes) {
    const auto * data = context.domain_data;
    input.drives_disabled = input.drives_disabled && cia402_disabled_feedback(
      EC_READ_U16(data + axis.control_word), EC_READ_U16(data + axis.status_word),
      EC_READ_U16(data + axis.error_code));
  }
  // Read the previous FRMW before this cycle requeues the shared datagram.
  // Complete PDO WKC alone does not guarantee receipt of this other datagram.
  if (context.reference_sample_pending && fresh_bus &&
    !context.dc_follower_status.failed && !context.follower_schedule_failed && !context.follower_getter_failed &&
    context.cycle_count >= context.reference_getter_retry_cycle)
  {
    const bool trace_getter = context.dc_trace && !context.dc_trace->frozen();
    if (trace_getter) {
      context.dc_trace_row.flags |= DcReferenceTraceRing::GetterCalled;
      context.dc_trace_row.getter_before_ns = steady_now_ns();
    }
    const int getter_result = ecrt_master_reference_clock_time(context.master, &input.ref32);
    if (trace_getter) {
      context.dc_trace_row.getter_after_ns = steady_now_ns();
      context.dc_trace_row.getter_result = getter_result;
      context.dc_trace_row.ref32 = input.ref32;
      if (getter_result == 0) {context.dc_trace_row.flags |= DcReferenceTraceRing::GetterSucceeded;}
    }
    input.valid = getter_result == 0;
    if (!input.valid) {
      ++context.reference_getter_errors;
      if (++context.follower_getter_consecutive_errors >= 4) {
        context.follower_getter_failed = true;
        if (context.follower_failure_cycle == 0) {context.follower_failure_cycle = context.cycle_count;}
      }
      // Bound libethercat's error-path stderr while leaving recovery attempts
      // inside the follower's 20 ms holdover window.
      context.reference_getter_retry_cycle = context.cycle_count + 5;
    } else {
      context.follower_getter_consecutive_errors = 0;
    }
  }
  context.reference_sample_pending = false;
  context.dc_follower_status = context.dc_follower.step(input);
  if (input.valid) {
    context.follower_last_ref32 = input.ref32;
    context.follower_ref_received_cycle = context.cycle_count;
  }
  const auto & status = context.dc_follower_status;
  if (status.calibrated_this_step) {
    context.follower_calibration_cycle = context.cycle_count;
    context.follower_calibration_delta_ns = status.calibration_delta_ns;
  }
  if (input.valid && status.rejected_because != DcReferenceFollower::Reject::None) {
    context.follower_reject_cycle = context.cycle_count;
    context.follower_reject_residual_ns = status.residual_ns;
    context.follower_reject_reason = static_cast<unsigned int>(status.rejected_because);
  }
  if (status.failed && context.follower_failure_cycle == 0) {
    context.follower_failure_cycle = context.cycle_count;
    context.follower_failure_residual_ns = status.residual_ns;
  }
}

void KaixuanCanSystem::poll_dc_startup_reset(
  MasterContext & context, const ec_domain_state_t & domain_state, std::uint64_t now_ns)
{
  if (!dc_reset_after_configuration_) {return;}
  using Reset = DcStartupReset;
  Reset::Input input;
  input.now_ns = now_ns;
  input.fresh_disabled = domain_state.wc_state == EC_WC_COMPLETE && context.probe_bus_ready;
  for (const auto & axis : context.axes) {
    const auto * data = context.domain_data;
    input.fresh_disabled = input.fresh_disabled && cia402_disabled_feedback(
      EC_READ_U16(data + axis.control_word), EC_READ_U16(data + axis.status_word),
      EC_READ_U16(data + axis.error_code));
  }
  if (context.dc_reset_action.kind != Reset::Kind::None) {
    input.reply.token = context.dc_reset_action.token;
    if (context.dc_reset_queue_rejected || context.dc_reset_request == nullptr) {
      input.reply.state = Reset::RequestState::Rejected;
    } else {
      switch (ecrt_reg_request_state(context.dc_reset_request)) {
        case EC_REQUEST_BUSY: input.reply.state = Reset::RequestState::Busy; break;
        case EC_REQUEST_ERROR: input.reply.state = Reset::RequestState::Error; break;
        case EC_REQUEST_SUCCESS:
          input.reply.state = Reset::RequestState::Success;
          input.reply.size = context.dc_reset_action.size;
          std::copy_n(ecrt_reg_request_data(context.dc_reset_request), input.reply.size,
            input.reply.bytes.begin());
          break;
        default: input.reply.state = Reset::RequestState::Rejected; break;
      }
    }
  }
  const auto output = context.dc_reset.step(input);
  context.dc_reset_status = output.status;
  if (output.discard_old_dc_samples) {
    context.startup_guard.reset();
    context.dc_monitor_pending = false;
    context.dc_sample_updated = false;
    context.dc_difference_ns = UINT32_MAX;
    context.dc_pending_queue_cycle = context.dc_sample_queue_cycle = 0;
    context.dc_pending_reference_age_cycles = context.dc_sample_reference_age_cycles = UINT64_MAX;
  }
  if (!output.status.pending) {
    context.dc_reset_action = {};
    context.dc_reset_queue_rejected = false;
  }
  if (output.action.kind != Reset::Kind::None) {
    context.dc_reset_action = output.action;
    context.dc_reset_queue_rejected = false;
    // The state machine owns each request through its terminal reply. Also
    // defend against accidental handle reuse without touching a BUSY buffer.
    if (context.dc_reset_request == nullptr ||
      ecrt_reg_request_state(context.dc_reset_request) == EC_REQUEST_BUSY)
    {
      context.dc_reset_queue_rejected = true;
      return;
    }
    int result;
    if (output.action.kind == Reset::Kind::Write) {
      std::copy_n(output.action.bytes.begin(), output.action.size,
        ecrt_reg_request_data(context.dc_reset_request));
      result = ecrt_reg_request_write(context.dc_reset_request,
        output.action.address, output.action.size);
    } else {
      result = ecrt_reg_request_read(context.dc_reset_request,
        output.action.address, output.action.size);
    }
    context.dc_reset_queue_rejected = result != 0;
  }
}

void KaixuanCanSystem::poll_bus_diagnostics(MasterContext & context, std::uint64_t now_ns)
{
  // Only the cyclic thread accesses request handles. The context retains the
  // active handle while the temporary adapter is reconstructed each cycle.
  struct Transport
  {
    MasterContext & context;
    ec_reg_request_t * reg;
    ec_sdo_request_t * sdo;
    std::size_t expected_size;
    int start(std::size_t slave, const ProbeSpec & spec)
    {
      context.probe_active_slave = slave;
      context.probe_active_kind = spec.kind;
      context.probe_expected_size = spec.size;
      reg = context.probe_registers[slave];
      sdo = context.probe_sdos[slave];
      expected_size = spec.size;
      if (spec.kind == 0) {
        return reg ? ecrt_reg_request_read(reg, spec.index, spec.size) : -ENODEV;
      }
      if (!sdo) {return -ENODEV;}
      const int result = ecrt_sdo_request_index(sdo, spec.index, spec.sub);
      return result == 0 ? ecrt_sdo_request_read(sdo) : result;
    }
    int state()
    {
      const auto value = context.probe_active_kind == 0 ?
        ecrt_reg_request_state(reg) : ecrt_sdo_request_state(sdo);
      switch (value) {
        case EC_REQUEST_BUSY: return BusProbe::Busy;
        case EC_REQUEST_SUCCESS: return BusProbe::Success;
        case EC_REQUEST_ERROR: return BusProbe::Error;
        default: return BusProbe::Unused;
      }
    }
    std::size_t size()
    {
      return context.probe_active_kind == 0 ? expected_size : ecrt_sdo_request_data_size(sdo);
    }
    const std::uint8_t * data()
    {
      return context.probe_active_kind == 0 ? ecrt_reg_request_data(reg) : ecrt_sdo_request_data(sdo);
    }
  } transport{context, context.probe_registers[context.probe_active_slave],
    context.probe_sdos[context.probe_active_slave], context.probe_expected_size};
  context.probe.step(now_ns, transport, bus_captures_[context.master_id], context.probe_bus_ready);
}

void KaixuanCanSystem::publish_bus_capture()
{
  if (!bus_diagnostics_) {return;}
  for (std::size_t master = 0; master < kMasterCount; ++master) {
    if (!master_is_active(master)) {continue;}
    std_msgs::msg::Int64MultiArray message;
    message.data.reserve(3 + 32 * kCaptureWidth);
    message.data = {1, static_cast<std::int64_t>(master), static_cast<std::int64_t>(kCaptureWidth)};
    CycleFrame frame;
    for (unsigned int i = 0; i < 32 && bus_captures_[master].pop(frame); ++i) {
      message.data.insert(message.data.end(), frame.begin(), frame.end());
    }
    // Idle heartbeat lets the recorder distinguish no requests from a dead
    // publisher (all optional objects can legitimately be disabled).
    if (message.data.size() > 3 || diagnostic_divider_ % 20 == 0) {
      bus_publisher_->publish(message);
    }
  }
}

void KaixuanCanSystem::publish_diagnostics()
{
  decltype(diagnostic_snapshots_) snapshots;
  {
    // This is the non-RT diagnostic executor. A single collision with the
    // cyclic exchange previously discarded an entire 200 ms publication.
    // Retry briefly without holding the mutex; never sleep in the PDO loop.
    std::unique_lock<std::mutex> lock(io_mutex_, std::defer_lock);
    for (unsigned attempt = 0; attempt < 4; ++attempt) {
      if (lock.try_lock()) {break;}
      if (attempt < 3) {
        ++diagnostic_snapshot_retries_;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    }
    if (!lock.owns_lock()) {++diagnostic_snapshot_drops_; return;}
    snapshots = diagnostic_snapshots_;
  }
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = diagnostic_node_->now();
  const auto now_ns = steady_now_ns();
  const auto add = [](auto & status, const char * key, auto value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = std::to_string(value);
      status.values.push_back(std::move(item));
    };
  for (std::size_t master = 0; master < kMasterCount; ++master) {
    if (!master_is_active(master)) {continue;}
    const auto & snapshot = snapshots[master];
    diagnostic_msgs::msg::DiagnosticStatus bus;
    bus.name = "ethercat/master" + std::to_string(master);
    bus.hardware_id = "master" + std::to_string(master);
    const bool stale = snapshot.sample_steady_ns == 0 ||
      now_ns - snapshot.sample_steady_ns > 200000000ULL;
    bus.level = stale ? 3 : (snapshot.wc_state == EC_WC_COMPLETE ? 0 : 2);
    bus.message = stale ? "cyclic feedback stale" :
      (bus.level == 0 ? "working counter complete" : "working counter incomplete");
    if (!stale && (snapshot.dc_follower_failed || snapshot.dc_reset_failed)) {
      bus.level = 2;
      bus.message = "DC clock qualification failed; drives disabled";
    }
    add(bus, "sample_steady_ns", snapshot.sample_steady_ns);
    add(bus, "diagnostic_snapshot_retries", diagnostic_snapshot_retries_);
    add(bus, "diagnostic_snapshot_drops", diagnostic_snapshot_drops_);
    add(bus, "cycle_count", snapshot.cycle_count);
    add(bus, "working_counter", snapshot.working_counter);
    add(bus, "wc_state", snapshot.wc_state);
    add(bus, "bad_wc_cycles", snapshot.bad_wc_cycles);
    add(bus, "capture_dropped_frames", cycle_captures_[master].dropped());
    add(bus, "max_cycle_lateness_ns", snapshot.max_cycle_lateness_ns);
    add(bus, "max_loop_interval_ns", snapshot.max_loop_interval_ns);
    add(bus, "bus_diagnostics_enabled", bus_diagnostics_);
    add(bus, "bus_probe_dropped_records", bus_captures_[master].dropped());
    add(bus, "max_bus_probe_work_ns", snapshot.max_probe_work_ns);
    add(bus, "startup_ready", snapshot.startup_ready);
    add(bus, "stop_cst_group_on_fault", stop_cst_group_on_fault_);
    add(bus, "dc_reset_after_configuration", dc_reset_after_configuration_);
    add(bus, "dc_reset_ready", snapshot.dc_reset_ready);
    add(bus, "dc_reset_failed", snapshot.dc_reset_failed);
    add(bus, "dc_reset_phase", snapshot.dc_reset_phase);
    add(bus, "dc_reset_error", snapshot.dc_reset_error);
    add(bus, "dc_reset_original_bandwidth", snapshot.dc_reset_original_bandwidth);
    add(bus, "dc_reset_original_filter", snapshot.dc_reset_original_filter);
    add(bus, "dc_reset_observation_start_ns", snapshot.dc_reset_observation_start_ns);
    add(bus, "dc_follow_reference_clock", follow_reference_clock_);
    add(bus, "dc_follower_proportional_gain", snapshot.dc_follower_config.proportional_gain);
    add(bus, "dc_follower_integral_gain", snapshot.dc_follower_config.integral_gain);
    add(bus, "dc_follower_filter_alpha", snapshot.dc_follower_config.filter_alpha);
    add(bus, "dc_follower_fine_entry_filtered_ns", snapshot.dc_follower_config.fine_entry_filtered_ns);
    add(bus, "dc_follower_bounded_handoff", snapshot.dc_follower_config.bumpless_fine_entry);
    add(bus, "dc_follower_acquisition_rate_feedforward", snapshot.dc_follower_config.acquisition_rate_feedforward);
    add(bus, "dc_follower_acquisition_rate_alpha", snapshot.dc_follower_config.acquisition_rate_alpha);
    add(bus, "dc_follower_ready", snapshot.dc_follower_ready);
    add(bus, "dc_follower_failed", snapshot.dc_follower_failed);
    add(bus, "dc_follower_locked", snapshot.dc_follower.locked);
    add(bus, "dc_follower_calibrated", snapshot.dc_follower.calibrated);
    add(bus, "dc_follower_acquiring_phase", snapshot.dc_follower.acquiring_phase);
    add(bus, "dc_follower_initial_residual_ns", snapshot.dc_follower.initial_residual_ns);
    add(bus, "dc_follower_acquisition_elapsed_ns", snapshot.dc_follower.acquisition_elapsed_ns);
    add(bus, "dc_follower_coarse_good_samples", snapshot.dc_follower.coarse_good_samples);
    add(bus, "dc_follower_baseline_restarts", snapshot.dc_follower.baseline_restarts);
    add(bus, "dc_follower_sample_accepted", snapshot.dc_follower.sample_accepted);
    add(bus, "dc_follower_residual_available", snapshot.dc_follower.residual_available);
    add(bus, "dc_follower_calibration_cycle", snapshot.follower_calibration_cycle);
    add(bus, "dc_follower_calibration_delta_ns", snapshot.follower_calibration_delta_ns);
    add(bus, "dc_follower_error", static_cast<unsigned int>(snapshot.dc_follower.failure));
    add(bus, "dc_follower_quality", static_cast<unsigned int>(snapshot.dc_follower.quality));
    add(bus, "dc_follower_base_ns", snapshot.dc_follower.base_ns);
    add(bus, "dc_follower_correction_ns", snapshot.dc_follower.next_correction_ns);
    add(bus, "dc_follower_measurement_ns", snapshot.dc_follower.measurement_error_ns);
    add(bus, "dc_follower_residual_ns", snapshot.dc_follower.residual_ns);
    add(bus, "dc_follower_filtered_ns", snapshot.dc_follower.filtered_error_ns);
    add(bus, "dc_follower_frequency_ns", snapshot.dc_follower.frequency_ns_per_cycle);
    add(bus, "dc_follower_sample_age_ns", snapshot.dc_follower.sample_age_ns);
    add(bus, "dc_follower_last_valid_age_ns", snapshot.dc_follower.last_valid_age_ns);
    add(bus, "dc_follower_disabled_ns", snapshot.dc_follower.disabled_observation_ns);
    add(bus, "dc_follower_good_samples", snapshot.dc_follower.consecutive_good_samples);
    add(bus, "dc_follower_accepted", snapshot.dc_follower.accepted_samples);
    add(bus, "dc_follower_rejected", snapshot.dc_follower.rejected_samples);
    add(bus, "dc_follower_matched_cycle", snapshot.reference_sample_cycle);
    add(bus, "dc_follower_ref32", snapshot.follower_last_ref32);
    add(bus, "dc_follower_ref_received_cycle", snapshot.follower_ref_received_cycle);
    add(bus, "dc_follower_getter_errors", snapshot.reference_getter_errors);
    add(bus, "dc_follower_getter_failed", snapshot.follower_getter_failed);
    add(bus, "dc_follower_getter_consecutive_errors", snapshot.follower_getter_consecutive_errors);
    add(bus, "dc_follower_schedule_failed", snapshot.follower_schedule_failed);
    add(bus, "dc_follower_skipped_periods", snapshot.follower_skipped_periods);
    add(bus, "dc_follower_failure_cycle", snapshot.follower_failure_cycle);
    add(bus, "dc_follower_failure_residual_ns", snapshot.follower_failure_residual_ns);
    add(bus, "dc_follower_last_reject_cycle", snapshot.follower_reject_cycle);
    add(bus, "dc_follower_last_reject_ns", snapshot.follower_reject_residual_ns);
    add(bus, "dc_follower_last_reject_reason", snapshot.follower_reject_reason);
    add(bus, "startup_bus_cycles", snapshot.startup_bus_cycles);
    add(bus, "startup_dc_samples", snapshot.startup_dc_samples);
    add(bus, "dc_difference_ns", snapshot.dc_difference_ns);
    add(bus, "dc_sample_queue_cycle", snapshot.dc_sample_queue_cycle);
    add(bus, "dc_sample_reference_age_cycles", snapshot.dc_sample_reference_age_cycles);
    add(bus, "fresh_reference_time", fresh_reference_time_);
    add(bus, "dc_reference_sync_cycles", reference_sync_cycles_);
    add(bus, "reference_sync_attempts", snapshot.reference_sync_attempts);
    add(bus, "reference_sync_queued", snapshot.reference_sync_queued);
    add(bus, "last_reference_sync_cycle", snapshot.last_reference_sync_cycle);
    add(bus, "timing_reference_sync_cycles", snapshot.timing_reference_sync_cycles);
    add(bus, "timing_window_start_ns", snapshot.timing_window_start_ns);
    add(bus, "timing_window_end_ns", snapshot.timing_window_end_ns);
    add(bus, "timing_window_cycles", snapshot.timing_window_cycles);
    add(bus, "slow_send_cycles_over100us", snapshot.slow_send_cycles);
    add(bus, "last_slow_send_ns", snapshot.last_slow_send_ns);
    add(bus, "sync_errors", snapshot.sync_errors);
    add(bus, "schedule_rebases", snapshot.schedule_rebases);
    static constexpr std::array<const char *, 9> timing_names = {
      "receive_process", "io_mutex_wait", "exchange", "wake_to_sync", "sync_call",
      "reference_age_at_send", "wake_to_send", "send_call", "wake_to_send_return"};
    for (std::size_t i = 0; i < timing_names.size(); ++i) {
      const std::string all_key = std::string("max_") + timing_names[i] + "_ns";
      const std::string window_key = std::string("window_max_") + timing_names[i] + "_ns";
      add(bus, all_key.c_str(), snapshot.timing_max_ns[i]);
      add(bus, window_key.c_str(), snapshot.timing_window_max_ns[i]);
    }
    message.status.push_back(bus);
    for (std::size_t axis = 0; axis < kJointsPerArm; ++axis) {
      const auto & data = snapshot.axes[axis];
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = info_.joints[master * kJointsPerArm + axis].name;
      status.hardware_id = bus.hardware_id + "/slave" + std::to_string(axis);
      status.level = stale ? 3 : ((data.error_code != 0 || (data.status_word & 8) || data.torque_guard_fault) ? 2 : bus.level);
      status.message = stale ? "feedback stale" : (status.level == 0 ? "PDO snapshot" : "drive/bus fault");
      const auto joint_index = master * kJointsPerArm + axis;
      const auto & limits = torque_parameters_[joint_index];
      const auto & trip = data.torque_fault_snapshot;
      const auto reason = static_cast<TorqueSafety::Fault>(data.torque_guard_fault);
      if (!stale && data.torque_guard_fault) {
        status.message = std::string("CST ") + TorqueSafety::fault_name(reason) +
          "; drives disabled; restart controller and send fresh zero to rearm";
      }
      if (!stale && logged_torque_fault_[joint_index] != data.torque_guard_fault) {
        logged_torque_fault_[joint_index] = data.torque_guard_fault;
        if (data.torque_guard_fault && trip.valid) {
          RCLCPP_ERROR(logger_,
            "CST guard %s fault=%u (%s): position=%.3fdeg, speed=%.3fdeg/s (limit %.3f), "
            "relative_displacement=%.3fdeg (limit %.3f); target torque=0, CW%u, stop latched",
            status.name.c_str(), data.torque_guard_fault, TorqueSafety::fault_name(reason),
            radians_to_degrees(trip.input.position), radians_to_degrees(trip.input.velocity),
            radians_to_degrees(limits.max_velocity_rad_s), radians_to_degrees(trip.displacement),
            radians_to_degrees(limits.max_displacement_rad), disabled_control_word());
        } else if (data.torque_guard_fault) {
          RCLCPP_ERROR(logger_, "CST guard %s fault=%u (%s); target torque=0, CW%u, stop latched",
            status.name.c_str(), data.torque_guard_fault, TorqueSafety::fault_name(reason), disabled_control_word());
        }
      }
      add(status, "sample_steady_ns", snapshot.sample_steady_ns);
      add(status, "actual_counts", data.actual_counts);
      add(status, "target_counts", data.target_counts);
      add(status, "velocity_raw", data.velocity_raw);
      add(status, "torque_raw", data.torque_raw);
      add(status, "effort_nm", data.effort_nm);
      add(status, "commanded_effort_nm", data.commanded_effort_nm);
      add(status, "target_torque_raw", data.target_torque_raw);
      add(status, "actual_mode", data.actual_mode);
      add(status, "effort_claimed", data.effort_claimed);
      add(status, "torque_armed", data.torque_armed);
      add(status, "torque_guard_fault", data.torque_guard_fault);
      add(status, "guard_fault_snapshot_valid", trip.valid);
      add(status, "guard_fault_position_rad", trip.input.position);
      add(status, "guard_fault_anchor_rad", trip.anchor);
      add(status, "guard_fault_displacement_rad", trip.displacement);
      add(status, "guard_fault_velocity_rad_s", trip.input.velocity);
      add(status, "guard_fault_command_nm", trip.valid ? trip.input.command : 0.0);
      add(status, "max_velocity_rad_s", limits.max_velocity_rad_s);
      add(status, "max_displacement_rad", limits.max_displacement_rad);
      add(status, "status_word", data.status_word);
      add(status, "control_word", data.control_word);
      add(status, "error_code", data.error_code);
      add(status, "last_nonzero_error_code", data.last_error_code);
      add(status, "fault_cycles", data.fault_cycles);
      add(status, "position_rad", data.position_rad);
      add(status, "velocity_rad_s", data.velocity_rad_s);
      message.status.push_back(std::move(status));
    }
  }
  diagnostic_publisher_->publish(message);
}

void KaixuanCanSystem::write_o6_frame(
  MasterContext & context, std::uint16_t control_word)
{
  auto & o6 = context.o6;
  auto * const output = context.domain_data + o6.rx_base;
  EC_WRITE_U16(output + 0, control_word);
  EC_WRITE_U16(output + 2, o6.command_sequence);
  EC_WRITE_U8(output + 4, 0x01);
  const auto request_mask = EC_READ_U8(output + 5);
  EC_WRITE_U8(output + 5, request_mask | kO6PressureRequestBit);
  EC_WRITE_U16(output + 6, 0x0000);
  for (std::size_t joint = 0; joint < kO6JointCount; ++joint)
  {
    EC_WRITE_U8(output + 8 + joint, o6.output_target[joint]);
    EC_WRITE_U8(output + 14 + joint, 0);
    EC_WRITE_U8(output + 20 + joint, 0);
    EC_WRITE_U8(output + 26 + joint, 0);
  }
}

void KaixuanCanSystem::reset_o6_handshake(
  MasterContext & context, bool keep_sequence)
{
  auto & o6 = context.o6;
  o6.handshake_state = O6HandshakeState::kDisabled;
  o6.disable_cycles = 0;
  o6.baseline_cycles = 0;
  o6.pending_generation = 0;
  if (!keep_sequence)
  {
    o6.command_sequence = 0;
    o6.sequence_initialized = false;
  }
}

void KaixuanCanSystem::exchange_o6_process_data(
  MasterContext & context, const ec_domain_state_t & domain_state,
  const ec_slave_config_state_t & slave_state, std::uint64_t now_ns)
{
  auto & o6 = context.o6;
  const auto * const input = context.domain_data + o6.tx_base;
  O6Snapshot snapshot{};
  snapshot.present = true;
  snapshot.operational = slave_state.operational != 0 && slave_state.al_state == 0x08;
  snapshot.working_counter_ok = domain_state.wc_state == EC_WC_COMPLETE;
  snapshot.working_counter = domain_state.working_counter;
  snapshot.wc_state = static_cast<std::uint32_t>(domain_state.wc_state);
  snapshot.al_state = static_cast<std::uint32_t>(slave_state.al_state);
  snapshot.cycle_count = context.cycle_count;
  snapshot.queue_count = context.queue_count;
  snapshot.send_count = context.send_count;
  snapshot.max_cycle_lateness_ns = context.max_cycle_lateness_ns;
  snapshot.max_loop_interval_ns = context.max_loop_interval_ns;
  snapshot.last_send_time_ns = context.last_send_time_ns;
  snapshot.last_complete_send_time_ns = context.last_complete_send_time_ns;
  snapshot.status_word = EC_READ_U16(input + 0);
  snapshot.applied_sequence = EC_READ_U16(input + 2);
  snapshot.position_sequence = EC_READ_U16(input + 4);
  snapshot.valid_mask = EC_READ_U8(input + 6);
  snapshot.active_can_id = EC_READ_U8(input + 7);
  for (std::size_t joint = 0; joint < kO6JointCount; ++joint)
  {
    snapshot.actual_position[joint] = EC_READ_U8(input + 8 + joint);
    snapshot.temperature[joint] = EC_READ_U8(input + 14 + joint);
    snapshot.joint_error[joint] = EC_READ_U8(input + 20 + joint);
  }
  snapshot.can_tx_count = EC_READ_U16(input + 26);
  snapshot.can_rx_count = EC_READ_U16(input + 28);
  snapshot.can_error_count = EC_READ_U8(input + 30);
  snapshot.timeout_count = EC_READ_U8(input + 31);
  snapshot.pressure_sequence = EC_READ_U16(input + 32);
  snapshot.finger_valid_mask = EC_READ_U8(input + 34);
  snapshot.pressure_state = EC_READ_U8(input + 35);
  for (std::size_t point = 0; point < kO6PressurePointCount; ++point)
  {
    snapshot.pressure[point] = EC_READ_U8(input + 36 + point);
  }
  snapshot.pressure_age_ms = EC_READ_U16(input + 236);
  snapshot.pressure_valid =
    snapshot.finger_valid_mask == kO6AllFingersValidMask &&
    snapshot.pressure_state == kO6PressureCompleteState &&
    snapshot.pressure_age_ms < kO6PressureMaxAgeMs;

  snapshot.imu_sequence = EC_READ_U16(input + 240);
  snapshot.imu_status = EC_READ_U16(input + 242);
  snapshot.accel_x_raw = EC_READ_S16(input + 244);
  snapshot.accel_y_raw = EC_READ_S16(input + 246);
  snapshot.accel_z_raw = EC_READ_S16(input + 248);
  snapshot.gyro_x_raw = EC_READ_S16(input + 250);
  snapshot.gyro_y_raw = EC_READ_S16(input + 252);
  snapshot.gyro_z_raw = EC_READ_S16(input + 254);
  snapshot.imu_temperature_centi_c = EC_READ_S16(input + 256);
  snapshot.imu_sample_age_ms = EC_READ_U16(input + 258);
  snapshot.imu_valid =
    (snapshot.imu_status & kO6ImuReadyMask) == kO6ImuReadyMask;
  snapshot.accel_x_mps2 = snapshot.accel_x_raw / kO6AccelLsbPerG * kStandardGravity;
  snapshot.accel_y_mps2 = snapshot.accel_y_raw / kO6AccelLsbPerG * kStandardGravity;
  snapshot.accel_z_mps2 = snapshot.accel_z_raw / kO6AccelLsbPerG * kStandardGravity;
  snapshot.gyro_x_rad_s = snapshot.gyro_x_raw / kO6GyroLsbPerDps * kPi / 180.0;
  snapshot.gyro_y_rad_s = snapshot.gyro_y_raw / kO6GyroLsbPerDps * kPi / 180.0;
  snapshot.gyro_z_rad_s = snapshot.gyro_z_raw / kO6GyroLsbPerDps * kPi / 180.0;
  snapshot.imu_temperature_c = snapshot.imu_temperature_centi_c / 100.0;

  const bool joint_errors_clear = std::all_of(
    snapshot.joint_error.begin(), snapshot.joint_error.end(),
    [](std::uint8_t error) {return error == 0;});
  snapshot.safe_inputs = snapshot.operational && snapshot.working_counter_ok &&
    (snapshot.status_word & 0x0001U) != 0 &&
    (snapshot.status_word & 0x0010U) == 0 &&
    (snapshot.valid_mask & 0x05U) == 0x05U && joint_errors_clear;

  O6Command command{};
  {
    std::lock_guard<std::mutex> lock(o6_mutex_);
    command = o6_commands_.at(context.master_id);
  }
  const bool command_fresh = command.update_time_ns != 0 &&
    (o6_command_timeout_ns_ == 0 ||
    (now_ns >= command.update_time_ns &&
    now_ns - command.update_time_ns <= static_cast<std::uint64_t>(o6_command_timeout_ns_)));
  const bool control_requested = command.enable_requested && command_fresh;

  if (o6_imu_only_)
  {
    reset_o6_handshake(context);
    write_o6_frame(context, 0x0002);
  }
  else if (!control_requested)
  {
    reset_o6_handshake(context);
    if ((snapshot.valid_mask & 0x01U) != 0)
    {
      o6.output_target = snapshot.actual_position;
    }
    write_o6_frame(context, 0);
  }
  else if (!snapshot.safe_inputs)
  {
    // Freeze on transient feedback faults without dropping an explicit enable.
    write_o6_frame(
      context,
      o6.handshake_state == O6HandshakeState::kDisabled ? 0 : 1);
  }
  else
  {
    switch (o6.handshake_state)
    {
      case O6HandshakeState::kDisabled:
        o6.output_target = snapshot.actual_position;
        if (!o6.sequence_initialized)
        {
          o6.command_sequence = snapshot.applied_sequence;
          o6.sequence_initialized = true;
        }
        write_o6_frame(context, 0);
        if (++o6.disable_cycles >= kO6DisableCycles)
        {
          o6.baseline_target = snapshot.actual_position;
          o6.output_target = o6.baseline_target;
          o6.baseline_cycles = 0;
          o6.handshake_state = O6HandshakeState::kEnableBaseline;
        }
        break;

      case O6HandshakeState::kEnableBaseline:
        o6.output_target = o6.baseline_target;
        write_o6_frame(context, 1);
        if (++o6.baseline_cycles >= kO6EnableBaselineCycles)
        {
          ++o6.command_sequence;
          o6.handshake_state = O6HandshakeState::kWaitBaselineAck;
        }
        break;

      case O6HandshakeState::kWaitBaselineAck:
        o6.output_target = o6.baseline_target;
        write_o6_frame(context, 1);
        if (snapshot.applied_sequence == o6.command_sequence)
        {
          o6.sent_generation = command.generation;
          o6.handshake_state = O6HandshakeState::kReady;
        }
        break;

      case O6HandshakeState::kReady:
        if (command.generation != o6.sent_generation)
        {
          o6.pending_target = command.target;
          o6.pending_generation = command.generation;
          o6.output_target = o6.pending_target;
          ++o6.command_sequence;
          o6.handshake_state = O6HandshakeState::kWaitCommandAck;
        }
        write_o6_frame(context, 1);
        break;

      case O6HandshakeState::kWaitCommandAck:
        o6.output_target = o6.pending_target;
        write_o6_frame(context, 1);
        if (snapshot.applied_sequence == o6.command_sequence)
        {
          o6.sent_generation = o6.pending_generation;
          o6.handshake_state = O6HandshakeState::kReady;
        }
        break;
    }
  }

  snapshot.command_sequence = o6.command_sequence;
  snapshot.handshake_state = static_cast<std::uint8_t>(o6.handshake_state);
  snapshot.ready = o6_imu_only_ ?
    snapshot.operational && snapshot.working_counter_ok && snapshot.imu_valid :
    o6.handshake_state == O6HandshakeState::kReady && snapshot.safe_inputs;
  {
    std::lock_guard<std::mutex> lock(o6_mutex_);
    o6_snapshots_.at(context.master_id) = snapshot;
  }
}

void KaixuanCanSystem::send_o6_disable_cycles(
  MasterContext & context, timespec & wake_time)
{
  reset_o6_handshake(context);
  for (std::uint64_t cycle = 0; cycle < kO6DisableCycles; ++cycle)
  {
    add_ns(wake_time, cycle_time_ns_);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, nullptr);
    ecrt_master_application_time(context.master, timespec_to_ns(wake_time));
    ecrt_master_receive(context.master);
    context.final_receive_ns = steady_now_ns();
    ecrt_domain_process(context.domain);
    write_o6_frame(context, o6_imu_only_ ? 0x0002 : 0x0000);
    ecrt_master_sync_reference_clock(context.master);
    ecrt_master_sync_slave_clocks(context.master);
    ecrt_domain_queue(context.domain);
    const auto send_start = steady_now_ns();
    context.previous_send_result = ecrt_master_send(context.master);
    context.previous_send_duration_ns = steady_now_ns() - send_start;
    context.last_send_time_ns = steady_now_ns();
  }
}

const char * KaixuanCanSystem::o6_namespace(std::size_t master_index)
{
  // Preserve the established right-hand topic names for existing tools.
  return master_index == 0 ? "/linkerhand_o6" : "/linkerhand_o6_left";
}

const char * KaixuanCanSystem::o6_hand_name(std::size_t master_index)
{
  return master_index == 0 ? "right" : "left";
}

bool KaixuanCanSystem::start_o6_ros_interfaces()
{
  try
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    for (std::size_t master_index = 0; master_index < kMasterCount; ++master_index)
    {
      if (!masters_[master_index].o6.configured)
      {
        continue;
      }
      auto & interface = o6_ros_interfaces_[master_index];
      interface.node = std::make_shared<rclcpp::Node>(
        std::string("gateway_") + o6_hand_name(master_index),
        o6_namespace(master_index));
      interface.target_subscription =
        interface.node->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "target_position", qos,
        [this, master_index](const std_msgs::msg::UInt8MultiArray::SharedPtr message)
        {receive_o6_target(master_index, message);});
      interface.enable_service = interface.node->create_service<std_srvs::srv::SetBool>(
        "enable",
        [this, master_index](
          const std_srvs::srv::SetBool::Request::SharedPtr request,
          std_srvs::srv::SetBool::Response::SharedPtr response)
        {set_o6_enabled(master_index, request, response);});
      interface.actual_publisher = interface.node->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "actual_position", qos);
      interface.temperature_publisher = interface.node->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "temperature", qos);
      interface.error_publisher = interface.node->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "joint_error", qos);
      interface.pressure_publisher = interface.node->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "pressure", qos);
      interface.status_publisher = interface.node->create_publisher<std_msgs::msg::UInt16MultiArray>(
        "status", qos);
      interface.pressure_status_publisher = interface.node->create_publisher<std_msgs::msg::UInt16MultiArray>(
        "pressure_status", qos);
      interface.imu_raw_publisher = interface.node->create_publisher<std_msgs::msg::Int16MultiArray>(
        "imu_raw", qos);
      interface.imu_si_publisher = interface.node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "imu_si", qos);
      interface.imu_status_publisher = interface.node->create_publisher<std_msgs::msg::UInt16MultiArray>(
        "imu_status", qos);
      interface.realtime_publisher = interface.node->create_publisher<std_msgs::msg::UInt64MultiArray>(
        "realtime_stats", qos);
      interface.online_publisher = interface.node->create_publisher<std_msgs::msg::Bool>("online", qos);
      interface.ready_publisher = interface.node->create_publisher<std_msgs::msg::Bool>("ready", qos);
      interface.pressure_valid_publisher = interface.node->create_publisher<std_msgs::msg::Bool>(
        "pressure_valid", qos);
      interface.publish_timer = interface.node->create_wall_timer(
        std::chrono::milliseconds(10),
        [this, master_index]() {publish_o6_status(master_index);});
      interface.executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      interface.executor->add_node(interface.node);
      interface.thread = std::thread(
        [&interface]() {interface.executor->spin();});
      RCLCPP_INFO(
        logger_, "%s O6 ROS interface ready under %s (disabled by default)",
        o6_hand_name(master_index), o6_namespace(master_index));
    }
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(logger_, "Cannot create O6 ROS interfaces: %s", exception.what());
    stop_o6_ros_interfaces();
    return false;
  }
  return true;
}

void KaixuanCanSystem::stop_o6_ros_interfaces()
{
  for (auto & interface : o6_ros_interfaces_)
  {
    if (interface.executor)
    {
      interface.executor->cancel();
    }
    if (interface.thread.joinable())
    {
      interface.thread.join();
    }
    if (interface.executor && interface.node)
    {
      interface.executor->remove_node(interface.node);
    }
    interface = O6RosInterface{};
  }
}

void KaixuanCanSystem::publish_o6_status(std::size_t master_index)
{
  O6Snapshot snapshot{};
  {
    std::lock_guard<std::mutex> lock(o6_mutex_);
    snapshot = o6_snapshots_.at(master_index);
  }
  auto & interface = o6_ros_interfaces_.at(master_index);

  std_msgs::msg::UInt8MultiArray bytes;
  bytes.data.assign(snapshot.actual_position.begin(), snapshot.actual_position.end());
  interface.actual_publisher->publish(bytes);
  bytes.data.assign(snapshot.temperature.begin(), snapshot.temperature.end());
  interface.temperature_publisher->publish(bytes);
  bytes.data.assign(snapshot.joint_error.begin(), snapshot.joint_error.end());
  interface.error_publisher->publish(bytes);
  bytes.data.assign(snapshot.pressure.begin(), snapshot.pressure.end());
  interface.pressure_publisher->publish(bytes);

  std_msgs::msg::UInt16MultiArray status;
  status.data = {
    snapshot.status_word, snapshot.applied_sequence, snapshot.position_sequence,
    snapshot.command_sequence, snapshot.valid_mask, snapshot.active_can_id,
    snapshot.can_tx_count, snapshot.can_rx_count, snapshot.can_error_count,
    snapshot.timeout_count, snapshot.handshake_state};
  interface.status_publisher->publish(status);

  std_msgs::msg::UInt16MultiArray pressure_status;
  pressure_status.data = {
    snapshot.pressure_sequence, snapshot.finger_valid_mask,
    snapshot.pressure_state, snapshot.pressure_age_ms};
  interface.pressure_status_publisher->publish(pressure_status);

  std_msgs::msg::Int16MultiArray imu_raw;
  imu_raw.data = {
    snapshot.accel_x_raw, snapshot.accel_y_raw, snapshot.accel_z_raw,
    snapshot.gyro_x_raw, snapshot.gyro_y_raw, snapshot.gyro_z_raw,
    snapshot.imu_temperature_centi_c};
  interface.imu_raw_publisher->publish(imu_raw);

  std_msgs::msg::Float64MultiArray imu_si;
  imu_si.data = {
    snapshot.accel_x_mps2, snapshot.accel_y_mps2, snapshot.accel_z_mps2,
    snapshot.gyro_x_rad_s, snapshot.gyro_y_rad_s, snapshot.gyro_z_rad_s,
    snapshot.imu_temperature_c};
  interface.imu_si_publisher->publish(imu_si);

  std_msgs::msg::UInt16MultiArray imu_status;
  imu_status.data = {
    snapshot.imu_sequence, snapshot.imu_status, snapshot.imu_sample_age_ms};
  interface.imu_status_publisher->publish(imu_status);

  std_msgs::msg::UInt64MultiArray realtime;
  realtime.data = {
    snapshot.cycle_count, snapshot.queue_count, snapshot.send_count,
    snapshot.working_counter, snapshot.wc_state, snapshot.al_state,
    snapshot.max_cycle_lateness_ns, snapshot.max_loop_interval_ns,
    snapshot.last_send_time_ns, snapshot.last_complete_send_time_ns};
  interface.realtime_publisher->publish(realtime);

  std_msgs::msg::Bool flag;
  flag.data = snapshot.operational && snapshot.working_counter_ok &&
    (snapshot.status_word & 0x0001U) != 0;
  interface.online_publisher->publish(flag);
  flag.data = snapshot.ready;
  interface.ready_publisher->publish(flag);
  flag.data = snapshot.pressure_valid;
  interface.pressure_valid_publisher->publish(flag);
}

void KaixuanCanSystem::receive_o6_target(
  std::size_t master_index,
  const std_msgs::msg::UInt8MultiArray::SharedPtr message)
{
  if (message->data.size() != kO6JointCount)
  {
    RCLCPP_ERROR(
      logger_, "Rejected %s O6 target: expected 6 uint8 values, got %zu",
      o6_hand_name(master_index), message->data.size());
    return;
  }

  std::lock_guard<std::mutex> lock(o6_mutex_);
  auto & command = o6_commands_.at(master_index);
  const auto & snapshot = o6_snapshots_.at(master_index);
  if (!command.enable_requested || !snapshot.ready || !snapshot.safe_inputs)
  {
    RCLCPP_WARN(
      logger_, "Rejected %s O6 target: explicitly enable it and wait for ready=true",
      o6_hand_name(master_index));
    return;
  }
  for (std::size_t joint = 0; joint < kO6JointCount; ++joint)
  {
    const auto difference = std::abs(
      static_cast<int>(message->data[joint]) -
      static_cast<int>(snapshot.actual_position[joint]));
    if (difference > o6_max_step_)
    {
      RCLCPP_ERROR(
        logger_, "Rejected O6 joint%zu target %u: actual=%u exceeds max step=%u",
        joint, message->data[joint], snapshot.actual_position[joint], o6_max_step_);
      return;
    }
  }
  const bool changed = !std::equal(
    message->data.begin(), message->data.end(), command.target.begin());
  if (changed)
  {
    std::copy(message->data.begin(), message->data.end(), command.target.begin());
    ++command.generation;
  }
  command.update_time_ns = steady_now_ns();
}

void KaixuanCanSystem::set_o6_enabled(
  std::size_t master_index,
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(o6_mutex_);
  auto & command = o6_commands_.at(master_index);
  const auto & snapshot = o6_snapshots_.at(master_index);
  if (!request->data)
  {
    command.enable_requested = false;
    command.update_time_ns = steady_now_ns();
    response->success = true;
    response->message = "O6 disabled; realtime loop is sending ControlWord=0";
    return;
  }
  if (!snapshot.safe_inputs)
  {
    response->success = false;
    response->message = "O6 safety inputs are not valid (OP/WC/online/valid/error)";
    return;
  }
  command.target = snapshot.actual_position;
  ++command.generation;
  command.update_time_ns = steady_now_ns();
  command.enable_requested = true;
  response->success = true;
  response->message = "O6 handshake requested at its current position";
}

bool KaixuanCanSystem::configure_realtime_thread(
  const MasterContext & context)
{
  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  CPU_SET(context.affinity, &cpu_set);
  int result = pthread_setaffinity_np(
    pthread_self(), sizeof(cpu_set), &cpu_set);
  if (result != 0)
  {
    RCLCPP_ERROR(
      logger_, "Master%u cannot bind to CPU%d: %s",
      context.master_id, context.affinity, std::strerror(result));
    return false;
  }

  sched_param parameters{};
  parameters.sched_priority = priority_;
  result = pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameters);
  if (result != 0)
  {
    RCLCPP_ERROR(
      logger_, "Master%u cannot set SCHED_FIFO/%d: %s. "
      "Log in again after joining the realtime group.",
      context.master_id, priority_, std::strerror(result));
    return false;
  }
  return true;
}

KaixuanCanSystem::Cia402State KaixuanCanSystem::decode_cia402_state(
  std::uint16_t status_word)
{
  const auto masked_6f = static_cast<std::uint16_t>(status_word & 0x006F);
  const auto masked_4f = static_cast<std::uint16_t>(status_word & 0x004F);
  if (masked_4f == 0x0040) {return Cia402State::kSwitchOnDisabled;}
  if (masked_6f == 0x0021) {return Cia402State::kReadyToSwitchOn;}
  if (masked_6f == 0x0023) {return Cia402State::kSwitchedOn;}
  if (masked_6f == 0x0027) {return Cia402State::kOperationEnabled;}
  if (masked_6f == 0x0007) {return Cia402State::kQuickStopActive;}
  if (masked_4f == 0x000F) {return Cia402State::kFaultReactionActive;}
  if (masked_4f == 0x0008) {return Cia402State::kFault;}
  if (masked_4f == 0x0000) {return Cia402State::kNotReadyToSwitchOn;}
  return Cia402State::kUnknown;
}

double KaixuanCanSystem::degrees_to_radians(double degrees)
{
  return degrees * kPi / 180.0;
}

double KaixuanCanSystem::radians_to_degrees(double radians)
{
  return radians * 180.0 / kPi;
}

std::uint64_t KaixuanCanSystem::timespec_to_ns(const timespec & value)
{
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
}

std::uint64_t KaixuanCanSystem::steady_now_ns()
{
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return timespec_to_ns(now);
}

void KaixuanCanSystem::add_ns(timespec & value, std::int64_t nanoseconds)
{
  value.tv_nsec += nanoseconds;
  while (value.tv_nsec >= 1000000000L)
  {
    value.tv_nsec -= 1000000000L;
    ++value.tv_sec;
  }
}

double KaixuanCanSystem::motor_rpm_to_joint_radians_per_second(
  std::int32_t motor_rpm, std::size_t joint_index) const
{
  const auto motor_turns_per_second = static_cast<double>(motor_rpm) / 60.0;
  return joint_direction_[joint_index] * motor_turns_per_second * 2.0 * kPi /
         joint_gear_ratio_[joint_index];
}

double KaixuanCanSystem::joint_counts_to_radians(
  std::int32_t counts, std::size_t joint_index) const
{
  return JointAngleMapping{joint_gear_ratio_[joint_index], joint_offset_deg_[joint_index],
    joint_direction_[joint_index], torque_parameters_[joint_index].normalize_position}.position(counts);
}

std::int32_t KaixuanCanSystem::radians_to_joint_counts(
  double radians, std::size_t joint_index) const
{
  return JointAngleMapping{joint_gear_ratio_[joint_index], joint_offset_deg_[joint_index],
    joint_direction_[joint_index], torque_parameters_[joint_index].normalize_position}.
    target_counts(radians, actual_counts_[joint_index]);
}

void KaixuanCanSystem::initialize_runtime()
{
  if (runtime_initialized_)
  {
    return;
  }
  if (mlockall(MCL_FUTURE | MCL_CURRENT) == -1)
  {
    RCLCPP_WARN(logger_, "mlockall failed: %s", std::strerror(errno));
  }
  runtime_initialized_ = true;
}

bool KaixuanCanSystem::all_joint_positions_ready() const
{
  for (std::size_t joint = 0; joint < kNumJoints; ++joint)
  {
    if (joint_is_active(joint) && !position_initialized_[joint])
    {
      return false;
    }
  }
  return true;
}

bool KaixuanCanSystem::master_is_active(std::size_t master_index) const
{
  return active_arm_ == "both" ||
         (active_arm_ == "right" && master_index == 0) ||
         (active_arm_ == "left" && master_index == 1);
}

bool KaixuanCanSystem::joint_is_active(std::size_t joint_index) const
{
  return master_is_active(joint_index / kJointsPerArm);
}

void KaixuanCanSystem::reset_runtime_state()
{
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    std::fill(hw_efforts_.begin(), hw_efforts_.end(), 0.0);
    std::fill(hw_commands_.begin(), hw_commands_.end(), cst_mode_ ?
      std::numeric_limits<double>::quiet_NaN() : 0.0);
    requested_efforts_.fill(std::numeric_limits<double>::quiet_NaN());
    measured_efforts_.fill(0.0);
    measured_positions_.fill(0.0);
    measured_velocities_.fill(0.0);
    effort_claimed_.fill(false);
    effort_write_ns_ = 0;
    for (auto & guard : torque_safety_) {guard.reset();}
    std::fill(commanded_counts_.begin(), commanded_counts_.end(), 0);
    actual_counts_.fill(0);
    std::fill(
      position_initialized_.begin(), position_initialized_.end(), false);
  }
  {
    std::lock_guard<std::mutex> lock(o6_mutex_);
    o6_commands_ = {};
    o6_snapshots_ = {};
  }
}

}  // namespace kaixuan_can_hardware

PLUGINLIB_EXPORT_CLASS(
  kaixuan_can_hardware::KaixuanCanSystem,
  hardware_interface::SystemInterface)
