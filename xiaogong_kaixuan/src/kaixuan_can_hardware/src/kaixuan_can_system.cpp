#include "kaixuan_can_hardware/kaixuan_can_system.hpp"

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
  hw_commands_.assign(kNumJoints, 0.0);
  joint_offset_deg_.assign(kNumJoints, 0.0);
  joint_direction_.assign(kNumJoints, 1.0);
  joint_gear_ratio_.assign(kNumJoints, 1.0);
  commanded_counts_.assign(kNumJoints, 0);
  position_initialized_.assign(kNumJoints, false);

  try
  {
    if (info_.hardware_parameters.count("active_arm"))
    {
      active_arm_ = info_.hardware_parameters.at("active_arm");
    }
    if (info_.hardware_parameters.count("bus_diagnostics")) {
      const auto & value = info_.hardware_parameters.at("bus_diagnostics");
      bus_diagnostics_ = value == "true" || value == "True" || value == "1";
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
    "shift=%ldns priority=%d o6_imu_only=%s",
    active_arm_.c_str(),
    masters_[0].affinity, masters_[1].affinity,
    static_cast<long>(cycle_time_ns_), static_cast<long>(shift_time_ns_),
    priority_, o6_imu_only_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KaixuanCanSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  stop_diagnostics();
  stop_o6_ros_interfaces();
  close_ethercat();
  diagnostic_snapshots_ = {};
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
      std::lock_guard<std::mutex> lock(io_mutex_);
      if (all_joint_positions_ready())
      {
        activated_ = true;
        hw_commands_ = hw_positions_;
        for (std::size_t joint = 0; joint < kNumJoints; ++joint)
        {
          if (!joint_is_active(joint))
          {
            continue;
          }
          commanded_counts_[joint] =
            radians_to_joint_counts(hw_commands_[joint], joint);
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
    logger_, "Timed out waiting for EtherCAT feedback. Missing: %s",
    not_ready.c_str());
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn KaixuanCanSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  deactivate_requested_ns_ = steady_now_ns();
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
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
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KaixuanCanSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(kNumJoints * 2);
  for (std::size_t i = 0; i < kNumJoints; ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
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
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type KaixuanCanSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(io_mutex_);
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
  }
  {
    std::lock_guard<std::mutex> lock(o6_mutex_);
    for (auto & command : o6_commands_)
    {
      command.enable_requested = false;
      command.update_time_ns = steady_now_ns();
    }
  }
  for (auto & context : masters_)
  {
    context.running.store(false);
  }
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
  // ROS/recorder may already have stopped. Only now, after both masters have
  // been released, serialize/fsync the shutdown evidence; never delay stopping
  // the cyclic threads on disk I/O. No new stop command or receive is added.
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

  ec_master_info_t master_info{};
  if (ecrt_master(context.master, &master_info) != 0)
  {
    RCLCPP_ERROR(logger_, "Failed to query Master%u", context.master_id);
    return false;
  }
  const bool has_o6 = master_info.slave_count == kMaxSlavesPerMaster;
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
  if (ecrt_slave_config_pdos(slave_config, EC_END, kSyncManagers) != 0)
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

  auto & axis = context.axes[slave_pos];
  axis = AxisHandle{};
  axis.joint_index = joint_index;
  axis.slave_pos = slave_pos;

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
    register_entry(axis.error_code, 0x603F, 0);
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

    ecrt_master_application_time(context.master, scheduled_time);
    ecrt_master_receive(context.master);
    context.final_receive_ns = steady_now_ns();
    ecrt_domain_process(context.domain);
    ec_domain_state_t domain_state{};
    ecrt_domain_state(context.domain, &domain_state);
    exchange_process_data(context, domain_state);
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
    ecrt_master_sync_reference_clock(context.master);
    ecrt_master_sync_slave_clocks(context.master);
    ecrt_domain_queue(context.domain);
    ++context.queue_count;
    const auto send_start_ns = steady_now_ns();
    context.previous_send_result = ecrt_master_send(context.master);
    context.previous_send_duration_ns = steady_now_ns() - send_start_ns;
    ++context.send_count;
    context.last_send_time_ns = steady_now_ns();
    if (domain_state.wc_state == EC_WC_COMPLETE)
    {
      context.last_complete_send_time_ns = context.last_send_time_ns;
    }

    add_ns(wake_time, cycle_time_ns_);
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (timespec_to_ns(now) >
      timespec_to_ns(wake_time) + static_cast<std::uint64_t>(cycle_time_ns_))
    {
      wake_time = now;
      add_ns(wake_time, cycle_time_ns_);
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
  std::lock_guard<std::mutex> lock(io_mutex_);
  auto & snapshot = diagnostic_snapshots_[context.master_id];
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
  for (const auto & axis : context.axes)
  {
    const auto joint = axis.joint_index;
    const auto * data = context.domain_data;
    const auto raw_counts = EC_READ_S32(data + axis.position_actual_value);
    const auto raw_velocity = EC_READ_S32(data + axis.velocity_actual_value);
    const auto status_word = EC_READ_U16(data + axis.status_word);

    const bool feedback_ready = status_word != 0 || raw_counts != 0;
    if (feedback_ready)
    {
      const auto actual_position = joint_counts_to_radians(raw_counts, joint);
      hw_positions_[joint] = actual_position;
      hw_velocities_[joint] =
        motor_rpm_to_joint_radians_per_second(raw_velocity, joint);
      if (!position_initialized_[joint])
      {
        position_initialized_[joint] = true;
        hw_commands_[joint] = actual_position;
        commanded_counts_[joint] = raw_counts;
      }
    }

    EC_WRITE_S8(data + axis.mode_of_operation, 8);
    if (position_initialized_[joint])
    {
      EC_WRITE_S32(data + axis.target_position, commanded_counts_[joint]);
    }
    std::uint16_t control_word = EC_READ_U16(data + axis.control_word);
    switch (decode_cia402_state(status_word))
    {
      case Cia402State::kNotReadyToSwitchOn:
      case Cia402State::kSwitchOnDisabled:
        control_word = 0x0006;
        break;
      case Cia402State::kReadyToSwitchOn:
        control_word = 0x0007;
        break;
      case Cia402State::kSwitchedOn:
      case Cia402State::kOperationEnabled:
        control_word = 0x000F;
        break;
      case Cia402State::kFault:
        control_word = 0x0080;
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
    diagnostic.status_word = status_word;
    diagnostic.control_word = control_word;
    diagnostic.error_code = EC_READ_U16(data + axis.error_code);
    if (diagnostic.error_code != 0) {
      diagnostic.last_error_code = diagnostic.error_code;
    }
    diagnostic.fault_cycles += (status_word & 0x0008U) != 0;
    diagnostic.position_rad = hw_positions_[joint];
    diagnostic.velocity_rad_s = hw_velocities_[joint];
    diagnostic.following_error = domain_state.wc_state == EC_WC_COMPLETE &&
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
    std::unique_lock<std::mutex> lock(io_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {return;}
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
    add(bus, "sample_steady_ns", snapshot.sample_steady_ns);
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
    message.status.push_back(bus);
    for (std::size_t axis = 0; axis < kJointsPerArm; ++axis) {
      const auto & data = snapshot.axes[axis];
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = info_.joints[master * kJointsPerArm + axis].name;
      status.hardware_id = bus.hardware_id + "/slave" + std::to_string(axis);
      status.level = stale ? 3 : ((data.error_code != 0 || (data.status_word & 8)) ? 2 : bus.level);
      status.message = stale ? "feedback stale" : (status.level == 0 ? "PDO snapshot" : "drive/bus fault");
      add(status, "sample_steady_ns", snapshot.sample_steady_ns);
      add(status, "actual_counts", data.actual_counts);
      add(status, "target_counts", data.target_counts);
      add(status, "velocity_raw", data.velocity_raw);
      add(status, "torque_raw", data.torque_raw);
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
  const auto turns =
    static_cast<double>(counts) /
    (static_cast<double>(kMotorCountsPerRevolution) * joint_gear_ratio_[joint_index]);
  const auto degrees =
    joint_direction_[joint_index] *
    (turns * 360.0 - joint_offset_deg_[joint_index]);
  return degrees_to_radians(degrees);
}

std::int32_t KaixuanCanSystem::radians_to_joint_counts(
  double radians, std::size_t joint_index) const
{
  const auto degrees =
    joint_direction_[joint_index] * radians_to_degrees(radians) +
    joint_offset_deg_[joint_index];
  const auto counts =
    degrees * static_cast<double>(
    kMotorCountsPerRevolution) * joint_gear_ratio_[joint_index] / 360.0;
  return static_cast<std::int32_t>(std::llround(counts));
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
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(commanded_counts_.begin(), commanded_counts_.end(), 0);
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
