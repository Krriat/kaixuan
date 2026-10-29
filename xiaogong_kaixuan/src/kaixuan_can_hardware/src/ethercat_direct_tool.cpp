#include <ecrt.h>

#include <pthread.h>
#include <poll.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{
constexpr std::size_t kAxisCount = 7;
constexpr std::uint32_t kVendorId = 0x00010203;
constexpr std::uint32_t kProductCode = 0x00000402;
constexpr std::uint32_t kRevision = 0x00050012;
constexpr std::uint16_t kDcAssignActivate = 0x0300;
constexpr std::int64_t kDefaultCycleNs = 1000000;
constexpr int kDefaultPriority = 90;
constexpr char kStateDirectory[] = "/tmp/ethercat_op_csp";

volatile std::sig_atomic_t g_stop_requested = 0;

// KGU CSP mapping required by the vendor specification.
const ec_pdo_entry_info_t kRxPdoEntries[] = {
  {0x6040, 0x00, 16},
  {0x607A, 0x00, 32},
  {0x6060, 0x00, 8},
};

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

struct Options
{
  unsigned int master_id{std::numeric_limits<unsigned int>::max()};
  unsigned int print_every{1000};
  std::int64_t cycle_ns{kDefaultCycleNs};
  int cpu{-1};
  int priority{kDefaultPriority};
  bool hold_csp_all{false};
  int hold_csp_mask{-1};
  int hold_csp_slave{-1};
  bool hold_op_all{false};
  int hold_op_slave{-1};
  bool validate_only{false};
  int move_slave{-1};
  double move_degrees{0.0};
  std::uint32_t velocity{50000};
  std::uint32_t acceleration{100000};
  std::uint32_t deceleration{100000};
};

struct Axis
{
  int control_word{-1};
  int status_word{-1};
  int mode{-1};
  int target_position{-1};
  int actual_position{-1};
  int actual_velocity{-1};
  int torque_actual{-1};
  int error_code{-1};
  std::int32_t command{0};
  double command_position{0.0};
  std::int32_t goal_position{0};
  double command_velocity{0.0};
  std::int32_t actual{0};
  std::int32_t velocity_actual{0};
  std::uint16_t status{0};
  std::int8_t mode_actual{0};
  bool initialized{false};
  bool configured{false};
};

void signal_handler(int)
{
  g_stop_requested = 1;
}

std::uint64_t timespec_to_ns(const timespec & value)
{
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
}

void add_ns(timespec & value, std::int64_t nanoseconds)
{
  value.tv_nsec += nanoseconds;
  while (value.tv_nsec >= 1000000000L)
  {
    value.tv_nsec -= 1000000000L;
    ++value.tv_sec;
  }
}

std::uint32_t counts_per_revolution(unsigned int slave)
{
  static constexpr std::array<std::uint32_t, kAxisCount> kGearRatios{
    101U, 101U, 81U, 81U, 101U, 101U, 101U};
  return 1048576U * kGearRatios.at(slave);
}

double joint_direction(unsigned int master_id, unsigned int slave)
{
  static constexpr std::array<double, 7> kRightDirections{
    -1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0};
  return master_id == 0 ? kRightDirections.at(slave) : 1.0;
}

std::string stop_file(unsigned int master_id)
{
  return std::string(kStateDirectory) + "/master" +
         std::to_string(master_id) + ".stop";
}

std::string move_command_file(unsigned int master_id)
{
  return std::string(kStateDirectory) + "/master" +
         std::to_string(master_id) + ".move";
}

long parse_long(const std::string & text, const char * name)
{
  std::size_t consumed = 0;
  long value = 0;
  try
  {
    value = std::stol(text, &consumed, 0);
  }
  catch (const std::exception &)
  {
    throw std::runtime_error(std::string("invalid ") + name + ": " + text);
  }
  if (consumed != text.size())
  {
    throw std::runtime_error(std::string("invalid ") + name + ": " + text);
  }
  return value;
}

double parse_double(const std::string & text, const char * name)
{
  std::size_t consumed = 0;
  double value = 0.0;
  try
  {
    value = std::stod(text, &consumed);
  }
  catch (const std::exception &)
  {
    throw std::runtime_error(std::string("invalid ") + name + ": " + text);
  }
  if (consumed != text.size() || !std::isfinite(value))
  {
    throw std::runtime_error(std::string("invalid ") + name + ": " + text);
  }
  return value;
}

void print_usage(const char * executable)
{
  std::cerr
    << "Usage:\n  " << executable
    << " --master_id 0|1 --validate_only\n  " << executable
    << " --master_id 0|1 --hold_csp_all [--print_every N]\n  "
    << executable
    << " --master_id 0|1 --hold_csp_mask 1..127 [--print_every N]\n  "
    << executable
    << " --master_id 0|1 --hold_csp_slave 0..6 [--print_every N]\n  "
    << executable
    << " --master_id 0|1 --hold_op_all [--print_every N]\n  "
    << executable
    << " --master_id 0|1 --hold_op_slave 0..6 [--print_every N]\n  "
    << executable
    << " --master_id 0|1 --move_slave 0..6 --move_degrees DEG"
    << " [--pp_velocity N --pp_acceleration N --pp_deceleration N]\n";
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index)
  {
    const std::string argument = argv[index];
    auto next_value = [&]() -> std::string
      {
        if (++index >= argc)
        {
          throw std::runtime_error("missing value after " + argument);
        }
        return argv[index];
      };

    if (argument == "--master_id")
    {
      options.master_id = static_cast<unsigned int>(
        parse_long(next_value(), "master_id"));
    }
    else if (argument == "--print_every")
    {
      options.print_every = static_cast<unsigned int>(
        parse_long(next_value(), "print_every"));
    }
    else if (argument == "--cycle_ns")
    {
      options.cycle_ns = parse_long(next_value(), "cycle_ns");
    }
    else if (argument == "--cpu")
    {
      options.cpu = static_cast<int>(parse_long(next_value(), "cpu"));
    }
    else if (argument == "--priority")
    {
      options.priority = static_cast<int>(parse_long(next_value(), "priority"));
    }
    else if (argument == "--hold_csp_all")
    {
      options.hold_csp_all = true;
    }
    else if (argument == "--hold_csp_mask")
    {
      options.hold_csp_mask = static_cast<int>(
        parse_long(next_value(), "hold_csp_mask"));
    }
    else if (argument == "--hold_csp_slave")
    {
      options.hold_csp_slave = static_cast<int>(
        parse_long(next_value(), "hold_csp_slave"));
    }
    else if (argument == "--hold_op_all")
    {
      options.hold_op_all = true;
    }
    else if (argument == "--hold_op_slave")
    {
      options.hold_op_slave = static_cast<int>(
        parse_long(next_value(), "hold_op_slave"));
    }
    else if (argument == "--validate_only")
    {
      options.validate_only = true;
    }
    else if (argument == "--move_slave")
    {
      options.move_slave = static_cast<int>(
        parse_long(next_value(), "move_slave"));
    }
    else if (argument == "--move_degrees")
    {
      options.move_degrees = parse_double(next_value(), "move_degrees");
    }
    else if (argument == "--pp_velocity")
    {
      const long value = parse_long(next_value(), "pp_velocity");
      if (value <= 0 || value > std::numeric_limits<std::uint32_t>::max())
      {
        throw std::runtime_error("pp_velocity is outside uint32 range");
      }
      options.velocity = static_cast<std::uint32_t>(value);
    }
    else if (argument == "--pp_acceleration")
    {
      const long value = parse_long(next_value(), "pp_acceleration");
      if (value <= 0 || value > std::numeric_limits<std::uint32_t>::max())
      {
        throw std::runtime_error("pp_acceleration is outside uint32 range");
      }
      options.acceleration = static_cast<std::uint32_t>(value);
    }
    else if (argument == "--pp_deceleration")
    {
      const long value = parse_long(next_value(), "pp_deceleration");
      if (value <= 0 || value > std::numeric_limits<std::uint32_t>::max())
      {
        throw std::runtime_error("pp_deceleration is outside uint32 range");
      }
      options.deceleration = static_cast<std::uint32_t>(value);
    }
    else if (argument == "--help" || argument == "-h")
    {
      print_usage(argv[0]);
      std::exit(0);
    }
    else
    {
      throw std::runtime_error("unknown argument: " + argument);
    }
  }

  if (options.master_id > 1)
  {
    throw std::runtime_error("master_id must be 0 or 1");
  }
  if (options.print_every == 0)
  {
    throw std::runtime_error("print_every must be greater than zero");
  }
  if (options.cycle_ns < 100000 || options.cycle_ns > 5000000)
  {
    throw std::runtime_error("cycle_ns must be between 100000 and 5000000");
  }
  if (options.priority < 1 || options.priority > 95)
  {
    throw std::runtime_error("priority must be between 1 and 95");
  }
  if (options.cpu < 0)
  {
    options.cpu = options.master_id == 0 ? 6 : 7;
  }
  const int selected_modes =
    static_cast<int>(options.hold_csp_all) +
    static_cast<int>(options.hold_csp_mask >= 0) +
    static_cast<int>(options.hold_csp_slave >= 0) +
    static_cast<int>(options.hold_op_all) +
    static_cast<int>(options.hold_op_slave >= 0) +
    static_cast<int>(options.validate_only) +
    static_cast<int>(options.move_slave >= 0);
  if (selected_modes != 1)
  {
    throw std::runtime_error(
            "select exactly one mode: --validate_only, --hold_csp_all, "
            "--hold_csp_mask, --hold_csp_slave, --hold_op_all, "
            "--hold_op_slave or --move_slave");
  }
  if (options.hold_csp_mask == 0 || options.hold_csp_mask > 0x7f)
  {
    throw std::runtime_error("hold_csp_mask must be between 1 and 127");
  }
  if (options.hold_op_slave < -1 ||
    options.hold_op_slave >= static_cast<int>(kAxisCount))
  {
    throw std::runtime_error("hold_op_slave must be between 0 and 6");
  }
  if (options.hold_csp_slave < -1 ||
    options.hold_csp_slave >= static_cast<int>(kAxisCount))
  {
    throw std::runtime_error("hold_csp_slave must be between 0 and 6");
  }
  if (options.move_slave >= static_cast<int>(kAxisCount))
  {
    throw std::runtime_error("move_slave must be between 0 and 6");
  }
  if (options.move_slave >= 0 && std::abs(options.move_degrees) > 45.0)
  {
    throw std::runtime_error(
            "refusing a relative move larger than 45 degrees");
  }
  if (options.velocity == 0 || options.acceleration == 0 ||
    options.deceleration == 0)
  {
    throw std::runtime_error(
            "velocity, acceleration and deceleration must be greater than zero");
  }
  return options;
}

bool configure_realtime(const Options & options)
{
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(options.cpu, &set);
  int result = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
  if (result != 0)
  {
    std::cerr << "cannot bind to CPU" << options.cpu << ": "
              << std::strerror(result) << '\n';
    return false;
  }

  sched_param parameters{};
  parameters.sched_priority = options.priority;
  result = pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameters);
  if (result != 0)
  {
    std::cerr << "cannot set SCHED_FIFO/" << options.priority << ": "
              << std::strerror(result) << '\n';
    return false;
  }
  return true;
}

std::uint16_t next_control_word(std::uint16_t status)
{
  const auto masked_6f = static_cast<std::uint16_t>(status & 0x006F);
  const auto masked_4f = static_cast<std::uint16_t>(status & 0x004F);
  if (masked_4f == 0x0040 || masked_4f == 0x0000)
  {
    return 0x0006;
  }
  if (masked_6f == 0x0021)
  {
    return 0x0007;
  }
  if (masked_6f == 0x0023 || masked_6f == 0x0027)
  {
    return 0x000F;
  }
  if (masked_4f == 0x0008)
  {
    return 0x0080;
  }
  return 0x0006;
}

bool operation_enabled(std::uint16_t status)
{
  return (status & 0x006F) == 0x0027;
}

int register_entry(
  ec_slave_config_t * config, ec_domain_t * domain,
  std::uint16_t index, std::uint8_t subindex,
  unsigned int master_id, unsigned int slave)
{
  const int offset = ecrt_slave_config_reg_pdo_entry(
    config, index, subindex, domain, nullptr);
  if (offset < 0)
  {
    throw std::runtime_error(
            "master" + std::to_string(master_id) + " slave" +
            std::to_string(slave) + " PDO " +
            std::to_string(index) + ":" + std::to_string(subindex) +
            " is not mapped");
  }
  return offset;
}

void print_snapshot(
  unsigned int master_id, std::uint64_t snapshot,
  const std::array<Axis, kAxisCount> & axes)
{
  std::cout << "master=" << master_id << " snapshot=" << snapshot << '\n';
  std::cout << "axis slave offset mode status position\n";
  for (std::size_t index = 0; index < axes.size(); ++index)
  {
    const auto & axis = axes[index];
    if (!axis.configured)
    {
      continue;
    }
    std::cout << index << ' ' << index << " 0 "
              << static_cast<int>(axis.mode_actual) << ' ' << axis.status << ' '
              << axis.actual << '\n';
  }
  std::cout << std::flush;
}

class DirectMaster
{
public:
  explicit DirectMaster(Options options)
  : options_(options)
  {
  }

  ~DirectMaster()
  {
    shutdown();
  }

  void configure()
  {
    master_ = ecrt_request_master(options_.master_id);
    if (master_ == nullptr)
    {
      throw std::runtime_error(
              "ecrt_request_master(" + std::to_string(options_.master_id) +
              ") failed; another process may own the master");
    }

    ec_master_info_t info{};
    if (ecrt_master(master_, &info) != 0)
    {
      throw std::runtime_error("failed to query EtherCAT master");
    }
    const bool selective_slave_mode =
      options_.hold_op_slave >= 0 || options_.hold_csp_slave >= 0;
    const int selected_slave = options_.hold_op_slave >= 0 ?
      options_.hold_op_slave : options_.hold_csp_slave;
    if ((!selective_slave_mode && info.slave_count != kAxisCount) ||
      (selective_slave_mode &&
      (selected_slave < 0 ||
      static_cast<unsigned int>(selected_slave) >= info.slave_count ||
      info.slave_count > kAxisCount)))
    {
      throw std::runtime_error(
              "master" + std::to_string(options_.master_id) +
              " slave count is incompatible with this mode, detected " +
              std::to_string(info.slave_count));
    }

    domain_ = ecrt_master_create_domain(master_);
    if (domain_ == nullptr)
    {
      throw std::runtime_error("failed to create process-data domain");
    }

    const auto slave_count = static_cast<std::uint16_t>(info.slave_count);
    for (std::uint16_t slave = 0; slave < slave_count; ++slave)
    {
      ec_slave_info_t slave_info{};
      if (ecrt_master_get_slave(master_, slave, &slave_info) != 0)
      {
        throw std::runtime_error("failed to read slave identity");
      }
      if (slave_info.vendor_id != kVendorId ||
        slave_info.product_code != kProductCode ||
        slave_info.revision_number != kRevision)
      {
        throw std::runtime_error(
                "master" + std::to_string(options_.master_id) + " slave" +
                std::to_string(slave) + " identity mismatch");
      }

      const bool configure_slave =
        (options_.hold_op_slave < 0 && options_.hold_csp_slave < 0) ||
        slave == static_cast<std::uint16_t>(options_.hold_op_slave) ||
        slave == static_cast<std::uint16_t>(options_.hold_csp_slave);
      if (!configure_slave)
      {
        continue;
      }

      auto * config = ecrt_master_slave_config(
        master_, 0, slave, kVendorId, kProductCode);
      if (config == nullptr)
      {
        throw std::runtime_error("failed to create slave configuration");
      }
      slave_configs_[slave] = config;
      if (ecrt_slave_config_pdos(config, EC_END, kSyncManagers) != 0)
      {
        throw std::runtime_error("failed to assign default PDOs");
      }
      if (ecrt_slave_config_dc(
          config, kDcAssignActivate,
          static_cast<std::uint32_t>(options_.cycle_ns), 0, 0, 0) != 0)
      {
        throw std::runtime_error("failed to configure distributed clocks");
      }

      auto & axis = axes_[slave];
      axis.configured = true;
      axis.control_word = register_entry(
        config, domain_, 0x6040, 0, options_.master_id, slave);
      axis.status_word = register_entry(
        config, domain_, 0x6041, 0, options_.master_id, slave);
      axis.mode = register_entry(
        config, domain_, 0x6060, 0, options_.master_id, slave);
      axis.target_position = register_entry(
        config, domain_, 0x607A, 0, options_.master_id, slave);
      axis.actual_position = register_entry(
        config, domain_, 0x6064, 0, options_.master_id, slave);
      axis.actual_velocity = register_entry(
        config, domain_, 0x606C, 0, options_.master_id, slave);
      axis.torque_actual = register_entry(
        config, domain_, 0x6077, 0, options_.master_id, slave);
      axis.error_code = register_entry(
        config, domain_, 0x603F, 0, options_.master_id, slave);
    }

    const std::size_t reference_slave = options_.hold_op_slave >= 0 ?
      static_cast<std::size_t>(options_.hold_op_slave) :
      (options_.hold_csp_slave >= 0 ?
      static_cast<std::size_t>(options_.hold_csp_slave) : 0U);
    if (ecrt_master_select_reference_clock(
        master_, slave_configs_[reference_slave]) != 0)
    {
      throw std::runtime_error(
              "failed to select slave" + std::to_string(reference_slave) +
              " as DC reference");
    }
    if (options_.validate_only)
    {
      return;
    }
    if (ecrt_master_activate(master_) != 0)
    {
      throw std::runtime_error("failed to activate EtherCAT master");
    }
    activated_ = true;
    domain_data_ = ecrt_domain_data(domain_);
    if (domain_data_ == nullptr)
    {
      throw std::runtime_error("domain data is null");
    }
  }

  int run()
  {
    if (options_.validate_only)
    {
      std::cout << "validation_ok master=" << options_.master_id
                << " slaves=7 vendor=0x00010203 product=0x00000402"
                << " revision=0x00050012 rxpdo=0x1601"
                << " txpdo=0x1a01 dc=0x0300\n";
      return 0;
    }
    std::filesystem::create_directories(kStateDirectory);
    std::error_code remove_error;
    std::filesystem::remove(stop_file(options_.master_id), remove_error);
    if (options_.hold_csp_all)
    {
      std::filesystem::remove(
        move_command_file(options_.master_id), remove_error);
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    {
      std::cerr << "warning: mlockall failed: " << std::strerror(errno) << '\n';
    }
    if (!configure_realtime(options_))
    {
      return 1;
    }

    std::cout << "direct_igh master=" << options_.master_id
              << " configured="
              << (options_.hold_op_slave >= 0 ?
              std::to_string(options_.hold_op_slave) :
              (options_.hold_csp_slave >= 0 ?
              std::to_string(options_.hold_csp_slave) : "all"))
              << " cycle_ns=" << options_.cycle_ns
              << " cpu=" << options_.cpu
              << " priority=" << options_.priority << '\n'
              << std::flush;

    timespec wake_time{};
    clock_gettime(CLOCK_MONOTONIC, &wake_time);
    add_ns(wake_time, options_.cycle_ns);

    std::uint64_t cycle = 0;
    std::uint64_t enabled_cycles = 0;
    bool move_started = false;

    while (!g_stop_requested)
    {
      if ((cycle % 100U) == 0U &&
        std::filesystem::exists(stop_file(options_.master_id)))
      {
        g_stop_requested = 1;
        break;
      }

      int sleep_result;
      do
      {
        sleep_result = clock_nanosleep(
          CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, nullptr);
      } while (sleep_result == EINTR);
      if (sleep_result != 0)
      {
        std::cerr << "clock_nanosleep failed: "
                  << std::strerror(sleep_result) << '\n';
        return 1;
      }

      ecrt_master_application_time(master_, timespec_to_ns(wake_time));
      ecrt_master_receive(master_);
      ecrt_domain_process(domain_);

      bool all_initialized = true;
      bool all_enabled = true;
      for (std::size_t slave = 0; slave < axes_.size(); ++slave)
      {
        auto & axis = axes_[slave];
        if (!axis.configured)
        {
          continue;
        }
        axis.actual = EC_READ_S32(domain_data_ + axis.actual_position);
        axis.velocity_actual =
          EC_READ_S32(domain_data_ + axis.actual_velocity);
        axis.status = EC_READ_U16(domain_data_ + axis.status_word);
        axis.mode_actual = 8;

        if (!axis.initialized && axis.status != 0)
        {
          axis.command_position = static_cast<double>(axis.actual);
          axis.goal_position = axis.actual;
          axis.command = axis.actual;
          axis.initialized = true;
        }
        all_initialized = all_initialized && axis.initialized;
        const bool enable_axis =
          !options_.hold_op_all && options_.hold_op_slave < 0 &&
          (options_.hold_csp_mask < 0 ||
          (options_.hold_csp_mask & (1 << slave)) != 0);
        all_enabled =
          all_enabled && (!enable_axis ||
          operation_enabled(axis.status));
        EC_WRITE_S8(domain_data_ + axis.mode, 8);
        if (axis.initialized)
        {
          const double dt =
            static_cast<double>(options_.cycle_ns) / 1.0e9;
          const double remaining =
            static_cast<double>(axis.goal_position) - axis.command_position;
          const double max_step = axis.command_velocity * dt;
          axis.command_position += std::clamp(
            remaining, -max_step, max_step);
          axis.command = static_cast<std::int32_t>(
            std::llround(axis.command_position));
          EC_WRITE_S32(domain_data_ + axis.target_position, axis.command);
        }
        EC_WRITE_U16(
          domain_data_ + axis.control_word,
          !enable_axis ? 0x0006 :
          (axis.initialized ? next_control_word(axis.status) : 0x0006));
      }

      if (options_.hold_op_all || options_.hold_op_slave >= 0)
      {
        all_enabled = all_initialized;
      }

      if (all_initialized && all_enabled)
      {
        ++enabled_cycles;
      }
      else
      {
        enabled_cycles = 0;
      }

      if (options_.hold_csp_all && enabled_cycles >= 100 &&
        (cycle % 100U) == 0U &&
        std::filesystem::exists(move_command_file(options_.master_id)))
      {
        const auto command_path = move_command_file(options_.master_id);
        std::ifstream command_file(command_path);
        std::string request_id;
        int slave = -1;
        double degrees = 0.0;
        double speed_degrees_per_second = 0.0;
        std::string extra;
        const bool parsed =
          static_cast<bool>(
          command_file >> request_id >> slave >> degrees >>
          speed_degrees_per_second) &&
          !(command_file >> extra);
        command_file.close();
        std::filesystem::remove(command_path, remove_error);

        if (!parsed || slave < 0 || slave >= static_cast<int>(kAxisCount) ||
          !std::isfinite(degrees) || std::abs(degrees) > 45.0 ||
          !std::isfinite(speed_degrees_per_second) ||
          speed_degrees_per_second <= 0.0 ||
          speed_degrees_per_second > 90.0)
        {
          std::cout << "move_command_rejected request=" << request_id
                    << " reason=invalid_command\n" << std::flush;
        }
        else
        {
          auto & selected_axis = axes_[static_cast<std::size_t>(slave)];
          const double delta =
            joint_direction(options_.master_id, static_cast<unsigned int>(slave)) *
            degrees * static_cast<double>(
            counts_per_revolution(static_cast<unsigned int>(slave))) / 360.0;
          const auto target = static_cast<std::int32_t>(std::llround(
              static_cast<double>(selected_axis.actual) + delta));
          selected_axis.goal_position = target;
          selected_axis.command_velocity =
            speed_degrees_per_second * static_cast<double>(
            counts_per_revolution(static_cast<unsigned int>(slave))) / 360.0;
          std::cout << "move_command_applied request=" << request_id
                    << " master=" << options_.master_id
                    << " slave=" << slave
                    << " degrees=" << degrees
                    << " speed_deg_s=" << speed_degrees_per_second
                    << " start=" << selected_axis.actual
                    << " target=" << target << '\n' << std::flush;
        }
      }

      if (options_.move_slave >= 0 && enabled_cycles >= 100)
      {
        auto & moving_axis =
          axes_[static_cast<std::size_t>(options_.move_slave)];
        if (!move_started)
        {
          const double delta =
            joint_direction(
              options_.master_id,
              static_cast<unsigned int>(options_.move_slave)) *
            options_.move_degrees *
            static_cast<double>(
            counts_per_revolution(
              static_cast<unsigned int>(options_.move_slave))) / 360.0;
          const auto move_target = static_cast<std::int32_t>(
            std::llround(static_cast<double>(moving_axis.actual) + delta));
          moving_axis.goal_position = move_target;
          moving_axis.command_velocity = static_cast<double>(options_.velocity);
          move_started = true;
          std::cout << "move_axis phase=4 target=" << move_target
                    << " start=" << moving_axis.actual
                    << " slave=" << options_.move_slave << '\n'
                    << "Enter another relative move as: slave(0-6) degrees([-45,45])\n"
                    << std::flush;
        }

        if ((cycle % 100U) == 0U)
        {
          pollfd input{};
          input.fd = STDIN_FILENO;
          input.events = POLLIN;
          if (poll(&input, 1, 0) > 0 && (input.revents & POLLIN) != 0)
          {
            std::string line;
            if (std::getline(std::cin, line))
            {
              std::istringstream stream(line);
              int slave = -1;
              double degrees = 0.0;
              std::string extra;
              if (!(stream >> slave >> degrees) || (stream >> extra) ||
                slave < 0 || slave >= static_cast<int>(kAxisCount) ||
                !std::isfinite(degrees) || std::abs(degrees) > 45.0)
              {
                std::cout << "invalid move; use: slave(0-6) degrees([-45,45])\n"
                          << std::flush;
              }
              else
              {
                auto & selected_axis = axes_[static_cast<std::size_t>(slave)];
                const double delta =
                  joint_direction(
                    options_.master_id, static_cast<unsigned int>(slave)) *
                  degrees * static_cast<double>(
                  counts_per_revolution(static_cast<unsigned int>(slave))) /
                  360.0;
                const auto target = static_cast<std::int32_t>(std::llround(
                    static_cast<double>(selected_axis.actual) + delta));
                selected_axis.goal_position = target;
                selected_axis.command_velocity =
                  static_cast<double>(options_.velocity);
                std::cout << "move_axis target=" << target
                          << " start=" << selected_axis.actual
                          << " slave=" << slave
                          << " degrees=" << degrees << '\n'
                          << std::flush;
              }
            }
          }
        }
      }

      if ((cycle % options_.print_every) == 0U)
      {
        print_snapshot(options_.master_id, cycle, axes_);
        if (!all_initialized)
        {
          std::cout << "waiting_for_valid_pdo_feedback\n" << std::flush;
        }
      }

      ecrt_master_sync_reference_clock(master_);
      ecrt_master_sync_slave_clocks(master_);
      ecrt_domain_queue(domain_);
      ecrt_master_send(master_);

      ++cycle;
      add_ns(wake_time, options_.cycle_ns);
      timespec now{};
      clock_gettime(CLOCK_MONOTONIC, &now);
      if (timespec_to_ns(now) >
        timespec_to_ns(wake_time) +
        static_cast<std::uint64_t>(options_.cycle_ns))
      {
        wake_time = now;
        add_ns(wake_time, options_.cycle_ns);
      }
    }

    send_safe_disable();
    std::filesystem::remove(stop_file(options_.master_id), remove_error);
    return 0;
  }

private:
  void send_safe_disable()
  {
    if (!activated_ || domain_data_ == nullptr)
    {
      return;
    }
    for (int cycle = 0; cycle < 20; ++cycle)
    {
      ecrt_master_receive(master_);
      ecrt_domain_process(domain_);
      for (auto & axis : axes_)
      {
        if (!axis.configured)
        {
          continue;
        }
        if (axis.initialized)
        {
          EC_WRITE_S32(domain_data_ + axis.target_position, axis.actual);
        }
        EC_WRITE_U16(domain_data_ + axis.control_word, 0x0006);
      }
      ecrt_domain_queue(domain_);
      ecrt_master_send(master_);
      usleep(1000);
    }
  }

  void shutdown()
  {
    if (master_ != nullptr)
    {
      if (activated_)
      {
        ecrt_master_deactivate(master_);
      }
      ecrt_release_master(master_);
    }
    master_ = nullptr;
    domain_ = nullptr;
    domain_data_ = nullptr;
    activated_ = false;
  }

  Options options_;
  ec_master_t * master_{nullptr};
  ec_domain_t * domain_{nullptr};
  std::uint8_t * domain_data_{nullptr};
  std::array<ec_slave_config_t *, kAxisCount> slave_configs_{};
  std::array<Axis, kAxisCount> axes_{};
  bool activated_{false};
};
}  // namespace

int main(int argc, char ** argv)
{
  try
  {
    const Options options = parse_options(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    DirectMaster master(options);
    master.configure();
    return master.run();
  }
  catch (const std::exception & exception)
  {
    std::cerr << "ethercat_direct_tool: " << exception.what() << '\n';
    print_usage(argv[0]);
    return 1;
  }
}




// #include <ecrt.h>

// #include <pthread.h>
// #include <poll.h>
// #include <sched.h>
// #include <sys/mman.h>
// #include <sys/stat.h>
// #include <time.h>
// #include <unistd.h>

// #include <algorithm>
// #include <array>
// #include <atomic>
// #include <cerrno>
// #include <cmath>
// #include <csignal>
// #include <cstdint>
// #include <cstdlib>
// #include <cstring>
// #include <filesystem>
// #include <fstream>
// #include <iostream>
// #include <limits>
// #include <sstream>
// #include <stdexcept>
// #include <string>

// namespace
// {

// // ============================================================
// // EtherCAT slave 数量配置
// //
// // Master 0: 7 个电机，slave 0 ~ 6
// // Master 1: 6 个电机，slave 0 ~ 5
// //
// // kAxisCount 保持 7，作为内部数组最大容量。
// // 不要改成 6，否则会影响 Master 0。
// // ============================================================

// constexpr std::size_t kAxisCount = 7;
// constexpr std::size_t kMaster0AxisCount = 7;
// constexpr std::size_t kMaster1AxisCount = 6;

// constexpr std::uint32_t kVendorId = 0x00000B95;
// constexpr std::uint32_t kProductCode = 0x90000300;
// constexpr std::uint16_t kDcAssignActivate = 0x0300;
// constexpr std::int64_t kDefaultCycleNs = 1000000;
// constexpr int kDefaultPriority = 90;
// constexpr char kStateDirectory[] = "/tmp/ethercat_op_csp";

// volatile std::sig_atomic_t g_stop_requested = 0;

// const ec_pdo_entry_info_t kRxPdoEntries[] = {
//   {0x6040, 0x00, 16},
//   {0x6060, 0x00, 16},
//   {0x607A, 0x00, 32},
//   {0x60FF, 0x00, 32},
//   {0x6071, 0x00, 16},
//   {0x6081, 0x00, 32},
//   {0x6083, 0x00, 32},
//   {0x6084, 0x00, 32},
//   {0x60FE, 0x01, 32},
// };

// const ec_pdo_entry_info_t kTxPdoEntries[] = {
//   {0x603F, 0x00, 16},
//   {0x6041, 0x00, 16},
//   {0x6061, 0x00, 16},
//   {0x6062, 0x00, 32},
//   {0x6064, 0x00, 32},
//   {0x606C, 0x00, 32},
//   {0x6077, 0x00, 16},
//   {0x60FD, 0x00, 32},
// };

// const ec_pdo_info_t kRxPdos[] = {
//   {0x1600, static_cast<unsigned int>(std::size(kRxPdoEntries)), kRxPdoEntries},
// };

// const ec_pdo_info_t kTxPdos[] = {
//   {0x1A00, static_cast<unsigned int>(std::size(kTxPdoEntries)), kTxPdoEntries},
// };

// const ec_sync_info_t kSyncManagers[] = {
//   {2, EC_DIR_OUTPUT, 1, kRxPdos, EC_WD_ENABLE},
//   {3, EC_DIR_INPUT, 1, kTxPdos, EC_WD_DISABLE},
//   {0xFF, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT},
// };

// struct Options
// {
//   unsigned int master_id{std::numeric_limits<unsigned int>::max()};
//   unsigned int print_every{1000};
//   std::int64_t cycle_ns{kDefaultCycleNs};
//   int cpu{-1};
//   int priority{kDefaultPriority};

//   bool hold_csp_all{false};
//   int hold_csp_mask{-1};
//   int hold_csp_slave{-1};

//   bool hold_op_all{false};
//   int hold_op_slave{-1};

//   bool validate_only{false};

//   int move_slave{-1};
//   double move_degrees{0.0};

//   std::uint32_t velocity{50000};
//   std::uint32_t acceleration{100000};
//   std::uint32_t deceleration{100000};
// };

// struct Axis
// {
//   int control_word{-1};
//   int status_word{-1};
//   int mode{-1};
//   int mode_display{-1};

//   int target_position{-1};

//   int profile_velocity{-1};
//   int profile_acceleration{-1};
//   int profile_deceleration{-1};

//   int actual_position{-1};
//   int actual_velocity{-1};

//   std::int32_t command{0};
//   double command_position{0.0};

//   std::int32_t goal_position{0};
//   double command_velocity{0.0};

//   std::int32_t actual{0};
//   std::int32_t velocity_actual{0};

//   std::uint16_t status{0};
//   std::int16_t mode_actual{0};

//   bool initialized{false};
//   bool configured{false};
// };

// void signal_handler(int)
// {
//   g_stop_requested = 1;
// }

// std::uint64_t timespec_to_ns(const timespec & value)
// {
//   return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
//          static_cast<std::uint64_t>(value.tv_nsec);
// }

// void add_ns(timespec & value, std::int64_t nanoseconds)
// {
//   value.tv_nsec += nanoseconds;

//   while (value.tv_nsec >= 1000000000L)
//   {
//     value.tv_nsec -= 1000000000L;
//     ++value.tv_sec;
//   }
// }

// std::uint32_t counts_per_revolution(unsigned int slave)
// {
//   return slave < 4 ? 524288U : 262144U;
// }

// double joint_direction(unsigned int master_id, unsigned int slave)
// {
//   static constexpr std::array<double, 7> kRightDirections{
//     -1.0,
//     1.0,
//     -1.0,
//     -1.0,
//     -1.0,
//     -1.0,
//     1.0
//   };

//   if (master_id == 0)
//   {
//     if (slave >= kRightDirections.size())
//     {
//       throw std::runtime_error(
//               "Master0 slave index out of direction table range");
//     }

//     return kRightDirections.at(slave);
//   }

//   // Master1 六个电机目前统一正方向
//   return 1.0;
// }

// std::string stop_file(unsigned int master_id)
// {
//   return std::string(kStateDirectory) + "/master" +
//          std::to_string(master_id) + ".stop";
// }

// std::string move_command_file(unsigned int master_id)
// {
//   return std::string(kStateDirectory) + "/master" +
//          std::to_string(master_id) + ".move";
// }

// long parse_long(const std::string & text, const char * name)
// {
//   std::size_t consumed = 0;
//   long value = 0;

//   try
//   {
//     value = std::stol(text, &consumed, 0);
//   }
//   catch (const std::exception &)
//   {
//     throw std::runtime_error(
//             std::string("invalid ") + name + ": " + text);
//   }

//   if (consumed != text.size())
//   {
//     throw std::runtime_error(
//             std::string("invalid ") + name + ": " + text);
//   }

//   return value;
// }

// double parse_double(const std::string & text, const char * name)
// {
//   std::size_t consumed = 0;
//   double value = 0.0;

//   try
//   {
//     value = std::stod(text, &consumed);
//   }
//   catch (const std::exception &)
//   {
//     throw std::runtime_error(
//             std::string("invalid ") + name + ": " + text);
//   }

//   if (consumed != text.size() || !std::isfinite(value))
//   {
//     throw std::runtime_error(
//             std::string("invalid ") + name + ": " + text);
//   }

//   return value;
// }

// void print_usage(const char * executable)
// {
//   std::cerr
//     << "Usage:\n  "
//     << executable
//     << " --master_id 0|1 --validate_only\n  "

//     << executable
//     << " --master_id 0|1 --hold_csp_all [--print_every N]\n  "

//     << executable
//     << " --master_id 0|1 --hold_csp_mask 1..127 [--print_every N]\n  "

//     << executable
//     << " --master_id 0|1 --hold_csp_slave 0..6 [--print_every N]\n  "

//     << executable
//     << " --master_id 0|1 --hold_op_all [--print_every N]\n  "

//     << executable
//     << " --master_id 0|1 --hold_op_slave 0..6 [--print_every N]\n  "

//     << executable
//     << " --master_id 0|1 --move_slave 0..6 --move_degrees DEG"
//     << " [--pp_velocity N --pp_acceleration N --pp_deceleration N]\n";
// }

// Options parse_options(int argc, char ** argv)
// {
//   Options options;

//   for (int index = 1; index < argc; ++index)
//   {
//     const std::string argument = argv[index];

//     auto next_value = [&]() -> std::string
//       {
//         if (++index >= argc)
//         {
//           throw std::runtime_error(
//                   "missing value after " + argument);
//         }

//         return argv[index];
//       };

//     if (argument == "--master_id")
//     {
//       options.master_id = static_cast<unsigned int>(
//         parse_long(next_value(), "master_id"));
//     }
//     else if (argument == "--print_every")
//     {
//       options.print_every = static_cast<unsigned int>(
//         parse_long(next_value(), "print_every"));
//     }
//     else if (argument == "--cycle_ns")
//     {
//       options.cycle_ns = parse_long(
//         next_value(),
//         "cycle_ns");
//     }
//     else if (argument == "--cpu")
//     {
//       options.cpu = static_cast<int>(
//         parse_long(next_value(), "cpu"));
//     }
//     else if (argument == "--priority")
//     {
//       options.priority = static_cast<int>(
//         parse_long(next_value(), "priority"));
//     }
//     else if (argument == "--hold_csp_all")
//     {
//       options.hold_csp_all = true;
//     }
//     else if (argument == "--hold_csp_mask")
//     {
//       options.hold_csp_mask = static_cast<int>(
//         parse_long(next_value(), "hold_csp_mask"));
//     }
//     else if (argument == "--hold_csp_slave")
//     {
//       options.hold_csp_slave = static_cast<int>(
//         parse_long(next_value(), "hold_csp_slave"));
//     }
//     else if (argument == "--hold_op_all")
//     {
//       options.hold_op_all = true;
//     }
//     else if (argument == "--hold_op_slave")
//     {
//       options.hold_op_slave = static_cast<int>(
//         parse_long(next_value(), "hold_op_slave"));
//     }
//     else if (argument == "--validate_only")
//     {
//       options.validate_only = true;
//     }
//     else if (argument == "--move_slave")
//     {
//       options.move_slave = static_cast<int>(
//         parse_long(next_value(), "move_slave"));
//     }
//     else if (argument == "--move_degrees")
//     {
//       options.move_degrees = parse_double(
//         next_value(),
//         "move_degrees");
//     }
//     else if (argument == "--pp_velocity")
//     {
//       const long value =
//         parse_long(next_value(), "pp_velocity");

//       if (value <= 0 ||
//         value > std::numeric_limits<std::uint32_t>::max())
//       {
//         throw std::runtime_error(
//                 "pp_velocity is outside uint32 range");
//       }

//       options.velocity =
//         static_cast<std::uint32_t>(value);
//     }
//     else if (argument == "--pp_acceleration")
//     {
//       const long value =
//         parse_long(next_value(), "pp_acceleration");

//       if (value <= 0 ||
//         value > std::numeric_limits<std::uint32_t>::max())
//       {
//         throw std::runtime_error(
//                 "pp_acceleration is outside uint32 range");
//       }

//       options.acceleration =
//         static_cast<std::uint32_t>(value);
//     }
//     else if (argument == "--pp_deceleration")
//     {
//       const long value =
//         parse_long(next_value(), "pp_deceleration");

//       if (value <= 0 ||
//         value > std::numeric_limits<std::uint32_t>::max())
//       {
//         throw std::runtime_error(
//                 "pp_deceleration is outside uint32 range");
//       }

//       options.deceleration =
//         static_cast<std::uint32_t>(value);
//     }
//     else if (argument == "--help" || argument == "-h")
//     {
//       print_usage(argv[0]);
//       std::exit(0);
//     }
//     else
//     {
//       throw std::runtime_error(
//               "unknown argument: " + argument);
//     }
//   }

//   if (options.master_id > 1)
//   {
//     throw std::runtime_error(
//             "master_id must be 0 or 1");
//   }

//   if (options.print_every == 0)
//   {
//     throw std::runtime_error(
//             "print_every must be greater than zero");
//   }

//   if (options.cycle_ns < 100000 ||
//     options.cycle_ns > 5000000)
//   {
//     throw std::runtime_error(
//             "cycle_ns must be between 100000 and 5000000");
//   }

//   if (options.priority < 1 ||
//     options.priority > 95)
//   {
//     throw std::runtime_error(
//             "priority must be between 1 and 95");
//   }

//   if (options.cpu < 0)
//   {
//     options.cpu =
//       options.master_id == 0 ? 6 : 7;
//   }

//   const int selected_modes =
//     static_cast<int>(options.hold_csp_all) +
//     static_cast<int>(options.hold_csp_mask >= 0) +
//     static_cast<int>(options.hold_csp_slave >= 0) +
//     static_cast<int>(options.hold_op_all) +
//     static_cast<int>(options.hold_op_slave >= 0) +
//     static_cast<int>(options.validate_only) +
//     static_cast<int>(options.move_slave >= 0);

//   if (selected_modes != 1)
//   {
//     throw std::runtime_error(
//             "select exactly one mode: --validate_only, "
//             "--hold_csp_all, --hold_csp_mask, "
//             "--hold_csp_slave, --hold_op_all, "
//             "--hold_op_slave or --move_slave");
//   }

//   if (options.hold_csp_mask == 0 ||
//     options.hold_csp_mask > 0x7f)
//   {
//     throw std::runtime_error(
//             "hold_csp_mask must be between 1 and 127");
//   }

//   if (options.hold_op_slave < -1 ||
//     options.hold_op_slave >=
//     static_cast<int>(kAxisCount))
//   {
//     throw std::runtime_error(
//             "hold_op_slave must be between 0 and 6");
//   }

//   if (options.hold_csp_slave < -1 ||
//     options.hold_csp_slave >=
//     static_cast<int>(kAxisCount))
//   {
//     throw std::runtime_error(
//             "hold_csp_slave must be between 0 and 6");
//   }

//   if (options.move_slave >=
//     static_cast<int>(kAxisCount))
//   {
//     throw std::runtime_error(
//             "move_slave must be between 0 and 6");
//   }

//   if (options.move_slave >= 0 &&
//     std::abs(options.move_degrees) > 45.0)
//   {
//     throw std::runtime_error(
//             "refusing a relative move larger than 45 degrees");
//   }

//   if (options.velocity == 0 ||
//     options.acceleration == 0 ||
//     options.deceleration == 0)
//   {
//     throw std::runtime_error(
//             "velocity, acceleration and deceleration "
//             "must be greater than zero");
//   }

//   // ----------------------------------------------------------
//   // Master1 只有 6 个 slave。
//   //
//   // 对 mask 模式做实际 slave 数量检查。
//   // Master1 最大允许 bit 0~5。
//   // ----------------------------------------------------------
//   if (options.master_id == 1)
//   {
//     if (options.hold_csp_mask >= 0 &&
//       (options.hold_csp_mask & ~0x3F) != 0)
//     {
//       throw std::runtime_error(
//               "master1 hold_csp_mask may only select "
//               "slave 0..5 (mask 0x01..0x3f)");
//     }

//     if (options.hold_op_slave >=
//       static_cast<int>(kMaster1AxisCount))
//     {
//       throw std::runtime_error(
//               "master1 hold_op_slave must be between 0 and 5");
//     }

//     if (options.hold_csp_slave >=
//       static_cast<int>(kMaster1AxisCount))
//     {
//       throw std::runtime_error(
//               "master1 hold_csp_slave must be between 0 and 5");
//     }

//     if (options.move_slave >=
//       static_cast<int>(kMaster1AxisCount))
//     {
//       throw std::runtime_error(
//               "master1 move_slave must be between 0 and 5");
//     }
//   }

//   return options;
// }

// bool configure_realtime(const Options & options)
// {
//   cpu_set_t set;

//   CPU_ZERO(&set);
//   CPU_SET(options.cpu, &set);

//   int result =
//     pthread_setaffinity_np(
//       pthread_self(),
//       sizeof(set),
//       &set);

//   if (result != 0)
//   {
//     std::cerr
//       << "cannot bind to CPU"
//       << options.cpu
//       << ": "
//       << std::strerror(result)
//       << '\n';

//     return false;
//   }

//   sched_param parameters{};
//   parameters.sched_priority = options.priority;

//   result =
//     pthread_setschedparam(
//       pthread_self(),
//       SCHED_FIFO,
//       &parameters);

//   if (result != 0)
//   {
//     std::cerr
//       << "cannot set SCHED_FIFO/"
//       << options.priority
//       << ": "
//       << std::strerror(result)
//       << '\n';

//     return false;
//   }

//   return true;
// }

// std::uint16_t next_control_word(std::uint16_t status)
// {
//   const auto masked_6f =
//     static_cast<std::uint16_t>(status & 0x006F);

//   const auto masked_4f =
//     static_cast<std::uint16_t>(status & 0x004F);

//   if (masked_4f == 0x0040 ||
//     masked_4f == 0x0000)
//   {
//     return 0x0006;
//   }

//   if (masked_6f == 0x0021)
//   {
//     return 0x0007;
//   }

//   if (masked_6f == 0x0023 ||
//     masked_6f == 0x0027)
//   {
//     return 0x000F;
//   }

//   if (masked_4f == 0x0008)
//   {
//     return 0x0080;
//   }

//   return 0x0006;
// }

// bool operation_enabled(std::uint16_t status)
// {
//   return (status & 0x006F) == 0x0027;
// }

// int register_entry(
//   ec_slave_config_t * config,
//   ec_domain_t * domain,
//   std::uint16_t index,
//   std::uint8_t subindex,
//   unsigned int master_id,
//   unsigned int slave)
// {
//   const int offset =
//     ecrt_slave_config_reg_pdo_entry(
//       config,
//       index,
//       subindex,
//       domain,
//       nullptr);

//   if (offset < 0)
//   {
//     throw std::runtime_error(
//             "master" +
//             std::to_string(master_id) +
//             " slave" +
//             std::to_string(slave) +
//             " PDO " +
//             std::to_string(index) +
//             ":" +
//             std::to_string(subindex) +
//             " is not mapped");
//   }

//   return offset;
// }

// void print_snapshot(
//   unsigned int master_id,
//   std::uint64_t snapshot,
//   const std::array<Axis, kAxisCount> & axes)
// {
//   std::cout
//     << "master="
//     << master_id
//     << " snapshot="
//     << snapshot
//     << '\n';

//   std::cout
//     << "axis slave offset mode status position\n";

//   for (std::size_t index = 0;
//     index < axes.size();
//     ++index)
//   {
//     const auto & axis = axes[index];

//     if (!axis.configured)
//     {
//       continue;
//     }

//     std::cout
//       << index
//       << ' '
//       << index
//       << " 0 "
//       << axis.mode_actual
//       << ' '
//       << axis.status
//       << ' '
//       << axis.actual
//       << '\n';
//   }

//   std::cout << std::flush;
// }

// class DirectMaster
// {
// public:
//   explicit DirectMaster(Options options)
//   : options_(options)
//   {
//   }

//   ~DirectMaster()
//   {
//     shutdown();
//   }

//   void configure()
//   {
//     master_ =
//       ecrt_request_master(options_.master_id);

//     if (master_ == nullptr)
//     {
//       throw std::runtime_error(
//               "ecrt_request_master(" +
//               std::to_string(options_.master_id) +
//               ") failed; another process may own the master");
//     }

//     ec_master_info_t info{};

//     if (ecrt_master(master_, &info) != 0)
//     {
//       throw std::runtime_error(
//               "failed to query EtherCAT master");
//     }

//     // --------------------------------------------------------
//     // 根据 Master ID 确定实际期望的 slave 数量
//     //
//     // Master 0 -> 7
//     // Master 1 -> 6
//     // --------------------------------------------------------

//     const std::size_t expected_axis_count =
//       options_.master_id == 1 ?
//       kMaster1AxisCount :
//       kMaster0AxisCount;

//     const bool selective_slave_mode =
//       options_.hold_op_slave >= 0 ||
//       options_.hold_csp_slave >= 0;

//     const int selected_slave =
//       options_.hold_op_slave >= 0 ?
//       options_.hold_op_slave :
//       options_.hold_csp_slave;

//     // --------------------------------------------------------
//     // slave 数量检查
//     //
//     // 非选择模式：
//     //   必须完全匹配 Master 对应的 slave 数量。
//     //
//     // 选择单个 slave：
//     //   slave 必须存在，并且不能超过 Master 实际数量。
//     // --------------------------------------------------------

//     if ((!selective_slave_mode &&
//       info.slave_count != expected_axis_count) ||
//       (selective_slave_mode &&
//       (selected_slave < 0 ||
//       static_cast<unsigned int>(selected_slave) >=
//       info.slave_count ||
//       info.slave_count > expected_axis_count)))
//     {
//       throw std::runtime_error(
//               "master" +
//               std::to_string(options_.master_id) +
//               " slave count is incompatible with this mode, detected " +
//               std::to_string(info.slave_count) +
//               ", expected " +
//               std::to_string(expected_axis_count));
//     }

//     // --------------------------------------------------------
//     // Master1 再做一次明确检查
//     // --------------------------------------------------------

//     if (options_.master_id == 1 &&
//       info.slave_count != kMaster1AxisCount)
//     {
//       throw std::runtime_error(
//               "master1 requires exactly 6 slaves, detected " +
//               std::to_string(info.slave_count));
//     }

//     // --------------------------------------------------------
//     // 创建 process data domain
//     // --------------------------------------------------------

//     domain_ =
//       ecrt_master_create_domain(master_);

//     if (domain_ == nullptr)
//     {
//       throw std::runtime_error(
//               "failed to create process-data domain");
//     }

//     const auto slave_count =
//       static_cast<std::uint16_t>(info.slave_count);

//     // --------------------------------------------------------
//     // 配置实际存在的 slave
//     // --------------------------------------------------------

//     for (std::uint16_t slave = 0;
//       slave < slave_count;
//       ++slave)
//     {
//       ec_slave_info_t slave_info{};

//       if (ecrt_master_get_slave(
//           master_,
//           slave,
//           &slave_info) != 0)
//       {
//         throw std::runtime_error(
//                 "failed to read slave identity");
//       }

//       if (slave_info.vendor_id != kVendorId ||
//         slave_info.product_code != kProductCode)
//       {
//         throw std::runtime_error(
//                 "master" +
//                 std::to_string(options_.master_id) +
//                 " slave" +
//                 std::to_string(slave) +
//                 " identity mismatch");
//       }

//       const bool configure_slave =
//         (options_.hold_op_slave < 0 &&
//         options_.hold_csp_slave < 0) ||
//         slave ==
//         static_cast<std::uint16_t>(
//           options_.hold_op_slave) ||
//         slave ==
//         static_cast<std::uint16_t>(
//           options_.hold_csp_slave);

//       if (!configure_slave)
//       {
//         continue;
//       }

//       auto * config =
//         ecrt_master_slave_config(
//           master_,
//           0,
//           slave,
//           kVendorId,
//           kProductCode);

//       if (config == nullptr)
//       {
//         throw std::runtime_error(
//                 "failed to create slave configuration");
//       }

//       slave_configs_[slave] = config;

//       if (ecrt_slave_config_pdos(
//           config,
//           EC_END,
//           kSyncManagers) != 0)
//       {
//         throw std::runtime_error(
//                 "failed to assign default PDOs");
//       }

//       if (ecrt_slave_config_dc(
//           config,
//           kDcAssignActivate,
//           static_cast<std::uint32_t>(
//             options_.cycle_ns),
//           0,
//           0,
//           0) != 0)
//       {
//         throw std::runtime_error(
//                 "failed to configure distributed clocks");
//       }

//       auto & axis = axes_[slave];

//       axis.configured = true;

//       axis.control_word =
//         register_entry(
//           config,
//           domain_,
//           0x6040,
//           0,
//           options_.master_id,
//           slave);

//       axis.status_word =
//         register_entry(
//           config,
//           domain_,
//           0x6041,
//           0,
//           options_.master_id,
//           slave);

//       axis.mode =
//         register_entry(
//           config,
//           domain_,
//           0x6060,
//           0,
//           options_.master_id,
//           slave);

//       axis.mode_display =
//         register_entry(
//           config,
//           domain_,
//           0x6061,
//           0,
//           options_.master_id,
//           slave);

//       axis.target_position =
//         register_entry(
//           config,
//           domain_,
//           0x607A,
//           0,
//           options_.master_id,
//           slave);

//       axis.profile_velocity =
//         register_entry(
//           config,
//           domain_,
//           0x6081,
//           0,
//           options_.master_id,
//           slave);

//       axis.profile_acceleration =
//         register_entry(
//           config,
//           domain_,
//           0x6083,
//           0,
//           options_.master_id,
//           slave);

//       axis.profile_deceleration =
//         register_entry(
//           config,
//           domain_,
//           0x6084,
//           0,
//           options_.master_id,
//           slave);

//       axis.actual_position =
//         register_entry(
//           config,
//           domain_,
//           0x6064,
//           0,
//           options_.master_id,
//           slave);

//       axis.actual_velocity =
//         register_entry(
//           config,
//           domain_,
//           0x606C,
//           0,
//           options_.master_id,
//           slave);
//     }

//     // --------------------------------------------------------
//     // DC reference slave
//     //
//     // 全体模式：
//     //   默认使用 slave 0
//     //
//     // 单 slave 模式：
//     //   使用选择的 slave
//     // --------------------------------------------------------

//     const std::size_t reference_slave =
//       options_.hold_op_slave >= 0 ?
//       static_cast<std::size_t>(
//         options_.hold_op_slave) :
//       (options_.hold_csp_slave >= 0 ?
//       static_cast<std::size_t>(
//         options_.hold_csp_slave) :
//       0U);

//     if (reference_slave >= expected_axis_count)
//     {
//       throw std::runtime_error(
//               "DC reference slave " +
//               std::to_string(reference_slave) +
//               " is outside expected slave range");
//     }

//     if (slave_configs_[reference_slave] == nullptr)
//     {
//       throw std::runtime_error(
//               "DC reference slave " +
//               std::to_string(reference_slave) +
//               " is not configured");
//     }

//     if (ecrt_master_select_reference_clock(
//         master_,
//         slave_configs_[reference_slave]) != 0)
//     {
//       throw std::runtime_error(
//               "failed to select slave" +
//               std::to_string(reference_slave) +
//               " as DC reference");
//     }

//     if (options_.validate_only)
//     {
//       return;
//     }

//     if (ecrt_master_activate(master_) != 0)
//     {
//       throw std::runtime_error(
//               "failed to activate EtherCAT master");
//     }

//     activated_ = true;

//     domain_data_ =
//       ecrt_domain_data(domain_);

//     if (domain_data_ == nullptr)
//     {
//       throw std::runtime_error(
//               "domain data is null");
//     }
//   }

//   int run()
//   {
//     if (options_.validate_only)
//     {
//       const std::size_t expected_axis_count =
//         options_.master_id == 1 ?
//         kMaster1AxisCount :
//         kMaster0AxisCount;

//       std::cout
//         << "validation_ok master="
//         << options_.master_id
//         << " slaves="
//         << expected_axis_count
//         << " vendor=0x00000b95"
//         << " product=0x90000300"
//         << " rxpdo=0x1600"
//         << " txpdo=0x1a00"
//         << " dc=0x0300\n";

//       return 0;
//     }

//     std::filesystem::create_directories(
//       kStateDirectory);

//     std::error_code remove_error;

//     std::filesystem::remove(
//       stop_file(options_.master_id),
//       remove_error);

//     if (options_.hold_csp_all)
//     {
//       std::filesystem::remove(
//         move_command_file(options_.master_id),
//         remove_error);
//     }

//     if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
//     {
//       std::cerr
//         << "warning: mlockall failed: "
//         << std::strerror(errno)
//         << '\n';
//     }

//     if (!configure_realtime(options_))
//     {
//       return 1;
//     }

//     std::cout
//       << "direct_igh master="
//       << options_.master_id
//       << " configured="
//       << (options_.hold_op_slave >= 0 ?
//           std::to_string(options_.hold_op_slave) :
//           (options_.hold_csp_slave >= 0 ?
//           std::to_string(options_.hold_csp_slave) :
//           "all"))
//       << " cycle_ns="
//       << options_.cycle_ns
//       << " cpu="
//       << options_.cpu
//       << " priority="
//       << options_.priority
//       << '\n'
//       << std::flush;

//     timespec wake_time{};

//     clock_gettime(
//       CLOCK_MONOTONIC,
//       &wake_time);

//     add_ns(
//       wake_time,
//       options_.cycle_ns);

//     std::uint64_t cycle = 0;
//     std::uint64_t enabled_cycles = 0;

//     bool move_started = false;

//     while (!g_stop_requested)
//     {
//       if ((cycle % 100U) == 0U &&
//         std::filesystem::exists(
//           stop_file(options_.master_id)))
//       {
//         g_stop_requested = 1;
//         break;
//       }

//       int sleep_result;

//       do
//       {
//         sleep_result =
//           clock_nanosleep(
//             CLOCK_MONOTONIC,
//             TIMER_ABSTIME,
//             &wake_time,
//             nullptr);
//       }
//       while (sleep_result == EINTR);

//       if (sleep_result != 0)
//       {
//         std::cerr
//           << "clock_nanosleep failed: "
//           << std::strerror(sleep_result)
//           << '\n';

//         return 1;
//       }

//       ecrt_master_application_time(
//         master_,
//         timespec_to_ns(wake_time));

//       ecrt_master_receive(master_);

//       ecrt_domain_process(domain_);

//       bool all_initialized = true;
//       bool all_enabled = true;

//       for (std::size_t slave = 0;
//         slave < axes_.size();
//         ++slave)
//       {
//         auto & axis = axes_[slave];

//         if (!axis.configured)
//         {
//           continue;
//         }

//         axis.actual =
//           EC_READ_S32(
//             domain_data_ +
//             axis.actual_position);

//         axis.velocity_actual =
//           EC_READ_S32(
//             domain_data_ +
//             axis.actual_velocity);

//         axis.status =
//           EC_READ_U16(
//             domain_data_ +
//             axis.status_word);

//         axis.mode_actual =
//           EC_READ_S16(
//             domain_data_ +
//             axis.mode_display);

//         if (!axis.initialized &&
//           axis.status != 0)
//         {
//           axis.command_position =
//             static_cast<double>(axis.actual);

//           axis.goal_position =
//             axis.actual;

//           axis.command =
//             axis.actual;

//           axis.initialized = true;
//         }

//         all_initialized =
//           all_initialized &&
//           axis.initialized;

//         const bool enable_axis =
//           !options_.hold_op_all &&
//           options_.hold_op_slave < 0 &&
//           (options_.hold_csp_mask < 0 ||
//           (options_.hold_csp_mask &
//           (1 << slave)) != 0);

//         all_enabled =
//           all_enabled &&
//           (!enable_axis ||
//           (operation_enabled(axis.status) &&
//           axis.mode_actual == 8));

//         EC_WRITE_S16(
//           domain_data_ +
//           axis.mode,
//           8);

//         if (axis.initialized)
//         {
//           const double dt =
//             static_cast<double>(
//               options_.cycle_ns) /
//             1.0e9;

//           const double remaining =
//             static_cast<double>(
//               axis.goal_position) -
//             axis.command_position;

//           const double max_step =
//             axis.command_velocity * dt;

//           axis.command_position +=
//             std::clamp(
//               remaining,
//               -max_step,
//               max_step);

//           axis.command =
//             static_cast<std::int32_t>(
//               std::llround(
//                 axis.command_position));

//           EC_WRITE_S32(
//             domain_data_ +
//             axis.target_position,
//             axis.command);
//         }

//         EC_WRITE_U32(
//           domain_data_ +
//           axis.profile_velocity,
//           options_.velocity);

//         EC_WRITE_U32(
//           domain_data_ +
//           axis.profile_acceleration,
//           options_.acceleration);

//         EC_WRITE_U32(
//           domain_data_ +
//           axis.profile_deceleration,
//           options_.deceleration);

//         EC_WRITE_U16(
//           domain_data_ +
//           axis.control_word,
//           !enable_axis ?
//           0x0006 :
//           (axis.initialized ?
//           next_control_word(axis.status) :
//           0x0006));
//       }

//       if (options_.hold_op_all ||
//         options_.hold_op_slave >= 0)
//       {
//         all_enabled = all_initialized;
//       }

//       if (all_initialized &&
//         all_enabled)
//       {
//         ++enabled_cycles;
//       }
//       else
//       {
//         enabled_cycles = 0;
//       }

//       // ------------------------------------------------------
//       // CSP all 模式下的相对运动命令
//       // ------------------------------------------------------

//       if (options_.hold_csp_all &&
//         enabled_cycles >= 100 &&
//         (cycle % 100U) == 0U &&
//         std::filesystem::exists(
//           move_command_file(options_.master_id)))
//       {
//         const auto command_path =
//           move_command_file(options_.master_id);

//         std::ifstream command_file(
//           command_path);

//         std::string request_id;
//         int slave = -1;
//         double degrees = 0.0;
//         double speed_degrees_per_second = 0.0;
//         std::string extra;

//         const bool parsed =
//           static_cast<bool>(
//             command_file >>
//             request_id >>
//             slave >>
//             degrees >>
//             speed_degrees_per_second) &&
//           !(command_file >> extra);

//         command_file.close();

//         std::filesystem::remove(
//           command_path,
//           remove_error);

//         // Master1 只能访问 slave 0~5
//         const std::size_t actual_axis_count =
//           options_.master_id == 1 ?
//           kMaster1AxisCount :
//           kMaster0AxisCount;

//         if (!parsed ||
//           slave < 0 ||
//           slave >=
//           static_cast<int>(actual_axis_count) ||
//           !std::isfinite(degrees) ||
//           std::abs(degrees) > 45.0 ||
//           !std::isfinite(
//             speed_degrees_per_second) ||
//           speed_degrees_per_second <= 0.0 ||
//           speed_degrees_per_second > 90.0)
//         {
//           std::cout
//             << "move_command_rejected request="
//             << request_id
//             << " reason=invalid_command\n"
//             << std::flush;
//         }
//         else
//         {
//           auto & selected_axis =
//             axes_[
//               static_cast<std::size_t>(
//                 slave)];

//           const double delta =
//             joint_direction(
//               options_.master_id,
//               static_cast<unsigned int>(
//                 slave)) *
//             degrees *
//             static_cast<double>(
//               counts_per_revolution(
//                 static_cast<unsigned int>(
//                   slave))) /
//             360.0;

//           const auto target =
//             static_cast<std::int32_t>(
//               std::llround(
//                 static_cast<double>(
//                   selected_axis.actual) +
//                 delta));

//           selected_axis.goal_position =
//             target;

//           selected_axis.command_velocity =
//             speed_degrees_per_second *
//             static_cast<double>(
//               counts_per_revolution(
//                 static_cast<unsigned int>(
//                   slave))) /
//             360.0;

//           std::cout
//             << "move_command_applied request="
//             << request_id
//             << " master="
//             << options_.master_id
//             << " slave="
//             << slave
//             << " degrees="
//             << degrees
//             << " speed_deg_s="
//             << speed_degrees_per_second
//             << " start="
//             << selected_axis.actual
//             << " target="
//             << target
//             << '\n'
//             << std::flush;
//         }
//       }

//       // ------------------------------------------------------
//       // --move_slave 模式
//       // ------------------------------------------------------

//       if (options_.move_slave >= 0 &&
//         enabled_cycles >= 100)
//       {
//         auto & moving_axis =
//           axes_[
//             static_cast<std::size_t>(
//               options_.move_slave)];

//         if (!move_started)
//         {
//           const double delta =
//             joint_direction(
//               options_.master_id,
//               static_cast<unsigned int>(
//                 options_.move_slave)) *
//             options_.move_degrees *
//             static_cast<double>(
//               counts_per_revolution(
//                 static_cast<unsigned int>(
//                   options_.move_slave))) /
//             360.0;

//           const auto move_target =
//             static_cast<std::int32_t>(
//               std::llround(
//                 static_cast<double>(
//                   moving_axis.actual) +
//                 delta));

//           moving_axis.goal_position =
//             move_target;

//           moving_axis.command_velocity =
//             static_cast<double>(
//               options_.velocity);

//           move_started = true;

//           std::cout
//             << "move_axis phase=4 target="
//             << move_target
//             << " start="
//             << moving_axis.actual
//             << " slave="
//             << options_.move_slave
//             << '\n'
//             << "Enter another relative move as: "
//                "slave(0-6) degrees([-45,45])\n"
//             << std::flush;
//         }

//         if ((cycle % 100U) == 0U)
//         {
//           pollfd input{};

//           input.fd = STDIN_FILENO;
//           input.events = POLLIN;

//           if (poll(
//               &input,
//               1,
//               0) > 0 &&
//             (input.revents & POLLIN) != 0)
//           {
//             std::string line;

//             if (std::getline(
//                 std::cin,
//                 line))
//             {
//               std::istringstream stream(line);

//               int slave = -1;
//               double degrees = 0.0;
//               std::string extra;

//               const std::size_t actual_axis_count =
//                 options_.master_id == 1 ?
//                 kMaster1AxisCount :
//                 kMaster0AxisCount;

//               if (!(stream >> slave >> degrees) ||
//                 (stream >> extra) ||
//                 slave < 0 ||
//                 slave >=
//                 static_cast<int>(
//                   actual_axis_count) ||
//                 !std::isfinite(degrees) ||
//                 std::abs(degrees) > 45.0)
//               {
//                 if (options_.master_id == 1)
//                 {
//                   std::cout
//                     << "invalid move; use: "
//                        "slave(0-5) degrees([-45,45])\n"
//                     << std::flush;
//                 }
//                 else
//                 {
//                   std::cout
//                     << "invalid move; use: "
//                        "slave(0-6) degrees([-45,45])\n"
//                     << std::flush;
//                 }
//               }
//               else
//               {
//                 auto & selected_axis =
//                   axes_[
//                     static_cast<std::size_t>(
//                       slave)];

//                 const double delta =
//                   joint_direction(
//                     options_.master_id,
//                     static_cast<unsigned int>(
//                       slave)) *
//                   degrees *
//                   static_cast<double>(
//                     counts_per_revolution(
//                       static_cast<unsigned int>(
//                         slave))) /
//                   360.0;

//                 const auto target =
//                   static_cast<std::int32_t>(
//                     std::llround(
//                       static_cast<double>(
//                         selected_axis.actual) +
//                       delta));

//                 selected_axis.goal_position =
//                   target;

//                 selected_axis.command_velocity =
//                   static_cast<double>(
//                     options_.velocity);

//                 std::cout
//                   << "move_axis target="
//                   << target
//                   << " start="
//                   << selected_axis.actual
//                   << " slave="
//                   << slave
//                   << " degrees="
//                   << degrees
//                   << '\n'
//                   << std::flush;
//               }
//             }
//           }
//         }
//       }

//       if ((cycle % options_.print_every) == 0U)
//       {
//         print_snapshot(
//           options_.master_id,
//           cycle,
//           axes_);

//         if (!all_initialized)
//         {
//           std::cout
//             << "waiting_for_valid_pdo_feedback\n"
//             << std::flush;
//         }
//       }

//       ecrt_master_sync_reference_clock(master_);
//       ecrt_master_sync_slave_clocks(master_);

//       ecrt_domain_queue(domain_);
//       ecrt_master_send(master_);

//       ++cycle;

//       add_ns(
//         wake_time,
//         options_.cycle_ns);

//       timespec now{};

//       clock_gettime(
//         CLOCK_MONOTONIC,
//         &now);

//       if (timespec_to_ns(now) >
//         timespec_to_ns(wake_time) +
//         static_cast<std::uint64_t>(
//           options_.cycle_ns))
//       {
//         wake_time = now;

//         add_ns(
//           wake_time,
//           options_.cycle_ns);
//       }
//     }

//     send_safe_disable();

//     std::filesystem::remove(
//       stop_file(options_.master_id),
//       remove_error);

//     return 0;
//   }

// private:

//   void send_safe_disable()
//   {
//     if (!activated_ ||
//       domain_data_ == nullptr)
//     {
//       return;
//     }

//     for (int cycle = 0;
//       cycle < 20;
//       ++cycle)
//     {
//       ecrt_master_receive(master_);

//       ecrt_domain_process(domain_);

//       for (auto & axis : axes_)
//       {
//         if (!axis.configured)
//         {
//           continue;
//         }

//         if (axis.initialized)
//         {
//           EC_WRITE_S32(
//             domain_data_ +
//             axis.target_position,
//             axis.actual);
//         }

//         EC_WRITE_U16(
//           domain_data_ +
//           axis.control_word,
//           0x0006);
//       }

//       ecrt_domain_queue(domain_);
//       ecrt_master_send(master_);

//       usleep(1000);
//     }
//   }

//   void shutdown()
//   {
//     if (master_ != nullptr)
//     {
//       if (activated_)
//       {
//         ecrt_master_deactivate(master_);
//       }

//       ecrt_release_master(master_);
//     }

//     master_ = nullptr;
//     domain_ = nullptr;
//     domain_data_ = nullptr;
//     activated_ = false;
//   }

//   Options options_;

//   ec_master_t * master_{nullptr};
//   ec_domain_t * domain_{nullptr};
//   std::uint8_t * domain_data_{nullptr};

//   // 最大 7 个，Master1 实际只使用前 6 个
//   std::array<
//     ec_slave_config_t *,
//     kAxisCount> slave_configs_{};

//   std::array<
//     Axis,
//     kAxisCount> axes_{};

//   bool activated_{false};
// };

// }  // namespace

// int main(int argc, char ** argv)
// {
//   try
//   {
//     const Options options =
//       parse_options(argc, argv);

//     std::signal(
//       SIGINT,
//       signal_handler);

//     std::signal(
//       SIGTERM,
//       signal_handler);

//     DirectMaster master(options);

//     master.configure();

//     return master.run();
//   }
//   catch (const std::exception & exception)
//   {
//     std::cerr
//       << "ethercat_direct_tool: "
//       << exception.what()
//       << '\n';

//     print_usage(argv[0]);

//     return 1;
//   }
// }
