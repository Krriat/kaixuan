#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ecrt.h>
#include "kaixuan_can_hardware/startup_guard.hpp"
#include "kaixuan_can_hardware/dc_startup_reset.hpp"
#include "kaixuan_can_hardware/dc_reference_follower.hpp"
#include "kaixuan_can_hardware/dc_reference_trace.hpp"
#include "kaixuan_can_hardware/torque_safety.hpp"
#include "kaixuan_can_hardware/joint_angle_mapping.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "kaixuan_can_hardware/cycle_capture.hpp"
#include "kaixuan_can_hardware/bus_probe.hpp"

namespace kaixuan_can_hardware
{

class KaixuanCanSystem final : public hardware_interface::SystemInterface
{
public:
  KaixuanCanSystem() = default;
  ~KaixuanCanSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kJointsPerArm = 7;
  static constexpr std::size_t kNumJoints = 14;
  static constexpr std::size_t kMasterCount = 2;
  static constexpr std::size_t kMaxSlavesPerMaster = 8;
  static constexpr std::size_t kO6JointCount = 6;
  static constexpr std::size_t kO6PressureFingerCount = 5;
  static constexpr std::size_t kO6PressureRows = 10;
  static constexpr std::size_t kO6PressureColumns = 4;
  static constexpr std::size_t kO6PressurePointCount =
    kO6PressureFingerCount * kO6PressureRows * kO6PressureColumns;
  static constexpr std::uint32_t kVendorId = 0x00010203;
  static constexpr std::uint32_t kProductCode = 0x00000402;
  static constexpr std::uint32_t kRevision = 0x00050012;
  static constexpr std::uint32_t kMotorCountsPerRevolution = 1048576U;
  static constexpr std::uint16_t kDcAssignActivate = 0x0300;
  static constexpr std::uint32_t kO6VendorId = 0x00000000;
  static constexpr std::uint32_t kO6ProductCode = 0x00000601;
  static constexpr std::uint32_t kO6Revision = 0x00010001;
  static constexpr std::uint16_t kO6DcAssignActivate = 0x0300;

  enum class Cia402State
  {
    kNotReadyToSwitchOn,
    kSwitchOnDisabled,
    kReadyToSwitchOn,
    kSwitchedOn,
    kOperationEnabled,
    kQuickStopActive,
    kFaultReactionActive,
    kFault,
    kUnknown,
  };

  struct AxisHandle
  {
    std::size_t joint_index{0};
    std::uint16_t slave_pos{0};
    int control_word{-1};
    int status_word{-1};
    int mode_of_operation{-1};
    int target_position{-1};
    int target_torque{-1};
    int mode_display{-1};
    int position_actual_value{-1};
    int velocity_actual_value{-1};
    int torque_actual_value{-1};
    int error_code{-1};
    unsigned torque_feedback_unit{0};
    bool velocity_counts_per_second{false};
    double rated_current_a{0.0};
    EnableSettle enable_settle{};
  };

  enum class O6HandshakeState : std::uint8_t
  {
    kDisabled = 0,
    kEnableBaseline = 1,
    kWaitBaselineAck = 2,
    kReady = 3,
    kWaitCommandAck = 4,
  };

  struct O6Handle
  {
    ec_slave_config_t * slave_config{nullptr};
    int rx_base{-1};
    int tx_base{-1};
    bool configured{false};
    O6HandshakeState handshake_state{O6HandshakeState::kDisabled};
    std::array<std::uint8_t, kO6JointCount> output_target{{128, 128, 128, 128, 128, 128}};
    std::array<std::uint8_t, kO6JointCount> baseline_target{};
    std::array<std::uint8_t, kO6JointCount> pending_target{};
    std::uint16_t command_sequence{0};
    std::uint64_t disable_cycles{0};
    std::uint64_t baseline_cycles{0};
    std::uint64_t sent_generation{0};
    std::uint64_t pending_generation{0};
    bool sequence_initialized{false};
  };

  struct O6Command
  {
    bool enable_requested{false};
    std::array<std::uint8_t, kO6JointCount> target{{128, 128, 128, 128, 128, 128}};
    std::uint64_t generation{0};
    std::uint64_t update_time_ns{0};
  };

  struct O6Snapshot
  {
    bool present{false};
    bool operational{false};
    bool working_counter_ok{false};
    bool safe_inputs{false};
    bool ready{false};
    bool pressure_valid{false};
    bool imu_valid{false};
    std::uint16_t status_word{0};
    std::uint16_t applied_sequence{0};
    std::uint16_t position_sequence{0};
    std::uint16_t command_sequence{0};
    std::uint16_t can_tx_count{0};
    std::uint16_t can_rx_count{0};
    std::uint8_t valid_mask{0};
    std::uint8_t active_can_id{0};
    std::uint8_t can_error_count{0};
    std::uint8_t timeout_count{0};
    std::uint8_t handshake_state{0};
    std::uint16_t pressure_sequence{0};
    std::uint8_t finger_valid_mask{0};
    std::uint8_t pressure_state{0};
    std::uint16_t pressure_age_ms{0};
    std::uint16_t imu_sequence{0};
    std::uint16_t imu_status{0};
    std::int16_t accel_x_raw{0};
    std::int16_t accel_y_raw{0};
    std::int16_t accel_z_raw{0};
    std::int16_t gyro_x_raw{0};
    std::int16_t gyro_y_raw{0};
    std::int16_t gyro_z_raw{0};
    std::int16_t imu_temperature_centi_c{0};
    std::uint16_t imu_sample_age_ms{0};
    double accel_x_mps2{0.0};
    double accel_y_mps2{0.0};
    double accel_z_mps2{0.0};
    double gyro_x_rad_s{0.0};
    double gyro_y_rad_s{0.0};
    double gyro_z_rad_s{0.0};
    double imu_temperature_c{0.0};
    std::uint32_t working_counter{0};
    std::uint32_t wc_state{0};
    std::uint32_t al_state{0};
    std::uint64_t cycle_count{0};
    std::uint64_t queue_count{0};
    std::uint64_t send_count{0};
    std::uint64_t max_cycle_lateness_ns{0};
    std::uint64_t max_loop_interval_ns{0};
    std::uint64_t last_send_time_ns{0};
    std::uint64_t last_complete_send_time_ns{0};
    std::array<std::uint8_t, kO6JointCount> actual_position{};
    std::array<std::uint8_t, kO6JointCount> temperature{};
    std::array<std::uint8_t, kO6JointCount> joint_error{};
    std::array<std::uint8_t, kO6PressurePointCount> pressure{};
  };

  struct O6RosInterface
  {
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::thread thread;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr target_subscription;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr actual_publisher;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr temperature_publisher;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr error_publisher;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pressure_publisher;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr status_publisher;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pressure_status_publisher;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr imu_raw_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr imu_si_publisher;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr imu_status_publisher;
    rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr realtime_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr online_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pressure_valid_publisher;
    rclcpp::TimerBase::SharedPtr publish_timer;
  };

  struct MasterContext
  {
    unsigned int master_id{0};
    int affinity{0};
    ec_master_t * master{nullptr};
    ec_domain_t * domain{nullptr};
    std::uint8_t * domain_data{nullptr};
    std::array<ec_slave_config_t *, kMaxSlavesPerMaster> slave_configs{};
    std::array<AxisHandle, kJointsPerArm> axes{};
    O6Handle o6{};
    std::thread cyclic_thread;
    std::atomic<bool> running{false};
    std::atomic<bool> thread_ready{false};
    std::atomic<bool> thread_failed{false};
    std::uint64_t cycle_count{0};
    std::uint64_t queue_count{0};
    std::uint64_t send_count{0};
    std::uint64_t last_cycle_time_ns{0};
    std::uint64_t max_cycle_lateness_ns{0};
    std::uint64_t max_loop_interval_ns{0};
    std::uint64_t last_send_time_ns{0};
    std::uint64_t last_complete_send_time_ns{0};
    bool master_activated{false};
    std::uint64_t scheduled_time_ns{0}, current_lateness_ns{0}, current_interval_ns{0};
    std::uint64_t previous_send_duration_ns{0};
    int previous_send_result{0};
    std::array<ec_reg_request_t *, kJointsPerArm> probe_registers{};
    std::array<ec_sdo_request_t *, kJointsPerArm> probe_sdos{};
    BusProbe probe{};
    std::size_t probe_active_slave{0}, probe_expected_size{0};
    unsigned probe_active_kind{0};
    bool probe_bus_ready{false};
    std::uint64_t max_probe_work_ns{0};
    std::uint64_t loop_exit_ns{0}, final_receive_ns{0};
    StartupGuard startup_guard{};
    ec_reg_request_t * dc_reset_request{nullptr};
    DcStartupReset dc_reset{};
    DcStartupReset::Status dc_reset_status{};
    DcStartupReset::Action dc_reset_action{};
    bool dc_reset_queue_rejected{false};
    DcReferenceFollower::Config dc_follower_config{};
    DcReferenceFollower dc_follower{};
    DcReferenceFollower::Output dc_follower_status{};
    std::unique_ptr<DcReferenceTraceRing> dc_trace;
    std::string dc_trace_directory;
    DcReferenceTraceRing::RawBracket reference_sample_raw{};
    DcReferenceTraceRing::Row dc_trace_row{};
    bool dc_trace_dump_attempted{false};
    bool reference_sample_pending{false}, follower_schedule_failed{false};
    bool follower_getter_failed{false};
    unsigned int follower_getter_consecutive_errors{0};
    std::uint64_t reference_sample_virtual_ns{0}, reference_sample_mono_ns{0};
    std::uint64_t reference_sample_host_ns{0};  // CLOCK_MONOTONIC_RAW in reference mode.
    std::uint64_t reference_sample_cycle{0}, reference_getter_retry_cycle{0};
    std::uint64_t reference_getter_errors{0}, follower_skipped_periods{0};
    std::uint32_t follower_last_ref32{0};
    std::uint64_t follower_failure_cycle{0}, follower_reject_cycle{0};
    std::int64_t follower_failure_residual_ns{0}, follower_reject_residual_ns{0};
    unsigned int follower_reject_reason{0};
    std::uint64_t follower_ref_received_cycle{0}, follower_calibration_cycle{0};
    std::int64_t follower_calibration_delta_ns{0};
    bool dc_monitor_pending{false}, dc_sample_updated{false};
    std::uint32_t dc_difference_ns{UINT32_MAX};
    std::uint64_t dc_pending_queue_cycle{0}, dc_pending_reference_age_cycles{UINT64_MAX};
    std::uint64_t dc_sample_queue_cycle{0}, dc_sample_reference_age_cycles{UINT64_MAX};
    std::array<std::uint64_t, 9> timing_max_ns{}, timing_window_max_ns{}, timing_completed_max_ns{};
    std::uint64_t timing_window_start_ns{0}, timing_window_cycles{0};
    std::uint64_t timing_completed_start_ns{0}, timing_completed_end_ns{0}, timing_completed_cycles{0};
    std::uint64_t timing_mutex_wait_ns{0}, slow_send_cycles{0}, last_slow_send_ns{0};
    std::uint64_t sync_errors{0}, schedule_rebases{0};
    std::uint64_t reference_sync_attempts{0}, reference_sync_queued{0}, last_reference_sync_cycle{0};
    std::uint64_t timing_reference_sync_cycles{0}, timing_completed_reference_sync_cycles{0};
  };

  // Fixed-size snapshots only; no ROS allocation or disk I/O in the cyclic loop.
  struct AxisDiagnostic
  {
    std::int32_t actual_counts{0}, target_counts{0}, velocity_raw{0};
    std::int16_t torque_raw{0};
    std::uint16_t status_word{0}, control_word{0}, error_code{0}, last_error_code{0};
    std::uint64_t fault_cycles{0};
    double position_rad{0}, velocity_rad_s{0};
    double effort_nm{0}, commanded_effort_nm{0};
    std::int16_t target_torque_raw{0};
    int actual_mode{0};
    unsigned torque_guard_fault{0};
    TorqueSafety::FaultSnapshot torque_fault_snapshot{};
    bool effort_claimed{false}, torque_armed{false};
    bool following_error{false};
  };
  struct MasterDiagnostic
  {
    std::uint64_t sample_steady_ns{0}, cycle_count{0}, bad_wc_cycles{0};
    std::uint64_t max_cycle_lateness_ns{0}, max_loop_interval_ns{0};
    std::uint64_t max_probe_work_ns{0};
    unsigned int working_counter{0}, wc_state{0};
    bool startup_ready{false};
    bool dc_reset_ready{true}, dc_reset_failed{false};
    unsigned int dc_reset_phase{0}, dc_reset_error{0};
    std::uint16_t dc_reset_original_bandwidth{0};
    std::uint8_t dc_reset_original_filter{0};
    std::uint64_t dc_reset_observation_start_ns{0};
    DcReferenceFollower::Config dc_follower_config{};
    DcReferenceFollower::Output dc_follower{};
    bool dc_follower_ready{true}, dc_follower_failed{false}, follower_schedule_failed{false};
    bool follower_getter_failed{false};
    unsigned int follower_getter_consecutive_errors{0};
    std::uint64_t reference_getter_errors{0}, follower_skipped_periods{0};
    std::uint64_t reference_sample_cycle{0};
    std::uint32_t follower_last_ref32{0};
    std::uint64_t follower_failure_cycle{0}, follower_reject_cycle{0};
    std::int64_t follower_failure_residual_ns{0}, follower_reject_residual_ns{0};
    unsigned int follower_reject_reason{0};
    std::uint64_t follower_ref_received_cycle{0}, follower_calibration_cycle{0};
    std::int64_t follower_calibration_delta_ns{0};
    unsigned int startup_bus_cycles{0}, startup_dc_samples{0};
    std::uint32_t dc_difference_ns{UINT32_MAX};
    std::uint64_t dc_sample_queue_cycle{0}, dc_sample_reference_age_cycles{UINT64_MAX};
    std::array<std::uint64_t, 9> timing_max_ns{}, timing_window_max_ns{};
    std::uint64_t timing_window_start_ns{0}, timing_window_end_ns{0}, timing_window_cycles{0};
    std::uint64_t slow_send_cycles{0}, last_slow_send_ns{0}, sync_errors{0}, schedule_rebases{0};
    std::uint64_t reference_sync_attempts{0}, reference_sync_queued{0}, last_reference_sync_cycle{0};
    std::uint64_t timing_reference_sync_cycles{0};
    std::array<AxisDiagnostic, kJointsPerArm> axes{};
  };
  std::array<MasterDiagnostic, kMasterCount> diagnostic_snapshots_{};
  std::array<CycleCapture<>, kMasterCount> cycle_captures_{};
  std::array<CycleCapture<128>, kMasterCount> bus_captures_{};
  rclcpp::Node::SharedPtr diagnostic_node_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr cycle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr bus_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> diagnostic_executor_;
  std::thread diagnostic_thread_;
  unsigned int diagnostic_divider_{0};
  std::uint64_t diagnostic_snapshot_retries_{0};
  std::uint64_t diagnostic_snapshot_drops_{0};
  void start_diagnostics();
  void stop_diagnostics();
  void publish_diagnostics();
  void publish_cycle_capture();
  void publish_bus_capture();
  void poll_bus_diagnostics(MasterContext & context, std::uint64_t now_ns);

  bool open_ethercat();
  void close_ethercat();
  bool configure_master(MasterContext & context);
  bool configure_axis(
    MasterContext & context, std::uint16_t slave_pos, std::size_t joint_index);
  bool configure_o6(MasterContext & context);
  void cyclic_loop(MasterContext & context);
  void exchange_process_data(MasterContext & context, const ec_domain_state_t & domain_state);
  void exchange_o6_process_data(
    MasterContext & context, const ec_domain_state_t & domain_state,
    const ec_slave_config_state_t & slave_state, std::uint64_t now_ns);
  void write_o6_frame(
    MasterContext & context, std::uint16_t control_word);
  void reset_o6_handshake(MasterContext & context, bool keep_sequence = true);
  void send_o6_disable_cycles(MasterContext & context, timespec & wake_time);
  bool configure_realtime_thread(const MasterContext & context);
  bool start_o6_ros_interfaces();
  void stop_o6_ros_interfaces();
  void publish_o6_status(std::size_t master_index);
  void receive_o6_target(
    std::size_t master_index,
    const std_msgs::msg::UInt8MultiArray::SharedPtr message);
  void set_o6_enabled(
    std::size_t master_index,
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
  static const char * o6_namespace(std::size_t master_index);
  static const char * o6_hand_name(std::size_t master_index);

  static Cia402State decode_cia402_state(std::uint16_t status_word);
  static double degrees_to_radians(double deg);
  static double radians_to_degrees(double rad);
  static std::uint64_t timespec_to_ns(const timespec & value);
  static std::uint64_t steady_now_ns();
  static void add_ns(timespec & value, std::int64_t nanoseconds);

  double motor_rpm_to_joint_radians_per_second(
    std::int32_t motor_rpm, std::size_t joint_index) const;
  double joint_counts_to_radians(
    std::int32_t counts, std::size_t joint_index) const;
  std::int32_t radians_to_joint_counts(
    double radians, std::size_t joint_index) const;
  void initialize_runtime();
  bool master_is_active(std::size_t master_index) const;
  bool joint_is_active(std::size_t joint_index) const;
  bool all_joint_positions_ready() const;
  void reset_runtime_state();

  rclcpp::Logger logger_{rclcpp::get_logger("kaixuan_can_hardware")};
  rclcpp::Clock clock_{RCL_STEADY_TIME};

  std::array<MasterContext, kMasterCount> masters_{};

  // 1 kHz EtherCAT cycle: 1 ms.
  std::int64_t cycle_time_ns_{1000000};
  std::int64_t shift_time_ns_{500000};
  int priority_{90};
  std::string active_arm_{"both"};
  bool bus_diagnostics_{false};
  bool fresh_reference_time_{true};
  std::uint32_t reference_sync_cycles_{1};
  bool dc_reset_after_configuration_{false};
  bool follow_reference_clock_{true};
  void poll_dc_startup_reset(MasterContext &, const ec_domain_state_t &, std::uint64_t);
  void poll_dc_reference_follower(MasterContext &, const ec_domain_state_t &, std::uint64_t);
  void dump_dc_reference_trace(MasterContext &);
  std::uint64_t deactivate_requested_ns_{0};
  std::int64_t o6_command_timeout_ns_{0};
  std::uint8_t o6_max_step_{255};
  bool o6_imu_only_{false};

  bool cst_mode_{false};
  std::uint16_t cst_disabled_control_word_{6};
  std::uint16_t disabled_control_word() const {return cst_mode_ ? cst_disabled_control_word_ : 6;}
  bool stop_cst_group_on_fault_{false};
  std::array<bool, kNumJoints> cst_selected_{}, effort_claimed_{};
  std::array<TorqueParameters, kNumJoints> torque_parameters_{};
  std::array<TorqueSafety, kNumJoints> torque_safety_{};
  // Accessed only by the non-real-time diagnostics publisher.
  std::array<unsigned, kNumJoints> logged_torque_fault_{};
  std::vector<std::string> joint_command_names_;
  std::array<double, kNumJoints> requested_efforts_{}, measured_efforts_{};
  std::array<double, kNumJoints> measured_positions_{}, measured_velocities_{};
  std::uint64_t effort_write_ns_{0}, effort_write_timeout_ns_{100000000};

  // ROS units: position in rad, velocity in rad/s, effort in output-joint N.m.
  // hw_commands_ contains position in CSP and effort in CST.
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_;
  std::vector<double> joint_offset_deg_;
  std::vector<double> joint_direction_;
  std::vector<double> joint_gear_ratio_;
  std::vector<std::int32_t> commanded_counts_;
  std::array<std::int32_t, kNumJoints> actual_counts_{};
  std::vector<bool> position_initialized_;
  std::mutex io_mutex_;

  std::array<O6Command, kMasterCount> o6_commands_{};
  std::array<O6Snapshot, kMasterCount> o6_snapshots_{};
  std::mutex o6_mutex_;
  std::array<O6RosInterface, kMasterCount> o6_ros_interfaces_{};

  bool configured_{false};
  bool activated_{false};
  bool runtime_initialized_{false};
};

}  // namespace kaixuan_can_hardware
