#include "modbus_ros2_control/hands/inspire/inspire_hand_hardware.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <limits>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace modbus_ros2_control
{
namespace
{
constexpr auto kLoggerName = "InspireHandHardware";

bool has_suffix(const std::string& value, const std::string& suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}  // namespace

hardware_interface::CallbackReturn InspireHandHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJointCount)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Inspire hand hardware expects exactly %zu joints, got %zu",
      kJointCount,
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.reserve(kJointCount);
  for (const auto& joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  if (!validate_joint_interfaces())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  load_parameters();
  upper_limits_ = default_upper_limits(joint_names_);

  if (read_tactile_enabled_)
  {
    for (const auto i : tactile_scan_regions_)
    {
      tactile_publishers_[i] = get_node()->create_publisher<sensor_msgs::msg::Image>(
        std::string("/hand/tactile/") + rh56e2_protocol::kTactileRegions[i].name,
        rclcpp::SensorDataQoS());
    }
    RCLCPP_INFO(rclcpp::get_logger(kLoggerName),
      "T1 tactile enabled: %zu regions, up to 96 words per %d ms (raw 0..4095)",
      tactile_scan_regions_.size(), tactile_period_ms_);
  }

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_positions_[i] = std::clamp(
      initial_value_for_joint(info_.joints[i]), lower_limits_[i], upper_limits_[i]);
    previous_positions_[i] = hw_positions_[i];
    hw_commands_[i] = hw_positions_[i];
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    feedback_positions_[i] = hw_positions_[i];
    feedback_efforts_[i] = 0.0;
  }

  last_command_values_ = joints_to_actuators(hw_commands_);
  pending_command_values_ = last_command_values_;

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Configured Inspire Modbus RTU hand: port=%s, baudrate=%d, id=%d, feedback=%s",
    serial_port_.c_str(),
    baudrate_,
    slave_id_,
    read_feedback_enabled_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn InspireHandHardware::on_activate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  if (!connect_modbus())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!initialize_hand())
  {
    disconnect_modbus();
    return hardware_interface::CallbackReturn::ERROR;
  }

  command_sent_ = false;
  pending_command_valid_ = false;
  pending_command_dirty_ = false;
  feedback_valid_ = false;

  std::array<double, kJointCount> initial_positions{};
  std::array<double, kJointCount> initial_efforts{};
  if (read_feedback_enabled_ && read_feedback(initial_positions, initial_efforts))
  {
    for (std::size_t i = 0; i < kJointCount; ++i)
    {
      hw_positions_[i] = initial_positions[i];
      hw_commands_[i] = hw_positions_[i];
      feedback_positions_[i] = initial_positions[i];
      feedback_efforts_[i] = initial_efforts[i];
    }
    feedback_valid_ = true;
  }

  last_command_values_ = joints_to_actuators(hw_commands_);
  pending_command_values_ = last_command_values_;
  tactile_region_index_ = 0;
  tactile_word_offset_ = 0;
  next_tactile_read_ = std::chrono::steady_clock::now();
  start_background_thread();

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Inspire Modbus RTU hand activated on %s",
    serial_port_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn InspireHandHardware::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  stop_background_thread();
  disconnect_modbus();
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Inspire Modbus RTU hand deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
InspireHandHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  state_interfaces.reserve(kJointCount * 3);

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
InspireHandHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  command_interfaces.reserve(kJointCount);

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    command_interfaces.push_back(
      std::make_shared<hardware_interface::CommandInterface>(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type InspireHandHardware::read(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& period)
{
  if (!modbus_ || !modbus_->isConnected())
  {
    return hardware_interface::return_type::ERROR;
  }

  if (background_running_.load())
  {
    update_background_period(period);
  }

  previous_positions_ = hw_positions_;

  if (read_feedback_enabled_)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (feedback_valid_)
    {
      hw_positions_ = feedback_positions_;
      hw_efforts_ = feedback_efforts_;
    }
  }
  else
  {
    hw_positions_ = hw_commands_;
    hw_efforts_.fill(0.0);
  }

  const double dt = period.seconds();
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_velocities_[i] = dt > std::numeric_limits<double>::epsilon()
                          ? (hw_positions_[i] - previous_positions_[i]) / dt
                          : 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type InspireHandHardware::write(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& /* period */)
{
  if (!modbus_ || !modbus_->isConnected())
  {
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    if (!std::isfinite(hw_commands_[i]))
    {
      return hardware_interface::return_type::ERROR;
    }
    hw_commands_[i] = std::clamp(hw_commands_[i], lower_limits_[i], upper_limits_[i]);
  }

  const auto actuator_values = joints_to_actuators(hw_commands_);
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (command_sent_ && !command_changed(actuator_values))
    {
      return hardware_interface::return_type::OK;
    }

    pending_command_values_ = actuator_values;
    pending_command_valid_ = true;
    pending_command_dirty_ = true;
  }

  return hardware_interface::return_type::OK;
}

void InspireHandHardware::load_parameters()
{
  const auto get_parameter = [this](const std::string& name, const std::string& fallback) {
    const auto it = info_.hardware_parameters.find(name);
    return it == info_.hardware_parameters.end() ? fallback : it->second;
  };

  serial_port_ = get_parameter("serial_port", serial_port_);
  hand_side_ = get_parameter("hand_side", hand_side_);
  baudrate_ = parse_int(get_parameter("baudrate", std::to_string(baudrate_)), baudrate_);
  background_period_ms_ = std::max(
    1, parse_int(get_parameter("background_period_ms", std::to_string(background_period_ms_)),
                 background_period_ms_));
  command_deadband_raw_ = std::clamp(
    parse_int(get_parameter("command_deadband_raw", std::to_string(command_deadband_raw_)),
              command_deadband_raw_),
    0,
    1000);
  default_speed_ = static_cast<uint16_t>(std::clamp(
    parse_int(get_parameter("default_speed", std::to_string(default_speed_)), default_speed_),
    0,
    static_cast<int>(rh56e2_protocol::kMaximumSpeed)));
  default_force_ = static_cast<uint16_t>(std::clamp(
    parse_int(get_parameter("default_force", std::to_string(default_force_)), default_force_),
    0,
    static_cast<int>(rh56e2_protocol::kMaximumForce)));
  read_feedback_enabled_ = parse_bool(
    get_parameter("read_feedback", read_feedback_enabled_ ? "true" : "false"),
    read_feedback_enabled_);

  read_tactile_enabled_ = parse_bool(get_parameter("read_tactile", "false"), false);
  tactile_period_ms_ = std::max(0,
    parse_int(get_parameter("tactile_period_ms", "0"), 0));
  tactile_feedback_period_ms_ = std::clamp(
    parse_int(get_parameter("tactile_feedback_period_ms", "100"), 100), 20, 200);
  tactile_scan_regions_.clear();
  // CSV selection, or "all" for the original 17-region scan.
  const auto selection = get_parameter("tactile_regions", "all");
  if (selection == "all") {
    for (std::size_t i = 0; i < rh56e2_protocol::kTactileRegions.size(); ++i) {
      tactile_scan_regions_.push_back(i);
    }
  } else {
    std::istringstream stream(selection);
    std::string name;
    while (std::getline(stream, name, ',')) {
      name.erase(std::remove_if(name.begin(), name.end(), [](unsigned char c) {
        return std::isspace(c);
      }), name.end());
      bool found = false;
      for (std::size_t i = 0; i < rh56e2_protocol::kTactileRegions.size(); ++i) {
        if (name == rh56e2_protocol::kTactileRegions[i].name) {
          if (std::find(tactile_scan_regions_.begin(), tactile_scan_regions_.end(), i) ==
              tactile_scan_regions_.end()) {tactile_scan_regions_.push_back(i);}
          found = true;
          break;
        }
      }
      if (!found) {throw std::invalid_argument("Unknown T1 tactile region: " + name);}
    }
    if (tactile_scan_regions_.empty()) {
      throw std::invalid_argument("tactile_regions must contain at least one region");
    }
  }

  // The manual specifies factory ID 1, independently of left/right hand.
  // A hand reconfigured to another ID must supply an explicit slave_id.
  const auto default_slave_id = rh56e2_protocol::kDefaultSlaveId;
  const auto slave_id_value = get_parameter("slave_id", "auto");
  if (slave_id_value == "auto" || slave_id_value == "AUTO" || slave_id_value.empty())
  {
    slave_id_ = default_slave_id;
  }
  else
  {
    slave_id_ = std::clamp(parse_int(slave_id_value, default_slave_id), 1, 254);
  }
}

bool InspireHandHardware::validate_joint_interfaces() const
{
  for (const auto& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must expose exactly one position command interface",
        joint.name.c_str());
      return false;
    }

    const auto has_state_interface = [&joint](const std::string& interface_name) {
      return std::any_of(
        joint.state_interfaces.begin(),
        joint.state_interfaces.end(),
        [&interface_name](const auto& interface) { return interface.name == interface_name; });
    };

    if (!has_state_interface(hardware_interface::HW_IF_POSITION) ||
        !has_state_interface(hardware_interface::HW_IF_VELOCITY) ||
        !has_state_interface(hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must expose position, velocity, and effort state interfaces",
        joint.name.c_str());
      return false;
    }
  }

  return true;
}

bool InspireHandHardware::connect_modbus()
{
  disconnect_modbus();

  modbus_ = std::make_unique<ModbusRtuCommunicator>(
    serial_port_,
    static_cast<uint32_t>(baudrate_),
    slave_id_,
    'N',
    8,
    1);

  if (!modbus_->connect())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to connect Inspire Modbus RTU hand: %s",
      modbus_->getLastError().c_str());
    modbus_.reset();
    return false;
  }

  return true;
}

void InspireHandHardware::disconnect_modbus()
{
  if (modbus_)
  {
    modbus_->disconnect();
    modbus_.reset();
  }
}

void InspireHandHardware::start_background_thread()
{
  if (background_running_.load())
  {
    return;
  }

  background_running_.store(true);
  background_thread_ = std::thread(&InspireHandHardware::background_loop, this);
}

void InspireHandHardware::stop_background_thread()
{
  background_running_.store(false);
  if (background_thread_.joinable())
  {
    background_thread_.join();
  }
}

void InspireHandHardware::update_background_period(const rclcpp::Duration& period)
{
  if (background_period_initialized_.load())
  {
    return;
  }

  int period_ms = static_cast<int>(period.seconds() * 1000.0);
  if (period_ms < 10)
  {
    period_ms = 10;
  }

  background_period_ms_ = period_ms;
  background_period_initialized_.store(true);

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Inspire Modbus RTU background loop configured: %d ms (%.1f Hz)",
    background_period_ms_,
    1000.0 / static_cast<double>(background_period_ms_));
}

void InspireHandHardware::background_loop()
{
  auto next_feedback_read = std::chrono::steady_clock::now();
  while (background_running_.load())
  {
    const auto loop_start = std::chrono::steady_clock::now();

    // Tactile and feedback share one serial connection. Avoid two feedback
    // transactions before every tactile region; command writes still run each loop.
    if (read_feedback_enabled_ &&
        (!read_tactile_enabled_ || loop_start >= next_feedback_read))
    {
      next_feedback_read = loop_start +
        std::chrono::milliseconds(tactile_feedback_period_ms_);
      std::array<double, kJointCount> positions{};
      std::array<double, kJointCount> efforts{};
      if (read_feedback(positions, efforts))
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        feedback_positions_ = positions;
        feedback_efforts_ = efforts;
        feedback_valid_ = true;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger(kLoggerName),
          *get_node()->get_clock(),
          2000,
          "No valid Inspire Modbus RTU feedback received from ID %d",
          slave_id_);
      }
    }

    std::array<uint16_t, kActuatorCount> actuator_values{};
    bool should_send = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (pending_command_valid_ && pending_command_dirty_ &&
          (!command_sent_ || command_changed(pending_command_values_)))
      {
        actuator_values = pending_command_values_;
        should_send = true;
        pending_command_dirty_ = false;
      }
    }

    if (should_send)
    {
      if (send_command(actuator_values))
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        last_command_values_ = actuator_values;
        command_sent_ = true;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger(kLoggerName),
          *get_node()->get_clock(),
          1000,
          "Failed to write Inspire Modbus RTU command to ID %d; command will be retried",
          slave_id_);
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_command_dirty_ = true;
      }
    }

    // Serial access stays on this thread, after ordinary feedback and commands.
    // Read at most one chunk; publish only a complete, valid region.
    if (read_tactile_enabled_ && std::chrono::steady_clock::now() >= next_tactile_read_)
    {
      read_tactile_chunk();
    }

    const auto elapsed = std::chrono::steady_clock::now() - loop_start;
    const auto target_period = std::chrono::milliseconds(background_period_ms_);
    if (elapsed < target_period)
    {
      std::this_thread::sleep_for(target_period - elapsed);
    }
    else
    {
      const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger(kLoggerName),
        *get_node()->get_clock(),
        5000,
        "Inspire Modbus RTU background loop is slower than expected: %ld ms (expected: %d ms)",
        elapsed_ms,
        background_period_ms_);
    }
  }
}

void InspireHandHardware::read_tactile_chunk()
{
  const auto& region = rh56e2_protocol::kTactileRegions[tactile_scan_regions_[tactile_region_index_]];
  const std::size_t total = region.rows * region.columns;
  const auto count = std::min(rh56e2_protocol::kTactileChunkWords,
    total - tactile_word_offset_);
  const auto address = static_cast<uint16_t>(region.address + 2 * tactile_word_offset_);
  auto* destination = tactile_values_.data() + tactile_word_offset_;
  const int received = modbus_->readHoldingRegisters(address, static_cast<int>(count), destination);

  if (received != static_cast<int>(count) ||
      std::any_of(destination, destination + count, [](uint16_t value) {
        return value > rh56e2_protocol::kMaximumTactileValue;
      }))
  {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger(kLoggerName), *get_node()->get_clock(), 5000,
      "T1 tactile read invalid: region=%s address=%u received=%d expected=%zu; "
      "no frame published, retry in 5s. Check model/firmware/map. %s",
      region.name, static_cast<unsigned>(address), received, count,
      received < 0 ? modbus_->getLastError().c_str() : "");
    tactile_word_offset_ = 0;  // Discard the partial region, never publish zeros on failure.
    next_tactile_read_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    return;
  }

  tactile_word_offset_ += count;
  if (tactile_word_offset_ == total)
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = get_node()->now();  // Host time at completion, not sensor timestamp.
    msg.header.frame_id = std::string("tactile_") + region.name;
    msg.height = region.rows;
    msg.width = region.columns;
    msg.encoding = "16UC1";
    msg.is_bigendian = false;
    msg.step = region.columns * 2;
    msg.data.resize(total * 2);
    for (std::size_t i = 0; i < total; ++i)
    {
      msg.data[2 * i] = static_cast<uint8_t>(tactile_values_[i] & 0xff);
      msg.data[2 * i + 1] = static_cast<uint8_t>(tactile_values_[i] >> 8);
    }
    tactile_publishers_[tactile_scan_regions_[tactile_region_index_]]->publish(msg);
    tactile_region_index_ = (tactile_region_index_ + 1) % tactile_scan_regions_.size();
    tactile_word_offset_ = 0;
  }
  next_tactile_read_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(tactile_period_ms_);
}

bool InspireHandHardware::initialize_hand()
{
  // Manual Table 55: six byte fields (1625..1630), packed into three words.
  std::array<uint16_t, rh56e2_protocol::kModeWordCount> modes{};
  modes.fill(0);
  if (modbus_->writeRegisters(kModeRegister, modes.size(), modes.data()) !=
      static_cast<int>(modes.size()))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to set Inspire speed-force protection mode; activation aborted");
    return false;
  }

  std::array<uint16_t, kActuatorCount> speeds{};
  speeds.fill(default_speed_);
  if (modbus_->writeRegisters(kSpeedSetRegister, speeds.size(), speeds.data()) !=
      static_cast<int>(speeds.size()))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to set Inspire hand speed register; activation aborted");
    return false;
  }

  std::array<uint16_t, kActuatorCount> forces{};
  forces.fill(default_force_);
  if (modbus_->writeRegisters(kForceSetRegister, forces.size(), forces.data()) !=
      static_cast<int>(forces.size()))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to set Inspire hand force register; activation aborted");
    return false;
  }

  return true;
}

bool InspireHandHardware::read_feedback(
  std::array<double, kJointCount>& positions,
  std::array<double, kJointCount>& efforts)
{
  std::array<uint16_t, kActuatorCount> angle_registers{};
  const auto angle_count = modbus_->readHoldingRegisters(
    kAngleActRegister, angle_registers.size(), angle_registers.data());
  if (angle_count != static_cast<int>(angle_registers.size()))
  {
    return false;
  }

  positions = actuators_to_joints(angle_registers);
  for (double position : positions)
  {
    if (!std::isfinite(position))
    {
      return false;  // ANGLE_ACT must be 0..1000; -1 is not a measured angle.
    }
  }

  std::array<uint16_t, kActuatorCount> force_registers{};
  const auto force_count = modbus_->readHoldingRegisters(
    kForceActRegister, force_registers.size(), force_registers.data());
  if (force_count == static_cast<int>(force_registers.size()))
  {
    for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index)
    {
      efforts[joint_index] = rh56e2_protocol::force_from_register(
        force_registers[rh56e2_protocol::kJointToActuator[joint_index]]);
      if (std::abs(efforts[joint_index]) > rh56e2_protocol::kMaximumMeasuredForce)
      {
        return false;
      }
    }
  }
  else
  {
    return false;  // Missing force data must not be reported as a measured zero.
  }

  return true;
}

bool InspireHandHardware::send_command(
  const std::array<uint16_t, kActuatorCount>& actuator_values)
{
  const auto result = modbus_->writeRegisters(
    kAngleSetRegister, actuator_values.size(), actuator_values.data());
  return result == static_cast<int>(actuator_values.size());
}

bool InspireHandHardware::command_changed(
  const std::array<uint16_t, kActuatorCount>& actuator_values) const
{
  for (std::size_t i = 0; i < kActuatorCount; ++i)
  {
    if (std::abs(static_cast<int>(actuator_values[i]) - static_cast<int>(last_command_values_[i])) >
        command_deadband_raw_)
    {
      return true;
    }
  }
  return false;
}

std::array<uint16_t, InspireHandHardware::kActuatorCount> InspireHandHardware::joints_to_actuators(
  const std::array<double, kJointCount>& joints) const
{
  std::array<uint16_t, kJointCount> values{};
  for (std::size_t joint = 0; joint < kJointCount; ++joint)
  {
    values[joint] = joint_to_actuator_value(joints[joint], joint);
  }
  return rh56e2_protocol::reverse_joint_order(values);
}

std::array<double, InspireHandHardware::kJointCount> InspireHandHardware::actuators_to_joints(
  const std::array<uint16_t, kActuatorCount>& actuators) const
{
  const auto values = rh56e2_protocol::reverse_joint_order(actuators);
  std::array<double, kJointCount> joints{};
  for (std::size_t joint = 0; joint < kJointCount; ++joint)
  {
    joints[joint] = actuator_value_to_joint(values[joint], joint);
  }
  return joints;
}

uint16_t InspireHandHardware::joint_to_actuator_value(
  double radians,
  std::size_t joint_index) const
{
  return rh56e2_protocol::angle_to_register(
    radians, lower_limits_[joint_index], upper_limits_[joint_index]);
}

double InspireHandHardware::actuator_value_to_joint(
  uint16_t value,
  std::size_t joint_index) const
{
  return rh56e2_protocol::register_to_angle(
    value, lower_limits_[joint_index], upper_limits_[joint_index]);
}

bool InspireHandHardware::parse_bool(const std::string& value, bool default_value)
{
  if (value == "true" || value == "1" || value == "True" || value == "TRUE")
  {
    return true;
  }
  if (value == "false" || value == "0" || value == "False" || value == "FALSE")
  {
    return false;
  }
  return default_value;
}

int InspireHandHardware::parse_int(const std::string& value, int default_value)
{
  try
  {
    return std::stoi(value, nullptr, 0);
  }
  catch (const std::exception&)
  {
    return default_value;
  }
}

double InspireHandHardware::initial_value_for_joint(
  const hardware_interface::ComponentInfo& joint)
{
  for (const auto& interface : joint.state_interfaces)
  {
    if (interface.name == hardware_interface::HW_IF_POSITION && !interface.initial_value.empty())
    {
      try
      {
        return std::stod(interface.initial_value);
      }
      catch (const std::exception&)
      {
        return 0.0;
      }
    }
  }
  return 0.0;
}

std::array<double, InspireHandHardware::kJointCount> InspireHandHardware::default_upper_limits(
  const std::vector<std::string>& joint_names)
{
  std::array<double, kJointCount> limits{1.1641, 0.5864, 1.4381, 1.4381, 1.4381, 1.4381};

  for (std::size_t i = 0; i < std::min(kJointCount, joint_names.size()); ++i)
  {
    const auto& name = joint_names[i];
    if (has_suffix(name, "thumb_joint1"))
    {
      limits[i] = 1.1641;
    }
    else if (has_suffix(name, "thumb_joint2"))
    {
      limits[i] = 0.5864;
    }
    else
    {
      limits[i] = 1.4381;
    }
  }

  return limits;
}

}  // namespace modbus_ros2_control

PLUGINLIB_EXPORT_CLASS(
  modbus_ros2_control::InspireHandHardware,
  hardware_interface::SystemInterface)
