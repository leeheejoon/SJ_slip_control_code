#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class HandControlNode : public rclcpp::Node
{
public:
  HandControlNode()
  : Node("hand_control_node")
  {
    tactile_noise_floor = declare_parameter("tactile_noise_floor", 20.0);
    tactile_contact_sum = declare_parameter("tactile_contact_sum", 1000.0);
    tactile_change_threshold = declare_parameter("tactile_change_threshold", 0.35);
    tactile_confirm_seconds = declare_parameter("tactile_confirm_seconds", 0.15);
    if (tactile_noise_floor < 0 || tactile_contact_sum <= 0 ||
        tactile_change_threshold <= 0 || tactile_confirm_seconds <= 0) {
      throw std::invalid_argument("촉각 판정 파라미터는 유효한 양수여야 합니다");
    }
    index_force_min = declare_parameter("index_force_min", 100.0);
    index_force_max = declare_parameter("index_force_max", 160.0);
    index_force_margin = declare_parameter("index_force_margin", 10.0);
    if (!std::isfinite(index_force_min) || !std::isfinite(index_force_max) ||
        !std::isfinite(index_force_margin) || index_force_min >= index_force_max ||
        index_force_margin < 0.0) {
      throw std::invalid_argument("검지 힘 범위 설정 오류");
    }
    slip_all_fingers = declare_parameter("slip_all_fingers", true);
    slip_step = declare_parameter("slip_step", 0.01);
    slip_max_step = declare_parameter("slip_max_step", 0.03);
    slip_max_extra = declare_parameter("slip_max_extra", 0.0);
    slip_settle_seconds = declare_parameter("slip_settle_seconds", 0.20);
    if (!std::isfinite(slip_step) || !std::isfinite(slip_max_step) ||
        slip_max_step < slip_step || !std::isfinite(slip_max_extra) ||
        !std::isfinite(slip_settle_seconds) || slip_step <= 0.0 ||
        slip_max_extra < 0.0 || slip_settle_seconds < 0.1) {
      throw std::invalid_argument("추가 조임 파라미터 범위 오류");
    }
    const auto calibration_regions = declare_parameter<std::vector<std::string>>(
      "tactile_calibration_regions", {"index_pad", "middle_pad", "thumb_distal"});
    if (calibration_regions.empty()) { throw std::invalid_argument("무접촉 보정 영역이 비어 있습니다"); }
    for (const auto & name : calibration_regions) {
      const auto it = std::find_if(tactile_regions.begin(), tactile_regions.end(),
        [&name](const char * value) { return name == value; });
      if (it == tactile_regions.end()) { throw std::invalid_argument("잘못된 촉각 영역: " + name); }
      calibration_required[std::distance(tactile_regions.begin(), it)] = true;
    }
    RCLCPP_INFO(get_logger(), "촉각 무접촉 보정 대기: 물체가 손에 닿지 않게 유지하세요 (파지 명령 보류)");
    command_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
        "/hand_joint_controller/target_joint_position", 10);

    state_sub_ =
      create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg)
        {
          receive_state(*msg);
        });

    for (std::size_t i = 0; i < tactile_regions.size(); ++i) {
      tactile_subs[i] = create_subscription<sensor_msgs::msg::Image>(
        std::string("/hand/tactile/") + tactile_regions[i],
        rclcpp::SensorDataQoS(),
        [this, i](sensor_msgs::msg::Image::ConstSharedPtr msg)
        {
          receive_tactile(i, *msg);
        });
    }

    timer_ = create_wall_timer(
      500ms, [this]() { control_step(); });
  }

private:
  const std::array<const char *, 17> tactile_regions{
    "little_tip", "little_distal", "little_pad",
    "ring_tip", "ring_distal", "ring_pad",
    "middle_tip", "middle_distal", "middle_pad",
    "index_tip", "index_distal", "index_pad",
    "thumb_tip", "thumb_distal", "thumb_middle", "thumb_pad", "palm"
  };
  const std::array<const char *, 17> tactile_labels{
    "소지끝", "소지말단", "소지안쪽",
    "약지끝", "약지말단", "약지안쪽",
    "중지끝", "중지말단", "중지안쪽",
    "검지끝", "검지말단", "검지안쪽",
    "엄지끝", "엄지말단", "엄지중간", "엄지안쪽", "손바닥"
  };
  std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, 17> tactile_subs{};
  std::array<uint16_t, 17> tactile_max{};
  std::array<bool, 17> tactile_received{};
  std::array<std::chrono::steady_clock::time_point, 17> tactile_received_at{};

  // 전체 배열과 기준 분포를 보관한다. 파지 후 촉각 변화로만 추가 조임한다.
  struct TactileState {
    std::vector<uint16_t> raw;
    std::vector<double> empty_upper;
    int empty_frames = 0;
    bool empty_contact = true;
    std::vector<double> values;
    std::vector<double> reference;
    uint32_t width = 0;
    uint32_t height = 0;
    int reference_frames = 0;
    int candidate = 0;  // 0: 정상, 1: 분포 변화 또는 합계 감소
    int candidate_frames = 0;
    int reported = 0;
    int near_zero_frames = 0;
    std::chrono::steady_clock::time_point candidate_start{};
  };
  std::array<TactileState, 17> tactile_state{};
  std::array<bool, 17> calibration_required{};
  bool tactile_calibrated = false;
  double tactile_noise_floor = 20.0;
  double tactile_contact_sum = 1000.0;
  double tactile_change_threshold = 0.35;
  double tactile_confirm_seconds = 0.15;
  bool object_loss_reported = false;
  bool object_loss_waiting = false;
  std::chrono::steady_clock::time_point object_loss_start{};

  const std::array<int, 17> tactile_joint{
    5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, -1};
  const std::array<double, 6> joint_upper{1.1641, 0.5864, 1.4381, 1.4381, 1.4381, 1.4381};
  std::array<double, 6> slip_extra{};
  std::array<bool, 6> slip_blocked{};
  std::array<std::chrono::steady_clock::time_point, 6> slip_settle_until{};
  bool slip_all_fingers = true;
  double slip_step = 0.01;
  double slip_max_step = 0.03;
  double slip_max_extra = 0.0;  // 0이면 누적 추가량 제한 해제
  double slip_settle_seconds = 0.20;

  void tighten_for_slip(std::size_t region, double distribution_change)
  {
    const int joint = tactile_joint[region];
    const auto now = std::chrono::steady_clock::now();
    if (joint < 1 || object_loss_reported ||
        now - last_state_received > 500ms || command_position.size() != 6) { return; }
    const auto affected = [this, joint](int index) {
      return index >= 1 && (slip_all_fingers || index == joint);
    };
    // 전체 모드에서는 모든 손가락이 하나의 대기 시간을 공유한다.
    const double requested = std::clamp(
      slip_step * distribution_change / tactile_change_threshold, slip_step, slip_max_step);
    double delta = requested;
    for (int index = 1; index < 6; ++index) {
      if (!affected(index)) { continue; }
      if (slip_blocked[index] || now < slip_settle_until[index]) { return; }
      const double extra_remaining = slip_max_extra > 0.0 ?
        slip_max_extra - slip_extra[index] : requested;
      const double remaining = std::min(extra_remaining, joint_upper[index] - command_position[index]);
      if (remaining <= 1e-9) {
        slip_blocked[index] = true;
        RCLCPP_INFO(get_logger(), "추가 조임 중단: %s, %s 한계 (%s)",
          joint_names[index].c_str(), joint_upper[index] - command_position[index] <= 1e-9 ?
          "관절각" : "누적 추가량", slip_all_fingers ? "전체 동일 증가량 모드" : "개별 모드");
        return;
      }
      delta = std::min(delta, remaining);
    }
    // 한 관절의 상한이 가까우면 공통 증가량도 줄여 모든 대상에 같은 각도를 더한다.
    for (int index = 1; index < 6; ++index) {
      if (!affected(index)) { continue; }
      command_position[index] += delta;
      slip_extra[index] += delta;
      slip_settle_until[index] = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(slip_settle_seconds));
    }
    std_msgs::msg::Float64MultiArray command;
    command.data = command_position;
    command_pub_->publish(command);
    RCLCPP_INFO(get_logger(), "미끄러짐 발생 → 추가 조임: 감지=%s, 대상=%s, 증가량=%.4f rad, 목표각=[%.4f, %.4f, %.4f, %.4f, %.4f]",
      tactile_labels[region], slip_all_fingers ? "다섯 손가락 전체" : joint_names[joint].c_str(), delta,
      command_position[1], command_position[2], command_position[3], command_position[4], command_position[5]);
    // 조임 직전 분포를 다음 비교 기준으로 삼고 짧은 대기 후 새 변화를 감시한다.
    for (std::size_t i = 0; i < tactile_state.size(); ++i) {
      if (!affected(tactile_joint[i])) { continue; }
      auto & state = tactile_state[i];
      if (state.reference_frames == 3 && now - tactile_received_at[i] <= 1s &&
          std::accumulate(state.values.begin(), state.values.end(), 0.0) > 100.0) {
        state.reference = state.values;
      }
      // 별도 무접촉 기준은 조임 후에도 바꾸지 않는다.
      state.candidate = 0;
      state.candidate_frames = 0;
      state.reported = 0;
    }
  }

  void inspect_tactile(std::size_t region, double interval)
  {
    if (!grasp_success) { return; }
    const int joint = tactile_joint[region];
    auto & state = tactile_state[region];
    const auto now = std::chrono::steady_clock::now();
    const double sum = std::accumulate(state.values.begin(), state.values.end(), 0.0);
    // 한 영역의 감소로 차단하지 않는다. 서로 다른 손가락의 기준 영역이
    // 모두 거의 0인 새 프레임을 연속 수신할 때만 전체 이탈을 판정한다.
    const bool near_empty = state.empty_contact;
    if (state.reference_frames == 3) {
      state.near_zero_frames = near_empty ?
        (interval > 1.0 ? 1 : state.near_zero_frames + 1) : 0;
    }
    std::array<bool, 6> referenced_fingers{};
    bool all_near_zero = true;
    for (std::size_t i = 0; i < tactile_state.size(); ++i) {
      if (tactile_state[i].reference_frames != 3 || tactile_joint[i] < 1) { continue; }
      referenced_fingers[tactile_joint[i]] = true;
      all_near_zero = all_near_zero && tactile_state[i].near_zero_frames >= 2 &&
        now - tactile_received_at[i] <= 1s;
    }
    const bool loss_candidate = all_near_zero &&
      std::count(referenced_fingers.begin(), referenced_fingers.end(), true) >= 2;
    if (!loss_candidate) {
      object_loss_waiting = false;
    } else if (!object_loss_waiting) {
      object_loss_start = now;
      object_loss_waiting = true;
    } else if (!object_loss_reported && now - object_loss_start >= 300ms) {
      object_loss_reported = true;
      RCLCPP_INFO(get_logger(), "물체 이탈 의심: 여러 손가락 기준 영역이 거의 0으로 지속, 전체 추가 조임 중단 (재시작 필요)");
    }
    // 거의 0인 영역에는 조임 명령을 보내지 않지만, 복구되면 다시 판정한다.
    if (object_loss_reported || (state.reference_frames == 3 && near_empty)) {
      state.candidate = 0;
      state.candidate_frames = 0;
      state.reported = 0;
      return;
    }
    if (joint >= 1 && now < slip_settle_until[joint]) { return; }
    // 무접촉 수준의 잔류값을 접촉 기준으로 저장하지 않는다.
    // 파지 완료 후 새로 수신한 접촉 프레임 세 개로 기준을 만든다.
    if (state.reference_frames < 3) {
      if (interval > 1.0) {
        state.reference.clear();
        state.reference_frames = 0;
      }
      if (state.empty_contact || sum < tactile_contact_sum) {
        state.reference.clear();
        state.reference_frames = 0;
        return;
      }
      if (state.reference.empty()) { state.reference.assign(state.values.size(), 0.0); }
      for (std::size_t j = 0; j < state.values.size(); ++j) {
        state.reference[j] += state.values[j] / 3.0;
      }
      if (++state.reference_frames == 3) {
        RCLCPP_INFO(get_logger(), "촉각 기준 준비: %s, 합계=%.0f, 수신간격=%.3f초",
          tactile_labels[region], sum, interval);
      }
      return;
    }
    const double reference_sum = std::accumulate(
      state.reference.begin(), state.reference.end(), 0.0);
    const double ratio = sum / reference_sum;
    double distribution_change = 0.0;
    if (sum > 0.0) {
      for (std::size_t j = 0; j < state.values.size(); ++j) {
        distribution_change += std::abs(
          state.values[j] / sum - state.reference[j] / reference_sum);
      }
    }
    const double drop = std::max(0.0, 1.0 - ratio);
    const int candidate = (distribution_change >= tactile_change_threshold || drop >= 0.3) ? 1 : 0;
    const double change_score = std::max(
      distribution_change, tactile_change_threshold * drop / 0.3);
    // 수신이 1초 이상 끊기면 연속 관측으로 세지 않는다.
    if (candidate != state.candidate || interval > 1.0) {
      state.candidate = candidate;
      state.candidate_frames = 0;
      state.candidate_start = now;
      state.reported = 0;
    }
    ++state.candidate_frames;
    if (candidate != 0 && state.reported != candidate &&
        state.candidate_frames >= 2 &&
        std::chrono::duration<double>(now - state.candidate_start).count() >=
        tactile_confirm_seconds)
    {
      state.reported = candidate;
      if (candidate == 1) { tighten_for_slip(region, change_score); }
    }
  }

  void receive_tactile(std::size_t region, const sensor_msgs::msg::Image & msg)
  {
    // 16-bit 원시 촉각 배열에서 부위별 최댓값 추출 (행 패딩 및 바이트 순서 반영)
    if (msg.encoding != "16UC1" || msg.width == 0 || msg.height == 0 ||
        static_cast<std::size_t>(msg.step) < static_cast<std::size_t>(msg.width) * 2 ||
        msg.data.size() < static_cast<std::size_t>(msg.step) * msg.height)
    {
      return;
    }

    std::vector<uint16_t> raw;
    raw.reserve(static_cast<std::size_t>(msg.width) * msg.height);
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(msg.width) * msg.height);
    uint16_t maximum = 0;
    for (std::size_t row = 0; row < msg.height; ++row) {
      for (std::size_t column = 0; column < msg.width; ++column) {
        const auto offset = row * msg.step + column * 2;
        const auto value = static_cast<uint16_t>(msg.is_bigendian ?
          (static_cast<uint16_t>(msg.data[offset]) << 8) | msg.data[offset + 1] :
          msg.data[offset] | (static_cast<uint16_t>(msg.data[offset + 1]) << 8));
        if (value > 4095) {
          return;
        }
        maximum = std::max(maximum, value);
        raw.push_back(value);
        values.push_back(std::max(0.0, static_cast<double>(value) - tactile_noise_floor));
      }
    }
    auto & state = tactile_state[region];
    if (state.width != msg.width || state.height != msg.height) {
      state = TactileState{};
      state.width = msg.width;
      state.height = msg.height;
    }
    state.raw = std::move(raw);
    state.values = std::move(values);
    if (!tactile_calibrated && state.empty_frames < 5) {
      if (state.empty_upper.empty()) { state.empty_upper.assign(state.raw.size(), 0.0); }
      for (std::size_t j = 0; j < state.raw.size(); ++j) {
        state.empty_upper[j] = std::max(state.empty_upper[j], static_cast<double>(state.raw[j]));
      }
      ++state.empty_frames;
      if (state.empty_frames == 5) {
        RCLCPP_INFO(get_logger(), "촉각 무접촉 기준 준비: %s (5프레임)", tactile_labels[region]);
      }
      bool ready = true;
      for (std::size_t i = 0; i < tactile_state.size(); ++i) {
        if (calibration_required[i] && tactile_state[i].empty_frames < 5) { ready = false; }
      }
      if (ready) {
        tactile_calibrated = true;
        RCLCPP_INFO(get_logger(), "촉각 무접촉 보정 완료: 이제 물체를 배치하고 초기 파지를 진행하세요");
      }
    }
    // 무접촉 때 관측한 점별 최댓값 + 허용 오차 20을 넘는 신호만 접촉 증거로 센다.
    if (state.empty_frames < 5) { return; }
    double excess_sum = 0.0;
    double excess_peak = 0.0;
    for (std::size_t j = 0; j < state.raw.size(); ++j) {
      const double excess = std::max(0.0, state.raw[j] - state.empty_upper[j] - 20.0);
      excess_sum += excess;
      excess_peak = std::max(excess_peak, excess);
    }
    state.empty_contact = excess_sum <= 100.0 && excess_peak <= 20.0;
    const auto now = std::chrono::steady_clock::now();
    const double interval = tactile_received[region] ?
      std::chrono::duration<double>(now - tactile_received_at[region]).count() : 0.0;
    tactile_max[region] = maximum;
    tactile_received[region] = true;
    tactile_received_at[region] = now;
    inspect_tactile(region, interval);
  }

  void log_tactile()
  {
    std::string text = "촉각데이터 : ";
    const auto current_time = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < tactile_regions.size(); ++i) {
      if (i != 0) {
        text += ", ";
      }
      text += tactile_labels[i];
      text += "=";
      if (!tactile_received[i]) {
        text += "미수신";
      }
      else if (current_time - tactile_received_at[i] > 10s) {
        text += "수신지연";
      }
      else {
        text += std::to_string(tactile_max[i]);
      }
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", text.c_str());
  }

  const std::array<std::string, 6> joint_names{
    "thumb_joint1",
    "thumb_joint2",
    "index_joint",
    "middle_joint",
    "ring_joint",
    "pinky_joint"
  };

  std::array<double, 6> position{};
  std::array<double, 6> effort{};

  std::array<double, 6> force_sum{};
  int force_count = 0;
  std::array<double, 6> force_zero{};
  bool force_zero_ready = false;

  double force_min = 80.0;
  double force_max = 140.0;
  double index_force_min = 100.0;
  double index_force_max = 160.0;
  double index_force_margin = 10.0;

  // [목표 하한, 목표 상한, 바깥쪽 미세 보정 폭]
  std::array<double, 3> initial_force_band(std::size_t joint) const
  {
    return joint == 2 ? std::array<double, 3>{index_force_min, index_force_max, index_force_margin} :
      std::array<double, 3>{force_min, force_max, 20.0};
  }

  std::vector<double> command_position;
  std::array<bool, 6> joint_stopped{};
  std::chrono::steady_clock::time_point last_state_received{};
  bool grasp_success = false;
  bool grasp_waiting = false;
  std::chrono::steady_clock::time_point grasp_wait_start{};

  void receive_state(const sensor_msgs::msg::JointState & msg)
  {
    // 명령 관절 순서에 맞는 메시지 인덱스 확인
    std::array<std::size_t, 6> indices{};

    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      bool found = false;

      for (std::size_t j = 0; j < msg.name.size(); ++j) {
        if (msg.name[j] == joint_names[i]) {
          if (j >= msg.position.size() || j >= msg.effort.size()) {
            return;
          }

          indices[i] = j;
          found = true;
          break;
        }
      }

      if (!found) {
        return;
      }
    }

    // 필요한 관절 데이터가 모두 있을 때만 상태 갱신
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      position[i] = msg.position[indices[i]];
      effort[i] = msg.effort[indices[i]];
    }

    // 첫 정상 수신 시 현재 각도를 목표 각도로 설정
    if (command_position.empty()) {
      command_position.assign(position.begin(), position.end());
      command_position[0] = 1.0;  // 엄지1 회전 관절은 1.0 rad로 고정
    }

    // 처음 10개 상태로 힘 영점 계산
    if (!force_zero_ready) {
      for (std::size_t i = 0; i < force_sum.size(); ++i) {
        force_sum[i] += effort[i];
      }

      ++force_count;

      if (force_count < 10) {
        return;
      }

      for (std::size_t i = 0; i < force_zero.size(); ++i) {
        force_zero[i] = force_sum[i] / 10.0;
      }

      force_zero_ready = true;
      RCLCPP_INFO(get_logger(), "손 힘 영점 계산 완료");
    }

    // 영점 보정
    for (std::size_t i = 0; i < effort.size(); ++i) {
      effort[i] -= force_zero[i];
    }

    last_state_received = std::chrono::steady_clock::now();

  }

  void control_step()
  {
    log_tactile();

    if (!force_zero_ready || !tactile_calibrated) {
      return;
    }

    if (std::chrono::steady_clock::now() - last_state_received > 500ms) {
      grasp_waiting = false;
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "힘: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
      effort[0], effort[1], effort[2],
      effort[3], effort[4], effort[5]);

    // 엄지1은 힘 제어에서 제외하고 고정 목표각 유지
    command_position[0] = 1.0;
    // 개별 도달을 확정하지 않고, 같은 0.5초 판단 시점에 다섯 관절의 힘을 함께 확인한다.
    const bool initial_grasp_stopped = std::all_of(
      joint_stopped.begin() + 1, joint_stopped.end(), [](bool stopped) { return stopped; });
    if (!initial_grasp_stopped) {
      bool all_in_range = true;
      for (std::size_t i = 1; i < effort.size(); ++i) {
        const auto band = initial_force_band(i);
        all_in_range = all_in_range && std::isfinite(effort[i]) &&
          effort[i] >= band[0] && effort[i] <= band[1];
      }
      if (all_in_range) {
        for (std::size_t i = 1; i < effort.size(); ++i) {
          command_position[i] = position[i];
          joint_stopped[i] = true;
        }
        RCLCPP_INFO(get_logger(), "다섯 손가락 동시 목표 힘 도달: 전체 정지 확정");
        // 빠른 판단은 초기 접근에만 적용한다. 정지 후에는 기존 1초 주기로 복귀.
        timer_->cancel();
        timer_ = create_wall_timer(1s, [this]() { control_step(); });
      } else {
        for (std::size_t i = 1; i < effort.size(); ++i) {
          if (!std::isfinite(effort[i])) { continue; }
          const auto band = initial_force_band(i);
          if (effort[i] < band[0] - band[2]) {
            command_position[i] += 0.015;
          } else if (effort[i] < band[0]) {
            command_position[i] += 0.001;
          } else if (effort[i] > band[1] + band[2]) {
            command_position[i] -= 0.015;
          } else if (effort[i] > band[1]) {
            command_position[i] -= 0.001;
          } else {
            // 범위 안에서는 현재 각도를 유지하되, 다음 주기에 벗어나면 다시 보정.
            command_position[i] = position[i];
          }
          command_position[i] = std::clamp(command_position[i], 0.0, joint_upper[i]);
        }
      }
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = command_position;
    command_pub_->publish(msg);

    // 다섯 관절 도달 후에는 힘 범위 재판정 없이 각도를 유지하며 3초 대기
    if (!grasp_success && std::all_of(
        joint_stopped.begin() + 1, joint_stopped.end(),
        [](bool stopped) { return stopped; }))
    {
      const auto current_time = std::chrono::steady_clock::now();
      if (!grasp_waiting) {
        grasp_wait_start = current_time;
        grasp_waiting = true;
        RCLCPP_INFO(get_logger(), "다섯 관절 목표 힘 도달: 각도 유지 후 3초 대기");
      }
      else if (current_time - grasp_wait_start >= 3s) {
        grasp_success = true;
        RCLCPP_INFO(get_logger(), "초기 파지 완료: 촉각 기준 수집 시작 (기준 준비 후 받침대 제거)");
      }
    }
  }

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr state_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HandControlNode>());
  rclcpp::shutdown();
  return 0;
}
