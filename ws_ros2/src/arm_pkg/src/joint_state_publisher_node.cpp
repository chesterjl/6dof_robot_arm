// Works for rviz and this node only for rviz 
/**
 * arm_publisher_node.cpp
 * ============================================================
 * ROS2 Joint-State Publisher for 6-DOF Servo Arm
 * Robot  : MG996R arm (servoarm URDF)
 * Joints : joint1 (waist/Z) → joint2 (shoulder/X)
 *          → joint3 (elbow/X) → joint4 (wrist_pitch/X)
 *          → joint5 (wrist_roll/Y)
 *          → joint6 (right_gripper/Z) + joint7 (left_gripper/Z)
 *
 * Sequence:
 *   HOME → REACH_LEFT → GRIP → LIFT → SWING_RIGHT
 *        → LOWER_RIGHT → RELEASE → HOME (loop)
 * ============================================================
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <cmath>
#include <array>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────
//  JOINT INDEX CONSTANTS  (matches msg.name order below)
// ─────────────────────────────────────────────────────────────
constexpr int J_WAIST        = 0;   // joint1  Z-axis  ±1.57
constexpr int J_SHOULDER      = 1;   // joint2  X-axis   0.0 → 1.57
constexpr int J_ELBOW         = 2;   // joint3  X-axis  ±1.57
constexpr int J_WRIST_PITCH   = 3;   // joint4  X-axis  ±1.57
constexpr int J_WRIST_ROLL    = 4;   // joint5  Y-axis  ±1.57
constexpr int J_GRIP_RIGHT    = 5;   // joint6  Z-axis  -0.5 → 0.0
constexpr int J_GRIP_LEFT     = 6;   // joint7  Z-axis  -0.5 → 0.0 (mimic)
constexpr int NUM_JOINTS      = 7;

// ─────────────────────────────────────────────────────────────
//  JOINT LIMITS  (from your URDF)
// ─────────────────────────────────────────────────────────────
constexpr double WAIST_MIN      = -1.57,  WAIST_MAX     =  1.57;
constexpr double SHOULDER_MIN   =  0.0,   SHOULDER_MAX  =  1.57;
constexpr double ELBOW_MIN      = -1.57,  ELBOW_MAX     =  1.57;
constexpr double WRIST_P_MIN    = -1.57,  WRIST_P_MAX   =  1.57;
constexpr double WRIST_R_MIN    = -1.57,  WRIST_R_MAX   =  1.57;
constexpr double GRIP_OPEN      =  -0.5;   // gripper open
constexpr double GRIP_CLOSED    = 0.0;   // gripper closed (grab object)

// ─────────────────────────────────────────────────────────────
//  MOTION SPEED
// ─────────────────────────────────────────────────────────────
constexpr double STEP           = 0.015;  // rad per 100 ms tick (~8.6 °/s)
constexpr double AT_TARGET_TOL  = 0.005;  // rad tolerance
constexpr int    HOLD_TICKS     = 25;     // 2.5 s hold at each waypoint

// ─────────────────────────────────────────────────────────────
//  POSE DEFINITIONS  [waist, shoulder, elbow, wrist_p, wrist_r, grip_r, grip_l]
// ─────────────────────────────────────────────────────────────
using Pose = std::array<double, NUM_JOINTS>;

//  Arm at rest – centred, raised, gripper open
const Pose POSE_HOME = {
   0.0,          // waist    – face forward
   0.3,          // shoulder – slightly raised
   0.0,          // elbow    – straight
   0.0,          // wrist pitch
   0.0,          // wrist roll
   GRIP_CLOSED,    // right gripper
   GRIP_CLOSED     // left  gripper
};

//  Swing left over the pick target
const Pose POSE_SWING_LEFT = {
   WAIST_MAX,    // waist full left  (+1.57)
   0.365,          // shoulder up
   0.0,
   0.0,
   0.0,
   GRIP_CLOSED,
   GRIP_CLOSED
};

//  Lower down to pick object on the left
const Pose POSE_REACH_LEFT = {
   WAIST_MAX,    // still facing left
   0.365, // shoulder down / reach
   -0.518,          // elbow bent forward
   -0.636,          // wrist angled for pick
   1.57,
   GRIP_OPEN,
   GRIP_CLOSED
};

//  Close gripper (grip object)
const Pose POSE_GRIP = {
   WAIST_MAX,
   0.365,
   -0.518,          // elbow bent forward
   -0.636,          // wrist angled for pick
   1.57,
   GRIP_CLOSED,
   GRIP_CLOSED
};

//  Lift straight up after gripping
const Pose POSE_LIFT = {
   WAIST_MAX,
   0.7,          // raise shoulder
   0.0,          // straighten elbow
   0.0,
   0.0,
   GRIP_CLOSED,  // keep grip
   GRIP_CLOSED
};

//  Swing right over drop target
const Pose POSE_SWING_RIGHT = {
   WAIST_MIN,    // waist full right (-1.57)
   0.365,
   0.0,
   0.0,
   0.0,
   GRIP_CLOSED,
   GRIP_CLOSED
};

//  Lower to drop position on the right
const Pose POSE_LOWER_RIGHT = {
   WAIST_MIN,
    0.365,
   -0.518,
   -0.636,
   -1.57,
   GRIP_CLOSED,
   GRIP_CLOSED
};

//  Release object
const Pose POSE_RELEASE = {
   WAIST_MIN,
    0.365,
   -0.518,
   -0.636,
   -1.57,
   GRIP_OPEN,
   GRIP_CLOSED
};

// ─────────────────────────────────────────────────────────────
//  STATE MACHINE
// ─────────────────────────────────────────────────────────────
enum class State : int {
  HOME = 0,
  HOLD_HOME,
  SWING_LEFT,
  REACH_LEFT,
  GRIP,
  HOLD_GRIP,
  LIFT,
  SWING_RIGHT,
  LOWER_RIGHT,
  RELEASE,
  HOLD_RELEASE,
  RETURN_HOME,
  COUNT_   // sentinel
};

static const char* STATE_NAMES[] = {
  "HOME", "HOLD_HOME", "SWING_LEFT", "REACH_LEFT",
  "GRIP", "HOLD_GRIP", "LIFT", "SWING_RIGHT",
  "LOWER_RIGHT", "RELEASE", "HOLD_RELEASE", "RETURN_HOME"
};

// ─────────────────────────────────────────────────────────────
//  NODE
// ─────────────────────────────────────────────────────────────
class ArmPublisher : public rclcpp::Node
{
public:
  ArmPublisher() : Node("arm_publisher_node")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);

    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&ArmPublisher::timerCallback, this));

    // Start at home pose immediately
    current_ = POSE_HOME;
    target_  = POSE_HOME;
    state_   = State::HOME;

    RCLCPP_INFO(get_logger(),
      "ArmPublisher started – Pick LEFT, Drop RIGHT cycle");
  }

private:
  // ── members ────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  Pose    current_;
  Pose    target_;
  State   state_    = State::HOME;
  int     hold_cnt_ = 0;

  // ── timer callback ─────────────────────────────────────────
  void timerCallback()
  {
    stepTowardsTarget();

    switch (state_)
    {
      // ── Idle at home ──────────────────────────────────────
      case State::HOME:
        setTarget(POSE_HOME);
        if (atTarget()) {
          state_    = State::HOLD_HOME;
          hold_cnt_ = 0;
        }
        break;

      case State::HOLD_HOME:
        if (++hold_cnt_ >= HOLD_TICKS)
          state_ = State::SWING_LEFT;
        break;

      // ── Pick sequence (left side) ─────────────────────────
      case State::SWING_LEFT:
        setTarget(POSE_SWING_LEFT);
        if (atTarget()) state_ = State::REACH_LEFT;
        break;

      case State::REACH_LEFT:
        setTarget(POSE_REACH_LEFT);
        if (atTarget()) state_ = State::GRIP;
        break;

      case State::GRIP:
        setTarget(POSE_GRIP);
        if (atTarget()) {
          state_    = State::HOLD_GRIP;
          hold_cnt_ = 0;
          RCLCPP_INFO(get_logger(), "✊  Object gripped!");
        }
        break;

      case State::HOLD_GRIP:
        if (++hold_cnt_ >= HOLD_TICKS)
          state_ = State::LIFT;
        break;

      // ── Carry sequence ────────────────────────────────────
      case State::LIFT:
        setTarget(POSE_LIFT);
        if (atTarget()) state_ = State::SWING_RIGHT;
        break;

      case State::SWING_RIGHT:
        setTarget(POSE_SWING_RIGHT);
        if (atTarget()) state_ = State::LOWER_RIGHT;
        break;

      // ── Drop sequence (right side) ────────────────────────
      case State::LOWER_RIGHT:
        setTarget(POSE_LOWER_RIGHT);
        if (atTarget()) state_ = State::RELEASE;
        break;

      case State::RELEASE:
        setTarget(POSE_RELEASE);
        if (atTarget()) {
          state_    = State::HOLD_RELEASE;
          hold_cnt_ = 0;
          RCLCPP_INFO(get_logger(), "✋  Object released!");
        }
        break;

      case State::HOLD_RELEASE:
        if (++hold_cnt_ >= HOLD_TICKS)
          state_ = State::RETURN_HOME;
        break;

      case State::RETURN_HOME:
        setTarget(POSE_HOME);
        if (atTarget()) {
          state_    = State::HOME;
          hold_cnt_ = 0;
          RCLCPP_INFO(get_logger(), "🔄  Cycle complete – returning home");
        }
        break;

      default:
        break;
    }

    publishJointState();
  }

  // ── motion helpers ─────────────────────────────────────────
  void setTarget(const Pose& p)
  {
    target_ = p;
  }

  void stepTowardsTarget()
  {
    for (int i = 0; i < NUM_JOINTS; ++i) {
      double diff = target_[i] - current_[i];
      if (std::fabs(diff) <= STEP)
        current_[i] = target_[i];
      else
        current_[i] += (diff > 0 ? STEP : -STEP);
    }
  }

  bool atTarget() const
  {
    for (int i = 0; i < NUM_JOINTS; ++i)
      if (std::fabs(current_[i] - target_[i]) > AT_TARGET_TOL)
        return false;
    return true;
  }

  // ── publish ────────────────────────────────────────────────
  void publishJointState()
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = get_clock()->now();

    // Joint names MUST match your URDF exactly
    msg.name = {
      "joint_1",   // waist
      "joint_2",   // shoulder
      "joint_3",   // elbow
      "joint_4",   // wrist_pitch
      "joint_5",   // wrist_roll
      "joint_6",   // right_gripper
      "joint_7"    // left_gripper  (mimic in URDF)
    };
    
    msg.position = {
      current_[J_WAIST],
      current_[J_SHOULDER],
      current_[J_ELBOW],
      current_[J_WRIST_PITCH],
      current_[J_WRIST_ROLL],
      current_[J_GRIP_RIGHT],
      current_[J_GRIP_LEFT]
    };

    publisher_->publish(msg);

    RCLCPP_DEBUG(get_logger(),
      "[%s] W:%.2f Sh:%.2f El:%.2f WP:%.2f WR:%.2f Gr:%.2f",
      STATE_NAMES[static_cast<int>(state_)],
      current_[J_WAIST],  current_[J_SHOULDER],
      current_[J_ELBOW],  current_[J_WRIST_PITCH],
      current_[J_WRIST_ROLL], current_[J_GRIP_RIGHT]);
  }
};

// ─────────────────────────────────────────────────────────────
//  MAIN
// ─────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmPublisher>());
  rclcpp::shutdown();
  return 0;
}