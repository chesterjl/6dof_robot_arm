#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp> 

using namespace std::chrono_literals;

class ArmGazeboNode : public rclcpp::Node
{
public:
  ArmGazeboNode()
  : Node("arm_gazebo_node",
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("use_sim_time", true)}))
  {
    arm_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);

    // Publishes direct commands to the single active gripper joint
    gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/gripper_action_controller/commands", 10);

    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&ArmGazeboNode::jointCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      100ms, std::bind(&ArmGazeboNode::timerCallback, this));

    // Keep size at 7 so the incoming subscriber doesn't throw out-of-bounds errors
    current_.resize(7, 0.0);
    joint_states_received_ = false;

    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════");
    RCLCPP_INFO(get_logger(), " Mimic-Aware 6-DOF+Gripper Node Active");
    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════");
  }

private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_; 
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<double> current_;
  bool joint_states_received_;

  // ── FIXED: Target configurations trimmed to 6 items [j1, j2, j3, j4, j5, j6]
  std::vector<double> LEFT  = {  1.56, 0.565, -0.518, -0.636,  1.56, 0.0 }; // Gripper Closed
  std::vector<double> MID   = {  0.00, 0.300,  0.000,  0.000,  0.00,  0.0 }; // Gripper Closed
  std::vector<double> RIGHT = { -1.56, 0.565, -0.518, -0.636, -1.56, -0.2 }; // Gripper Open

  double tolerance_ = 0.08;
  
  const double TRANSIT_DURATION = 4.5; 
  const int MOVE_TIMEOUT = 65;

  enum State {
    WAIT_FOR_STATES,
    PICK_LEFT,
    HOLD_LEFT,
    DEFAULT,
    HOLD_DEFAULT,
    PLACE_RIGHT,
    HOLD_PLACE,
    DEFAULT_BACK,
  };

  State state_ = WAIT_FOR_STATES;
  int wait_counter_    = 0;
  int move_counter_    = 0;
  const int HOLD_TICKS = 25; 
  bool command_sent_ = false;

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.empty()) return;

    for (size_t i = 0; i < msg->name.size(); i++)
    {
      if      (msg->name[i] == "joint_1") current_[0] = msg->position[i];
      else if (msg->name[i] == "joint_2") current_[1] = msg->position[i];
      else if (msg->name[i] == "joint_3") current_[2] = msg->position[i];
      else if (msg->name[i] == "joint_4") current_[3] = msg->position[i];
      else if (msg->name[i] == "joint_5") current_[4] = msg->position[i];
      else if (msg->name[i] == "joint_6") current_[5] = msg->position[i];
      else if (msg->name[i] == "joint_7") current_[6] = msg->position[i]; 
    }

    if (!joint_states_received_) {
      joint_states_received_ = true;
    }
  }

  bool atTarget(const std::vector<double>& target)
  {
    // ── FIXED: Only check up to index 5 (joint_6). Ignore joint_7 status.
    for (int i = 0; i < 6; i++) {
      if (std::fabs(current_[i] - target[i]) > tolerance_) return false;
    }
    return true;
  }

  void sendSplitTrajectory(const std::vector<double>& target, double duration_sec)
  {
    auto now = this->get_clock()->now();

    // 1. Pack Arm Trajectory (5 Joints)
    trajectory_msgs::msg::JointTrajectory arm_msg;
    arm_msg.header.stamp = now;
    arm_msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5"};
    
    trajectory_msgs::msg::JointTrajectoryPoint arm_point;
    arm_point.positions = {target[0], target[1], target[2], target[3], target[4]};
    arm_point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0}; 
    arm_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    arm_msg.points.push_back(arm_point);

    // 2. Pack Gripper Command
    // ── FIXED: Only send target[5] (joint_6). Gazebo handles the rest.
    std_msgs::msg::Float64MultiArray gripper_msg;
    gripper_msg.data = {target[5]};

    arm_pub_->publish(arm_msg);
    gripper_pub_->publish(gripper_msg);

    command_sent_ = true;
    move_counter_ = 0;
  }

  void timerCallback()
  {
    switch (state_)
    {
      case WAIT_FOR_STATES:
        if (joint_states_received_) { state_ = PICK_LEFT; }
        return;

      case PICK_LEFT:
        if (!command_sent_) sendSplitTrajectory(LEFT, TRANSIT_DURATION);
        move_counter_++;
        if (atTarget(LEFT) || move_counter_ >= MOVE_TIMEOUT) {
          state_ = HOLD_LEFT; wait_counter_ = 0; command_sent_ = false;
        }
        break;

      case HOLD_LEFT:
        if (++wait_counter_ >= HOLD_TICKS) { state_ = DEFAULT; command_sent_ = false; }
        break;

      case DEFAULT:
        if (!command_sent_) sendSplitTrajectory(MID, TRANSIT_DURATION);
        move_counter_++;
        if (atTarget(MID) || move_counter_ >= MOVE_TIMEOUT) {
          state_ = HOLD_DEFAULT; wait_counter_ = 0; command_sent_ = false;
        }
        break;

      case HOLD_DEFAULT:
        if (++wait_counter_ >= HOLD_TICKS) { state_ = PLACE_RIGHT; command_sent_ = false; }
        break;

      case PLACE_RIGHT:
        if (!command_sent_) sendSplitTrajectory(RIGHT, TRANSIT_DURATION);
        move_counter_++;
        if (atTarget(RIGHT) || move_counter_ >= MOVE_TIMEOUT) {
          state_ = HOLD_PLACE; wait_counter_ = 0; command_sent_ = false;
        }
        break;

      case HOLD_PLACE:
        if (++wait_counter_ >= HOLD_TICKS) { state_ = DEFAULT_BACK; command_sent_ = false; }
        break;

      case DEFAULT_BACK:
        if (!command_sent_) sendSplitTrajectory(MID, TRANSIT_DURATION);
        move_counter_++;
        if (atTarget(MID) || move_counter_ >= MOVE_TIMEOUT) {
          state_ = PICK_LEFT; wait_counter_ = 0; command_sent_ = false;
        }
        break;
    }
  }
};

#ifndef TESTING_EXLUDE_MAIN
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmGazeboNode>());
  rclcpp::shutdown();
  return 0;
}
#endif