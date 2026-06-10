#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <thread>
#include <chrono>

using namespace moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_(std::make_shared<rclcpp::Node>("mtc_node", options)) {}

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
    return node_->get_node_base_interface();
  }

  void initTask() {
    task_ = Task("servoarm_pick_place");
    task_.loadRobotModel(node_);

    auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

    // Match this to your SRDF planning group name, NOT the controller name
    const std::string arm_group = "arm_controller";

    // ── STAGE 1: Current State ───────────────────────────────────────────────
    {
      auto current_state = std::make_unique<stages::CurrentState>("current_state");
      task_.add(std::move(current_state));
      // If you need a raw pointer to this stage later (e.g. to feed properties
      // into a downstream stage), declare it BEFORE the unique_ptr:
      //   stages::CurrentState* current_state_ptr = nullptr;
      //   auto current_state = std::make_unique<stages::CurrentState>("...");
      //   current_state_ptr = current_state.get();   // safe: raw ptr before move
      //   task_.add(std::move(current_state));        // then move
    }

    // ── STAGE 2: Connect ─────────────────────────────────────────────────────
    {
      stages::Connect::GroupPlannerVector planners = {
        { arm_group, joint_interpolation }
      };
      auto connect = std::make_unique<stages::Connect>("connect_to_home", planners);
      connect->setTimeout(5.0);
      connect->properties().configureInitFrom(Stage::PARENT);
      task_.add(std::move(connect));
    }

    // ── STAGE 3: Move to Named Pose ───────────────────────────────────────────
    {
      auto move_to_home = std::make_unique<stages::MoveTo>("move_to_home", joint_interpolation);
      move_to_home->setGroup(arm_group);
      move_to_home->setGoal("home");  // must match a named state in your SRDF
      task_.add(std::move(move_to_home));
    }

    RCLCPP_INFO(node_->get_logger(), "MTC Task Pipeline Initialized Successfully.");
  }

  void planAndExecute() {
    try {
      task_.plan(5);
      RCLCPP_INFO(node_->get_logger(), "Planning complete!");

      if (task_.numSolutions() > 0) {
        task_.execute(*task_.solutions().front());
        RCLCPP_INFO(node_->get_logger(), "Execution triggered successfully!");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "MTC failed to find a valid solution.");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "MTC Exception caught: %s", e.what());
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  Task task_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task = std::make_shared<MTCTaskNode>(options);

  std::thread plan_thread([mtc_task]() {
    std::this_thread::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(rclcpp::get_logger("mtc_node"), "Initializing MTC Task Tree...");
    mtc_task->initTask();
    RCLCPP_INFO(rclcpp::get_logger("mtc_node"), "Generating Trajectory Plans...");
    mtc_task->planAndExecute();
  });

  rclcpp::spin(mtc_task->get_node_base_interface());

  if (plan_thread.joinable()) {
    plan_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}