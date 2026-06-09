#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <algorithm>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#define TESTING_EXLUDE_MAIN
#include "../../src/arm_publisher_gazebo.cpp"

class ArmPublisherGazeboTest : public ::testing::Test
{
    protected:
        void SetUp() override {
            // Initialize rclcpp once per test execution
            if (!rclcpp::ok()) {
                rclcpp::init(0, nullptr);
            }
            node = std::make_shared<ArmGazeboNode>();
        }

        void TearDown() override {
            node.reset();
            rclcpp::shutdown();
        }
        
        // FIXED: Pointer type corrected to point to the actual Node class
        std::shared_ptr<ArmGazeboNode> node;
};

// Test 1: Verifies node name and topic registration
TEST_F(ArmPublisherGazeboTest, TestNodeCreation) {
    EXPECT_EQ(std::string(node->get_name()), std::string("arm_gazebo_node"));

    auto publishers = node->get_publishers_info_by_topic("/arm_controller/joint_trajectory");
    EXPECT_FALSE(publishers.empty());

    auto gripper_publishers = node->get_publishers_info_by_topic("/gripper_controller/joint_trajectory");
    EXPECT_FALSE(gripper_publishers.empty());

    auto subscriptions = node->get_subscriptions_info_by_topic("/joint_states");
    EXPECT_FALSE(subscriptions.empty());
}

// Test 2: Verifies expected topic types are strictly correct
TEST_F(ArmPublisherGazeboTest, PublishesExpectedTopicsAndTypes)
{
    // FIXED: Removed local node instantiation, utilizing fixture 'node'
    auto topics = node->get_topic_names_and_types();

    const auto check_topic = [&](const std::string & topic_name, const std::string & expected_type) {
        auto it = topics.find(topic_name);
        ASSERT_NE(it, topics.end()) << "Topic not found: " << topic_name;
        EXPECT_NE(std::find(it->second.begin(), it->second.end(), expected_type), it->second.end())
          << "Unexpected type for topic " << topic_name;
    };

    check_topic("/arm_controller/joint_trajectory", "trajectory_msgs/msg/JointTrajectory");
    check_topic("/gripper_controller/joint_trajectory", "trajectory_msgs/msg/JointTrajectory");
    check_topic("/joint_states", "sensor_msgs/msg/JointState");
}

// Test 3: Verifies specific parameters like Sim Time overrides
TEST_F(ArmPublisherGazeboTest, HasCorrectNodeNameAndUseSimTime)
{
    // FIXED: Removed local node instantiation, utilizing fixture 'node'
    EXPECT_STREQ(node->get_name(), "arm_gazebo_node");
    ASSERT_TRUE(node->has_parameter("use_sim_time"));

    rclcpp::Parameter use_sim_time;
    node->get_parameter("use_sim_time", use_sim_time);
    EXPECT_TRUE(use_sim_time.as_bool());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}