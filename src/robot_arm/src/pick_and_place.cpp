#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <my_robot_interfaces/srv/get_position.hpp>
#include <my_robot_interfaces/srv/plan_joint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;
using FollowJT = control_msgs::action::FollowJointTrajectory;
using GetPosition = my_robot_interfaces::srv::GetPosition;
using GetPositionIK = moveit_msgs::srv::GetPositionIK;
using PlanJoint = my_robot_interfaces::srv::PlanJoint;

class PickAndPlaceNode : public rclcpp::Node {
public:
  PickAndPlaceNode()
  : Node("pick_and_place_node")
  {
    target_color_ = this->declare_parameter<std::string>("target_color", "red");

    arm_joint_names_ = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint",
    };

    gripper_joint_names_ = {
      "hande_left_finger_joint",
      "hande_right_finger_joint",
    };

    // Service clients
    perception_client_ = this->create_client<GetPosition>("/get_position");
    plan_client_ = this->create_client<PlanJoint>("/plan_joint");
    ik_client_ = this->create_client<GetPositionIK>("/compute_ik");

    // Action clients
    arm_action_client_ = rclcpp_action::create_client<FollowJT>(
      this, "/joint_trajectory_controller/follow_joint_trajectory");
    gripper_action_client_ = rclcpp_action::create_client<FollowJT>(
      this, "/robotiq_gripper_controller/follow_joint_trajectory");

    // Joint state subscription
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 20,
      std::bind(&PickAndPlaceNode::jointStateCb, this, std::placeholders::_1));
  }

  bool run()
  {
    if (!waitForInterfaces()) {
      return false;
    }

    RCLCPP_INFO(get_logger(), "pick_and_place_node scaffold is running");
    RCLCPP_INFO(get_logger(), "target_color: %s", target_color_.c_str());

    /*
      Plan (left intentionally as comments for you to implement):

      1. Read current joint state from /joint_states.
      2. Move arm to home configuration.
      3. Open gripper.
      4. Call /get_position with target_color to get perceived x, y, z.
      5. Build W1 above-object pose using perceived x/y and carry height.
      6. Solve IK for W1 using /compute_ik.
      7. Plan and execute to W1 using /plan_joint + arm FollowJointTrajectory.
      8. Build W2 pick-depth pose.
      9. Solve IK for W2, then plan and execute.
      10. Close gripper.
      11. Lift back to carry height (W3).
      12. Move in-plane to bin XY at carry height (W4).
      13. Descend at bin (W5).
      14. Open gripper.
      15. Retreat to carry height (W6).
      16. Return arm to home.

      Suggested helper methods to implement next:
      - queryPerception()
      - solveIkForPose()
      - planArm(start_conf, goal_conf)
      - executeArmTrajectory(traj)
      - commandGripper(position)
      - getCurrentArmJoints()
    */

    return true;
  }

private:
  template<typename ServiceT>
  bool waitForServiceWithRetry(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const std::string & service_name,
    int timeout_sec)
  {
    for (int waited = 0; waited < timeout_sec; ++waited) {
      if (client->wait_for_service(1s)) {
        return true;
      }
      RCLCPP_INFO(
        get_logger(),
        "Waiting for service '%s'... (%d/%d s)",
        service_name.c_str(),
        waited + 1,
        timeout_sec);
    }
    return false;
  }

  bool waitForActionWithRetry(
    const rclcpp_action::Client<FollowJT>::SharedPtr & client,
    const std::string & name,
    int timeout_sec)
  {
    for (int waited = 0; waited < timeout_sec; ++waited) {
      if (client->wait_for_action_server(1s)) {
        return true;
      }
      RCLCPP_INFO(
        get_logger(),
        "Waiting for action '%s'... (%d/%d s)",
        name.c_str(),
        waited + 1,
        timeout_sec);
    }
    return false;
  }

  bool waitForInterfaces()
  {
    if (!waitForServiceWithRetry<GetPosition>(perception_client_, "/get_position", 30)) {
      RCLCPP_ERROR(get_logger(), "Service /get_position not available");
      return false;
    }
    if (!waitForServiceWithRetry<PlanJoint>(plan_client_, "/plan_joint", 30)) {
      RCLCPP_ERROR(get_logger(), "Service /plan_joint not available");
      return false;
    }
    if (!waitForServiceWithRetry<GetPositionIK>(ik_client_, "/compute_ik", 30)) {
      RCLCPP_ERROR(get_logger(), "Service /compute_ik not available");
      return false;
    }
    if (!waitForActionWithRetry(arm_action_client_, "/joint_trajectory_controller/follow_joint_trajectory", 30)) {
      RCLCPP_ERROR(get_logger(), "Arm FollowJointTrajectory action not available");
      return false;
    }
    if (!waitForActionWithRetry(gripper_action_client_, "/robotiq_gripper_controller/follow_joint_trajectory", 30)) {
      RCLCPP_ERROR(get_logger(), "Gripper FollowJointTrajectory action not available");
      return false;
    }
    return true;
  }

  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      joint_position_map_[msg->name[i]] = msg->position[i];
    }
    have_joint_state_ = true;
  }

  std::string target_color_;

  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> gripper_joint_names_;

  rclcpp::Client<GetPosition>::SharedPtr perception_client_;
  rclcpp::Client<PlanJoint>::SharedPtr plan_client_;
  rclcpp::Client<GetPositionIK>::SharedPtr ik_client_;

  rclcpp_action::Client<FollowJT>::SharedPtr arm_action_client_;
  rclcpp_action::Client<FollowJT>::SharedPtr gripper_action_client_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::mutex joint_state_mutex_;
  std::map<std::string, double> joint_position_map_;
  bool have_joint_state_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickAndPlaceNode>();
  const bool ok = node->run();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
