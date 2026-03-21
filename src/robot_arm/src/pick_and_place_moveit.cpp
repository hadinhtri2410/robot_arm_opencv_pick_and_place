#include <chrono>
#include <algorithm>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <my_robot_interfaces/srv/get_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

const double tau = 2 * M_PI;
using GetPosition = my_robot_interfaces::srv::GetPosition;
using AttachLink = linkattacher_msgs::srv::AttachLink;
using DetachLink = linkattacher_msgs::srv::DetachLink;

class PickAndPlaceNode : public rclcpp::Node {
public:
  PickAndPlaceNode()
  : Node("pick_and_place_moveit_node")
  {
    target_color_ = this->declare_parameter<std::string>("target_color", "red");
    pre_pick_offset_ = this->declare_parameter<double>("pre_pick_offset", 0.15);
    pick_height_offset_ = this->declare_parameter<double>("pick_height_offset", 0.085);
    place_height_offset_ = this->declare_parameter<double>("place_height_offset", 0.12);
    gripper_close_target_ = this->declare_parameter<double>("gripper_close_target", 0.018);
    perception_client_ = this->create_client<GetPosition>("/get_position");
    attach_client_ = this->create_client<AttachLink>("/ATTACHLINK");
    detach_client_ = this->create_client<DetachLink>("/DETACHLINK");

    arm_joint_names_ = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint",
    };

    gripper_joint_names_ = {"hande_finger_distance"};
  }

  bool close_gripper(moveit::planning_interface::MoveGroupInterface & gripper_group)
  {
    const double finger_target = std::clamp(gripper_close_target_, 0.0, 0.05);
    RCLCPP_INFO(
      this->get_logger(),
      "Closing gripper to hande_finger_distance=%.3f",
      finger_target);
    gripper_group.setJointValueTarget("hande_finger_distance", finger_target);
    const bool ok = (gripper_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_WARN(this->get_logger(), "Gripper close command failed");
    }
    return ok;
  }

  bool open_gripper(moveit::planning_interface::MoveGroupInterface & gripper_group)
  {
    RCLCPP_INFO(this->get_logger(), "Opening gripper");
    gripper_group.setJointValueTarget("hande_finger_distance", 0.06);
    const bool ok = (gripper_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_WARN(this->get_logger(), "Gripper open command failed");
    }
    return ok;
  }

  bool attach_block(const std::string & block_model)
  {
    auto req = std::make_shared<AttachLink::Request>();
    req->model1_name = "cobot";
    req->link1_name = "hande_left_finger";
    req->model2_name = block_model;
    req->link2_name = "block_link";
    auto future = attach_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "ATTACHLINK service timed out");
      return false;
    }
    auto res = future.get();
    RCLCPP_INFO(this->get_logger(), "ATTACHLINK: %s", res->message.c_str());
    return res->success;
  }

  bool detach_block(const std::string & block_model)
  {
    auto req = std::make_shared<DetachLink::Request>();
    req->model1_name = "cobot";
    req->link1_name = "hande_left_finger";
    req->model2_name = block_model;
    req->link2_name = "block_link";
    auto future = detach_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "DETACHLINK service timed out");
      return false;
    }
    auto res = future.get();
    RCLCPP_INFO(this->get_logger(), "DETACHLINK: %s", res->message.c_str());
    return res->success;
  }

  bool query_target_with_retry(
    const std::string & target_type, double & x, double & y, double & z,
    int max_attempts = 120)
  {
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
      auto req = std::make_shared<GetPosition::Request>();
      req->get_position = true;
      req->target_color = target_color_;
      req->target_type = target_type;

      auto result_future = perception_client_->async_send_request(req);
      const auto status = result_future.wait_for(std::chrono::seconds(2));
      if (status != std::future_status::ready) {
        RCLCPP_WARN(
          this->get_logger(),
          "Call to /get_position timed out for type '%s' (attempt %d/%d)",
          target_type.c_str(), attempt, max_attempts);
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        continue;
      }

      auto res = result_future.get();
      if (res->found) {
        x = res->x_position;
        y = res->y_position;
        z = res->z_position;
        RCLCPP_INFO(
          this->get_logger(),
          "Perception %s found: x=%.3f y=%.3f z=%.3f",
          target_type.c_str(), x, y, z);
        return true;
      }

      if (attempt % 10 == 0) {
        RCLCPP_INFO(
          this->get_logger(),
          "Waiting for %s detection of color '%s'... (%d/%d)",
          target_type.c_str(), target_color_.c_str(), attempt, max_attempts);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    RCLCPP_ERROR(
      this->get_logger(),
      "Timed out waiting for %s detection for color '%s'",
      target_type.c_str(), target_color_.c_str());
    return false;
  }

  bool pick(
    moveit::planning_interface::MoveGroupInterface & move_group,
    moveit::planning_interface::MoveGroupInterface & gripper_group)
  {
    RCLCPP_INFO(this->get_logger(), "Starting pick sequence");
    open_gripper(gripper_group);

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!query_target_with_retry("block", x, y, z)) {
      return false;
    }

    const float position_x = static_cast<float>(x);
    const float position_y = static_cast<float>(y);
    const float position_z = static_cast<float>(z + pre_pick_offset_);

    geometry_msgs::msg::PoseStamped pick_pose;
    pick_pose.header.frame_id = "base_link";
    pick_pose.pose.position.x = position_x;
    pick_pose.pose.position.y = position_y;
    pick_pose.pose.position.z = position_z;

    tf2::Quaternion orientation;
    orientation.setRPY(M_PI, 0, 0);
    pick_pose.pose.orientation = tf2::toMsg(orientation);

    move_group.setPoseTarget(pick_pose);
    auto ok = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to pre-pick pose");
      return false;
    }

    const double target_pick_z = z + pick_height_offset_;
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Start from the current pose after reaching pre-pick
    geometry_msgs::msg::Pose down_pose = move_group.getCurrentPose().pose;
    down_pose.position.x = x;
    down_pose.position.y = y;
    down_pose.position.z = target_pick_z;
    waypoints.push_back(down_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.005;   // 5 mm
    const double jump_threshold = 0.0;

    // Let robot settle after move() before computing Cartesian path
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    move_group.setStartStateToCurrentState();

    double fraction = move_group.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(this->get_logger(), "Cartesian path fraction: %.3f", fraction);

    if (fraction < 0.90) {
    RCLCPP_ERROR(this->get_logger(), "Failed to compute full downward Cartesian path");
    return false;
    }

    ok = (move_group.execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute downward Cartesian path");
    return false;
    }

    if (!close_gripper(gripper_group)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Gripper close did not report success; continuing to transfer motion");
    }

    // Attach block to gripper via link attacher so it doesn't slip
    const std::string block_model = target_color_ + "_block";
    attach_block(block_model);

    move_group.clearPoseTargets();
    return true;
  }

  bool place(
    moveit::planning_interface::MoveGroupInterface & move_group,
    moveit::planning_interface::MoveGroupInterface & gripper_group)
  {
    RCLCPP_INFO(this->get_logger(), "Starting place sequence");
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!query_target_with_retry("bin", x, y, z)) {
      return false;
    }

    geometry_msgs::msg::PoseStamped place_pose;
    place_pose.header.frame_id = "base_link";
    place_pose.pose.position.x = x;
    place_pose.pose.position.y = y;
    place_pose.pose.position.z = z + place_height_offset_;

    tf2::Quaternion orientation;
    orientation.setRPY(M_PI, 0, 0);
    const auto desired_orientation = tf2::toMsg(orientation);

    auto execute_pose_target =
      [&](const geometry_msgs::msg::PoseStamped & target, const char * error_msg) -> bool
      {
        move_group.setStartStateToCurrentState();
        move_group.setPoseTarget(target);
        const bool ok = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok) {
          RCLCPP_ERROR(this->get_logger(), "%s", error_msg);
          return false;
        }
        return true;
      };

    // 1) Lift to a safe transfer height from current pose.
    geometry_msgs::msg::PoseStamped lift_pose;
    lift_pose.header.frame_id = "base_link";
    lift_pose.pose = move_group.getCurrentPose().pose;
    lift_pose.pose.orientation = desired_orientation;
    const double transfer_z = z + pre_pick_offset_;
    if (lift_pose.pose.position.z < transfer_z) {
      lift_pose.pose.position.z = transfer_z;
      if (!execute_pose_target(lift_pose, "Failed to lift to transfer height before place")) {
        return false;
      }
    }

    // 2) Move above bin at place height (skip separate descend step).
    place_pose.pose.orientation = desired_orientation;
    if (!execute_pose_target(place_pose, "Failed to move above bin for place")) {
      return false;
    }

    geometry_msgs::msg::PoseStamped above_place_pose;
    above_place_pose.header.frame_id = "base_link";
    above_place_pose.pose = place_pose.pose;
    above_place_pose.pose.position.z = transfer_z;

    // Detach block before opening gripper
    const std::string block_model = target_color_ + "_block";
    if (!detach_block(block_model)) {
      RCLCPP_ERROR(this->get_logger(), "Detach failed; block is still attached");
      return false;
    }

if (!open_gripper(gripper_group)) {
  RCLCPP_WARN(this->get_logger(), "Gripper open command failed after detach");
}

    // 4) Retreat back up after releasing.
    above_place_pose.pose.position.z = transfer_z;
    if (!execute_pose_target(above_place_pose, "Failed to retreat after place")) {
      return false;
    }

    move_group.clearPoseTargets();
    return true;
  }

  bool run()
  {
    RCLCPP_INFO(this->get_logger(), "pick_and_place_moveit_node started");
    if (!perception_client_->wait_for_service(std::chrono::seconds(30))) {
      RCLCPP_ERROR(this->get_logger(), "Service /get_position not available");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Perception service is available");
    moveit::planning_interface::MoveGroupInterface arm(shared_from_this(), "ur3e_manipulator");
    moveit::planning_interface::MoveGroupInterface gripper_group(shared_from_this(), "gripper");
    arm.startStateMonitor(2.0);
    arm.setPoseReferenceFrame("base_link");
    arm.setPlanningTime(5.0);
    arm.setNumPlanningAttempts(10);
    arm.setMaxVelocityScalingFactor(0.2);
    arm.setMaxAccelerationScalingFactor(0.2);

    // Add table and bins as collision objects so MoveIt plans around them.
    // Table in world: center (0, 0, 0.375), size 1.2 x 0.8 x 0.75m.
    // Robot base_link in world: (-0.55, -0.35, 0.75).
    // Table center in base_link: (0.55, 0.35, -0.375).
    // Bin model origins in base_link (x, y, z): red(0.14, 0.335, 0.0),
    // blue(0.34, 0.335, 0.0), yellow(0.54, 0.335, 0.0).
    {
      moveit::planning_interface::PlanningSceneInterface psi;
      std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

      moveit_msgs::msg::CollisionObject table;
      table.header.frame_id = "base_link";
      table.id = "table";
      table.operation = moveit_msgs::msg::CollisionObject::ADD;

      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions = {1.2, 0.8, 0.75};

      geometry_msgs::msg::Pose table_pose;
      table_pose.position.x = 0.55;
      table_pose.position.y = 0.35;
      table_pose.position.z = -0.375;
      table_pose.orientation.w = 1.0;

      table.primitives.push_back(box);
      table.primitive_poses.push_back(table_pose);
      collision_objects.push_back(table);

      auto make_bin_collision_object =
        [&](const std::string & id, double center_x, double center_y) -> moveit_msgs::msg::CollisionObject
        {
          moveit_msgs::msg::CollisionObject bin;
          bin.header.frame_id = "base_link";
          bin.id = id;
          bin.operation = moveit_msgs::msg::CollisionObject::ADD;

          auto add_box =
            [&](double sx, double sy, double sz, double px, double py, double pz)
            {
              shape_msgs::msg::SolidPrimitive primitive;
              primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
              primitive.dimensions = {sx, sy, sz};

              geometry_msgs::msg::Pose pose;
              pose.position.x = center_x + px;
              pose.position.y = center_y + py;
              pose.position.z = pz;
              pose.orientation.w = 1.0;

              bin.primitives.push_back(primitive);
              bin.primitive_poses.push_back(pose);
            };

          // Bottom plate.
          add_box(0.10, 0.10, 0.005, 0.0, 0.0, 0.0025);
          add_box(0.005, 0.10, 0.025, -0.0475, 0.0, 0.0175);   // left
          add_box(0.005, 0.10, 0.025, 0.0475, 0.0, 0.0175);    // right
          add_box(0.09, 0.005, 0.025, 0.0, 0.0475, 0.0175);    // front
          add_box(0.09, 0.005, 0.025, 0.0, -0.0475, 0.0175);   // back

          return bin;
        };

      collision_objects.push_back(make_bin_collision_object("red_bin_collision", 0.14, 0.335));
      collision_objects.push_back(make_bin_collision_object("blue_bin_collision", 0.34, 0.335));
      collision_objects.push_back(make_bin_collision_object("yellow_bin_collision", 0.54, 0.335));

      psi.applyCollisionObjects(collision_objects);
      RCLCPP_INFO(this->get_logger(), "Added table + bin collision objects to planning scene");
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for MoveIt current state");
    const auto state = arm.getCurrentState(10.0);
    if (!state) {
      RCLCPP_ERROR(this->get_logger(), "No current robot state from MoveIt after timeout");
      return false;
    }

    // Make a visible initial move to validate MoveIt/controller wiring.
    RCLCPP_INFO(this->get_logger(), "Moving arm to named target: home");
    arm.setNamedTarget("home");
    if (arm.move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to home target");
      return false;
    }

    if (!pick(arm, gripper_group)) {
      RCLCPP_ERROR(this->get_logger(), "Pick sequence failed");
      return false;
    }
    if (!place(arm, gripper_group)) {
      RCLCPP_ERROR(this->get_logger(), "Place sequence failed");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Pick and place sequence complete");
    return true;
  }

private:
  std::string target_color_;
  double pre_pick_offset_;
  double pick_height_offset_;
  double place_height_offset_;
  double gripper_close_target_;
  rclcpp::Client<GetPosition>::SharedPtr perception_client_;
  rclcpp::Client<AttachLink>::SharedPtr attach_client_;
  rclcpp::Client<DetachLink>::SharedPtr detach_client_;
  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> gripper_joint_names_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickAndPlaceNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread t([&executor]() {executor.spin();});

  node->run();

  executor.cancel();
  t.join();
  rclcpp::shutdown();
  return 0;
}
