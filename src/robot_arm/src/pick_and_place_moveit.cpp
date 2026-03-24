#include <chrono>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <my_robot_interfaces/srv/get_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

namespace fs = std::filesystem;

const double tau = 2 * M_PI;
using GetPosition = my_robot_interfaces::srv::GetPosition;
using AttachLink = linkattacher_msgs::srv::AttachLink;
using DetachLink = linkattacher_msgs::srv::DetachLink;
using SetEntityState = gazebo_msgs::srv::SetEntityState;

class PickAndPlaceNode : public rclcpp::Node {
public:
  PickAndPlaceNode()
  : Node("pick_and_place_moveit_node")
  {
    target_color_ = this->declare_parameter<std::string>("target_color", "red");
    pre_pick_offset_ = this->declare_parameter<double>("pre_pick_offset", 0.15);
    pick_height_offset_ = this->declare_parameter<double>("pick_height_offset", 0.12);
    place_height_offset_ = this->declare_parameter<double>("place_height_offset", 0.22);
    gripper_close_target_ = this->declare_parameter<double>("gripper_close_target", 0.018);
    num_iterations_ = this->declare_parameter<int>("num_iterations", 1000);
    output_dir_ = this->declare_parameter<std::string>("output_dir", "trajectory_data");

    perception_client_ = this->create_client<GetPosition>("/get_position");
    attach_client_ = this->create_client<AttachLink>("/ATTACHLINK");
    detach_client_ = this->create_client<DetachLink>("/DETACHLINK");
    set_entity_client_ = this->create_client<SetEntityState>("/set_entity_state");

    arm_joint_names_ = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint",
    };

    gripper_joint_names_ = {"hande_finger_distance"};
    rng_.seed(std::random_device{}());
  }

  // ─── Gripper helpers ───────────────────────────────────────────────

  bool close_gripper(moveit::planning_interface::MoveGroupInterface & gripper_group)
  {
    const double finger_target = std::clamp(gripper_close_target_, 0.0, 0.05);
    RCLCPP_INFO(this->get_logger(),
      "Closing gripper to hande_finger_distance=%.3f", finger_target);
    gripper_group.setJointValueTarget("hande_finger_distance", finger_target);
    return gripper_group.move() == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool open_gripper(moveit::planning_interface::MoveGroupInterface & gripper_group)
  {
    RCLCPP_INFO(this->get_logger(), "Opening gripper");
    gripper_group.setJointValueTarget("hande_finger_distance", 0.06);
    return gripper_group.move() == moveit::core::MoveItErrorCode::SUCCESS;
  }

  // ─── Link attacher helpers ─────────────────────────────────────────

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

  // ─── Perception ────────────────────────────────────────────────────

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
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        continue;
      }

      auto res = result_future.get();
      if (res->found) {
        x = res->x_position;
        y = res->y_position;
        z = res->z_position;
        RCLCPP_INFO(this->get_logger(),
          "Perception %s found: x=%.3f y=%.3f z=%.3f",
          target_type.c_str(), x, y, z);
        return true;
      }

      if (attempt % 10 == 0) {
        RCLCPP_INFO(this->get_logger(),
          "Waiting for %s detection of color '%s'... (%d/%d)",
          target_type.c_str(), target_color_.c_str(), attempt, max_attempts);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    RCLCPP_ERROR(this->get_logger(),
      "Timed out waiting for %s detection for color '%s'",
      target_type.c_str(), target_color_.c_str());
    return false;
  }

  // ─── Gazebo model teleportation ────────────────────────────────────

  bool set_model_pose(const std::string & model_name,
    double wx, double wy, double wz)
  {
    auto req = std::make_shared<SetEntityState::Request>();
    req->state.name = model_name;
    req->state.pose.position.x = wx;
    req->state.pose.position.y = wy;
    req->state.pose.position.z = wz;
    req->state.pose.orientation.w = 1.0;
    req->state.reference_frame = "world";

    auto future = set_entity_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(),
        "SetEntityState timed out for %s", model_name.c_str());
      return false;
    }
    return future.get()->success;
  }

  // Robot base in world: (-0.55, -0.35, 0.75)
  static constexpr double robot_wx_ = -0.55;
  static constexpr double robot_wy_ = -0.35;
  static constexpr double robot_wz_ = 0.75;

  // Convert base_link coords to world coords
  void base_to_world(double bx, double by, double bz,
    double & wx, double & wy, double & wz)
  {
    wx = bx + robot_wx_;
    wy = by + robot_wy_;
    wz = bz + robot_wz_;
  }

  // Randomize block and bin positions, returns positions in base_link frame
  struct SceneLayout {
    // Block position in base_link
    double block_x, block_y;
    // Bin position in base_link
    double bin_x, bin_y;
  };

  SceneLayout randomize_scene(
    moveit::planning_interface::PlanningSceneInterface & psi)
  {
    // Reachable area on table in base_link frame
    // x: distance in front of robot, y: lateral
    std::uniform_real_distribution<double> block_x_dist(0.12, 0.40);
    std::uniform_real_distribution<double> block_y_dist(0.35, 0.55);
    std::uniform_real_distribution<double> bin_x_dist(0.12, 0.45);
    std::uniform_real_distribution<double> bin_y_dist(0.25, 0.40);

    SceneLayout layout;
    // Keep generating until block and bin are at least 0.12m apart
    do {
      layout.block_x = block_x_dist(rng_);
      layout.block_y = block_y_dist(rng_);
      layout.bin_x = bin_x_dist(rng_);
      layout.bin_y = bin_y_dist(rng_);
    } while (std::hypot(layout.block_x - layout.bin_x,
                         layout.block_y - layout.bin_y) < 0.12);

    // Move ALL non-target blocks and bins off the table so they don't interfere
    const std::vector<std::string> all_colors = {"red", "blue", "yellow"};
    for (const auto & color : all_colors) {
      if (color == target_color_) continue;
      set_model_pose(color + "_block", 5.0, 5.0, 0.0);  // far away
      set_model_pose(color + "_bin", 5.0, 5.0, 0.0);
    }

    // Teleport the target block and bin in Gazebo (world frame)
    double wx, wy, wz;
    const std::string block_model = target_color_ + "_block";
    const std::string bin_model = target_color_ + "_bin";

    base_to_world(layout.block_x, layout.block_y, 0.015, wx, wy, wz);
    set_model_pose(block_model, wx, wy, wz);
    RCLCPP_INFO(this->get_logger(),
      "Teleported %s to base_link (%.3f, %.3f)",
      block_model.c_str(), layout.block_x, layout.block_y);

    base_to_world(layout.bin_x, layout.bin_y, 0.0, wx, wy, wz);
    set_model_pose(bin_model, wx, wy, wz);
    RCLCPP_INFO(this->get_logger(),
      "Teleported %s to base_link (%.3f, %.3f)",
      bin_model.c_str(), layout.bin_x, layout.bin_y);

    // Update the bin collision object in MoveIt
    update_bin_collision(psi, target_color_ + "_bin_collision",
      layout.bin_x, layout.bin_y);

    // Remove non-target bin collisions from MoveIt
    std::vector<std::string> remove_ids;
    for (const auto & color : all_colors) {
      if (color == target_color_) continue;
      remove_ids.push_back(color + "_bin_collision");
    }
    psi.removeCollisionObjects(remove_ids);

    // Wait for perception to detect the new positions
    std::this_thread::sleep_for(std::chrono::seconds(2));

    return layout;
  }

  void update_bin_collision(
    moveit::planning_interface::PlanningSceneInterface & psi,
    const std::string & bin_id, double cx, double cy)
  {
    moveit_msgs::msg::CollisionObject bin;
    bin.header.frame_id = "base_link";
    bin.id = bin_id;
    bin.operation = moveit_msgs::msg::CollisionObject::ADD;

    auto add_box = [&](double sx, double sy, double sz,
                       double px, double py, double pz)
    {
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions = {sx, sy, sz};

      geometry_msgs::msg::Pose pose;
      pose.position.x = cx + px;
      pose.position.y = cy + py;
      pose.position.z = pz;
      pose.orientation.w = 1.0;

      bin.primitives.push_back(primitive);
      bin.primitive_poses.push_back(pose);
    };

    add_box(0.10, 0.10, 0.005, 0.0, 0.0, 0.0025);
    add_box(0.005, 0.10, 0.025, -0.0475, 0.0, 0.0175);
    add_box(0.005, 0.10, 0.025, 0.0475, 0.0, 0.0175);
    add_box(0.09, 0.005, 0.025, 0.0, 0.0475, 0.0175);
    add_box(0.09, 0.005, 0.025, 0.0, -0.0475, 0.0175);

    psi.applyCollisionObjects({bin});
  }

  // ─── Trajectory logging ────────────────────────────────────────────

  void save_trajectory(
    const moveit_msgs::msg::RobotTrajectory & traj,
    const std::string & label,
    const SceneLayout & layout,
    double planning_time_ms,
    int iteration)
  {
    const auto & jt = traj.joint_trajectory;
    if (jt.points.empty()) return;

    std::ostringstream filename;
    filename << output_dir_ << "/traj_"
             << std::setfill('0') << std::setw(5) << iteration
             << "_" << label << ".json";

    std::ofstream file(filename.str());
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open %s", filename.str().c_str());
      return;
    }

    file << std::setprecision(8);
    file << "{\n";
    file << "  \"iteration\": " << iteration << ",\n";
    file << "  \"label\": \"" << label << "\",\n";
    file << "  \"planning_time_ms\": " << planning_time_ms << ",\n";

    // Scene info
    file << "  \"scene\": {\n";
    file << "    \"block_x\": " << layout.block_x << ",\n";
    file << "    \"block_y\": " << layout.block_y << ",\n";
    file << "    \"bin_x\": " << layout.bin_x << ",\n";
    file << "    \"bin_y\": " << layout.bin_y << "\n";
    file << "  },\n";

    // Joint names
    file << "  \"joint_names\": [";
    for (size_t i = 0; i < jt.joint_names.size(); i++) {
      file << "\"" << jt.joint_names[i] << "\"";
      if (i + 1 < jt.joint_names.size()) file << ", ";
    }
    file << "],\n";

    // Start and goal joints
    file << "  \"start_joints\": [";
    for (size_t i = 0; i < jt.points.front().positions.size(); i++) {
      file << jt.points.front().positions[i];
      if (i + 1 < jt.points.front().positions.size()) file << ", ";
    }
    file << "],\n";

    file << "  \"goal_joints\": [";
    for (size_t i = 0; i < jt.points.back().positions.size(); i++) {
      file << jt.points.back().positions[i];
      if (i + 1 < jt.points.back().positions.size()) file << ", ";
    }
    file << "],\n";

    // Full trajectory
    file << "  \"num_waypoints\": " << jt.points.size() << ",\n";
    file << "  \"trajectory\": [\n";
    for (size_t w = 0; w < jt.points.size(); w++) {
      file << "    [";
      for (size_t j = 0; j < jt.points[w].positions.size(); j++) {
        file << jt.points[w].positions[j];
        if (j + 1 < jt.points[w].positions.size()) file << ", ";
      }
      file << "]";
      if (w + 1 < jt.points.size()) file << ",";
      file << "\n";
    }
    file << "  ]\n";
    file << "}\n";

    file.close();

    // Accumulate iteration totals
    iter_planning_time_ms_ += planning_time_ms;
    iter_total_waypoints_ += static_cast<int>(jt.points.size());

    RCLCPP_INFO(this->get_logger(), "Saved %s (%zu waypoints)",
      filename.str().c_str(), jt.points.size());
  }

  // ─── Plan, log, and execute (replaces move()) ─────────────────────

  bool plan_move_and_log(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const std::string & label,
    const SceneLayout & layout,
    int iteration)
  {
    // Let robot settle so trajectory start matches current state
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    move_group.setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto t0 = std::chrono::steady_clock::now();
    bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    double plan_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "Planning failed for %s", label.c_str());
      return false;
    }

    save_trajectory(plan.trajectory_, label, layout, plan_ms, iteration);

    ok = (move_group.execute(plan.trajectory_) ==
          moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "Execution failed for %s", label.c_str());
      return false;
    }
    return true;
  }

  // ─── Pick sequence ─────────────────────────────────────────────────

  bool pick(
    moveit::planning_interface::MoveGroupInterface & move_group,
    moveit::planning_interface::MoveGroupInterface & gripper_group,
    const SceneLayout & layout, int iteration)
  {
    RCLCPP_INFO(this->get_logger(), "Starting pick sequence");
    open_gripper(gripper_group);

    double x = 0.0, y = 0.0, z = 0.0;
    if (!query_target_with_retry("block", x, y, z)) return false;

    // Move to pre-pick
    geometry_msgs::msg::PoseStamped pick_pose;
    pick_pose.header.frame_id = "base_link";
    pick_pose.pose.position.x = x;
    pick_pose.pose.position.y = y;
    pick_pose.pose.position.z = z + pre_pick_offset_;

    tf2::Quaternion orientation;
    orientation.setRPY(M_PI, 0, 0);
    pick_pose.pose.orientation = tf2::toMsg(orientation);

    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(pick_pose);
    if (!plan_move_and_log(move_group, "pre_pick", layout, iteration)) {
      return false;
    }

    // Cartesian descent to grasp
    const double target_pick_z = z + pick_height_offset_;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose down_pose = move_group.getCurrentPose().pose;
    down_pose.position.x = x;
    down_pose.position.y = y;
    down_pose.position.z = target_pick_z;
    waypoints.push_back(down_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    move_group.setStartStateToCurrentState();

    auto t0 = std::chrono::steady_clock::now();
    double fraction = move_group.computeCartesianPath(
      waypoints, 0.005, 0.0, trajectory);
    double plan_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    RCLCPP_INFO(this->get_logger(),
      "Cartesian descent fraction: %.3f (from z=%.3f to z=%.3f)",
      fraction, move_group.getCurrentPose().pose.position.z, target_pick_z);

    if (fraction < 0.50) {
      RCLCPP_ERROR(this->get_logger(), "Cartesian descent fraction too low: %.3f", fraction);
      return false;
    }

    save_trajectory(trajectory, "pick_descend", layout, plan_ms, iteration);

    if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute pick descent");
      return false;
    }

    // Grasp
    close_gripper(gripper_group);
    attach_block(target_color_ + "_block");

    move_group.clearPoseTargets();
    return true;
  }

  // ─── Place sequence ────────────────────────────────────────────────

  bool place(
    moveit::planning_interface::MoveGroupInterface & move_group,
    moveit::planning_interface::MoveGroupInterface & gripper_group,
    const SceneLayout & layout, int iteration)
  {
    RCLCPP_INFO(this->get_logger(), "Starting place sequence");
    double x = 0.0, y = 0.0, z = 0.0;
    if (!query_target_with_retry("bin", x, y, z)) return false;

    tf2::Quaternion orientation;
    orientation.setRPY(M_PI, 0, 0);
    const auto desired_orientation = tf2::toMsg(orientation);
    const double transfer_z = z + pre_pick_offset_;

    // 1) Lift
    geometry_msgs::msg::PoseStamped lift_pose;
    lift_pose.header.frame_id = "base_link";
    lift_pose.pose = move_group.getCurrentPose().pose;
    lift_pose.pose.orientation = desired_orientation;
    if (lift_pose.pose.position.z < transfer_z) {
      lift_pose.pose.position.z = transfer_z;
      move_group.setStartStateToCurrentState();
      move_group.setPoseTarget(lift_pose);
      if (!plan_move_and_log(move_group, "place_lift", layout, iteration)) {
        return false;
      }
    }

    // 2) Move above bin
    geometry_msgs::msg::PoseStamped place_pose;
    place_pose.header.frame_id = "base_link";
    place_pose.pose.position.x = x;
    place_pose.pose.position.y = y;
    place_pose.pose.position.z = z + place_height_offset_;
    place_pose.pose.orientation = desired_orientation;

    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(place_pose);
    if (!plan_move_and_log(move_group, "place_above_bin", layout, iteration)) {
      return false;
    }

    // 3) Release
    detach_block(target_color_ + "_block");
    open_gripper(gripper_group);

    // 4) Retreat
    geometry_msgs::msg::PoseStamped retreat_pose;
    retreat_pose.header.frame_id = "base_link";
    retreat_pose.pose = place_pose.pose;
    retreat_pose.pose.position.z = transfer_z;

    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(retreat_pose);
    if (!plan_move_and_log(move_group, "place_retreat", layout, iteration)) {
      return false;
    }

    move_group.clearPoseTargets();
    return true;
  }

  // ─── Main loop ─────────────────────────────────────────────────────

  bool run()
  {
    RCLCPP_INFO(this->get_logger(), "pick_and_place_moveit_node started (data collection mode)");

    // Wait for services
    if (!perception_client_->wait_for_service(std::chrono::seconds(30))) {
      RCLCPP_ERROR(this->get_logger(), "Service /get_position not available");
      return false;
    }
    if (!set_entity_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Service /set_entity_state not available");
      return false;
    }

    fs::create_directories(output_dir_);

    moveit::planning_interface::MoveGroupInterface arm(
      shared_from_this(), "ur3e_manipulator");
    moveit::planning_interface::MoveGroupInterface gripper_group(
      shared_from_this(), "gripper");
    moveit::planning_interface::PlanningSceneInterface psi;

    arm.startStateMonitor(2.0);
    arm.setPoseReferenceFrame("base_link");
    arm.setPlanningTime(5.0);
    arm.setNumPlanningAttempts(10);
    arm.setMaxVelocityScalingFactor(0.2);
    arm.setMaxAccelerationScalingFactor(0.2);

    // Add table collision (fixed across all iterations)
    {
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
      psi.applyCollisionObjects({table});
      RCLCPP_INFO(this->get_logger(), "Added table collision object");
    }

    const auto state = arm.getCurrentState(10.0);
    if (!state) {
      RCLCPP_ERROR(this->get_logger(), "No current robot state");
      return false;
    }

    int success_count = 0;

    // Write CSV header for iteration summary
    {
      std::string summary_path = output_dir_ + "/iteration_summary.csv";
      if (!fs::exists(summary_path)) {
        std::ofstream header(summary_path);
        header << "trial,planning_success,planning_time_s,num_waypoints\n";
        header.close();
      }
    }

    for (int iter = 0; iter < num_iterations_; iter++) {
      RCLCPP_INFO(this->get_logger(),
        "===== Iteration %d/%d (successes: %d) =====",
        iter + 1, num_iterations_, success_count);

      // Reset per-iteration accumulators
      iter_planning_time_ms_ = 0.0;
      iter_total_waypoints_ = 0;

      // Go home first — settle before planning
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      arm.setStartStateToCurrentState();
      arm.setNamedTarget("home");
      if (arm.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Failed to move home, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        arm.setStartStateToCurrentState();
        arm.setNamedTarget("home");
        if (arm.move() != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_ERROR(this->get_logger(), "Failed to move home twice, skipping");
          continue;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // Randomize scene
      SceneLayout layout = randomize_scene(psi);

      // Run pick and place
      bool ok = pick(arm, gripper_group, layout, iter);
      if (ok) {
        ok = place(arm, gripper_group, layout, iter);
      }

      if (ok) {
        success_count++;
        RCLCPP_INFO(this->get_logger(),
          "Iteration %d SUCCEEDED (total: %d) | planning_time=%.3fs waypoints=%d",
          iter + 1, success_count,
          iter_planning_time_ms_ / 1000.0, iter_total_waypoints_);
      } else {
        RCLCPP_WARN(this->get_logger(),
          "Iteration %d FAILED, continuing to next | planning_time=%.3fs waypoints=%d",
          iter + 1, iter_planning_time_ms_ / 1000.0, iter_total_waypoints_);
        // Make sure block is detached if pick succeeded but place failed
        detach_block(target_color_ + "_block");
        open_gripper(gripper_group);
      }

      // Append to iteration summary CSV
      {
        std::ofstream csv(output_dir_ + "/iteration_summary.csv", std::ios::app);
        csv << (iter + 1) << ","
            << (ok ? "Yes" : "No") << ","
            << std::fixed << std::setprecision(2) << (iter_planning_time_ms_ / 1000.0) << ","
            << iter_total_waypoints_ << "\n";
        csv.close();
      }
    }

    RCLCPP_INFO(this->get_logger(),
      "Data collection complete: %d/%d successful iterations, saved to %s",
      success_count, num_iterations_, output_dir_.c_str());
    return true;
  }

private:
  std::string target_color_;
  double pre_pick_offset_;
  double pick_height_offset_;
  double place_height_offset_;
  double gripper_close_target_;
  int num_iterations_;
  std::string output_dir_;
  rclcpp::Client<GetPosition>::SharedPtr perception_client_;
  rclcpp::Client<AttachLink>::SharedPtr attach_client_;
  rclcpp::Client<DetachLink>::SharedPtr detach_client_;
  rclcpp::Client<SetEntityState>::SharedPtr set_entity_client_;
  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> gripper_joint_names_;
  std::mt19937 rng_;

  // Per-iteration accumulators
  double iter_planning_time_ms_ = 0.0;
  int iter_total_waypoints_ = 0;
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
