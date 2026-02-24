#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>


int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("robot_model_and_state_tutorial", node_options);
    const auto& logger = node->get_logger();
    
    robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
    moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    RCLCPP_INFO(logger, "Model frame: %s", kinematic_model->getModelFrame().c_str());
    moveit::core::RobotStatePtr robot_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
    robot_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("ur3e_manipulator");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); i++){
        RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    joint_values[0] = 1.0;
    robot_state->setJointGroupPositions(joint_model_group, joint_values);
    RCLCPP_INFO_STREAM(logger, "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));
    robot_state->enforceBounds();
    RCLCPP_INFO_STREAM(logger, "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));
    //Forward kinematics
    robot_state->setToRandomPositions(joint_model_group);
    const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("tool0");
    RCLCPP_INFO_STREAM(logger, "End effector position: \n" << end_effector_state.matrix());
    RCLCPP_INFO_STREAM(logger, "End effector translation: \n" << end_effector_state.translation());
    RCLCPP_INFO_STREAM(logger, "End effector rotation: \n" << end_effector_state.rotation());
    //Inverse kinematics
    double timeout = 0.1;
    bool found_ik = robot_state->setFromIK(joint_model_group, end_effector_state, timeout);
    if (found_ik){
        robot_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); i++){
            RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    } else {
        RCLCPP_INFO(logger, "Did not find IK solution");
    }
    //Get the Jacobian
    Eigen::Vector3d& reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    robot_state->getJacobian(joint_model_group,
                             robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                             reference_point_position, jacobian);
    RCLCPP_INFO_STREAM(logger, "Jacobian: \n" << jacobian);
    //check collisions
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene::PlanningScene planning_scene(kinematic_model);
    moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    collision_request.contacts = true;
    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result, current_state);
    RCLCPP_INFO_STREAM(logger, "Test state is " << (collision_result.collision ? "in" : "not in") << " collision");

    //add the world model
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(7);
    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = kinematic_model->getModelFrame();
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.5;
    collision_objects[0].primitives[0].dimensions[1] = 1.0;
    collision_objects[0].primitives[0].dimensions[2] = 0.5;
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = 0.25;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[1].id = "box1";
    collision_objects[1].header.frame_id = kinematic_model->getModelFrame();
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.5;
    collision_objects[1].primitives[0].dimensions[1] = 1.0;
    collision_objects[1].primitives[0].dimensions[2] = 0.5;
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.5;
    collision_objects[1].primitive_poses[0].position.y = 0.5;
    collision_objects[1].primitive_poses[0].position.z = 0.25;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;

    collision_objects[2].id = "box2";
    collision_objects[2].header.frame_id = kinematic_model->getModelFrame();
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.5;
    collision_objects[2].primitives[0].dimensions[1] = 1.0;
    collision_objects[2].primitives[0].dimensions[2] = 0.5;
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.5;
    collision_objects[2].primitive_poses[0].position.y = -0.5;
    collision_objects[2].primitive_poses[0].position.z = 0.25;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;

    collision_objects[2].id = "box2";
    collision_objects[2].header.frame_id = kinematic_model->getModelFrame();
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.5;
    collision_objects[2].primitives[0].dimensions[1] = 1.0;
    collision_objects[2].primitives[0].dimensions[2] = 0.5;
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.5;
    collision_objects[2].primitive_poses[0].position.y = -0.5;
    collision_objects[2].primitive_poses[0].position.z = 0.25;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;

    collision_objects[3].id = "box3";
    collision_objects[3].header.frame_id = kinematic_model->getModelFrame();
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
    collision_objects[3].primitives[0].dimensions.resize(3);
    collision_objects[3].primitives[0].dimensions[0] = 0.5;
    collision_objects[3].primitives[0].dimensions[1] = 1.0;
    collision_objects[3].primitives[0].dimensions[2] = 0.5;
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = 0.5;
    collision_objects[3].primitive_poses[0].position.y = -1.5;
    collision_objects[3].primitive_poses[0].position.z = 0.25;
    collision_objects[3].primitive_poses[0].orientation.w = 1.0;

    collision_objects[3].id = "box3";
    collision_objects[3].header.frame_id = kinematic_model->getModelFrame();
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
    collision_objects[3].primitives[0].dimensions.resize(3);
    collision_objects[3].primitives[0].dimensions[0] = 0.5;
    collision_objects[3].primitives[0].dimensions[1] = 1.0;
    collision_objects[3].primitives[0].dimensions[2] = 0.5;
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = 0.5;
    collision_objects[3].primitive_poses[0].position.y = -1.5;
    collision_objects[3].primitive_poses[0].position.z = 0.25;
    collision_objects[3].primitive_poses[0].orientation.w = 1.0;

    collision_objects[4].id = "block1";
    collision_objects[4].header.frame_id = kinematic_model->getModelFrame();
    collision_objects[4].primitives.resize(1);
    collision_objects[4].primitives[0].type = collision_objects[4].primitives[0].BOX;
    collision_objects[4].primitives[0].dimensions.resize(3);
    collision_objects[4].primitives[0].dimensions[0] = 0.5;
    collision_objects[4].primitives[0].dimensions[1] = 1.0;
    collision_objects[4].primitives[0].dimensions[2] = 0.5;
    collision_objects[4].primitive_poses.resize(1);
    collision_objects[4].primitive_poses[0].position.x = 0.5;
    collision_objects[4].primitive_poses[0].position.y = -2.5;
    collision_objects[4].primitive_poses[0].position.z = 0.25;
    collision_objects[4].primitive_poses[0].orientation.w = 1.0;

    collision_objects[5].id = "block2";
    collision_objects[5].header.frame_id = kinematic_model->getModelFrame();
    collision_objects[5].primitives.resize(1);
    collision_objects[5].primitives[0].type = collision_objects[5].primitives[0].BOX;
    collision_objects[5].primitives[0].dimensions.resize(3);
    collision_objects[5].primitives[0].dimensions[0] = 0.5;
    collision_objects[5].primitives[0].dimensions[1] = 1.0;
    collision_objects[5].primitives[0].dimensions[2] = 0.5;
    collision_objects[5].primitive_poses.resize(1);
    collision_objects[5].primitive_poses[0].position.x = 0.5;
    collision_objects[5].primitive_poses[0].position.y = -3.5;
    collision_objects[5].primitive_poses[0].position.z = 0.25;
    collision_objects[5].primitive_poses[0].orientation.w = 1.0;

    collision_objects[6].id = "block3";
    collision_objects[6].header.frame_id = kinematic_model->getModelFrame();
    collision_objects[6].primitives.resize(1);
    collision_objects[6].primitives[0].type = collision_objects[6].primitives[0].BOX;
    collision_objects[6].primitives[0].dimensions.resize(3);
    collision_objects[6].primitives[0].dimensions[0] = 0.5;
    collision_objects[6].primitives[0].dimensions[1] = 1.0;
    collision_objects[6].primitives[0].dimensions[2] = 0.5;
    collision_objects[6].primitive_poses.resize(1);
    collision_objects[6].primitive_poses[0].position.x = 0.5;
    collision_objects[6].primitive_poses[0].position.y = -4.5;
    collision_objects[6].primitive_poses[0].position.z = 0.25;
    collision_objects[6].primitive_poses[0].orientation.w = 1.0;

    planning_scene.processCollisionObjectMsg(collision_objects[0]);
    planning_scene.processCollisionObjectMsg(collision_objects[1]);
    planning_scene.processCollisionObjectMsg(collision_objects[2]);
    planning_scene.processCollisionObjectMsg(collision_objects[3]);
    planning_scene.processCollisionObjectMsg(collision_objects[4]);
    planning_scene.processCollisionObjectMsg(collision_objects[5]);
    planning_scene.processCollisionObjectMsg(collision_objects[6]);
    
    collision_result.clear();
    
    return 0;
}
