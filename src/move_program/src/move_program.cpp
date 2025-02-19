#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>
  (
    "move_program",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("move_program");

  moveit::planning_interface::MoveGroupInterface move_group(node, "right_arm");

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  auto joints = move_group.getJoints();
  std::ostringstream joint_names_stream;
  for (const auto& joint : joints) {
      joint_names_stream << joint << " "; 
  }
  RCLCPP_INFO(logger, "Joint: %s", joint_names_stream.str().c_str());

  RCLCPP_INFO(logger, "End effector name: %s", move_group.getEndEffectorLink().c_str());

  move_group.setStartStateToCurrentState();
  move_group.setPlanningTime(10.0);

  //tf2::Quaternion tf2_quat;
  //tf2_quat.setRPY(1.5, -0.08, 1.46);

  //geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

  geometry_msgs::msg::Pose GoalPose;
  //GoalPose.orientation = msg_quat;
  GoalPose.orientation.x = 0.0;
  GoalPose.orientation.y = 0.0;
  GoalPose.orientation.z = 0.0;
  GoalPose.orientation.w = 1.0;
  GoalPose.position.x = 0.5;
  GoalPose.position.y = 0.1;
  GoalPose.position.z = 0.232;
  move_group.setPoseTarget(GoalPose);
  //move_group.setGoalTolerance(0.01);

  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  auto const outcome = static_cast<bool>(move_group.plan(plan1));

  if(outcome){
    RCLCPP_INFO(logger, "Executing the plan...");
    move_group.execute(plan1);
  }
  else{
    RCLCPP_ERROR(logger, "Plan not executed!");
  }

  rclcpp::shutdown();
  return 0;
}