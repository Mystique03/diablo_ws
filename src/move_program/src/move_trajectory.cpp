#include <memory>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include "rclcpp/rclcpp.hpp"
#include <string>

static const rclcpp::Logger logger = rclcpp::get_logger("move_group_demo");
static const std::string planning_group = "right_arm";

class TestTrajectory : public rclcpp::Node{
    public:
        TestTrajectory(std::shared_ptr<rclcpp::Node> move_group_node)
            : Node("move_trajectory"),
              move_group_arm(move_group_node, planning_group),
              joint_model_group_arm(
                move_group_arm.getCurrentState()->getJointModelGroup(
                    planning_group
                )
              ) {

                this->timer_ = 
                    this->create_wall_timer(std::chrono::milliseconds(500),
                                            std::bind(&TestTrajectory::timer_callback, this)); 
              }

    void get_info(){
        RCLCPP_INFO(logger, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());
        RCLCPP_INFO(logger, "End Effector link: %s", move_group_arm.getEndEffectorLink().c_str());
        RCLCPP_INFO(logger, "Available planning groups:");
        std::copy(move_group_arm.getJointModelGroupNames().begin(), move_group_arm.getJointModelGroupNames().end(),
                    std::ostream_iterator<std::string>(std::cout, ", "));
    }

    void current_state(){
        RCLCPP_INFO(logger, "Get robot current state");

        current_state_arm = move_group_arm.getCurrentState(10);
        current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
                                                    this->joint_group_positions_arm);
    }

    void plan_arm_joint_space(){
        RCLCPP_INFO(logger, "Planning joint state space");

        joint_group_positions_arm[0] = -0.54;    //shoulder pitch
        joint_group_positions_arm[1] = -1.31;    //shoulder roll
        joint_group_positions_arm[2] = -1.08;    //elbow
        joint_group_positions_arm[3] = 1.27;    //wrist

        move_group_arm.setJointValueTarget(joint_group_positions_arm);

        bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                            moveit::planning_interface::MoveItErrorCode::SUCCESS);

    }

    void timer_callback(){
        this->timer_->cancel();
        get_info();
        current_state();
        plan_arm_joint_space();
    }

private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<double> joint_group_positions_arm;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    rclcpp::TimerBase::SharedPtr timer_;

    moveit::planning_interface::MoveGroupInterface move_group_arm;
    const moveit::core::JointModelGroup *joint_model_group_arm;

    moveit::core::RobotStatePtr current_state_arm;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = 
            rclcpp::Node::make_shared("move_group_demo", node_options);

    rclcpp::executors::SingleThreadedExecutor planner_executor;
    std::shared_ptr<TestTrajectory> planner_node =
        std::make_shared<TestTrajectory>(move_group_node);
    planner_executor.add_node(planner_node);
    planner_executor.spin();

    rclcpp::shutdown();
    return 0;
}
