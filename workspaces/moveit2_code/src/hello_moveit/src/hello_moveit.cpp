#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

#include "robot_interfaces/action/robot_joints.hpp"
#include "robot_interfaces/srv/plan_to_pose.hpp"

class MotionPlannerNode : public rclcpp::Node
{
public:
  using RobotJoints = robot_interfaces::action::RobotJoints;
  using PlanToPose = robot_interfaces::srv::PlanToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<RobotJoints>;

  MotionPlannerNode()
  : Node("hello_moveit_planner")
  {
    RCLCPP_INFO(this->get_logger(), "Node constructed, but MoveGroupInterface init deferred.");
  }

  void init()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "fanuc_arm");

    action_client_ = rclcpp_action::create_client<RobotJoints>(shared_from_this(), "joint_controller");

    service_ = this->create_service<PlanToPose>(
      "plan_and_execute_pose",
      std::bind(&MotionPlannerNode::handle_pose_request, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(this->get_logger(), "Service ready: plan_and_execute_pose");
  }

private:
  void handle_pose_request(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<PlanToPose::Request> request,
    std::shared_ptr<PlanToPose::Response> response)
  {
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(request->target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (!success)
    {
      response->success = false;
      response->message = "Motion planning failed.";
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      response->success = false;
      response->message = "Action server not available.";
      return;
    }

    const auto& traj = plan.trajectory_.joint_trajectory;

    for (const auto& point : traj.points)
    {
      RobotJoints::Goal goal;
      goal.joint_state.name = traj.joint_names;
      goal.joint_state.position = point.positions;
      goal.velocity = 10;
      goal.acceleration = 10;
      goal.cnt_val = 100;

      auto future_goal = action_client_->async_send_goal(goal);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal) != rclcpp::FutureReturnCode::SUCCESS)
      {
        response->success = false;
        response->message = "Failed to send goal.";
        return;
      }

      auto goal_handle = future_goal.get();
      if (!goal_handle)
      {
        response->success = false;
        response->message = "Goal was rejected.";
        return;
      }

      auto future_result = action_client_->async_get_result(goal_handle);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) != rclcpp::FutureReturnCode::SUCCESS)
      {
        response->success = false;
        response->message = "Failed to get result.";
        return;
      }

      if (!future_result.get().result->success)
      {
        response->success = false;
        response->message = "One of the trajectory points failed.";
        return;
      }
    }

    response->success = true;
    response->message = "Trajectory executed successfully.";
  }

  rclcpp::Service<PlanToPose>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Client<RobotJoints>::SharedPtr action_client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionPlannerNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
