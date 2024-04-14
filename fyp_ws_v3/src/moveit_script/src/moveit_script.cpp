#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include "robotiq_3f_gripper_ros2_interfaces/srv/robotiq3_f_gripper_output_service.hpp"

#include <chrono>
#include <cstdlib>
#include <fstream>

#include <geometry_msgs/msg/pose_stamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <rclcpp/clock.hpp>

using std::placeholders::_1;
class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "transformed_pose", 1, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
  void topic_callback(const geometry_msgs::msg::PoseStamped& msg) const
  {
  //Set PTP pose position
  geometry_msgs::msg::PoseStamped obj_pose;
  obj_pose = msg;

  if (obj_pose.pose.position.z < 0.5 and obj_pose.pose.position.z != NULL)
  {

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = obj_pose.pose.orientation.w;
    target_pose.orientation.x = obj_pose.pose.orientation.x;
    target_pose.orientation.y = obj_pose.pose.orientation.y;
    target_pose.orientation.z = obj_pose.pose.orientation.z;
    target_pose.position.x = obj_pose.pose.position.x;
    target_pose.position.y = obj_pose.pose.position.y;
    target_pose.position.z = obj_pose.pose.position.z + 0.3;

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("moveit_script", node_options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("moveit_script");

    //Create Gripper Client and Initialize
    using namespace std::chrono_literals;
    std::shared_ptr<rclcpp::Node> gripper_node = rclcpp::Node::make_shared("gripper_client");
    rclcpp::Client<robotiq_3f_gripper_ros2_interfaces::srv::Robotiq3FGripperOutputService>::SharedPtr client =
      gripper_node->create_client<robotiq_3f_gripper_ros2_interfaces::srv::Robotiq3FGripperOutputService>("Robotiq3FGripper/OutputRegistersService");
      //Create request to be sent to Gripper Server
      auto request = std::make_shared<robotiq_3f_gripper_ros2_interfaces::srv::Robotiq3FGripperOutputService::Request>();
      request -> output_registers.r_act = 1; //Activate Gripper
      request -> output_registers.r_mod = 1; //Pinch Mode
      request -> output_registers.r_gto = 1;
      request -> output_registers.r_atr = 0;
      request -> output_registers.r_spa = 255; //Speed, 255: Max Speed
      request -> output_registers.r_fra = 0;   //Force, 255: Max Force     

      while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        // return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(gripper_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper Initialisation Done");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call gripper service");
    }

    // Create the MoveIt MoveGroup Interface
    moveit::planning_interface::MoveGroupInterface move_group_interface(node, "ur_manipulator");
    auto const robot_model = move_group_interface.getCurrentState()->getRobotModel();

    //Construct Time Parameterization Namespace
    trajectory_processing::TimeOptimalTrajectoryGeneration totg; //Same namespace construstor used to compute time stamps for first moveJ and first moveL

    //Set Velocity and Acceleration Scaling
    double vel_scaling = 0.5, moveL_vel_scaling = 0.05, accel_scaling = 1.0;
    move_group_interface.setMaxVelocityScalingFactor(vel_scaling);
    move_group_interface.setMaxAccelerationScalingFactor(accel_scaling);

    //Set Home Pose
    geometry_msgs::msg::Pose home_pose;
    home_pose.orientation.w = 0.0;
    home_pose.orientation.x = -0.458;
    home_pose.orientation.y = 0.888;
    home_pose.orientation.z = 0.0;
    home_pose.position.x = 0.18;
    home_pose.position.y = 0.33;
    home_pose.position.z = 0.7;
    
    //Set Pose as target end-point
    move_group_interface.setPoseTarget(target_pose);

    //Plan trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);


    //Calculate Execution Time and Execute trajectory
    if (success){
      robot_trajectory::RobotTrajectory plan_traj(move_group_interface.getCurrentState()->getRobotModel(), "ur_manipulator");
      plan_traj.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), plan.trajectory_);
      totg.computeTimeStamps(plan_traj, vel_scaling, accel_scaling);
      auto moveJ_time = plan_traj.getDuration(); //Get Trajectory Time
      move_group_interface.execute(plan);
      }

    //Dynamic Grasping
    std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    geometry_msgs::msg::Pose target_pose2 = move_group_interface.getCurrentPose().pose;
    target_pose2.position.x += 0.1;
    target_pose2.position.y -= 0.1;
    target_pose2.position.z -= 0.13;
    approach_waypoints.push_back(target_pose2);

    //Compute Cartersian Path Trajectory to approach object
    moveit_msgs::msg::RobotTrajectory trajectory_approach;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    move_group_interface.computeCartesianPath(approach_waypoints, eef_step, jump_threshold, trajectory_approach);

    //Utilize Time Parameterization for velocity control
    robot_trajectory::RobotTrajectory robot_traj(move_group_interface.getCurrentState()->getRobotModel(), "ur_manipulator");
    robot_traj.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory_approach);
    totg.computeTimeStamps(robot_traj, moveL_vel_scaling, accel_scaling);
    auto moveL_time = robot_traj.getDuration();
    robot_traj.getRobotTrajectoryMsg(trajectory_approach);
    plan.trajectory_ = trajectory_approach;

    //Plan and Execute MoveL Trajectory
    move_group_interface.asyncExecute(trajectory_approach);

    sleep(moveL_time*0.99);

    // // Static Grasping
    // std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    // geometry_msgs::msg::Pose target_pose2 = move_group_interface.getCurrentPose().pose;
    // target_pose2.position.z -= 0.13;
    // approach_waypoints.push_back(target_pose2);

    // moveit_msgs::msg::RobotTrajectory trajectory_approach;
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;

    // move_group_interface.computeCartesianPath(approach_waypoints, eef_step, jump_threshold, trajectory_approach);
    // //Utilize Time Parameterization for velocity control
    // robot_trajectory::RobotTrajectory robot_traj(move_group_interface.getCurrentState()->getRobotModel(), "ur_manipulator");
    // robot_traj.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory_approach);
    // totg.computeTimeStamps(robot_traj, moveL_vel_scaling, accel_scaling);
    // auto moveL_time = robot_traj.getDuration();
    // robot_traj.getRobotTrajectoryMsg(trajectory_approach);
    // plan.trajectory_ = trajectory_approach;
    // move_group_interface.execute(trajectory_approach);

    //Close Gripper
    request -> output_registers.r_pra = 120; //Position of Gripper, 255: Fully Closed, 0: Fully Opened
    request -> output_registers.r_spa = 60; //Speed, 255: Max Speed

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(gripper_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper Movement Done");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call gripper service");
    }

    //Compute Cartersian Path Trajectory to retreat from object
    std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
    target_pose2.position.z += 0.1;
    retreat_waypoints.push_back(target_pose2);

    moveit_msgs::msg::RobotTrajectory trajectory_retreat;

    move_group_interface.computeCartesianPath(retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);
    move_group_interface.execute(trajectory_retreat);

    geometry_msgs::msg::Pose target_pose3;
    target_pose3.orientation.w = home_pose.orientation.w;
    target_pose3.orientation.x = home_pose.orientation.x;
    target_pose3.orientation.y = home_pose.orientation.y;
    target_pose3.orientation.z = home_pose.orientation.z;
    target_pose3.position.x = 0.380;
    target_pose3.position.y = -0.280;
    target_pose3.position.z = 0.7;


    //Set Pose as target end-point
    move_group_interface.setPoseTarget(target_pose3);

    //Plan trajectory
    success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);


    //Calculate Execution Time and Execute trajectory
    if (success){
      move_group_interface.execute(plan);
      }

    //Open Gripper
    request -> output_registers.r_pra = 0; //Position of Gripper, 255: Fully Closed, 0: Fully Opened
    request -> output_registers.r_spa = 255; //Speed, 255: Max Speed

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    result = client->async_send_request(request);

    //Move back to Home Pose
    move_group_interface.setPoseTarget(home_pose);
    success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success){
      move_group_interface.execute(plan);}

    executor.cancel();
    }
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}