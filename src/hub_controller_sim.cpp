#include <memory>
#include <string>
#include <vector>
# include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <std_msgs/msg/string.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"

class HubControllerSim : public rclcpp::Node
{
public:
	HubControllerSim()
	: Node("hub_controller_sim")
	{
		cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
			"cmd", 10, std::bind(&HubControllerSim::cmd_callback, this, std::placeholders::_1));
		stop_sub = this->create_subscription<std_msgs::msg::String>(
			"stop", 10, std::bind(&HubControllerSim::stop_callback, this, std::placeholders::_1));
	}

	static std::shared_ptr<HubControllerSim> make(std::vector<std::string> joint_names)
	{
		auto hub_controller_sim_ptr = std::make_shared<HubControllerSim>();
		hub_controller_sim_ptr->joint_names = joint_names;
		rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client;
  		action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    hub_controller_sim_ptr->get_node_base_interface(),
    hub_controller_sim_ptr->get_node_graph_interface(),
    hub_controller_sim_ptr->get_node_logging_interface(),
    hub_controller_sim_ptr->get_node_waitables_interface(),
    "/joint_trajectory_controller/follow_joint_trajectory");
		if (action_client == nullptr)
		{
			RCLCPP_ERROR(hub_controller_sim_ptr->get_logger(), "could not succesfully create action client");
			return nullptr;
		}
		hub_controller_sim_ptr->set_action_client(action_client);
		hub_controller_sim_ptr->init_pos();
		return hub_controller_sim_ptr;
	}

private:
	std::vector<std::string> joint_names;
	rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_ptr;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stop_sub;

	void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
	{
		// send command to go to location
		float pos_x = msg->linear.x;
		float pos_y = msg->linear.y;
		float pos_z = msg->linear.z;
    RCLCPP_INFO(this->get_logger(), "received goal x: %f, y: %f, z: %f)", pos_x, pos_y, pos_z);
		int success = move_to(pos_x, pos_y, pos_z);
	}

	void common_result_response(
	  const rclcpp_action::ClientGoalHandle
	  <control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
	{
	  printf("common_result_response time: %f\n", rclcpp::Clock().now().seconds());
		rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
	  common_resultcode = result.code;
		int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
	  common_action_result_code = result.result->error_code;
	  switch (result.code) {
	    case rclcpp_action::ResultCode::SUCCEEDED:
	      printf("SUCCEEDED result code\n");
	      break;
	    case rclcpp_action::ResultCode::ABORTED:
	      printf("Goal was aborted\n");
	      return;
	    case rclcpp_action::ResultCode::CANCELED:
	      printf("Goal was canceled\n");
	      return;
	    default:
	      printf("Unknown result code\n");
	      return;
	  }
	}

	void common_feedback(
  	rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
  	const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
	{
	  std::cout << "feedback->desired.positions :";
	  for (auto & x : feedback->desired.positions) {
	    std::cout << x << "\t";
	  }
	  std::cout << std::endl;
	  std::cout << "feedback->desired.velocities :";
	  for (auto & x : feedback->desired.velocities) {
	    std::cout << x << "\t";
	  }
	  std::cout << std::endl;
	}

	void common_goal_response(
	  std::shared_future<rclcpp_action::ClientGoalHandle
	  <control_msgs::action::FollowJointTrajectory>::SharedPtr> future)
	{
		bool common_goal_accepted = false;
	  RCLCPP_DEBUG(
	    this->get_logger(), "common_goal_response time: %f",
	    rclcpp::Clock().now().seconds());
	  auto goal_handle = future.get();
	  if (!goal_handle) {
	    common_goal_accepted = false;
	    printf("Goal rejected\n");
	  } else {
	    common_goal_accepted = true;
	    printf("Goal accepted\n");
	  }
	}

	void stop_callback(const std_msgs::msg::String::SharedPtr msg)
	{
		//set speed to zero
	}

	void set_action_client(rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr _action_client)
	{
		action_client_ptr = std::move(_action_client);
	}

  int move_to(const float x, const float y, float z)
	{
				// create goal
		std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
		trajectory_msgs::msg::JointTrajectoryPoint point;
		point.positions.resize(joint_names.size());
		point.positions[0] = x;
		point.positions[1] = z;

		// goal options
		rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
  	opt.goal_response_callback = std::bind(&HubControllerSim::common_goal_response, this, std::placeholders::_1);
  	opt.result_callback = std::bind(&HubControllerSim::common_result_response, this, std::placeholders::_1);
  	opt.feedback_callback = std::bind(&HubControllerSim::common_feedback, this, std::placeholders::_1, std::placeholders::_2);

		// send  goal
  	control_msgs::action::FollowJointTrajectory_Goal goal_msg;
  	goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
  	goal_msg.trajectory.joint_names = joint_names;
  	goal_msg.trajectory.points = points;

    auto goal_handle_future = action_client_ptr->async_send_goal(goal_msg, opt);
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "send goal call failed :(");
      return 1;
    }
    RCLCPP_INFO(this->get_logger(), "send goal call ok :)");

    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
      goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      return 1;
    }
    RCLCPP_INFO(this->get_logger(), "Goal was accepted by server");

    // Wait for the server to be done with the goal
    auto result_future = action_client_ptr->async_get_result(goal_handle);
    RCLCPP_INFO(this->get_logger(), "Waiting for result");
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "get result call failed :(");
      return 1;
    }

	}
	// Sets position to zero for both joints.
	void init_pos()
	{
    move_to(0.0, 0.0, 0.0);
	}

};


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	std::vector<std::string> joint_names = {"x_axis_arm_to_holder", "x_axis_arm_joint" };
	rclcpp::spin(HubControllerSim::make(joint_names));
	// if (!hub_controller)
	// 	return
	// rclcpp::spin(hub_controller);
	rclcpp::shutdown();
	return 0;
}