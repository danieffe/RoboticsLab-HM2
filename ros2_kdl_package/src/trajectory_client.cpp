#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_kdl_package/action/execute_trajectory.hpp"
#include <chrono>

using namespace std::chrono_literals;
using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;

class TrajectoryClient : public rclcpp::Node
{
public:
  TrajectoryClient() : Node("trajectory_client")
  {
    client_ = rclcpp_action::create_client<ExecuteTrajectory>(
        this, "execute_trajectory");

    // Attendi che il server sia disponibile
    while (!client_->wait_for_action_server(1s)) {
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }

    // Crea il messaggio obiettivo
    ExecuteTrajectory::Goal goal_msg;
    goal_msg.total_time = 2.0;
    goal_msg.traj_duration = 1.5;
    goal_msg.acc_duration = 0.5;
    goal_msg.end_position_x = 0.3;
    goal_msg.end_position_y = 0.0;
    goal_msg.end_position_z = 0.4;
    goal_msg.kp = 1.0;

    // Definisci le callback
    rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions options;

    options.feedback_callback =
        [](rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::SharedPtr,
           const std::shared_ptr<const ExecuteTrajectory::Feedback> feedback)
        {
          RCLCPP_INFO(rclcpp::get_logger("trajectory_client"),
                      "Position error: %.4f", feedback->position_error);
        };

    options.result_callback =
        [](const rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::WrappedResult &result)
        {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            RCLCPP_INFO(rclcpp::get_logger("trajectory_client"), "Trajectory executed successfully!");
          else
            RCLCPP_WARN(rclcpp::get_logger("trajectory_client"), "Trajectory failed or was canceled.");
        };

    // Invia il goal
    client_->async_send_goal(goal_msg, options);
  }

private:
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryClient>());
  rclcpp::shutdown();
  return 0;
}

