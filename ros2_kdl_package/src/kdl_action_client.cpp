#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "kdl_interfaces/action/trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <chrono>
#include <functional>
#include <memory>

using namespace std::placeholders;
using namespace std::chrono_literals;

using KdlActionTrajectory = kdl_interfaces::action::Trajectory;
using GoalHandleTrajectory = rclcpp_action::ClientGoalHandle<KdlActionTrajectory>;

class KdlActionClient : public rclcpp::Node
{
public:
    KdlActionClient()
        : Node("kdl_action_client")
    {
        declare_parameter("traj_type", "linear");
        get_parameter("traj_type", traj_type_);
        RCLCPP_INFO(get_logger(), "Current trajectory type is: '%s'", traj_type_.c_str());

        declare_parameter("s_type", "trapezoidal");
        get_parameter("s_type", s_type_);
        RCLCPP_INFO(get_logger(), "Current s type is: '%s'", s_type_.c_str());

        declare_parameter("total_time", 1.5);
        declare_parameter("trajectory_len", 150);
        declare_parameter("Kp", 5.0);
        get_parameter("total_time", this->total_time);
        get_parameter("trajectory_len", this->trajectory_len);
        get_parameter("Kp", this->Kp);
        RCLCPP_INFO(get_logger(), "Current total time is: '%f'", this->total_time);
        RCLCPP_INFO(get_logger(), "Current trajectory length is: '%d'", this->trajectory_len);
        RCLCPP_INFO(get_logger(), "Current Kp is: '%f'", this->Kp);

        declare_parameter("ctrl", "velocity_ctrl");
        get_parameter("ctrl", this->ctrl_mode_);
        RCLCPP_INFO(get_logger(), "Control mode selected: '%s'", this->ctrl_mode_.c_str());

        declare_parameter("lambda", 1.0);
        get_parameter("lambda", this->lambda_);
        RCLCPP_INFO(get_logger(), "Lambda value for null-space control: '%f'", this->lambda_);

        client_ptr_ = rclcpp_action::create_client<KdlActionTrajectory>(
            this, "trajectory_action");

        this->declare_parameter("end_position_x", 0.0);
        this->declare_parameter("end_position_y", 0.0);
        this->declare_parameter("end_position_z", 0.0);
        this->declare_parameter("traj_duration", 10.0);
        this->declare_parameter("acc_duration", 1.0);

        timer_ = this->create_wall_timer(
            1s, std::bind(&KdlActionClient::send_goal, this));
    }

private:
    rclcpp_action::Client<KdlActionTrajectory>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string traj_type_;
    std::string s_type_;
    double total_time = 6.0;
    int trajectory_len = 0;
    double Kp = 0;
    std::string ctrl_mode_;
    double lambda_;

    void send_goal()
    {
        double end_x, end_y, end_z, traj_duration, acc_duration;
        this->get_parameter("end_position_x", end_x);
        this->get_parameter("end_position_y", end_y);
        this->get_parameter("end_position_z", end_z);
        this->get_parameter("traj_duration", traj_duration);
        this->get_parameter("acc_duration", acc_duration);

        timer_->cancel();

        if (!client_ptr_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
            rclcpp::shutdown();
            return;
        }
        // Construct goal message
        auto goal_msg = KdlActionTrajectory::Goal();
        goal_msg.end_position.x = end_x;
        goal_msg.end_position.y = end_y;
        goal_msg.end_position.z = end_z;
        goal_msg.traj_duration = traj_duration;
        goal_msg.acc_duration = acc_duration;
        goal_msg.total_time = total_time;
        goal_msg.trajectory_len = trajectory_len;
        goal_msg.kp = Kp;
        goal_msg.lambda_val = lambda_;

        goal_msg.traj_type = traj_type_;
        goal_msg.s_type = s_type_;
        goal_msg.ctrl = ctrl_mode_;

        RCLCPP_INFO(this->get_logger(),
                    "Sending goal -> Target (%.2f, %.2f, %.2f), Duration: %.1f s, Acc: %.1f s",
                    end_x, end_y, end_z, traj_duration, acc_duration);
        auto send_goal_options = rclcpp_action::Client<KdlActionTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&KdlActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&KdlActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&KdlActionClient::result_callback, this, _1);

        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleTrajectory::SharedPtr &goal_handle)
    {
        if (!goal_handle)
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
        else
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
    }

    void feedback_callback(
        GoalHandleTrajectory::SharedPtr,
        const std::shared_ptr<const KdlActionTrajectory::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Feedback -> Error: %.4f | Pos: (%.2f, %.2f, %.2f)",
                    feedback->position_error_magnitude,
                    feedback->current_position.x,
                    feedback->current_position.y,
                    feedback->current_position.z);
    }

    void result_callback(const GoalHandleTrajectory::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            RCLCPP_INFO(this->get_logger(), "Action SUCCEEDED! Trajectory completed.");
        else
            RCLCPP_ERROR(this->get_logger(), "Action failed or was cancelled.");

        RCLCPP_INFO(this->get_logger(), "Server message: %s",
                    result.result->message.c_str());
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KdlActionClient>();
    rclcpp::spin(node);
    return 0;
}
