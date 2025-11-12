// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl_interfaces/action/trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"

using KdlActionTrajectory = kdl_interfaces::action::Trajectory;
using GoalHandleTrajectory = rclcpp_action::ServerGoalHandle<KdlActionTrajectory>;
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
public:
    Iiwa_pub_sub()
        : Node("kdl_action_server"),
          node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
    {
        declare_parameter("cmd_interface", "position");
        get_parameter("cmd_interface", cmd_interface_);

        RCLCPP_INFO(get_logger(), "Current cmd interface is: '%s'", cmd_interface_.c_str());

        if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort"))
        {
            RCLCPP_ERROR(get_logger(), "Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead...");
            return;
        }

        iteration_ = 0;
        t_ = 0;
        joint_state_available_ = false;

        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "/robot_state_publisher");
        while (!parameters_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});

        // Crea struttura KDLRobot
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree))
        {
            std::cout << "Failed to retrieve robot_description param!";
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);

        // Inizializza array e limiti
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96;
        q_max.data << 2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96;
        robot_->setJntLimits(q_min, q_max);
        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_positions_cmd_.resize(nj);
        joint_velocities_cmd_.resize(nj);
        joint_efforts_cmd_.resize(nj);
        joint_efforts_cmd_.data.setZero();

        // Subscriber a jnt states
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

        // Attesa primo joint_state
        while (!joint_state_available_)
        {
            RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
            rclcpp::spin_some(node_handle_);
        }

        // KDL initialization
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        // Compute EE frame
        init_cart_pose_ = robot_->getEEFrame();

        // Compute IK
        KDL::JntArray q(nj);
        robot_->getInverseKinematics(init_cart_pose_, q);

        // Initialize controller
        controller_ = std::make_unique<KDLController>(*robot_);

        // Setup Publisher e Timer di Mantenimento
        if (cmd_interface_ == "position")
        {
            cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                             std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            for (long int i = 0; i < joint_positions_.data.size(); ++i)
            {
                desired_commands_[i] = joint_positions_(i);
            }
        }
        else if (cmd_interface_ == "velocity")
        {
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), // 1 Hz
                                             std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            for (long int i = 0; i < joint_velocities_.data.size(); ++i)
            {
                desired_commands_[i] = joint_velocities_(i);
            }
        }
        else if (cmd_interface_ == "effort")
        {
            cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                             std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i)
            {
                desired_commands_[i] = joint_efforts_cmd_(i);
            }
        }

        // Setup Action Server
        using namespace std::placeholders;
        action_server_ = rclcpp_action::create_server<KdlActionTrajectory>(
            this,
            "trajectory_action",
            std::bind(&Iiwa_pub_sub::handle_goal, this, _1, _2),
            std::bind(&Iiwa_pub_sub::handle_cancel, this, _1),
            std::bind(&Iiwa_pub_sub::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Action server trajectory_action successfully started.");
    }

private:
    void cmd_publisher()
    {
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState &sensor_msg)
    {
        joint_state_available_ = true;
        for (unsigned int i = 0; i < sensor_msg.position.size(); i++)
        {
            joint_positions_.data[i] = sensor_msg.position[i];
            joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
    }

    // Action Server Handlers
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const KdlActionTrajectory::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Received trajectory goal request.");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleTrajectory> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received cancellation request.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleTrajectory> goal_handle)
    {
        std::thread{std::bind(&Iiwa_pub_sub::execute_trajectory, this, goal_handle)}.detach();
    }

    void execute_trajectory(const std::shared_ptr<GoalHandleTrajectory> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Starting trajectory execution for Action Goal...");
        if (timer_)
        {
            timer_->cancel();
        }

        const auto goal = goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(), "Goal end_position: [%.4f, %.4f, %.4f], traj_d: %.3f, acc_d: %.3f",
                    goal->end_position.x, goal->end_position.y, goal->end_position.z,
                    goal->traj_duration, goal->acc_duration);

        Eigen::Vector3d action_end_position;
        action_end_position << goal->end_position.x, goal->end_position.y, goal->end_position.z;

        // Extract parameters from goal message
        double total_time = goal->total_time;
        double action_traj_duration = goal->traj_duration;
        double action_acc_duration = goal->acc_duration;
        std::string traj_type_ = goal->traj_type;
        std::string s_type_ = goal->s_type;
        double Kp = goal->kp;
        double lambda_ = goal->lambda_val;
        int trajectory_len = goal->trajectory_len;
        std::string ctrl_mode_ = goal->ctrl;
        double traj_radius = 0.15;

        // Obtain current end-effector position
        KDL::Frame current_cartpos = robot_->getEEFrame();
        Eigen::Vector3d action_init_position(Eigen::Vector3d(current_cartpos.p.data));

        // Replan the trajectory with the new parameters
        KDLPlanner action_planner(action_traj_duration, action_acc_duration, action_init_position, action_end_position);

        // Initialize Action loop
        double t_action = 0;
        int loop_rate = trajectory_len / action_traj_duration;
        double dt = 1.0 / loop_rate;
        rclcpp::Rate rate(loop_rate);

        // Initialize feedback and result
        auto feedback = std::make_shared<KdlActionTrajectory::Feedback>();
        auto result = std::make_shared<KdlActionTrajectory::Result>();

        while (t_action < action_traj_duration)
        {
            if (goal_handle->is_canceling())
            {
                RCLCPP_INFO(this->get_logger(), "Goal canceled. Stopping robot.");
                result->success = false;
                result->message = "Goal cancelled by client.";
                goal_handle->canceled(result);
                if (timer_)
                {
                    timer_->reset();
                }
                return;
            }

            // --- Trajectory Control Logic ---

            t_action += dt;
            // Retrieve the trajectory point based on the trajectory type (use action_planner)
            if (traj_type_ == "linear")
            {
                planner_ = KDLPlanner(action_traj_duration, action_acc_duration, action_init_position, action_end_position); // currently using trapezoidal velocity profile
                if (s_type_ == "trapezoidal")
                {
                    p_ = planner_.linear_traj_trapezoidal(t_action);
                }
                else if (s_type_ == "cubic")
                {
                    p_ = planner_.linear_traj_cubic(t_action);
                }
            }
            else if (traj_type_ == "circular")
            {
                planner_ = KDLPlanner(action_traj_duration, action_init_position, traj_radius, action_acc_duration);
                if (s_type_ == "trapezoidal")
                {
                    p_ = planner_.circular_traj_trapezoidal(t_action);
                }
                else if (s_type_ == "cubic")
                {
                    p_ = planner_.circular_traj_cubic(t_action);
                }
            }

            // Compute EE frame (robot_->getEEFrame() uses the latest joint_state received)
            KDL::Frame cartpos = robot_->getEEFrame();

            // Compute desired Frame (Rotation maintained, Position from trajectory)
            KDL::Frame desFrame;
            desFrame.M = init_cart_pose_.M;
            desFrame.p = toKDL(p_.pos);

            // compute errors
            Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
            Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));

            // ---  Feedback Publishing ---
            feedback->position_error_magnitude = error.norm();
            feedback->current_position.x = cartpos.p.x();
            feedback->current_position.y = cartpos.p.y();
            feedback->current_position.z = cartpos.p.z();
            goal_handle->publish_feedback(feedback);

            // --- Command Calculation (Control Logic) ---
            if (cmd_interface_ == "position")
            {
                KDL::Frame nextFrame;
                nextFrame.M = cartpos.M;
                nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp * error)) * dt;

                joint_positions_cmd_ = joint_positions_;
                robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                for (long int i = 0; i < joint_positions_.data.size(); ++i)
                {
                    desired_commands_[i] = joint_positions_cmd_(i);
                }
            }
            else if (cmd_interface_ == "velocity")
            {
                if (ctrl_mode_ == "velocity_ctrl")
                {
                    Vector6d cartvel;
                    cartvel << p_.vel + Kp * error, o_error;
                    joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
                }
                else if (ctrl_mode_ == "velocity_ctrl_null")
                {
                    joint_velocities_cmd_.data = this->controller_->velocity_ctrl_null(desFrame, Kp, lambda_);
                }
                for (long int i = 0; i < joint_velocities_.data.size(); ++i)
                {
                    desired_commands_[i] = joint_velocities_cmd_(i);
                }
            }
            else if (cmd_interface_ == "effort")
            {
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i)
                {
                    desired_commands_[i] = 0.0;
                }
            }

            // Publish commands
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
            rate.sleep();
        }

        if (goal_handle->is_canceling())
        {
            RCLCPP_INFO(this->get_logger(), "Goal canceled just before completion.");
            result->success = false;
            result->message = "Goal cancelled by client near completion.";
            goal_handle->canceled(result);
            if (timer_)
            {
                timer_->reset();
            }
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully.");
        result->success = true;
        result->message = "Trajectory execution finished.";

        // Send success result
        goal_handle->succeed(result);
        if (cmd_interface_ == "position")
        {
            for (long int i = 0; i < joint_positions_cmd_.data.size(); ++i)
            {
                desired_commands_[i] = joint_positions_cmd_(i);
            }
        }
        else if (cmd_interface_ == "velocity")
        {
            for (long int i = 0; i < desired_commands_.size(); ++i)
            {
                desired_commands_[i] = 0.0;
            }
        }

        std_msgs::msg::Float64MultiArray hold_msg;
        hold_msg.data = desired_commands_;
        cmdPublisher_->publish(hold_msg);
        if (timer_)
        {
            timer_->reset();
        }
    }

    // Member definitions (without unnecessary initialization logic)
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp_action::Server<KdlActionTrajectory>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Node::SharedPtr node_handle_;

    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_positions_cmd_;
    KDL::JntArray joint_velocities_cmd_;
    KDL::JntArray joint_efforts_cmd_;

    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;

    trajectory_point p_;
    int iteration_;
    bool joint_state_available_;
    double t_;
    std::string cmd_interface_;
    KDL::Frame init_cart_pose_;
    std::unique_ptr<KDLController> controller_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}