#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_parser/kdl_parser.hpp"
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
public:
Iiwa_pub_sub()
: Node("ros2_kdl_node_vision"),
node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
{

joint_state_available_ = false;
aruco_pose_available_ = false;


auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");

while (!parameters_client->wait_for_service(1s)) {
if (!rclcpp::ok()) {
RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
rclcpp::shutdown();
}
RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
}
// Get the robot_description parameter
auto parameter = parameters_client->get_parameters({"robot_description"});

KDL::Tree robot_tree;
if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
std::cout << "Failed to retrieve robot_description param!";
}
robot_ = std::make_shared<KDLRobot>(robot_tree);
nj = robot_->getNrJnts();
KDL::JntArray q_min(nj), q_max(nj);
q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;
q_max.data << 2.96,2.09,2.96,2.09,2.96,2.09,2.96;
robot_->setJntLimits(q_min,q_max);
joint_positions_.resize(nj);
joint_velocities_.resize(nj);
joint_velocities_cmd_.resize(nj); 

// --- Subscribers ---
jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
"/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

// --- Aruco pose Subscriber ---
image_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
"/aruco_single/pose", 100, std::bind(&Iiwa_pub_sub::imageCallback, this, std::placeholders::_1));

// --- Wait for joint states and Aruco pose messages ---
while(!joint_state_available_){
RCLCPP_INFO(this->get_logger(), "Waiting for /joint_states ...");
rclcpp::spin_some(node_handle_);
}
while(!aruco_pose_available_){
RCLCPP_INFO(this->get_logger(), "Waiting for /aruco_single/pose ...");
rclcpp::spin_some(node_handle_);
}

// --- Robot setup ---
robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
KDL::Frame f_T_ee = KDL::Frame::Identity();
robot_->addEE(f_T_ee);
robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
init_cart_pose_ = robot_->getEEFrame();


controller_ = std::make_unique<KDLController>(*robot_);
s_d << 0, 0, 1; 



cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
std::bind(&Iiwa_pub_sub::cmd_publisher, this));

desired_commands_.resize(nj, 0.0);


std_msgs::msg::Float64MultiArray cmd_msg;
cmd_msg.data = desired_commands_;
cmdPublisher_->publish(cmd_msg);

RCLCPP_INFO(this->get_logger(), "Ready. Starting Vision Control...");
}

private:


void cmd_publisher(){

robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

double total_time = 50.0;
int trajectory_len = total_time * 100;
double dt = 1.0 / (trajectory_len / total_time);
t_ += dt;

if (t_ < total_time){
KDL::Frame cartpos = robot_->getEEFrame();
J_cam = robot_->getEEJacobian();

Eigen::VectorXd q0_dot = Eigen::VectorXd::Zero(nj);

KDL::Frame cartpos_camera = cartpos * KDL::Frame(
    KDL::Rotation::RotY(-1.5708),   
    KDL::Vector(0.0, 0.0, 0.02)     
);

KDL::Jacobian J_c_camera(J_cam.columns());
KDL::changeBase(J_cam, cartpos_camera.M, J_c_camera);


joint_velocities_.data = controller_->vision_ctrl(
    pose_in_camera_frame, 
    cartpos_camera,       
    J_c_camera,           
    q0_dot
);


robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

// Set joint velocity commands
for (long int i = 0; i < nj; ++i) {
desired_commands_[i] = joint_velocities_(i);
}
}
else{
// Trajectory time over: send zero velocities
RCLCPP_INFO_ONCE(this->get_logger(), "===> Trajectory completed. Sending zero velocity commands. <===");
for (long int i = 0; i < nj; ++i) {
desired_commands_[i] = 0.0;
}
}
// Create msg and publish commands
std_msgs::msg::Float64MultiArray cmd_msg;
cmd_msg.data = desired_commands_;
cmdPublisher_->publish(cmd_msg);
}

// ========= CALLBACKS =========

void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
joint_state_available_ = true;
for (unsigned int i = 0; i < nj; i++){
joint_positions_.data[i] = sensor_msg.position[i];
joint_velocities_.data[i] = sensor_msg.velocity[i];
}
}

void imageCallback(const geometry_msgs::msg::PoseStamped& msg) {
RCLCPP_INFO_ONCE(this->get_logger(), "===> Aruco pose received! <===");
aruco_pose_available_ = true;


// Extract position and orientation from the PoseStamped message
const auto position = msg.pose.position;
const auto orientation = msg.pose.orientation;

// Convert to KDL::Frame
KDL::Vector kdl_position(position.x, position.y, position.z);
KDL::Rotation kdl_rotation = KDL::Rotation::Quaternion(
orientation.x, orientation.y, orientation.z, orientation.w
);

// Save to private variable
pose_in_camera_frame.M = kdl_rotation;
pose_in_camera_frame.p = kdl_position;
}

// --- Private Variables ---

// ROS
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr image_sub_; 
rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Node::SharedPtr node_handle_;

// Commands
std::vector<double> desired_commands_;
KDL::JntArray joint_velocities_cmd_;

// Robot State (KDL)
std::shared_ptr<KDLRobot> robot_;
KDL::JntArray joint_positions_;
KDL::JntArray joint_velocities_;
KDL::Frame init_cart_pose_;

// Controller
std::unique_ptr<KDLController> controller_;
KDL::Jacobian J_cam;
Eigen::Vector3d s_d;
KDL::Frame pose_in_camera_frame;

// Node State
unsigned int nj;
double t_ = 0.0;
bool joint_state_available_;
bool aruco_pose_available_;
};

int main( int argc, char** argv )
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
rclcpp::shutdown();
return 1;
}