#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "tutorial_interfaces/msg/falconpos.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>


using namespace std::chrono_literals;


/////////////////// global variables ///////////////////
const std::string urdf_path = "/home/michael/FOR_TESTING/panda.urdf";
const unsigned int n_joints = 7;

const std::vector<double> lower_joint_limits {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
const std::vector<double> upper_joint_limits {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

const bool display_time = true;

KDL::Tree panda_tree;
KDL::Chain panda_chain;

KDL::Rotation orientation;
bool got_orientation = false;

std::vector<double> tcp_pos {0.3069, 0.0, 0.4853};   // initialized the same as the "home" position


/////////////////// function declarations ///////////////////
void compute_ik(std::vector<double>& p, std::vector<double>& origin, std::vector<double>& curr_joint_vals, std::vector<double>& control_joint_vals);

bool within_limits(std::vector<double>& vals);
bool create_tree();
void get_chain();

void print_joint_vals(std::vector<double>& joint_vals);



/////////////// DEFINITION OF NODE CLASS //////////////

class CartesianController : public rclcpp::Node
{
public:
  
  std::vector<double> offset_pos {0.0, 0.0, 0.0};
  std::vector<double> origin {0.3069, 0.0, 0.4853}; //////// can change the task-space origin point! ////////

  std::vector<double> curr_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> control_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool control = false;
  
  const int mapping_ratio = 7;    /////// this ratio is {end-effector movement} / {Falcon movement}

  CartesianController()
  : Node("cartesian_controller")
  { 
    //Create publisher and subscriber and timer
    controller_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10);
    controller_timer_ = this->create_wall_timer(5ms, std::bind(&CartesianController::controller_publisher, this));    // controls at 200 Hz

    //Create publisher and subscriber and timer
    tcp_pos_pub_ = this->create_publisher<tutorial_interfaces::msg::Falconpos>("tcp_position", 10);
    tcp_pos_timer_ = this->create_wall_timer(5ms, std::bind(&CartesianController::tcp_pos_publisher, this));    // publishes at 200 Hz

    joint_vals_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&CartesianController::joint_states_callback, this, std::placeholders::_1));

    falcon_pos_sub_ = this->create_subscription<tutorial_interfaces::msg::Falconpos>(
      "falcon_position", 10, std::bind(&CartesianController::falcon_pos_callback, this, std::placeholders::_1));

    //Create Panda tree and get its kinematic chain
    if (!create_tree()) rclcpp::shutdown();
    get_chain();

  }

private:
  
  ///////////////////////////////////// JOINT CONTROLLER /////////////////////////////////////
  void controller_publisher()
  { 
    if (control == true) {

      auto message = trajectory_msgs::msg::JointTrajectory();
      message.joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

      ///////// compute IK /////////
      compute_ik(offset_pos, origin, curr_joint_vals, control_joint_vals);

      std::cout << "The new joint values [control] are ";
      print_joint_vals(control_joint_vals);

      if (!within_limits(control_joint_vals)) {
        std::cout << "--------\nThese violate the joint limits of the Panda arm, shutting down now !!!\n---------" << std::endl;
        rclcpp::shutdown();
      }
      
      auto point = trajectory_msgs::msg::JointTrajectoryPoint();
      point.positions = control_joint_vals;
      // point.time_from_start.nanosec = 100000;     //// matching the publishing frequency

      message.points = {point};

      // RCLCPP_INFO(this->get_logger(), "Publishing controller joint values");
      controller_pub_->publish(message);
    }
  }

  ///////////////////////////////////// TCP POSITION PUBLISHER /////////////////////////////////////
  void tcp_pos_publisher()
  { 
    auto message = tutorial_interfaces::msg::Falconpos();
    message.x = tcp_pos.at(0);
    message.y = tcp_pos.at(1);
    message.z = tcp_pos.at(2);

    // RCLCPP_INFO(this->get_logger(), "Publishing controller joint values");
    tcp_pos_pub_->publish(message);
  }

  ///////////////////////////////////// JOINT STATES SUBSCRIBER /////////////////////////////////////
  void joint_states_callback(const sensor_msgs::msg::JointState & msg)
  { 
    auto data = msg.position;

    // write into our class variable for storing joint values
    curr_joint_vals.at(0) = data.at(0);
    curr_joint_vals.at(1) = data.at(1);
    curr_joint_vals.at(2) = data.at(7);
    curr_joint_vals.at(3) = data.at(2);
    curr_joint_vals.at(4) = data.at(3);
    curr_joint_vals.at(5) = data.at(4);
    curr_joint_vals.at(6) = data.at(5);

    // std::cout << "The current joint values [Gazebo] are ";
    // print_joint_vals(curr_joint_vals);

    /// if this the first iteration, change the control flag and start partying!
    if (control == false) control = true;
  }

  ///////////////////////////////////// FALCON SUBSCRIBER /////////////////////////////////////
  void falcon_pos_callback(const tutorial_interfaces::msg::Falconpos & msg)
  { 
    offset_pos.at(0) = msg.x / 100 * mapping_ratio;
    offset_pos.at(1) = msg.y / 100 * mapping_ratio;
    offset_pos.at(2) = msg.z / 100 * mapping_ratio;
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr controller_pub_;
  rclcpp::TimerBase::SharedPtr controller_timer_;

  rclcpp::Publisher<tutorial_interfaces::msg::Falconpos>::SharedPtr tcp_pos_pub_;
  rclcpp::TimerBase::SharedPtr tcp_pos_timer_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_vals_sub_;

  rclcpp::Subscription<tutorial_interfaces::msg::Falconpos>::SharedPtr falcon_pos_sub_;
  
};



/////////////////////////////// my own ik function ///////////////////////////////

void compute_ik(std::vector<double>& offset, std::vector<double>& origin, std::vector<double>& curr_joint_vals, std::vector<double>& control_joint_vals) {

  auto start = std::chrono::high_resolution_clock::now();

	//Create solvers
	KDL::ChainFkSolverPos_recursive fk_solver(panda_chain);
	KDL::ChainIkSolverVel_pinv vel_ik_solver(panda_chain, 0.0001, 1000);
	KDL::ChainIkSolverPos_NR ik_solver(panda_chain, fk_solver, vel_ik_solver, 1000);

  //Create the KDL array of current joint values
  KDL::JntArray jnt_pos_start(n_joints);
  for (unsigned int i=0; i<n_joints; i++) {
    jnt_pos_start(i) = curr_joint_vals.at(i);
  }

  //Write in the initial orientation if not already done so
  if (!got_orientation) {
    //Compute current tcp position
    KDL::Frame tcp_pos_start;
    fk_solver.JntToCart(jnt_pos_start, tcp_pos_start);
    orientation = tcp_pos_start.M;
    got_orientation = true;
  }

  for (unsigned int i=0; i<3; i++) {
    tcp_pos.at(i) = origin.at(i) + offset.at(i);
  }

  //Create the task-space goal object
  KDL::Vector vec_tcp_pos_goal(offset.at(0) + origin.at(0), offset.at(1) + origin.at(1), offset.at(2) + origin.at(2));
  KDL::Frame tcp_pos_goal(orientation, vec_tcp_pos_goal);

  //Compute inverse kinematics
  KDL::JntArray jnt_pos_goal(n_joints);
  ik_solver.CartToJnt(jnt_pos_start, tcp_pos_goal, jnt_pos_goal);

  //Change the control joint values and finish the function
  for (unsigned int i=0; i<n_joints; i++) {
    control_joint_vals.at(i) = jnt_pos_goal.data(i);
  }

  if (display_time) {
    auto finish = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
    std::cout << "Execution of my IK solver function took " << duration.count() << " [microseconds]" << std::endl;
  }
  
}


///////////////// other helper functions /////////////////

bool within_limits(std::vector<double>& vals) {
  for (unsigned int i=0; i<n_joints; i++) {
    if (vals.at(i) > upper_joint_limits.at(i) || vals.at(i) < lower_joint_limits.at(i)) return false;
  }
  return true;
}

bool create_tree() {
  if (!kdl_parser::treeFromFile(urdf_path, panda_tree)){
		std::cout << "Failed to construct kdl tree" << std::endl;
   	return false;
  }
  return true;
}

void get_chain() {
  panda_tree.getChain("panda_link0", "panda_grasptarget", panda_chain);
}

void print_joint_vals(std::vector<double>& joint_vals) {
  
  std::cout << "[ ";
  for (unsigned int i=0; i<joint_vals.size(); i++) {
    std::cout << joint_vals.at(i) << ' ';
  }
  std::cout << "]" << std::endl;
}




//////////////////// MAIN FUNCTION ///////////////////

int main(int argc, char * argv[])
{   
  rclcpp::init(argc, argv);

  std::shared_ptr<CartesianController> michael = std::make_shared<CartesianController>();

  rclcpp::spin(michael);

  rclcpp::shutdown();
  return 0;
}

