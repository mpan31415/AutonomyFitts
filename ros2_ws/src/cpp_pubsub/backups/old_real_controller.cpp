#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
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
// const std::string urdf_path = "/home/michael/FOR_TESTING/panda.urdf";
const std::string urdf_path = "/home/michael/HRI/ros2_ws/src/cpp_pubsub/urdf/panda.urdf";
const unsigned int n_joints = 7;

const std::vector<double> lower_joint_limits {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
const std::vector<double> upper_joint_limits {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

const bool display_time = true;

KDL::Tree panda_tree;
KDL::Chain panda_chain;

KDL::Rotation orientation;
bool got_orientation = false;

std::vector<double> tcp_pos {0.3069, 0.0, 0.4853};   // initialized the same as the "home" position

//////// global dictionaries ////////
std::vector< std::vector<double> > alphas_dict {
  {0.0, 0.0, 0.0},  // 0
  {0.2, 0.2, 0.2},  // 1
  {0.4, 0.4, 0.4},  // 2
  {0.6, 0.6, 0.6},  // 3
  {0.8, 0.8, 0.8},  // 4
  {1.0, 1.0, 1.0}   // 5
};


/////////////////// function declarations ///////////////////
void compute_ik(std::vector<double>& desired_tcp_pos, std::vector<double>& curr_vals, std::vector<double>& res_vals);

///// hard-coded trajectories /////
// void get_robot_control0(double t, std::vector<double>& vals);
// void get_robot_control1(double t, std::vector<double>& vals);

bool within_limits(std::vector<double>& vals);
bool create_tree();
void get_chain();

void print_joint_vals(std::vector<double>& joint_vals);


void get_rotation_matrix(int axis, double angle, std::vector<std::vector<double>> &T);       // axes are: {1-x, 2-y, 3-z}
void matrix_mult_vector(std::vector<std::vector<double>> &mat, std::vector<double> &vec, std::vector<double> &result);


/////////////// DEFINITION OF NODE CLASS //////////////

class RealController : public rclcpp::Node
{
public:

  // parameters name list
  std::vector<std::string> param_names = {"part_id", "traj_id", "auto_id"};
  int part_id {0};
  int traj_id {0};
  int auto_id {0};
  
  // std::vector<double> origin {0.3059, 0.0, 0.4846}; //////// can change the task-space origin point! ////////
  // std::vector<double> origin {0.4559, 0.0, 0.3346}; //////// can change the task-space origin point! ////////
  std::vector<double> origin {0.4559, 0.0, 0.3846}; //////// can change the task-space origin point! ////////

  std::vector<double> human_offset {0.0, 0.0, 0.0};
  std::vector<double> robot_offset {0.0, 0.0, 0.0};

  std::vector<double> curr_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> ik_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> message_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool control = false;
  
  const double mapping_ratio = 2.0;    /////// this ratio is {end-effector movement} / {Falcon movement}
  const int control_freq = 1000;   // the rate at which the "controller_publisher" function is called in [Hz]
  const double latency = 2.0;  // this is the artificial latency introduced into the joint points published
  const int tcp_pub_frequency = 20;   // in [Hz]

  // used to initially smoothly incorporate the Falcon offset
  const int smoothing_time = 5;   /// smoothing time in [seconds]
  const int max_smoothing_count = control_freq * smoothing_time;
  int count = 0;

  double w = 0.0;  // this is the weight used for initial smoothing, which goes from 0 -> 1 as count goes from 0 -> max_smoothing_count

  // alpha values in {x, y, z} dimensions, used for convex combination of human and robot input
  // alpha values correspond to the share of HUMAN INPUT
  // they have to be in the range of [0, 1], and here, we are only initializing them
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;

  // trajectory recording
  const int traj_duration = 10;   // in [seconds]
  int max_recording_count = control_freq * traj_duration;
  int recording_count = 0;
  bool record_flag = false;

  // for robot trajectory following
  double t_param = 0.0;

  // for transformations
  std::vector<std::vector<double>> T {{0,0,0}, {0,0,0}, {0,0,0}};
  std::vector<double> pre_point {0, 0, 0};


  ////////////////////////////////////////////////////////////////////////
  RealController()
  : Node("real_controller")
  { 
    // parameter stuff
    this->declare_parameter(param_names.at(0), 0);
    this->declare_parameter(param_names.at(1), 0);
    this->declare_parameter(param_names.at(2), 0);
    // this->declare_parameter(param_names.at(2), rclcpp::PARAMETER_INTEGER);
    
    std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);
    part_id = std::stoi(params.at(0).value_to_string().c_str());
    traj_id = std::stoi(params.at(1).value_to_string().c_str());
    auto_id = std::stoi(params.at(2).value_to_string().c_str());
    print_params();

    // update {ax, ay, az} values using the parameter "auto_id"
    ax = alphas_dict.at(auto_id).at(0);
    ay = alphas_dict.at(auto_id).at(1);
    az = alphas_dict.at(auto_id).at(2);

    // joint controller publisher & timer
    // controller_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10);
    controller_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("desired_joint_vals", 10);
    controller_timer_ = this->create_wall_timer(1ms, std::bind(&RealController::controller_publisher, this));    // controls at 1000 Hz 

    // tcp position publisher & timer
    tcp_pos_pub_ = this->create_publisher<tutorial_interfaces::msg::Falconpos>("tcp_position", 10);
    tcp_pos_timer_ = this->create_wall_timer(50ms, std::bind(&RealController::tcp_pos_publisher, this));    // publishes at 20 Hz

    // recording flag publisher & timer
    record_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("record", 10);
    record_flag_timer_ = this->create_wall_timer(2ms, std::bind(&RealController::record_flag_publisher, this));    // publishes at 500 Hz

    joint_vals_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&RealController::joint_states_callback, this, std::placeholders::_1));

    falcon_pos_sub_ = this->create_subscription<tutorial_interfaces::msg::Falconpos>(
      "falcon_position", 10, std::bind(&RealController::falcon_pos_callback, this, std::placeholders::_1));

    //Create Panda tree and get its kinematic chain
    if (!create_tree()) rclcpp::shutdown();
    get_chain();

    // get the correct transformation for the given trajectory 
    switch (traj_id) {
      case 2: get_rotation_matrix(2, 45, T); break;
    }

  }

private:
  
  ///////////////////////////////////// JOINT CONTROLLER /////////////////////////////////////
  void controller_publisher()
  { 
    if (control == true) {

      auto traj_message = trajectory_msgs::msg::JointTrajectory();
      traj_message.joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

      // get the robot control offset in Cartesian space (calling the corresponding function of the traj_id)
      t_param = (double) (count - max_smoothing_count) / max_recording_count * 2 * M_PI;   // for circle, parametrized in the range [0, 2pi]
      switch (traj_id) {
        case 0: get_robot_control0(t_param); break;
        case 1: get_robot_control1(t_param); break;
        case 2: get_robot_control2(t_param); break;
      }

      // perform the convex combination of robot and human offsets
      // also adding the origin and thus representing it as tcp_pos in the robot's base frame
      tcp_pos.at(0) = origin.at(0) + ax * human_offset.at(0) + (1-ax) * robot_offset.at(0);
      tcp_pos.at(1) = origin.at(1) + ay * human_offset.at(1) + (1-ay) * robot_offset.at(1);
      tcp_pos.at(2) = origin.at(2) + az * human_offset.at(2) + (1-az) * robot_offset.at(2);

      ///////// compute IK /////////
      compute_ik(tcp_pos, curr_joint_vals, ik_joint_vals);

      ///////// initial smooth transitioning from current position to Falcon-mapped position /////////
      count++;  // increase count
      if (count <= max_smoothing_count) w = pow((double)count / max_smoothing_count, 2.0);  // use cubic increase to make it smoother
      std::cout << "The current count is " << count << std::endl;
      std::cout << "The current weight is " << w << std::endl;
      for (unsigned int i=0; i<n_joints; i++) message_joint_vals.at(i) = w * ik_joint_vals.at(i) + (1-w) * curr_joint_vals.at(i);

      ///////// check limits /////////
      if (!within_limits(message_joint_vals)) {
        std::cout << "--------\nThese violate the joint limits of the Panda arm, shutting down now !!!\n---------" << std::endl;
        rclcpp::shutdown();
      }

      
      ///////// prepare the trajectory message, introducing artificial latency /////////
      // auto point = trajectory_msgs::msg::JointTrajectoryPoint();
      // point.positions = message_joint_vals;
      // point.time_from_start.nanosec = (int)(1000 / control_freq) * latency * 1000000;     //// => {milliseconds} * 1e6

      // traj_message.points = {point};

      // std::cout << "The joint values [MESSAGE] are ";
      // print_joint_vals(message_joint_vals);
      // controller_pub_->publish(traj_message);


      ///////// prepare the desired_joint_vals message /////////
      auto q_desired = sensor_msgs::msg::JointState();
      q_desired.position = message_joint_vals;
      controller_pub_->publish(q_desired);





      //////////////////////// NOW PUBLISH THE TCP_POS ////////////////////////
      // // note: this is in meters
      // auto tcp_message = tutorial_interfaces::msg::Falconpos();
      // tcp_message.x = tcp_pos.at(0);
      // tcp_message.y = tcp_pos.at(1);
      // tcp_message.z = tcp_pos.at(2);

      // // RCLCPP_INFO(this->get_logger(), "Publishing controller joint values");
      // tcp_pos_pub_->publish(tcp_message);


      // set the record flag as either true or false, depending on the number of trajectory points executed
      if (count == max_smoothing_count && !record_flag && recording_count == 0) {
        record_flag = true;
        std::cout << "\n\n\n\n\n\n======================= RECORD FLAG IS SET TO => TRUE =======================\n\n\n\n\n\n" << std::endl;
      }
      if (record_flag) {
        if (recording_count < max_recording_count) {
          recording_count++;
        } else {
        record_flag = false;
        std::cout << "\n\n\n\n\n\n======================= RECORD FLAG IS SET TO => FALSE =======================\n\n\n\n\n\n" << std::endl;
        }
      }
    }
  }

  /////////////////////////////// robot control (trajectory following) functions ///////////////////////////////

  void get_robot_control0(double t)
  {
    // vertical circle of radius 0.1m, parametrized in the range [0, 2pi]
    double r = 0.1;
    double x = 0.0;
    double y = r * sin(t);
    double z = r * cos(t);
    robot_offset.at(0) = x;
    robot_offset.at(1) = y;
    robot_offset.at(2) = z;
    // if recording hasn't started, move the robot to the desired starting position
    if (t < 0.0) {
      robot_offset.at(0) = 0.00;
      robot_offset.at(1) = 0.00;
      robot_offset.at(2) = r;
    }
    // if recording has finished, keep the robot at the finishing position
    if (t > 2*M_PI) {
      robot_offset.at(0) = 0.00;
      robot_offset.at(1) = 0.00;
      robot_offset.at(2) = r;
    }
  }

  void get_robot_control1(double t) 
  {
    // horizontal circle of radius 0.1m, parametrized in the range [0, 2pi]
    double r = 0.1;
    double x = -r * cos(t);
    double y = r * sin(t);
    double z = 0.0;
    robot_offset.at(0) = x;
    robot_offset.at(1) = y;
    robot_offset.at(2) = z;
    // if recording hasn't started, move the robot to the desired starting position
    if (t < 0.0) {
      robot_offset.at(0) = -r;
      robot_offset.at(1) = 0.00;
      robot_offset.at(2) = 0.00;
    }
    // if recording has finished, keep the robot at the finishing position
    if (t > 2*M_PI) {
      robot_offset.at(0) = -r;
      robot_offset.at(1) = 0.00;
      robot_offset.at(2) = 0.00;
    }
  }

  void get_robot_control2(double t) 
  {
    // horizontal circle of radius 0.1m, parametrized in the range [0, 2pi]
    double r = 0.1;
    double x = -r * cos(t);
    double y = r * sin(t);
    double z = 0.0;

    pre_point = {x, y, z};
    matrix_mult_vector(T, pre_point, robot_offset);

    // if recording hasn't started, move the robot to the desired starting position
    if (t < 0.0) {
      pre_point = {-r, 0.0, 0.0};
      matrix_mult_vector(T, pre_point, robot_offset);
    }
    // if recording has finished, keep the robot at the finishing position
    if (t > 2*M_PI) {
      pre_point = {-r, 0.0, 0.0};
      matrix_mult_vector(T, pre_point, robot_offset);
    }
  }

  ///////////////////////////////////// TCP POSITION PUBLISHER /////////////////////////////////////
  void tcp_pos_publisher()
  { 
    if (record_flag) {
      // note: this is in meters
      auto message = tutorial_interfaces::msg::Falconpos();
      message.x = tcp_pos.at(0);
      message.y = tcp_pos.at(1);
      message.z = tcp_pos.at(2);

      tcp_pos_pub_->publish(message);
    }
  }

  ///////////////////////////////////// TRAJ RECORD FLAG PUBLISHER /////////////////////////////////////
  void record_flag_publisher()
  { 
    auto message = std_msgs::msg::Bool();
    message.data = record_flag;

    // RCLCPP_INFO(this->get_logger(), "Publishing controller joint values");
    record_flag_pub_->publish(message);
  }

  ///////////////////////////////////// JOINT STATES SUBSCRIBER /////////////////////////////////////
  void joint_states_callback(const sensor_msgs::msg::JointState & msg)
  { 
    auto data = msg.position;

    // write into our class variable for storing joint values
    for (unsigned int i=0; i<n_joints; i++) {
      curr_joint_vals.at(i) = data.at(i);
    }

    /// if this the first iteration, change the control flag and start partying!
    if (control == false) control = true;
  }

  ///////////////////////////////////// FALCON SUBSCRIBER /////////////////////////////////////
  void falcon_pos_callback(const tutorial_interfaces::msg::Falconpos & msg)
  { 
    human_offset.at(0) = msg.x / 100 * mapping_ratio;
    human_offset.at(1) = msg.y / 100 * mapping_ratio;
    human_offset.at(2) = msg.z / 100 * mapping_ratio;
    // std::cout << "x = " << human_offset.at(0) << ", " << "y = " << human_offset.at(1) << ", " << "z = " << human_offset.at(2) << std::endl;
  }

  ///////////////////////////////////// FUNCTION TO PRINT PARAMETERS /////////////////////////////////////
  void print_params() {
    for (unsigned int i=0; i<10; i++) std::cout << "\n";
    std::cout << "\n\nThe current parameters [real_controller] are as follows:\n" << std::endl;
    std::cout << "Participant ID = " << part_id << "\n" << std::endl;
    std::cout << "Trajectory ID = " << traj_id << "\n" << std::endl;
    std::cout << "Autonomy ID = " << auto_id << "\n" << std::endl;
    for (unsigned int i=0; i<10; i++) std::cout << "\n";
  }

  // rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr controller_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr controller_pub_;
  rclcpp::TimerBase::SharedPtr controller_timer_;

  rclcpp::Publisher<tutorial_interfaces::msg::Falconpos>::SharedPtr tcp_pos_pub_;
  rclcpp::TimerBase::SharedPtr tcp_pos_timer_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr record_flag_pub_;
  rclcpp::TimerBase::SharedPtr record_flag_timer_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_vals_sub_;

  rclcpp::Subscription<tutorial_interfaces::msg::Falconpos>::SharedPtr falcon_pos_sub_;
  
};



/////////////////////////// util functions ///////////////////////////

void get_rotation_matrix(int axis, double angle, std::vector<std::vector<double>> &T)
{    
  double th = angle / 180 * M_PI;
  switch (axis) {
      case 1: T.at(0) = {1,0,0}; T.at(1) = {0,cos(th),-sin(th)}; T.at(2) = {0,sin(th),cos(th)}; break;
      case 2: T.at(0) = {cos(th),0,sin(th)}; T.at(1) = {0,1,0}; T.at(2) = {-sin(th),0,cos(th)}; break;
      case 3: T.at(0) = {cos(th),-sin(th),0}; T.at(1) = {sin(th),cos(th),0}; T.at(2) = {0,0,1}; break;
  }
}


void matrix_mult_vector(std::vector<std::vector<double>> &mat, std::vector<double> &vec, std::vector<double> &result) 
{   
  for (size_t i=0; i<mat.size(); i++) {
      auto row = mat.at(i);
      double sum {0};
      for (size_t j=0; j<row.size(); j++) {
          sum += row.at(j) * vec.at(j);
      }
      result.at(i) = sum;
  }
}



/////////////////////////////// my own ik function ///////////////////////////////

void compute_ik(std::vector<double>& desired_tcp_pos, std::vector<double>& curr_vals, std::vector<double>& res_vals) {

  auto start = std::chrono::high_resolution_clock::now();

	//Create solvers
	KDL::ChainFkSolverPos_recursive fk_solver(panda_chain);
	KDL::ChainIkSolverVel_pinv vel_ik_solver(panda_chain, 0.0001, 1000);
	KDL::ChainIkSolverPos_NR ik_solver(panda_chain, fk_solver, vel_ik_solver, 1000);

  //Create the KDL array of current joint values
  KDL::JntArray jnt_pos_start(n_joints);
  for (unsigned int i=0; i<n_joints; i++) {
    jnt_pos_start(i) = curr_vals.at(i);
  }

  //Write in the initial orientation if not already done so
  if (!got_orientation) {
    //Compute current tcp position
    KDL::Frame tcp_pos_start;
    fk_solver.JntToCart(jnt_pos_start, tcp_pos_start);
    orientation = tcp_pos_start.M;
    got_orientation = true;
  }

  //Create the task-space goal object
  // KDL::Vector vec_tcp_pos_goal(origin.at(0), origin.at(1), origin.at(2));
  KDL::Vector vec_tcp_pos_goal(desired_tcp_pos.at(0), desired_tcp_pos.at(1), desired_tcp_pos.at(2));
  KDL::Frame tcp_pos_goal(orientation, vec_tcp_pos_goal);

  //Compute inverse kinematics
  KDL::JntArray jnt_pos_goal(n_joints);
  ik_solver.CartToJnt(jnt_pos_start, tcp_pos_goal, jnt_pos_goal);

  //Change the control joint values and finish the function
  for (unsigned int i=0; i<n_joints; i++) {
    res_vals.at(i) = jnt_pos_goal.data(i);
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

  std::shared_ptr<RealController> michael = std::make_shared<RealController>();

  rclcpp::spin(michael);

  rclcpp::shutdown();
  return 0;
}

