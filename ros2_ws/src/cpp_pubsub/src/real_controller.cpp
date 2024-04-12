#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "tutorial_interfaces/msg/falconpos.hpp"
#include "tutorial_interfaces/msg/pos_info.hpp"

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

#include <algorithm>

#include <iostream>
#include <fstream>
#include <sstream>


using namespace std::chrono_literals;


/////////////////// global variables ///////////////////
const std::string urdf_path = "/home/michael/AutonomyFitts/ros2_ws/src/cpp_pubsub/urdf/panda.urdf";
const unsigned int n_joints = 7;

const std::vector<double> lower_joint_limits {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
const std::vector<double> upper_joint_limits {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

const bool disp_ik_solve_time = false;

KDL::Tree panda_tree;
KDL::Chain panda_chain;

KDL::Rotation orientation;
bool got_orientation = false;

std::vector<double> tcp_pos {0.5059, 0.0, 0.4346};   // initialized the a self-defined "home" position

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

bool within_limits(std::vector<double>& vals);
bool create_tree();
void get_chain();

void print_joint_vals(std::vector<double>& joint_vals);


/////////////// DEFINITION OF NODE CLASS //////////////

class RealController : public rclcpp::Node
{
public:

  // parameters name list
  std::vector<std::string> param_names = {"free_drive", "mapping_ratio", "use_depth", "part_id", "alpha_id", "ring_id"};
  int free_drive {0};
  double mapping_ratio {3.0};
  int part_id {0};
  int alpha_id {0};
  int ring_id {0};
  
  std::vector<double> origin {0.5059, 0.0, 0.4346}; //////// can change the task-space origin point! ////////

  std::vector<double> human_offset {0.0, 0.0, 0.0};
  std::vector<double> robot_offset {0.0, 0.0, 0.0};

  std::vector<double> curr_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> ik_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> message_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::vector<double> initial_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> final_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> home_joint_vals {0, -M_PI_4/2, 0, -5 * M_PI_4/2, 0, M_PI_2, M_PI_4};

  bool control = false;
  
  const int control_freq = 500;   // the rate at which the "controller_publisher" function is called in [Hz]
  const int tcp_pub_frequency = 40;   // in [Hz]

  // step 1: prep-time (listens to joint values)
  // step 2: smoothing -> used to initially smoothly incorporate the Falcon offset (and then float for a while before starting)
  // step 3: complete the ring (however long it takes)
  // step 4: shift control back to robot
  // step 5: move the robot back to self-defined home position

  // times for each step
  const int prep_time = 5;
  const int require_initial_vals_time = 3;
  const int smoothing_time = 5;
  const int float_time = 2;     // time to float at starting position
  const int shifting_time = 3;
  const int homing_time = 2;
  const int shutdown_time = 1;

  // calculations of the required counts (based on control frequency)
  const int max_prep_count = control_freq * prep_time;
  int prep_count = 0;

  const int required_initial_vals = control_freq * require_initial_vals_time;   // get initial joint values for X seconds
  int initial_joint_vals_count = 0;

  const int max_smoothing_count = control_freq * smoothing_time;

  int max_shifting_count = shifting_time * control_freq;

  int max_homing_count = homing_time * control_freq;

  int max_shutdown_count = shutdown_time * control_freq;


  // IMPORTANT: BIG BOSS COUNTER HERE
  int count = 0;

  // alpha values = amount of HUMAN INPUT, in the range [0, 1]
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;

  // initial alpha values (from experimental setting)
  double iax = 0.0;
  double iay = 0.0;
  double iaz = 0.0;


  // Fitts ring parameters (fixed)
  std::vector<double> fitts_ring_origin {0.5059, 0.0, 0.4};
  int n_targets = 9;
  double r_small = 0.1;
  double r_big = 0.14;
  double w_small = 0.02;
  double w_big = 0.04;
  int curr_target_id = 0;   // in the range [0, 8]

  // Active Fitts ring parameters (for the current trial)
  // int ring_id = 1;
  double r_radius = 0.0;
  double w_target = 0.0;


  ////////////////////////////////////////////////////////////////////////
  RealController()
  : Node("real_controller")
  { 
    // parameter stuff
    this->declare_parameter(param_names.at(0), 0);
    this->declare_parameter(param_names.at(1), 3.0);
    this->declare_parameter(param_names.at(2), 0);
    this->declare_parameter(param_names.at(3), 0);
    this->declare_parameter(param_names.at(4), 0);
    
    std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);
    free_drive = std::stoi(params.at(0).value_to_string().c_str());
    mapping_ratio = std::stod(params.at(1).value_to_string().c_str());
    part_id = std::stoi(params.at(2).value_to_string().c_str());
    alpha_id = std::stoi(params.at(3).value_to_string().c_str());
    ring_id = std::stoi(params.at(4).value_to_string().c_str());

    // overwrite alpha_id if the free drive mode is activated
    if (free_drive == 1) alpha_id = 5;

    print_params();

    // update {ax, ay, az} values using the parameter "alpha_id"
    ax = alphas_dict.at(alpha_id).at(0);
    ay = alphas_dict.at(alpha_id).at(1);
    az = alphas_dict.at(alpha_id).at(2);
    
    // also store them into the initial alpha values
    iax = ax;
    iay = ay;
    iaz = az;

    // write the Fitts ring parameters
    switch (ring_id) {
      case 1: r_radius = r_small; w_target = w_big;   break;
      case 2: r_radius = r_big;   w_target = w_big;   break;
      case 3: r_radius = r_small; w_target = w_small; break;
      case 4: r_radius = r_big;   w_target = w_small; break;
    }

    // joint controller publisher & timer
    controller_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("desired_joint_vals", 10);
    controller_timer_ = this->create_wall_timer(2ms, std::bind(&RealController::controller_publisher, this));    // controls at 500 Hz

    // tcp position publisher & timer
    tcp_pos_pub_ = this->create_publisher<tutorial_interfaces::msg::PosInfo>("tcp_position", 10);
    // tcp_pos_timer_ = this->create_wall_timer(25ms, std::bind(&RealController::tcp_pos_publisher, this));    // publishes at 40 Hz

    // second_last_point publisher
    last_point_pub_ = this->create_publisher<std_msgs::msg::Bool>("last_point", 10);  // publishes only once

    // controller count publisher, same frequency as the controller
    // count_pub_ = this->create_publisher<std_msgs::msg::Float64>("controller_count", 10);

    // countdown publisher, only publishes at whole second points during smoothing
    countdown_pub_ = this->create_publisher<std_msgs::msg::Float64>("countdown", 10);

    joint_vals_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "franka/joint_states", 10, std::bind(&RealController::joint_states_callback, this, std::placeholders::_1));

    falcon_pos_sub_ = this->create_subscription<tutorial_interfaces::msg::Falconpos>(
      "falcon_position", 10, std::bind(&RealController::falcon_pos_callback, this, std::placeholders::_1));

    // create the curr_target_id subscriber
    curr_target_id_sub_ = this->create_subscription<std_msgs::msg::Int16>(
    "curr_target_id", 10, std::bind(&MarkerPublisher::curr_target_id_callback, this, std::placeholders::_1));

    //Create Panda tree and get its kinematic chain
    if (!create_tree()) rclcpp::shutdown();
    get_chain();

  }

private:
  
  ///////////////////////////////////// JOINT CONTROLLER /////////////////////////////////////
  void controller_publisher()
  { 
    if (!control) {

      prep_count++;
      if (prep_count % control_freq == 0) std::cout << "The prep_count is currently " << prep_count << "\n" << std::endl; 
      if (prep_count == max_prep_count) control = true;

      if (prep_count > max_prep_count - control_freq*2) {
        ///////// warm-up the wait-set 2 seconds before actual control /////////
        ///////// here we need to publish the initial_joint_vals /////////
        auto q_desired = sensor_msgs::msg::JointState();
        q_desired.position = initial_joint_vals;
        controller_pub_->publish(q_desired);
      }
      

    } else {

      // get the robot control offset in Cartesian space (calling the corresponding function of the ring_id)
      t_param = (double) (count - max_smoothing_count) / max_recording_count * 2 * M_PI;   // t_param is in the range [0, 2pi], but can be out of range
      get_robot_control(t_param);      

      // gradually change control authority to fully robot after 10 second trajectory
      if (count > max_smoothing_count+max_recording_count && count <= max_smoothing_count+max_recording_count+max_shifting_count) {
        double shift_t = (double) (count - max_smoothing_count - max_recording_count) / max_shifting_count;
        ax = (1.0 - shift_t) * iax;
        ay = (1.0 - shift_t) * iay;
        az = (1.0 - shift_t) * iaz;
      }
      // write the joint values at the final trajectory position
      if (count == max_smoothing_count+max_recording_count+max_shifting_count) {
        for (size_t i=0; i<7; i++) final_joint_vals.at(i) = curr_joint_vals.at(i);
      }
      
      // perform the convex combination of robot and human offsets
      // also adding the origin and thus representing it as tcp_pos in the robot's base frame
      tcp_pos.at(0) = origin.at(0) + ax * human_offset.at(0) + (1-ax) * robot_offset.at(0);
      tcp_pos.at(1) = origin.at(1) + ay * human_offset.at(1) + (1-ay) * robot_offset.at(1);
      tcp_pos.at(2) = origin.at(2) + az * human_offset.at(2) + (1-az) * robot_offset.at(2);

      ///////// compute IK /////////
      compute_ik(tcp_pos, curr_joint_vals, ik_joint_vals);

      ///////////// publish the tcp position message /////////////
      if (record_flag && ((count - max_smoothing_count) % (control_freq / 40) == 0)) RealController::tcp_pos_publisher();

      ///////// initial smooth transitioning from current position to Falcon-mapped position /////////
      count++;  // increase count

      if (count <= max_smoothing_count) {
        double ratio = 0.0;
        if (count <= max_smoothing_count - control_freq * float_time) {
          // get lerp position using time
          ratio = (double) count / (max_smoothing_count - control_freq * float_time);    // need to get there early and "float"
        } else {
          ratio = 1.0;
        }
        // std::cout << "The smoothing ratio is " << ratio << std::endl;

        for (unsigned int i=0; i<n_joints; i++) message_joint_vals.at(i) = ratio * ik_joint_vals.at(i) + (1-ratio) * initial_joint_vals.at(i);

      } else {
        for (unsigned int i=0; i<n_joints; i++) message_joint_vals.at(i) = ik_joint_vals.at(i);
      }

      // bring it home boys
      if (count > max_smoothing_count + max_recording_count + max_shifting_count) {
        double hr = 0.0;
        if (count <= max_smoothing_count + max_recording_count + max_shifting_count + max_homing_count) {
          hr = (double) (count - max_smoothing_count - max_recording_count - max_shifting_count) / max_homing_count;
        } else {
          hr = 1.0;
        }
        for (size_t i=0; i<7; i++) message_joint_vals.at(i) = hr * home_joint_vals.at(i) + (1-hr) * final_joint_vals.at(i);
      }
      // shutdown down 1 second after homing
      if (count == max_smoothing_count + max_recording_count + max_shifting_count + max_homing_count + max_shutdown_count) {
        std::cout << "\n    Trial finished cleanly! Shutting down now ... Bye-bye!    \n" << std::endl;
        rclcpp::shutdown();
      }

      ///////// check limits /////////
      if (!within_limits(message_joint_vals)) {
        std::cout << "--------\nThese violate the joint limits of the Panda arm, shutting down now !!!\n---------" << std::endl;
        rclcpp::shutdown();
      }

      ///////// prepare and publish the desired_joint_vals message /////////
      auto q_desired = sensor_msgs::msg::JointState();
      q_desired.position = message_joint_vals;
      controller_pub_->publish(q_desired);

      // set the record flag as true
      if ((count == max_smoothing_count) && (!record_flag)) {
        record_flag = true;
        std::cout << "\n\n\n\n\n\n======================= RECORD FLAG IS SET TO => TRUE =======================\n\n\n\n\n\n" << std::endl;
      }

      // set the record flag as false
      if ((count == max_smoothing_count + max_recording_count) && (record_flag == true)) {
        std::cout << "\n\n\n\n\n\n======================= RECORD FLAG IS SET TO => FALSE =======================\n\n\n\n\n\n" << std::endl;
        record_flag = false; 
      }

      // ///////////// publish the controller count message /////////////
      // auto count_msg = std_msgs::msg::Float64();
      // count_msg.data = count;
      // count_pub_->publish(count_msg);

      ///////////// check if need to publish the countdown message /////////////
      if (count % control_freq == 0) {
        auto count_msg = std_msgs::msg::Float64();
        count_msg.data = count / control_freq;
        countdown_pub_->publish(count_msg);
      }
    }
  }

  /////////////// CALLBACK LISTENING TO THE CURRENT TARGET ID ///////////////
  void curr_target_id_callback(const std_msgs::msg::Int16 & msg)
  { 
    curr_target_id = msg.data;
    std::cout << "\n\nReceived a new incoming target ID = " << curr_target_id << " !!!\n\n" << std::endl;
  }

  ///////////////////////////////////// TCP POSITION PUBLISHER /////////////////////////////////////
  void tcp_pos_publisher()
  { 
    // if (count > max_smoothing_count + max_recording_count - tcp_pub_frequency) {
    //   auto lp = std_msgs::msg::Bool();
    //   lp.data = true;
    //   std::cout << "\n\n\n\n\n\n======================= SETTING LAST POINT TO => TRUE =======================\n\n\n\n\n\n" << std::endl;
    //   last_point_pub_->publish(lp);
    // }

    // note: this is in meters
    auto message = tutorial_interfaces::msg::PosInfo();

    message.ref_position = {
      origin.at(0) + ref_offset.at(0),
      origin.at(1) + ref_offset.at(1),
      origin.at(2) + ref_offset.at(2)
    };

    message.human_position = {
      origin.at(0) + human_offset.at(0),
      origin.at(1) + human_offset.at(1),
      origin.at(2) + human_offset.at(2)
    };

    message.robot_position = {
      origin.at(0) + robot_offset.at(0),
      origin.at(1) + robot_offset.at(1),
      origin.at(2) + robot_offset.at(2)
    };

    message.tcp_position = {
      tcp_pos.at(0),
      tcp_pos.at(1),
      tcp_pos.at(2)
    };

    message.time_from_start = (double) (count - max_smoothing_count) / max_recording_count * 10;    // out of total of 10 seconds

    tcp_pos_pub_->publish(message);
    
  }

  ///////////////////////////////////// JOINT STATES SUBSCRIBER /////////////////////////////////////
  void joint_states_callback(const sensor_msgs::msg::JointState & msg)
  { 
    auto data = msg.position;
    for (unsigned int i=0; i<n_joints; i++) {
      curr_joint_vals.at(i) = data.at(i);
    }
    // get and store initial joint values if haven't received enough messages
    if (initial_joint_vals_count < required_initial_vals) {
      for (unsigned int i=0; i<n_joints; i++) {
        initial_joint_vals.at(i) = data.at(i);
      }
      initial_joint_vals_count++;

      // print_joint_vals(initial_joint_vals);
    }
  }

  ///////////////////////////////////// FALCON SUBSCRIBER /////////////////////////////////////
  void falcon_pos_callback(const tutorial_interfaces::msg::Falconpos & msg)
  { 
    human_offset.at(0) = msg.x / 100 * mapping_ratio;
    human_offset.at(1) = msg.y / 100 * mapping_ratio;
    human_offset.at(2) = msg.z / 100 * mapping_ratio;
  }

  /////////////////////////////// robot control function ///////////////////////////////
  void get_robot_control(double t) 
  { 
    int within_traj_count = count - max_smoothing_count;

    // make sure t = [0, 2pi], wtj = [0, 5000]
    if (t < 0.0) {t = 0.0; within_traj_count = 0;}
    if (t > 2*M_PI) {t = 2*M_PI; within_traj_count = 5000;}

    // assign the noise
    double noise = robot_noise_vector.at(within_traj_count);
    if (within_traj_count%100==0) std::cout << "noise_value = " << noise << std::endl;

    // compute reference position and assign into ref_position vector
    ref_offset.at(0) = 0.0;
    if (use_depth) ref_offset.at(0) = abs(t-M_PI) / M_PI * depth - (depth/2);
    ref_offset.at(1) = t / (2*M_PI) * width - (width/2);
    ref_offset.at(2) = (ph*height) * (sin(pa*(t+ps)) + sin(pb*(t+ps)) + sin(pc*(t+ps)));

    // compute robot target = reference position + noise
    robot_offset.at(0) = ref_offset.at(0);
    robot_offset.at(1) = ref_offset.at(1);
    robot_offset.at(2) = ref_offset.at(2) + noise;
    // robot_offset.at(2) = ref_offset.at(2);
  }


  ///////////////////////////////////// FUNCTION TO PRINT PARAMETERS /////////////////////////////////////
  void print_params() {
    for (unsigned int i=0; i<10; i++) std::cout << "\n";
    std::cout << "\n\nThe current parameters [real_controller] are as follows:\n" << std::endl;
    std::cout << "Free drive mode = " << free_drive << "\n" << std::endl;
    std::cout << "Mapping ratio = " << mapping_ratio << "\n" << std::endl;
    std::cout << "Participant ID = " << part_id << "\n" << std::endl;
    std::cout << "Alpha ID = " << alpha_id << "\n" << std::endl;
    std::cout << "Ring ID = " << ring_id << "\n" << std::endl;
    for (unsigned int i=0; i<10; i++) std::cout << "\n";
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr controller_pub_;
  rclcpp::TimerBase::SharedPtr controller_timer_;

  rclcpp::Publisher<tutorial_interfaces::msg::PosInfo>::SharedPtr tcp_pos_pub_;
  // rclcpp::TimerBase::SharedPtr tcp_pos_timer_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr last_point_pub_;

  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr count_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr countdown_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_vals_sub_;

  rclcpp::Subscription<tutorial_interfaces::msg::Falconpos>::SharedPtr falcon_pos_sub_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr curr_target_id_sub_;
  
};




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

  if (disp_ik_solve_time) {
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

