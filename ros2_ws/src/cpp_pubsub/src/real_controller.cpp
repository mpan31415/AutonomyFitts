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

std::vector<double> cosine_interpolate(std::vector<double> start, std::vector<double> end, double mu);
std::vector<double> lerp(std::vector<double> start, std::vector<double> end, double mu);
bool within_limits(std::vector<double>& vals);
bool create_tree();
void get_chain();

void print_joint_vals(std::vector<double>& joint_vals);



/////////////// DEFINITION OF NODE CLASS //////////////

class RealController : public rclcpp::Node
{
public:

  // parameters name list
  std::vector<std::string> param_names = {"free_drive", "mapping_ratio", "part_id", "alpha_id", "ring_id"};
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

  const int control_freq = 500;   // the rate at which the "controller_publisher" function is called in [Hz]
  const int tcp_pub_frequency = 40;   // in [Hz]

  // step 1: prep-time (listens to joint values)
  // step 2: smoothing -> used to initially smoothly incorporate the Falcon offset (and then float for a while before starting)
  // step 3: complete the ring (however long it takes)
  // step 4: shift control back to robot
  // step 5: move the robot back to self-defined home position

  // times for each step
  const int prep_time = 4;
  const int require_initial_vals_time = 2;
  const int smoothing_time = 5;
  const int float_time = 2;     // time to float at starting position
  const int shifting_time = 2;
  const int homing_time = 4;
  const int shutdown_time = 1;

  ////// IMPORTANT TIME: ROBOT'S DEFINED MOVEMENT TIME FROM EACH TARGET TO THE NEXT //////
  const int robot_movement_time = 2;    // seconds

  // calculations of the required counts (based on control frequency)
  int prep_count = 0;
  const int max_prep_count = control_freq * prep_time;

  int initial_joint_vals_count = 0;
  const int required_initial_vals = control_freq * require_initial_vals_time;   // get initial joint values for X seconds

  int smoothing_count = 0;
  const int float_count = control_freq * float_time;
  const int max_smoothing_count = control_freq * smoothing_time;
  
  int movement_count = 0;
  const int max_movement_count = control_freq * robot_movement_time;
  
  int shifting_count = 0;
  const int max_shifting_count = control_freq * shifting_time;

  int homing_count = 0;
  const int max_homing_count = control_freq * homing_time;

  int shutdown_count = 0;
  const int max_shutdown_count = control_freq * shutdown_time;


  // OVERALL COUNTER
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
  std::vector<double> fitts_ring_origin {0.5059, 0.0, 0.4346};
  int n_targets = 9;
  double r_small = 0.06;
  double r_big   = 0.12;
  double w_small = 0.01;
  double w_big   = 0.02;

  // Fitts ring parameters (for the current trial)
  double r_radius = 0.0;
  double w_target = 0.0;

  ///////// OHTER TASK PARAMETERS /////////
  int task_state = 1;
  int last_target_id = 0;     // in the range [0, 8]
  int curr_target_id = 0;     // in the range [0, 8]
  int incoming_target_id = 0; // in the range [0, 8]

  std::vector<double> last_target_vec {0.5059, 0.0, 0.4346};
  std::vector<double> curr_target_vec {0.5059, 0.0, 0.4346};


  bool ring_finished = false;


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
    tcp_pos_timer_ = this->create_wall_timer(25ms, std::bind(&RealController::tcp_pos_publisher, this));    // publishes at 40 Hz

    // countdown publisher, only publishes at whole second points during smoothing
    countdown_pub_ = this->create_publisher<std_msgs::msg::Int16>("countdown", 10);

    joint_vals_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "franka/joint_states", 10, std::bind(&RealController::joint_states_callback, this, std::placeholders::_1));

    falcon_pos_sub_ = this->create_subscription<tutorial_interfaces::msg::Falconpos>(
      "falcon_position", 10, std::bind(&RealController::falcon_pos_callback, this, std::placeholders::_1));

    // create the curr_target_id subscriber
    curr_target_id_sub_ = this->create_subscription<std_msgs::msg::Int16>(
    "curr_target_id", 10, std::bind(&RealController::incoming_target_id_callback, this, std::placeholders::_1));

    // create the ring_finished flag subscriber
    ring_finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "ring_finished", 10, std::bind(&RealController::ring_finished_callback, this, std::placeholders::_1));

    //Create Panda tree and get its kinematic chain
    if (!create_tree()) rclcpp::shutdown();
    get_chain();

  }

private:
  
  ///////////////////////////////////// JOINT CONTROLLER /////////////////////////////////////
  void controller_publisher()
  { 
    count++;

    if (task_state == 1) /////////////////////////////////////////////////////////////////////////////
    {
      // initial 5 second waiting
      prep_count++;
      if (prep_count % control_freq == 0) std::cout << "The prep_count is currently " << prep_count << "\n" << std::endl; 
      if (prep_count == max_prep_count) task_state = 2;

      if (prep_count > max_prep_count - control_freq * 2) {
        ///////// warm-up the wait-set 2 seconds before actual control /////////
        ///////// here we need to publish the initial_joint_vals /////////
        ///////// prepare the trajectory message, introducing artificial latency /////////
        send_control_joint_vals(initial_joint_vals);
      }
      // publish countdown = 5 to marker publisher
      send_count(5);


    } 
    else if (task_state == 2) /////////////////////////////////////////////////////////////////////////////
    {
      // 5 seconds to move to the start position and float
      smoothing_count++;
      if (smoothing_count % control_freq == 0) {
        std::cout << "The smoothing_count is currently " << smoothing_count << "\n" << std::endl;
        int cd_count = (int)(max_smoothing_count - smoothing_count) / control_freq; // [4 -> 0]
        send_count(cd_count);
      }
      if (smoothing_count == max_smoothing_count) task_state = 3;
      get_smoothing_target();

      // perform the convex combination of robot and human offsets, tcp_pos is in robot's base frame
      tcp_pos.at(0) = origin.at(0) + ax * human_offset.at(0) + (1-ax) * robot_offset.at(0);
      tcp_pos.at(1) = origin.at(1) + ay * human_offset.at(1) + (1-ay) * robot_offset.at(1);
      tcp_pos.at(2) = origin.at(2) + az * human_offset.at(2) + (1-az) * robot_offset.at(2);
      compute_ik(tcp_pos, curr_joint_vals, ik_joint_vals);

      /////// perform smoothing / floating accordingly
      double ratio = 0.0;
      if (smoothing_count <= max_smoothing_count - float_count) {
        // get lerp position using time
        ratio = (double) smoothing_count / (max_smoothing_count - float_count);    // need to get there early and "float"
      } else {
        ratio = 1.0;
      }
      // std::cout << "The smoothing ratio is " << ratio << std::endl;
      for (unsigned int i=0; i<n_joints; i++) {message_joint_vals.at(i) = ratio * ik_joint_vals.at(i) + (1-ratio) * initial_joint_vals.at(i);}
      send_control_joint_vals(message_joint_vals);


    }
    else if (task_state == 3) /////////////////////////////////////////////////////////////////////////////
    {
      // in the process of reaching towards the new target (until completing the ring)
      if (curr_target_id == incoming_target_id) 
      { 
        // case 1: not yet at the new incoming target
        if (movement_count < max_movement_count) {
          // case 1.1: at least robot's part is not finished yet
          movement_count++;
          double mu = (double)(movement_count) / (double)(max_movement_count);   // mu in [0, 1]
          update_robot_offset(mu);
        }
        // ((((((((( case 1.2: robot done, human is still lacking ))))))))) -> do nothing to the robot_offset vector
        // std::cout << "ax = " << ax << " ay = " << ay << " az = " << az << std::endl;
        // std::cout << "R = (" << robot_offset.at(0) << ", " << robot_offset.at(1) << ", " << robot_offset.at(2) << ") !!!";
        // std::cout << "H = (" << human_offset.at(0) << ", " << human_offset.at(1) << ", " << human_offset.at(2) << ") !!!" << std::endl;
        tcp_pos.at(0) = origin.at(0) + ax * human_offset.at(0) + (1-ax) * robot_offset.at(0);
        tcp_pos.at(1) = origin.at(1) + ay * human_offset.at(1) + (1-ay) * robot_offset.at(1);
        tcp_pos.at(2) = origin.at(2) + az * human_offset.at(2) + (1-az) * robot_offset.at(2);
        compute_ik(tcp_pos, curr_joint_vals, ik_joint_vals);
        for (unsigned int i=0; i<n_joints; i++) {message_joint_vals.at(i) = ik_joint_vals.at(i);}
        send_control_joint_vals(message_joint_vals);

        if (ring_finished) {
          // ring is finished, advance to next task_state
          std::cout << "The Fitts ring is finished! " << "\n" << std::endl;
          task_state = 4;
          // send large count number to indicate ring has finished
          send_count(10);
        }

      } 
      else 
      {
        // case 2: received new incoming_target_id, previous target reached
        // ring not yet finished, set incoming target as new active target, update target vectors
        last_target_id = curr_target_id;
        curr_target_id = incoming_target_id;
        update_target_vectors();
        movement_count = 0;
      }

      
    } 
    else if (task_state == 4) /////////////////////////////////////////////////////////////////////////////
    {
      // Ring is finished, gradually shift control entirely back to robot
      if (shifting_count < max_shifting_count) {
        // not yet finished shifting
        shifting_count++;
        double mu = (double)(shifting_count) / (double)(max_shifting_count);
        ax = (1.0 - mu) * iax;
        ay = (1.0 - mu) * iay;
        az = (1.0 - mu) * iaz;
        tcp_pos.at(0) = origin.at(0) + ax * human_offset.at(0) + (1-ax) * robot_offset.at(0);
        tcp_pos.at(1) = origin.at(1) + ay * human_offset.at(1) + (1-ay) * robot_offset.at(1);
        tcp_pos.at(2) = origin.at(2) + az * human_offset.at(2) + (1-az) * robot_offset.at(2);
        compute_ik(tcp_pos, curr_joint_vals, ik_joint_vals);
        for (unsigned int i=0; i<n_joints; i++) {message_joint_vals.at(i) = ik_joint_vals.at(i);}
        send_control_joint_vals(message_joint_vals);

      } else {
        // finished shifting back to robot, write the joint values at the final trajectory position 
        for (size_t i=0; i<7; i++) final_joint_vals.at(i) = curr_joint_vals.at(i);
        std::cout << "\n\n Finished shifting control back to robot \n\n" << std::endl;
        // advance to next task_state
        task_state = 5;
      }
      

    }
    else if (task_state == 5) /////////////////////////////////////////////////////////////////////////////
    {
      // Go back to self-defined "home" position
      if (homing_count < max_homing_count) {
        // have not returned home yet
        homing_count++;
        double hr = (double)(homing_count) / (double)(max_homing_count);
        for (size_t i=0; i<7; i++) message_joint_vals.at(i) = hr * home_joint_vals.at(i) + (1-hr) * final_joint_vals.at(i);
        send_control_joint_vals(message_joint_vals);

      } else {
        // finished homing, advance to next task_state
        std::cout << "\n\n Finished homing the robot \n\n" << std::endl;
        task_state = 6;
      }


    }
    else if (task_state == 6) /////////////////////////////////////////////////////////////////////////////
    {
      // Wait and then shutdown node
      if (shutdown_count < max_shutdown_count) {
        shutdown_count++;
        send_control_joint_vals(home_joint_vals);
      } else {
        std::cout << "\n    Trial finished cleanly! Shutting down now ... Bye-bye!    \n" << std::endl;
        rclcpp::shutdown();
      }

    }


  }


  /////////////////////////////// send control joint values function ///////////////////////////////
  void send_control_joint_vals(std::vector<double> desired_joint_vals) 
  {
    ///////// check limits /////////
    if (!within_limits(desired_joint_vals)) {
      print_joint_vals(desired_joint_vals);
      std::cout << "--------\nThese violate the joint limits of the Panda arm, shutting down now !!!\n---------" << std::endl;
      rclcpp::shutdown();
    }
    // Real Robot
    auto q_desired = sensor_msgs::msg::JointState();
    q_desired.position = desired_joint_vals;
    controller_pub_->publish(q_desired);
    // std::cout << "--------\n[REAL] Publishing control joint values now !!!\n---------" << std::endl;
    // std::cout << "--------\n[FAKE] Publishing control joint values now !!!\n---------" << std::endl;
  }

  /////////////////////////////// update target vectors function ///////////////////////////////
  void update_target_vectors() 
  {
    // double last_theta = ((double)(last_target_id)/(double)(n_targets))*2*M_PI;   // in radians
    double curr_theta = ((double)(curr_target_id)/(double)(n_targets))*2*M_PI;   // in radians
    // update last target vector (using current tcp_pos)
    last_target_vec.at(0) = tcp_pos.at(0) - origin.at(0);
    last_target_vec.at(1) = tcp_pos.at(1) - origin.at(1);
    last_target_vec.at(2) = tcp_pos.at(2) - origin.at(2);
    // update curr target vector
    curr_target_vec.at(0) = 0.0;
    curr_target_vec.at(1) = r_radius*sin(curr_theta);
    curr_target_vec.at(2) = r_radius*cos(curr_theta);
    std::cout << "Finished updating the target vectors! \n" << std::endl;
    // // update last target vector (using calculation)
    // last_target_vec.at(0) = 0.0;
    // last_target_vec.at(1) = origin.at(1) + r_radius*sin(last_theta);
    // last_target_vec.at(2) = origin.at(2) + r_radius*cos(last_theta);
  }

  /////////////////////////////// initial 5 sec smoothing control function ///////////////////////////////
  void get_smoothing_target() 
  {
    robot_offset.at(0) = 0.0;
    robot_offset.at(1) = 0.0;
    robot_offset.at(2) = r_radius;
    // used only initially, when the robot smoothes towards and floats at the first target
    last_target_vec.at(0) = 0.0;
    last_target_vec.at(1) = 0.0;
    last_target_vec.at(2) = r_radius;
    curr_target_vec.at(0) = 0.0;
    curr_target_vec.at(1) = 0.0;
    curr_target_vec.at(2) = r_radius;
  }

  /////////////////////////////// robot control (trajectory following) function ///////////////////////////////
  void update_robot_offset(double mu) 
  {
    // cosine interpolate between last and current target vectors
    std::vector<double> interpolated = cosine_interpolate(last_target_vec, curr_target_vec, mu);
    // std::vector<double> interpolated = lerp(last_target_vec, curr_target_vec, mu);
    // update robot target vector
    robot_offset.at(0) = 0.0;
    robot_offset.at(1) = interpolated.at(1);
    robot_offset.at(2) = interpolated.at(2);
  }

  ///////////////////////////////////// TCP POSITION PUBLISHER /////////////////////////////////////
  void tcp_pos_publisher()
  { 
    auto message = tutorial_interfaces::msg::PosInfo();
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
    message.time_from_start = (double) (count - max_prep_count - max_smoothing_count) / control_freq;    // seconds
    tcp_pos_pub_->publish(message);
  }

  /////////////////////////////// sends count to marker publisher ///////////////////////////////
  void send_count(int co) 
  {
    auto msg = std_msgs::msg::Int16();
    msg.data = co;
    countdown_pub_->publish(msg);
  }

  /////////////// CALLBACK LISTENING TO THE CURRENT TARGET ID ///////////////
  void ring_finished_callback(const std_msgs::msg::Bool & msg)
  {
    ring_finished = msg.data;
    // std::cout << "\n\nRing finished = " << ring_finished << " !!!\n\n" << std::endl;
  }

  /////////////// CALLBACK LISTENING TO THE CURRENT TARGET ID ///////////////
  void incoming_target_id_callback(const std_msgs::msg::Int16 & msg)
  { 
    incoming_target_id = msg.data;
    // std::cout << "\n\nReceived a new incoming target ID = " << incoming_target_id << " !!!\n\n" << std::endl;
  }

  ///////////////////////////////////// JOINT STATES SUBSCRIBER /////////////////////////////////////
  void joint_states_callback(const sensor_msgs::msg::JointState & msg)
  { 
    auto data = msg.position;
    // write into our class variable for storing joint values
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
    // std::cout << "x = " << human_offset.at(0) << ", " << "y = " << human_offset.at(1) << ", " << "z = " << human_offset.at(2) << std::endl;
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
  rclcpp::TimerBase::SharedPtr tcp_pos_timer_;

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr countdown_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_vals_sub_;

  rclcpp::Subscription<tutorial_interfaces::msg::Falconpos>::SharedPtr falcon_pos_sub_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr curr_target_id_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ring_finished_sub_;
  
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

// Function to cosine interpolate between two vectors and return a new vector
std::vector<double> cosine_interpolate(std::vector<double> start, std::vector<double> end, double mu) {
  // mu in [0, 1]
  std::vector<double> interp_vec = {0.0, 0.0, 0.0};
  double angle = mu * M_PI;
  double interp_ratio = (1.0 - cos(angle)) * 0.5;                                   // in [0, 1]
  for (unsigned int i=0; i<start.size(); i++) {
    double interp_point = start.at(i) + interp_ratio * (end.at(i) - start.at(i));   // in [start, end]
    interp_vec.at(i) = interp_point;
  }
  return interp_vec;
}

// Function to linearly interpolate between two vectors
std::vector<double> lerp(std::vector<double> start, std::vector<double> end, double mu) {
  // mu in [0, 1]
  std::vector<double> lerp_vec = {0.0, 0.0, 0.0};
  for (unsigned int i=0; i<start.size(); i++) {
    lerp_vec.at(i) = start.at(i) + (end.at(i) - start.at(i)) * mu;
  }
  return lerp_vec;
}

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

