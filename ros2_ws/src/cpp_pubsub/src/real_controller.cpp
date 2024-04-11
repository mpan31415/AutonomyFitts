#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
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

const bool display_time = false;

KDL::Tree panda_tree;
KDL::Chain panda_chain;

KDL::Rotation orientation;
bool got_orientation = false;

std::vector<double> tcp_pos {0.5059, 0.0, 0.4346};   // initialized the same as the "home" position

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

void readCSV(const std::string& filename, std::vector<double>& dataArray);
double linearInterpolate(double y1, double y2, double mu);
double cosineInterpolate(double y1, double y2, double mu);
std::vector<double> linear_interpolate_vec(std::vector<double> old_vec, int num_interp);
std::vector<double> cosine_interpolate_vec(std::vector<double> old_vec, int num_interp);

bool within_limits(std::vector<double>& vals);
bool create_tree();
void get_chain();
double get_min(double a, double b);

void print_joint_vals(std::vector<double>& joint_vals);


/////////////// DEFINITION OF NODE CLASS //////////////

class RealController : public rclcpp::Node
{
public:

  // parameters name list
  std::vector<std::string> param_names = {"free_drive", "mapping_ratio", "use_depth", "part_id", "alpha_id", "traj_id"};
  int free_drive {0};
  double mapping_ratio {3.0};
  int use_depth {0};
  int part_id {0};
  int alpha_id {0};
  int traj_id {0};
  
  std::vector<double> origin {0.5059, 0.0, 0.4346}; //////// can change the task-space origin point! ////////

  std::vector<double> human_offset {0.0, 0.0, 0.0};
  std::vector<double> ref_offset {0.0, 0.0, 0.0};
  std::vector<double> robot_offset {0.0, 0.0, 0.0};

  std::vector<double> curr_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> ik_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> message_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool control = false;
  
  const int control_freq = 500;   // the rate at which the "controller_publisher" function is called in [Hz]
  const int tcp_pub_frequency = 40;   // in [Hz]

  // step 1: prep-time
  const int prep_time = 5;    // seconds
  const int max_prep_count = prep_time * control_freq;
  int prep_count = 0;

  std::vector<double> initial_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const int required_initial_vals = control_freq * 3;   // get initial joint values for 3 seconds
  int initial_joint_vals_count = 0;

  // step 2: smoothing -> used to initially smoothly incorporate the Falcon offset
  const int smoothing_time = 5;   /// smoothing time in [seconds]
  const int max_smoothing_count = control_freq * smoothing_time;
  const int float_time = 2;     // time to float at starting position [seconds]

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

  // trajectory recording
  const int traj_duration = 10;   // in [seconds]
  int max_recording_count = control_freq * traj_duration;
  bool record_flag = false;

  // for robot trajectory following
  double t_param = 0.0;

  // sine curve parameters (initialization)
  int pa = 0;
  int pb = 0;
  int pc = 0;
  double ps = 0.0;
  double ph = 0.0;

  double depth = 0.1;
  double width = 0.3;
  double height = 0.1;

  // for gradually shifting control to robot after 10 second trajectory
  const int shifting_time = 3;   // seconds
  int max_shifting_count = shifting_time * control_freq;

  // for moving to home after trajectory finishes
  std::vector<double> final_joint_vals {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // home joint values
  std::vector<double> home_joint_vals {0, -M_PI_4/2, 0, -5 * M_PI_4/2, 0, M_PI_2, M_PI_4};

  // for gradually homing the robot after the 3-second shifting
  const int homing_time = 2;    // seconds
  int max_homing_count = homing_time * control_freq;

  // wait 1 second before shutting down node
  const int shutdown_time = 1;    // second
  int max_shutdown_count = shutdown_time * control_freq;

  // empty noise file index string & noise vector
  std::string nid {};
  std::vector<double> robot_noise_vector;


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
    this->declare_parameter(param_names.at(5), 0);
    
    std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);
    free_drive = std::stoi(params.at(0).value_to_string().c_str());
    mapping_ratio = std::stod(params.at(1).value_to_string().c_str());
    use_depth = std::stod(params.at(2).value_to_string().c_str());
    part_id = std::stoi(params.at(3).value_to_string().c_str());
    alpha_id = std::stoi(params.at(4).value_to_string().c_str());
    traj_id = std::stoi(params.at(5).value_to_string().c_str());

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

    // write the sine curve parameters
    switch (traj_id) {
      case 0: pa = 1; pb = 1; pc = 4; ps = M_PI;     ph = 0.25; nid = "8"; break;
      case 1: pa = 2; pb = 3; pc = 4; ps = 4*M_PI/3; ph = 0.25; nid = "9"; break;
      case 2: pa = 1; pb = 3; pc = 4; ps = M_PI;     ph = 0.25; nid = "7"; break;
      case 3: pa = 2; pb = 2; pc = 5; ps = M_PI;     ph = 0.2;  nid = "5"; break;
      case 4: pa = 2; pb = 3; pc = 5; ps = 8*M_PI/5; ph = 0.2;  nid = "2"; break;
      case 5: pa = 2; pb = 4; pc = 5; ps = M_PI;     ph = 0.2;  nid = "4"; break;
    }

    // joint controller publisher & timer
    controller_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("desired_joint_vals", 10);
    controller_timer_ = this->create_wall_timer(2ms, std::bind(&RealController::controller_publisher, this));    // controls at 500 Hz

    // tcp position publisher & timer
    tcp_pos_pub_ = this->create_publisher<tutorial_interfaces::msg::PosInfo>("tcp_position", 10);
    // tcp_pos_timer_ = this->create_wall_timer(25ms, std::bind(&RealController::tcp_pos_publisher, this));    // publishes at 40 Hz

    // recording flag publisher & timer
    record_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("record", 10);
    record_flag_timer_ = this->create_wall_timer(2ms, std::bind(&RealController::record_flag_publisher, this));    // publishes at 500 Hz

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

    //Create Panda tree and get its kinematic chain
    if (!create_tree()) rclcpp::shutdown();
    get_chain();

    // read the noise data csv file
    std::string noise_file {"noise" + nid + ".csv"};
    generate_noise_vector(noise_file);
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

      // get the robot control offset in Cartesian space (calling the corresponding function of the traj_id)
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

  ///////////////////////////////////// TCP POSITION PUBLISHER /////////////////////////////////////
  void tcp_pos_publisher()
  { 
    if (count > max_smoothing_count + max_recording_count - tcp_pub_frequency) {
      auto lp = std_msgs::msg::Bool();
      lp.data = true;
      std::cout << "\n\n\n\n\n\n======================= SETTING LAST POINT TO => TRUE =======================\n\n\n\n\n\n" << std::endl;
      last_point_pub_->publish(lp);
    }

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

  ///////////////////////////////////// TRAJ RECORD FLAG PUBLISHER /////////////////////////////////////
  void record_flag_publisher()
  { 
    auto message = std_msgs::msg::Bool();
    message.data = record_flag;
    record_flag_pub_->publish(message);
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

  ///////////////////////////////////// FUNCTION TO READ NOISE CSV AND INTERPOLATE /////////////////////////////////////
  void generate_noise_vector(const std::string filename) {

    std::cout << "Noise filename = " << filename << std::endl;

    std::string csv_file_name {"/home/michael/HRI/ros2_ws/src/cpp_pubsub/robot_noise/noise_csv_files/"+filename};

    std::vector<double> raw_data;
    const int num_interp = 49;

    // read csv file
    readCSV(csv_file_name, raw_data);
    std::cout << "Length of raw noise array = " << raw_data.size() << std::endl;

    // Display the data stored in the array
    // std::cout << "Data in the array: ";
    // for (const auto& value : raw_data) {
    //     std::cout << value << " ";
    // }
    // std::cout << std::endl;

    // interpolate (linear / cosine)
    robot_noise_vector = linear_interpolate_vec(raw_data, num_interp);
    std::cout << "Success! Length of new noise vector = " << robot_noise_vector.size() << std::endl;
  }

  ///////////////////////////////////// FUNCTION TO PRINT PARAMETERS /////////////////////////////////////
  void print_params() {
    for (unsigned int i=0; i<10; i++) std::cout << "\n";
    std::cout << "\n\nThe current parameters [real_controller] are as follows:\n" << std::endl;
    std::cout << "Free drive mode = " << free_drive << "\n" << std::endl;
    std::cout << "Mapping ratio = " << mapping_ratio << "\n" << std::endl;
    std::cout << "Use depth parameter = " << use_depth << "\n" << std::endl;
    std::cout << "Participant ID = " << part_id << "\n" << std::endl;
    std::cout << "Alpha ID = " << alpha_id << "\n" << std::endl;
    std::cout << "Trajectory ID = " << traj_id << "\n" << std::endl;
    for (unsigned int i=0; i<10; i++) std::cout << "\n";
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr controller_pub_;
  rclcpp::TimerBase::SharedPtr controller_timer_;

  rclcpp::Publisher<tutorial_interfaces::msg::PosInfo>::SharedPtr tcp_pos_pub_;
  // rclcpp::TimerBase::SharedPtr tcp_pos_timer_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr record_flag_pub_;
  rclcpp::TimerBase::SharedPtr record_flag_timer_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr last_point_pub_;

  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr count_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr countdown_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_vals_sub_;

  rclcpp::Subscription<tutorial_interfaces::msg::Falconpos>::SharedPtr falcon_pos_sub_;
  
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

  if (display_time) {
    auto finish = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
    std::cout << "Execution of my IK solver function took " << duration.count() << " [microseconds]" << std::endl;
  }
  
}


///////////////// Noise helper functions /////////////////

// Function to read CSV file and store data in a C++ array
void readCSV(const std::string& filename, std::vector<double>& dataArray) {
    std::ifstream file(filename);

    if (file.is_open()) {
        std::string line;
        getline(file, line); // Read the entire line from the CSV

        std::stringstream ss(line);
        std::string value;

        while (getline(ss, value, ',')) {
            // Assuming the CSV contains integers; you can modify this part based on your data type
            double dataValue = std::stod(value);
            dataArray.push_back(dataValue);
        }

        file.close();
    } else {
        std::cerr << "Unable to open the file: " << filename << std::endl;
    }
}

// Function to linearly interpolate between two numbers for a given mu
double linearInterpolate(double y1, double y2, double mu) { return (y2 - y1) * mu + y1; }

// Function to cosine interpolate between two numbers for a given mu
double cosineInterpolate(double y1, double y2, double mu) {
    double angle = mu * M_PI;
    double mu2 = (1.0 - std::cos(angle)) * 0.5;
    double res = linearInterpolate(y1, y2, mu2);
    return res;
}

// Function to linearly interpolate a vector and return a new vector
std::vector<double> linear_interpolate_vec(std::vector<double> old_vec, int num_interp) {
    int num_old_points = old_vec.size();
    std::vector<double> new_vec = {old_vec.at(0)};
    for (int i=0; i<num_old_points-1; i++) {
        for (int j=1; j<num_interp+2; j++) {
            double mu = (double)j / (double)(num_interp+1);
            double lin_x = linearInterpolate(old_vec.at(i), old_vec.at(i+1), mu);
            new_vec.push_back(lin_x);
        }
    }
    return new_vec;
}

// Function to cosine interpolate a vector and return a new vector
std::vector<double> cosine_interpolate_vec(std::vector<double> old_vec, int num_interp) {
    int num_old_points = old_vec.size();
    std::vector<double> new_vec = {old_vec.at(0)};
    for (int i=0; i<num_old_points-1; i++) {
        for (int j=1; j<num_interp+2; j++) {
            double mu = (double)j / (double)(num_interp+1);
            double lin_x = cosineInterpolate(old_vec.at(i), old_vec.at(i+1), mu);
            new_vec.push_back(lin_x);
        }
    }
    return new_vec;
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

double get_min(double a, double b) {
  if (a<b) return a;
  return b;
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

