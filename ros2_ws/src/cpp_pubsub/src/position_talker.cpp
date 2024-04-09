#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "tutorial_interfaces/msg/falconpos.hpp"

#include <stdio.h>
#include "dhdc.h"


using namespace std::chrono_literals;


/////////////// DEFINITION OF PUBLISHER CLASS //////////////

class PositionTalker : public rclcpp::Node
{
public:

  // parameters name list
  std::vector<std::string> param_names = {"mapping_ratio", "use_depth", "part_id", "alpha_id", "traj_id"};
  double mapping_ratio {3.0};
  int use_depth {0};
  int part_id {0};
  int alpha_id {0};
  int traj_id {0};

  // other arrays
  double p[3] {0.0, 0.0, 0.0};
  double v[3] {0.0, 0.0, 0.0};
  double f[3] {0.0, 0.0, 0.0};
  double K[3] {200.0, 50.0, 50.0};     //////////////////// -> this is the initial gain vector K, will be changed after a few seconds!
  double C[3] {5.0, 5.0, 5.0};      //////////// -> damping vector C, having values higher than 5 will likely cause vibrations
  int choice;

  const int pub_freq = 500;    // publishing rate in [Hz]

  ///////// -> this is the centering / starting Falcon pos, but is NOT THE ORIGIN => ORIGIN IS ALWAYS (0, 0, 0)
  ///////// -> max bounds are around +-0.05m (5cm)
  ///////// -> this depends on the alpha_id parameter
  std::vector<double> centering {0.00, 0.00, 0.00};   ///////// -> note: this is in [meters]
  // guide:
  // {x, y, z} = {1, 2, 3} DOFS = {in/out, left/right, up/down}
  // positive axes directions are {out, right, up}
  

  // sine curve's first point is currently all the same
  // std::vector<double> first_point {0.06, -0.16, -0.01};


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  PositionTalker(int a_choice)
  : Node("position_talker")
  { 
    choice = a_choice;

    // parameter stuff
    this->declare_parameter(param_names.at(0), 3.0);
    this->declare_parameter(param_names.at(1), 0);
    this->declare_parameter(param_names.at(2), 0);
    this->declare_parameter(param_names.at(3), 0);
    this->declare_parameter(param_names.at(4), 0);
    
    std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);
    mapping_ratio = std::stod(params.at(0).value_to_string().c_str());
    use_depth = std::stoi(params.at(1).value_to_string().c_str());
    part_id = std::stoi(params.at(2).value_to_string().c_str());
    alpha_id = std::stoi(params.at(3).value_to_string().c_str());
    traj_id = std::stoi(params.at(4).value_to_string().c_str());
    print_params();

    // update first point if not using depth
    // if (use_depth == 0) first_point = {0.01, -0.16, -0.01};

    // update centering position using "post_point" computed above
    // for (size_t i=0; i<3; i++) centering.at(i) = first_point.at(i) / mapping_ratio;

    // publisher
    publisher_ = this->create_publisher<tutorial_interfaces::msg::Falconpos>("falcon_position", 10);
    timer_ = this->create_wall_timer(2ms, std::bind(&PositionTalker::timer_callback, this));       ///////// publishing at 500 Hz /////////
  }


private:

  void timer_callback()
  { 
    ///////////////////////// FALCON STUFF /////////////////////////
    dhdGetPosition(&(p[0]), &(p[1]), &(p[2]));
    dhdGetLinearVelocity (&(v[0]), &(v[1]), &(v[2]));

    if (count < count_thres2) {
      // gradually perform centering {in increasing levels of K = 1000 -> K = 2000, after 1 -> 2 seconds}
      for (int i=0; i<3; i++) f[i] = - K[i] * (p[i] - centering[i]) - C[i] * v[i];
    } else {
      switch (choice) {
        case 0: for (int i=0; i<3; i++) f[i] = 0; break;
        case 1: for (int i=0; i<1; i++) f[i] = - K[i] * (p[i] - centering[i]) - C[i] * v[i]; break;
        case 2: for (int i=0; i<2; i++) f[i] = - K[i] * (p[i] - centering[i]) - C[i] * v[i]; break;
        case 3: for (int i=0; i<3; i++) f[i] = - K[i] * (p[i] - centering[i]) - C[i] * v[i]; break;
      }
    }

    if (dhdSetForceAndTorqueAndGripperForce (f[0], f[1], f[2], 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
      printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
      printf ("\n\n=============================== THANK YOU FOR FLYING WITH FALCON ===============================\n\n");
      rclcpp::shutdown();
    }

    // generate and publish the message
    auto message = tutorial_interfaces::msg::Falconpos();
    message.x = p[0] * 100;
    message.y = p[1] * 100;
    message.z = p[2] * 100;
    // RCLCPP_INFO(this->get_logger(), "Publishing position: px = %.3f, py = %.3f, pz = %.3f  [in cm]", message.x, message.y, message.z);
    publisher_->publish(message);



    if (dhdKbHit() && dhdKbGet() == 'q') {
        printf ("\n\n=============================== THANK YOU FOR FLYING WITH FALCON ===============================\n\n");
        rclcpp::shutdown();
    }

    // // only run if the button is pressed (or) the reset condition is active
    // if (dhdGetButton (0) == DHD_ON || reset == true) {
    //   if (!reset) {
    //     reset = true;
    //     printf ("\n\n=============================== RE-CENTERING IN 1 SECOND ===============================\n\n");
    //   }
    //   reset_count++;
    //   if (reset_count == count_thres1) {
    //     // this is 1 second after the button has been pressed
    //     // need to reset {count, reset_count, reset, K, C}
    //     printf ("\n\n=============================== CENTERING NOW !!! ===============================\n\n");
    //     count = 0;
    //     reset_count = 0;
    //     reset = false;

    //     // restore the K and C gain vectors to initial values
    //     K[0] = 200.0; K[1] = 50.0; K[2] = 50.0;
    //     C[0] = 5.0;   C[1] = 5.0;  C[2] = 5.0;
    //   }
    // }

    count++;
    if (count > count_thres1) {
      K[0] = 500.0; K[1] = 250.0; K[2] = 250.0;
    }
    if (count > count_thres2) {
      K[0] = 2000.0; K[1] = 1500.0; K[2] = 1500.0;
    }

  }

  void print_params() {
    for (unsigned int i=0; i<10; i++) std::cout << "\n";
    std::cout << "\n\nThe current parameters [position_publisher] are as follows:\n" << std::endl;
    std::cout << "Mapping ratio = " << mapping_ratio << "\n" << std::endl;
    std::cout << "Use depth parameter = " << use_depth << "\n" << std::endl;
    std::cout << "Participant ID = " << part_id << "\n" << std::endl;
    std::cout << "Alpha ID = " << alpha_id << "\n" << std::endl;
    std::cout << "Trajectory ID = " << traj_id << "\n" << std::endl;
    for (unsigned int i=0; i<10; i++) std::cout << "\n";
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::Falconpos>::SharedPtr publisher_;

  const int count_thres1 = 1 * pub_freq;   // 1 second
  const int count_thres2 = 1.5 * pub_freq;   // 1.5 seconds
  const int count_thres3 = 2 * pub_freq;   // 2 seconds

  int count {0};
  int reset_count {0};

  bool reset = false;

};



//////////////////// MAIN FUNCTION ///////////////////

int main(int argc, char * argv[])
{   

  rclcpp::init(argc, argv);

  // message
  printf ("Force Dimension - Position Example (By Michael Pan) %s\n", dhdGetSDKVersionStr());
  printf ("Copyright (C) 2001-2022 Force Dimension\n");
  printf ("All Rights Reserved.\n\n");

  // open the first available device
  if (dhdOpen () < 0) {
    printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    dhdSleep (2.0);
    return -1;
  }

  // identify device
  printf ("%s device detected\n\n", dhdGetSystemName());

  // display instructions
  printf ("      'q' to perform landing :) \n\n");

  // enable force and button emulation, disable velocity threshold
  dhdEnableExpertMode ();
  dhdSetVelocityThreshold (0);
  dhdEnableForce (DHD_ON);
  dhdEmulateButton (DHD_ON);



  ///////////////// CHOOSE YOUR MODE! /////////////////

  // guide:
  // {x, y, z} = {1, 2, 3} DOFS = {in/out, left/right, up/down}
  // positive axes directions are {out, right, up}

  // int choice = 0;
  int choice = 1;      // in/out direction locked

  ///////////////// CHOOSE YOUR MODE! /////////////////



  std::shared_ptr<PositionTalker> michael = std::make_shared<PositionTalker>(choice);

  rclcpp::spin(michael);

//   rclcpp::shutdown();
  return 0;
}
