#include <chrono>
#include <functional>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tutorial_interfaces/msg/falconpos.hpp"
#include "tutorial_interfaces/msg/pos_info.hpp"

using namespace std::chrono_literals;


/////////  functions to show markers in Rviz  /////////
void generate_ref_ball(visualization_msgs::msg::Marker &ref_marker, double x, double y, double z, visualization_msgs::msg::Marker &traj_marker);
void generate_tcp_marker(visualization_msgs::msg::Marker &tcp_marker);
visualization_msgs::msg::Marker generate_countdown(int count, std::vector<double> &center);
visualization_msgs::msg::Marker generate_progress(double percentage, double line_width, std::vector<double> center, double height, double width, int id);

void generate_traj_marker(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points,
                          double pa, double pb, double pc, double ps, double ph, double height, double width, double depth);

void generate_progress_bar(visualization_msgs::msg::Marker &progress_bar, std::vector<double> center, double height, double width);

visualization_msgs::msg::Marker generate_cleaner_marker();


class MarkerPublisher : public rclcpp::Node
{
  public:

    // parameters name list
    std::vector<std::string> param_names = {"part_id", "alpha_id", "traj_id"};
    int part_id {0};
    int alpha_id {0};
    int traj_id {0};
    
     //////// KEEP CONSISTENT WITH REAL CONTROLLER ////////
    std::vector<double> origin {0.5059, 0.0, 0.4346};
    const int max_points = 200;

    // for the reference tcp marker
    std::vector<double> ref_pos {0.0, 0.0, 0.0};

    // for the progress bar
    // std::vector<double> bar_center {-0.2, 0.0, 1.0};
    std::vector<double> bar_center {0.3, 0.0, 0.05};
    double bar_width {0.5};
    double bar_height {0.1};

    const int pub_freq = 50;   // [Hz]
    const int control_freq = 500;    // [Hz]
    const int max_smoothing_time = 5;   // [seconds]
    const double max_smoothing_count = control_freq * max_smoothing_time;

    const int max_recording_time = 10;    // [seconds]

    double line_width = bar_width / (pub_freq * max_smoothing_time);

    int my_marker_id {5};

    // int controller_count {0};

    int controller_seconds {0};
    int countdown_count {5};
    
    std::vector<visualization_msgs::msg::Marker> progress_bar_lines;

    // sine curve parameters (initialization)
    int pa = 0;
    int pb = 0;
    int pc = 0;
    double ps = 0.0;
    double ph = 0.0;
    
    double traj_height = 0.1;
    double traj_width = 0.3;
    double traj_depth = 0.1;
  

    MarkerPublisher()
    : Node("marker_publisher")
    { 
      // parameter stuff
      this->declare_parameter(param_names.at(0), 0);
      this->declare_parameter(param_names.at(1), 0);
      this->declare_parameter(param_names.at(2), 0);
      
      std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);
      part_id = std::stoi(params.at(0).value_to_string().c_str());
      alpha_id = std::stoi(params.at(1).value_to_string().c_str());
      traj_id = std::stoi(params.at(2).value_to_string().c_str());
      print_params();

      // write the sine curve parameters
      switch (traj_id) {
        case 0: pa = 1; pb = 1; pc = 4; ps = M_PI;     ph = 0.4; break;
        case 1: pa = 2; pb = 3; pc = 4; ps = 4*M_PI/3; ph = 0.3; break;
        case 2: pa = 1; pb = 3; pc = 4; ps = M_PI;     ph = 0.3; break;
        case 3: pa = 2; pb = 2; pc = 5; ps = M_PI;     ph = 0.2; break;
        case 4: pa = 2; pb = 3; pc = 5; ps = 8*M_PI/5; ph = 0.2; break;
        case 5: pa = 2; pb = 4; pc = 5; ps = M_PI;     ph = 0.2; break;
      }

      // generate the trajectory marker
      generate_traj_marker(traj_marker_, origin, max_points, pa, pb, pc, ps, ph, traj_height, traj_width, traj_depth);

      // generate the empty progress bar
      generate_progress_bar(progress_bar_, bar_center, bar_height, bar_width);

      // create the marker publisher
      marker_timer_ = this->create_wall_timer(20ms, std::bind(&MarkerPublisher::marker_callback, this));  // publish this at 50 Hz
      marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

      // reference tcp position subscriber
      ref_sub_ = this->create_subscription<tutorial_interfaces::msg::PosInfo>(
      "tcp_position", 10, std::bind(&MarkerPublisher::ref_callback, this, std::placeholders::_1));

      // controller count subscriber
      // count_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      // "controller_count", 10, std::bind(&MarkerPublisher::count_callback, this, std::placeholders::_1));

      count_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "countdown", 10, std::bind(&MarkerPublisher::count_callback, this, std::placeholders::_1));

      // marker cleaner & wipe all existing markers
      // auto cleaner_marker_arr = visualization_msgs::msg::MarkerArray();
      // auto cleaner_marker = generate_cleaner_marker();
      // cleaner_marker_arr.markers.push_back(cleaner_marker);
      // marker_pub_->publish(cleaner_marker_arr);
    }


  private:

    void marker_callback()
    { 
      auto marker_array_msg = visualization_msgs::msg::MarkerArray();

      // add in the certain ones
      // marker_array_msg.markers.push_back(progress_bar_);
      marker_array_msg.markers.push_back(traj_marker_);

      generate_tcp_marker(tcp_marker_);
      marker_array_msg.markers.push_back(tcp_marker_);
      
      generate_ref_ball(ref_marker_, ref_pos.at(0), ref_pos.at(1), ref_pos.at(2), traj_marker_);
      marker_array_msg.markers.push_back(ref_marker_);

      // // add progress bar if needed
      // if (controller_count > 0.0 && controller_count < max_smoothing_count) {

      //   double percentage = controller_count / max_smoothing_count;
      //   auto progress_marker = generate_progress(percentage, line_width, bar_center, bar_height, bar_width, my_marker_id);
      //   my_marker_id++;

      //   progress_bar_lines.push_back(progress_marker);
      //   for (size_t i=0; i<progress_bar_lines.size(); i++) marker_array_msg.markers.push_back(progress_bar_lines.at(i));
      // }

      // display countdown numbers when during smoothing
      if (countdown_count >= 0 || countdown_count == -10) {
        auto countdown_text = generate_countdown(countdown_count, bar_center);
        marker_array_msg.markers.push_back(countdown_text);
      }
      
      marker_pub_->publish(marker_array_msg);

    }

    void ref_callback(const tutorial_interfaces::msg::PosInfo & msg) 
    { 
      ref_pos.at(0) = msg.robot_position[0];
      ref_pos.at(1) = msg.robot_position[1];
      ref_pos.at(2) = msg.robot_position[2];
    }

    void count_callback(const std_msgs::msg::Float64 & msg) {
      controller_seconds = (int) msg.data;
      if (controller_seconds != 0) {
        countdown_count = max_smoothing_time - controller_seconds;
      }
    }

    void print_params() {
      for (unsigned int i=0; i<10; i++) std::cout << "\n";
      std::cout << "\n\nThe current parameters [marker_publisher] are as follows:\n" << std::endl;
      std::cout << "Participant ID = " << part_id << "\n" << std::endl;
      std::cout << "Alpha ID = " << alpha_id << "\n" << std::endl;
      std::cout << "Trajectory ID = " << traj_id << "\n" << std::endl;
      for (unsigned int i=0; i<10; i++) std::cout << "\n";
    }

    rclcpp::TimerBase::SharedPtr marker_timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Subscription<tutorial_interfaces::msg::PosInfo>::SharedPtr ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr count_sub_;

    visualization_msgs::msg::Marker traj_marker_;
    visualization_msgs::msg::Marker ref_marker_;
    visualization_msgs::msg::Marker tcp_marker_;
    visualization_msgs::msg::Marker progress_bar_;
    
};


/////////////////////////////////// FUNCTIONS TO GENERATE REFERENCE BALL ///////////////////////////////////
void generate_ref_ball(visualization_msgs::msg::Marker &ref_marker, double x, double y, double z, visualization_msgs::msg::Marker &traj_marker) 
{
  // fill-in the tcp_marker message
  ref_marker.header.frame_id = "/panda_link0";
  ref_marker.header.stamp = rclcpp::Clock().now();
  ref_marker.ns = "marker_publisher";
  ref_marker.action = visualization_msgs::msg::Marker::ADD;
  ref_marker.id = 0;
  ref_marker.type = visualization_msgs::msg::Marker::SPHERE;

  // diameters of the sphere in x, y, z directions [cm]
  ref_marker.scale.x = 0.02;
  ref_marker.scale.y = 0.02;
  ref_marker.scale.z = 0.02;

  // sphere is green
  ref_marker.color.g = 1.0;
  ref_marker.color.a = 1.0;
  
  if (x == 0.0) {
    // use first point of traj_marker_
    ref_marker.pose.position.x = traj_marker.points.at(0).x;
    ref_marker.pose.position.y = traj_marker.points.at(0).y;
    ref_marker.pose.position.z = traj_marker.points.at(0).z;
  } else {
    // use the reading from "robot position"
    ref_marker.pose.position.x = x;
    ref_marker.pose.position.y = y;
    ref_marker.pose.position.z = z;
  }
}


/////////////////////////////////// FUNCTIONS TO GENERATE TCP MARKER ///////////////////////////////////
void generate_tcp_marker(visualization_msgs::msg::Marker &tcp_marker) 
{
  // fill-in the tcp_marker message
  tcp_marker.header.frame_id = "/panda_hand_tcp";
  tcp_marker.header.stamp = rclcpp::Clock().now();
  tcp_marker.ns = "marker_publisher";
  tcp_marker.action = visualization_msgs::msg::Marker::ADD;
  tcp_marker.id = 1;
  tcp_marker.type = visualization_msgs::msg::Marker::SPHERE;

  // diameters of the sphere in x, y, z directions [cm]
  tcp_marker.scale.x = 0.01;
  tcp_marker.scale.y = 0.01;
  tcp_marker.scale.z = 0.01;

  // sphere is red
  tcp_marker.color.r = 1.0;
  tcp_marker.color.a = 1.0;
  
  // zero offset from the panda tcp link frame
  tcp_marker.pose.position.x = 0.0;
  tcp_marker.pose.position.y = 0.0;
  tcp_marker.pose.position.z = 0.0;
}


/////////////////////////////////// FUNCTIONS TO GENERATE COUNTDOWN TEXT ///////////////////////////////////
visualization_msgs::msg::Marker generate_countdown(int count, std::vector<double> &center)
{ 
  auto text = visualization_msgs::msg::Marker();

  // fill-in the text message
  text.header.frame_id = "/panda_link0";
  text.header.stamp = rclcpp::Clock().now();
  text.ns = "marker_publisher";
  text.action = visualization_msgs::msg::Marker::ADD;
  text.id = 10;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

  // height of 'A' is 10 cm
  text.scale.z = 0.2;

  // set text color and opacity
  switch (count) {
    case 5: text.color.r = 1.0; break;
    case 4: text.color.r = 1.0; break;
    case 3: text.color.r = 1.0; break;
    case 2: text.color.r = 1.0; text.color.g = 1.0; break;
    case 1: text.color.r = 1.0; text.color.g = 1.0; break;
    case 0: text.color.g = 1.0; break;
  }
  text.color.a = 1.0;

  // what to display
  text.text = std::to_string(count);

  // let's go!
  if (count == 0) {
    text.text = "Go!";
  }

  // stop right there!
  if (count == -10) {
    text.text = "Stop!";
    text.color.r = 1.0;
  }

  // constant offset from panda base link
  text.pose.position.x = center.at(0);
  text.pose.position.y = center.at(1);
  text.pose.position.z = center.at(2) + 0.05;

  return text;
}


/////////////////////////////////// FUNCTIONS TO GENERATE PROGRESS ///////////////////////////////////
visualization_msgs::msg::Marker generate_progress(double percentage, double line_width, std::vector<double> center, double height, double width, int id)
{ 
  auto progress = visualization_msgs::msg::Marker();

  // fill-in the progress message
  progress.header.frame_id = "/panda_link0";
  progress.header.stamp = rclcpp::Clock().now();
  progress.ns = "marker_publisher";
  progress.action = visualization_msgs::msg::Marker::ADD;
  progress.id = id;
  progress.type = visualization_msgs::msg::Marker::LINE_LIST;

  // width of line strip
  progress.scale.x = line_width;

  // line strip is green
  progress.color.g = 1.0;
  progress.color.a = 1.0;
  
  // Create the vertices for the points and lines
  geometry_msgs::msg::Point top;
  geometry_msgs::msg::Point bottom;

  top.x = center.at(0) - height/2; 
  top.y = center.at(1) - (width/2) + percentage * width;
  top.z = center.at(2);

  bottom.x = center.at(0) + height/2;
  bottom.y = center.at(1) - (width/2) + percentage * width;
  bottom.z = center.at(2);

  progress.points.push_back(top);
  progress.points.push_back(bottom);

  return progress;
}


/////////////////////////////////// FUNCTIONS TO GENERATE REFERENCE TRAJECTORY MARKERS ///////////////////////////////////
void generate_traj_marker(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points,
                          double pa, double pb, double pc, double ps, double ph, double height, double width, double depth)
{
  // fill-in the traj_marker message
  traj_marker.header.frame_id = "/panda_link0";
  traj_marker.header.stamp = rclcpp::Clock().now();
  traj_marker.ns = "marker_publisher";
  traj_marker.action = visualization_msgs::msg::Marker::ADD;
  traj_marker.id = 2;
  traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  traj_marker.scale.x = 0.015;  // make this 1.5 cm

  // Line strip is blue
  traj_marker.color.b = 1.0;
  traj_marker.color.a = 1.0;

  // Create the vertices for the points and lines
  for (int count=0; count<=max_points; count++) {

    double t = (double) count / max_points * 2 * M_PI;   // parametrized in the range [0, 2pi]

    double x = abs(t-M_PI) / M_PI * depth - (depth/2);
    double y = t / (2*M_PI) * width - (width/2);
    double z = (ph*height) * (sin(pa*(t+ps)) + sin(pb*(t+ps)) + sin(pc*(t+ps)));

    geometry_msgs::msg::Point p;
    p.x = x + origin.at(0);
    p.y = y + origin.at(1);
    p.z = z + origin.at(2);

    traj_marker.points.push_back(p);
  }
}


/////////////////////////////////// FUNCTIONS TO GENERATE TCP MARKER ///////////////////////////////////
void generate_progress_bar(visualization_msgs::msg::Marker &progress_bar, std::vector<double> center, double height, double width) 
{
  // fill-in the progress_bar message
  progress_bar.header.frame_id = "/panda_link0";
  progress_bar.header.stamp = rclcpp::Clock().now();
  progress_bar.ns = "marker_publisher";
  progress_bar.action = visualization_msgs::msg::Marker::ADD;
  progress_bar.id = 3;
  progress_bar.type = visualization_msgs::msg::Marker::LINE_LIST;

  // width of line 
  progress_bar.scale.x = 0.02;

  // Points are green
  progress_bar.color.g = 1.0;
  progress_bar.color.a = 1.0;

  // define the positions of the four corners of the progress bar
  geometry_msgs::msg::Point tl;
  geometry_msgs::msg::Point tr;
  geometry_msgs::msg::Point br;
  geometry_msgs::msg::Point bl;

  tl.x = center.at(0) - height/2; tl.y = center.at(1) - width/2; tl.z = center.at(2);
  tr.x = center.at(0) - height/2; tr.y = center.at(1) + width/2; tr.z = center.at(2);
  br.x = center.at(0) + height/2; br.y = center.at(1) + width/2; br.z = center.at(2);
  bl.x = center.at(0) + height/2; bl.y = center.at(1) - width/2; bl.z = center.at(2);

  // add them sequentially, forming line segments between each consecutive pair
  progress_bar.points.push_back(tl);
  progress_bar.points.push_back(tr);
  progress_bar.points.push_back(tr);
  progress_bar.points.push_back(br);
  progress_bar.points.push_back(br);
  progress_bar.points.push_back(bl);
  progress_bar.points.push_back(bl);
  progress_bar.points.push_back(tl);
}


/////////////////////////////////// FUNCTIONS TO CLEAR ALL MARKERS ///////////////////////////////////
visualization_msgs::msg::Marker generate_cleaner_marker() 
{ 
  auto cleaner_marker = visualization_msgs::msg::Marker();

  // fill-in the cleaner_marker message
  cleaner_marker.header.frame_id = "/panda_link0";
  cleaner_marker.header.stamp = rclcpp::Clock().now();
  cleaner_marker.ns = "marker_publisher";
  cleaner_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  cleaner_marker.id = 4;

  return cleaner_marker;
}



/////////////////////////// THE MAIN FUNCTION ///////////////////////////
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}