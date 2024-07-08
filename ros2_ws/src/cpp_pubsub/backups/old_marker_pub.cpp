#include <chrono>
#include <functional>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tutorial_interfaces/msg/falconpos.hpp"

using namespace std::chrono_literals;


/////////  functions to show markers in Rviz  /////////
void generate_tcp_marker(visualization_msgs::msg::Marker &traj_marker);
void generate_marker_training(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points);
void generate_marker1(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points);
void generate_marker2(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points);
void generate_marker3(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points);

void get_rotation_matrix(int axis, double angle, std::vector<std::vector<double>> &T);       // axes are: {1-x, 2-y, 3-z}
void matrix_mult_vector(std::vector<std::vector<double>> &mat, std::vector<double> &vec, std::vector<double> &result);


class MarkerPublisher : public rclcpp::Node
{
  public:

    // parameters name list
    std::vector<std::string> param_names = {"part_id", "traj_id", "auto_id"};
    int part_id {0};
    int traj_id {0};
    int auto_id {0};
    
     //////// KEEP CONSISTENT WITH REAL CONTROLLER ////////
    std::vector<double> origin {0.4559, 0.0, 0.3846};
    const int max_points = 200;


    MarkerPublisher()
    : Node("marker_publisher")
    { 
      // parameter stuff
      this->declare_parameter(param_names.at(0), 0);
      this->declare_parameter(param_names.at(1), 0);
      this->declare_parameter(param_names.at(2), 0);
      // this->declare_parameter(param_names.at(0), rclcpp::PARAMETER_INTEGER);
      
      std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);
      part_id = std::stoi(params.at(0).value_to_string().c_str());
      traj_id = std::stoi(params.at(1).value_to_string().c_str());
      auto_id = std::stoi(params.at(2).value_to_string().c_str());
      print_params();

      switch (traj_id) {
        case 0: generate_marker_training(traj_marker_, origin, max_points); break;
        case 1: generate_marker1(traj_marker_, origin, max_points); break;
        case 2: generate_marker2(traj_marker_, origin, max_points); break;
        case 3: generate_marker3(traj_marker_, origin, max_points); break;
      }

      // create the marker publisher
      marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
      marker_timer_ = this->create_wall_timer(50ms, std::bind(&MarkerPublisher::marker_callback, this));  // publish this at 20 Hz
    }


  private:

    void marker_callback()
    { 
      marker_pub_->publish(traj_marker_);    // this is a continuous line, good!

      generate_tcp_marker(tcp_marker_);
      marker_pub_->publish(tcp_marker_);    // this is a sphere (point), good!
    }

    void print_params() {
      for (unsigned int i=0; i<10; i++) std::cout << "\n";
      std::cout << "\n\nThe current parameters [marker_publisher] are as follows:\n" << std::endl;
      std::cout << "Participant ID = " << part_id << "\n" << std::endl;
      std::cout << "Trajectory ID = " << traj_id << "\n" << std::endl;
      std::cout << "Autonomy ID = " << auto_id << "\n" << std::endl;
      for (unsigned int i=0; i<10; i++) std::cout << "\n";
    }

    rclcpp::TimerBase::SharedPtr marker_timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<tutorial_interfaces::msg::Falconpos>::SharedPtr tcp_pos_sub_;
    visualization_msgs::msg::Marker traj_marker_;
    visualization_msgs::msg::Marker tcp_marker_;
};


/////////////////////////////////// FUNCTIONS TO GENERATE REFERENCE TRAJECTORY MARKERS ///////////////////////////////////
void generate_tcp_marker(visualization_msgs::msg::Marker &traj_marker) 
{
  // fill-in the traj_marker message
  traj_marker.header.frame_id = "/panda_hand_tcp";
  traj_marker.header.stamp = rclcpp::Clock().now();
  traj_marker.ns = "marker_publisher";
  traj_marker.action = visualization_msgs::msg::Marker::ADD;
  traj_marker.id = 20;
  traj_marker.type = visualization_msgs::msg::Marker::SPHERE;

  // diameters of the sphere in x, y, z directions [cm]
  traj_marker.scale.x = 0.03;
  traj_marker.scale.y = 0.03;
  traj_marker.scale.z = 0.03;

  // sphere is red
  traj_marker.color.r = 245.0;
  traj_marker.color.a = 1.0;
  
  // zero offset from the panda tcp link frame
  traj_marker.pose.position.x = 0.0;
  traj_marker.pose.position.y = 0.0;
  traj_marker.pose.position.z = 0.0;
}

/////////////////////////////////// FUNCTIONS TO GENERATE REFERENCE TRAJECTORY MARKERS ///////////////////////////////////
void generate_marker_training(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points) 
{
  double r = 0.1;
  double h = 0.2;

  // fill-in the traj_marker message
  traj_marker.header.frame_id = "/panda_link0";
  traj_marker.header.stamp = rclcpp::Clock().now();
  traj_marker.ns = "marker_publisher";
  traj_marker.action = visualization_msgs::msg::Marker::ADD;
  traj_marker.id = 0;
  traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  traj_marker.scale.x = 0.02;  // make this 2 cm

  // Line strip is blue
  traj_marker.color.b = 1.0;
  traj_marker.color.a = 1.0;

  // Create the vertices for the points and lines
  for (int count=0; count<=max_points; count++) {

    double t = (double) count / max_points * 2 * M_PI;   // for circle, parametrized in the range [0, 2pi]

    double x = r * sin(t*2) + origin.at(0);
    double y = r * cos(t*2) + origin.at(1);
    double z = -h/2 + t/(2*M_PI) * h + origin.at(2);

    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;

    traj_marker.points.push_back(p);
  }
}

void generate_marker1(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points) 
{
  double r = 0.1;

  // fill-in the traj_marker message
  traj_marker.header.frame_id = "/panda_link0";
  traj_marker.header.stamp = rclcpp::Clock().now();
  traj_marker.ns = "marker_publisher";
  traj_marker.action = visualization_msgs::msg::Marker::ADD;
  traj_marker.id = 0;
  traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  traj_marker.scale.x = 0.02;  // make this 2 cm

  // Line strip is blue
  traj_marker.color.b = 1.0;
  traj_marker.color.a = 1.0;

  // Create the vertices for the points and lines
  for (int count=0; count<=max_points; count++) {

    double t = (double) count / max_points * 2 * M_PI;   // for circle, parametrized in the range [0, 2pi]

    double x = 0.0 + origin.at(0);
    double y = r * sin(t) + origin.at(1);
    double z = r * cos(t) + origin.at(2);

    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;

    traj_marker.points.push_back(p);
  }
}

void generate_marker2(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points)
{
  double r = 0.1;

  // fill-in the traj_marker message
  traj_marker.header.frame_id = "/panda_link0";
  traj_marker.header.stamp = rclcpp::Clock().now();
  traj_marker.ns = "marker_publisher";
  traj_marker.action = visualization_msgs::msg::Marker::ADD;
  traj_marker.id = 0;
  traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  traj_marker.scale.x = 0.02;  // make this 2 cm

  // Line strip is blue
  traj_marker.color.b = 1.0;
  traj_marker.color.a = 1.0;

  // Create the vertices for the points and lines
  for (int count=0; count<=max_points; count++) {

    double t = (double) count / max_points * 2 * M_PI;   // for circle, parametrized in the range [0, 2pi]

    double x = -r * cos(t) + origin.at(0);
    double y = r * sin(t) + origin.at(1);
    double z = 0.0 + origin.at(2);

    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;

    traj_marker.points.push_back(p);
  }
}

void generate_marker3(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points)
{
  double r = 0.1;

  // fill-in the traj_marker message
  traj_marker.header.frame_id = "/panda_link0";
  traj_marker.header.stamp = rclcpp::Clock().now();
  traj_marker.ns = "marker_publisher";
  traj_marker.action = visualization_msgs::msg::Marker::ADD;
  traj_marker.id = 0;
  traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  traj_marker.scale.x = 0.02;  // make this 2 cm

  // Line strip is blue
  traj_marker.color.b = 1.0;
  traj_marker.color.a = 1.0;

  // get rotation matrix
  std::vector<std::vector<double>> T {{0,0,0},{0,0,0},{0,0,0}};
  get_rotation_matrix(2, 45, T);

  std::vector<double> pre_point {0,0,0};
  std::vector<double> post_point {0,0,0};

  // Create the vertices for the points and lines
  for (int count=0; count<=max_points; count++) {

    double t = (double) count / max_points * 2 * M_PI;   // for circle, parametrized in the range [0, 2pi]

    double x = -r * cos(t);
    double y = r * sin(t);
    double z = 0.0;

    pre_point = {x, y, z};
    matrix_mult_vector(T, pre_point, post_point);

    geometry_msgs::msg::Point p;
    p.x = post_point.at(0) + origin.at(0);
    p.y = post_point.at(1) + origin.at(1);
    p.z = post_point.at(2) + origin.at(2);

    traj_marker.points.push_back(p);
  }
}





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




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}