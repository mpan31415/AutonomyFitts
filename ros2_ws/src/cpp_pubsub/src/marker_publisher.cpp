#include <chrono>
#include <functional>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tutorial_interfaces/msg/falconpos.hpp"
#include "tutorial_interfaces/msg/pos_info.hpp"

using namespace std::chrono_literals;


/////////  functions to show markers in Rviz  /////////
void generate_fitts_ring(visualization_msgs::msg::Marker &spheres, 
                         double ring_center_x, double ring_center_y, double ring_center_z,
                         int num_targets, double ring_radius, double target_width, int target_id);

void generate_tcp_arrow(visualization_msgs::msg::Marker &arrow);
void generate_tcp_marker(visualization_msgs::msg::Marker &tcp_marker);



class MarkerPublisher : public rclcpp::Node
{
  public:

    // parameters name list
    std::vector<std::string> param_names = {"ring_id"};
    int ring_id {0};
    
    //////// KEEP CONSISTENT WITH REAL CONTROLLER ////////
    std::vector<double> origin {0.5059, 0.0, 0.4346};

    const int pub_freq = 50;   // [Hz]

    // Fitts ring parameters (fixed)
    std::vector<double> fitts_ring_origin {0.5059, 0.0, 0.4346};
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


    MarkerPublisher()
    : Node("marker_publisher")
    { 
      // parameter stuff
      this->declare_parameter(param_names.at(0), 1);
      
      std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);
      ring_id = std::stoi(params.at(0).value_to_string().c_str());
      print_params();

      // create the marker publisher
      marker_timer_ = this->create_wall_timer(20ms, std::bind(&MarkerPublisher::marker_callback, this));  // publish this at 50 Hz
      marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

      // create the curr_target_id subscriber
      curr_target_id_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "curr_target_id", 10, std::bind(&MarkerPublisher::curr_target_id_callback, this, std::placeholders::_1));
    }


  private:

    void marker_callback()
    { 
      auto marker_array_msg = visualization_msgs::msg::MarkerArray();

      // tcp position projection onto the Fitts plane
      // generate_tcp_marker(tcp_marker_);
      generate_tcp_arrow(tcp_marker_);
      marker_array_msg.markers.push_back(tcp_marker_);

      switch (ring_id) {
        case 1: r_radius = r_small; w_target = w_big;   break;
        case 2: r_radius = r_big;   w_target = w_big;   break;
        case 3: r_radius = r_small; w_target = w_small; break;
        case 4: r_radius = r_big;   w_target = w_small; break;
      }

      // Fitts ring
      generate_fitts_ring(fitts_marker_, fitts_ring_origin.at(0), fitts_ring_origin.at(1), fitts_ring_origin.at(2), 
                          n_targets, r_radius, w_target, curr_target_id);
      marker_array_msg.markers.push_back(fitts_marker_);
      
      // update markers in Rviz
      marker_pub_->publish(marker_array_msg);
    }

    /////////////// CALLBACK LISTENING TO THE CURRENT TARGET ID ///////////////
    void curr_target_id_callback(const std_msgs::msg::Int16 & msg)
    { 
      curr_target_id = msg.data;
      std::cout << "\n\nReceived a new incoming target ID = " << curr_target_id << " !!!\n\n" << std::endl;
    }

    /////////////// FUNCTION TO PRINT THE PARAMETERS ///////////////
    void print_params() {
      for (unsigned int i=0; i<10; i++) std::cout << "\n";
      std::cout << "\n\nThe current parameters [marker_publisher] are as follows:\n" << std::endl;
      std::cout << "Ring ID = " << ring_id << "\n" << std::endl;
      for (unsigned int i=0; i<10; i++) std::cout << "\n";
    }

    rclcpp::TimerBase::SharedPtr marker_timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr curr_target_id_sub_;

    visualization_msgs::msg::Marker fitts_marker_;
    visualization_msgs::msg::Marker tcp_marker_;
    
};


/////////////////////////////////// FUNCTIONS TO GENERATE FITTS RING ///////////////////////////////////
void generate_fitts_ring(visualization_msgs::msg::Marker &spheres, 
                         double ring_center_x, double ring_center_y, double ring_center_z,
                         int num_targets, double ring_radius, double target_width, int target_id)
{
  // make sure the marker is cleared
  spheres.points.clear();
  spheres.colors.clear();

  // fill-in the spheres marker message
  spheres.header.frame_id = "/panda_link0";
  spheres.header.stamp = rclcpp::Clock().now();
  spheres.ns = "marker_publisher";
  spheres.action = visualization_msgs::msg::Marker::ADD;
  spheres.id = 0;
  spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  spheres.scale.x = target_width / 2; // Sphere radius
  spheres.scale.y = target_width / 2; // Sphere radius
  spheres.scale.z = 0.2; // Not used for SPHERE_LIST

  // pose relative to the parent frame
  spheres.pose.position.x = ring_center_x;
  spheres.pose.position.y = ring_center_y;
  spheres.pose.position.z = ring_center_z;
  spheres.pose.orientation.x = 0.5;
  spheres.pose.orientation.y = 0.5;
  spheres.pose.orientation.z = 0.5;
  spheres.pose.orientation.w = 0.5;

  // Add different colors and positions for each sphere in the list
  for (int i=0; i < num_targets; i++) {

    // point position
    geometry_msgs::msg::Point p;
    double theta = ((double)(i)/(double)(num_targets))*2*M_PI;   // in radians
    p.x = ring_radius * sin(theta); // Position spheres along the x-axis
    p.y = ring_radius * cos(theta);
    p.z = 0.0;
    
    // assign the correct color
    // dark green - already done, red - current target, light green - not yet done
    std_msgs::msg::ColorRGBA color;
    if (i<target_id) {
      // light green - already done (half see-through)
      color.g = 1.0;
      color.a = 0.3;
    } else if (i==target_id) {
      // red - current target
      color.r = 1.0;
      color.a = 1.0;
    } else {
      // dark green - not yet done
      color.g = 1.0;
      color.a = 1.0;
    }

    // add sphere and color to list
    spheres.points.push_back(p);
    spheres.colors.push_back(color);
  }
}


/////////////////////////////////// FUNCTIONS TO GENERATE TCP ARROW ///////////////////////////////////
void generate_tcp_arrow(visualization_msgs::msg::Marker &arrow)
{
  arrow.header.frame_id = "/panda_hand_tcp";
  // arrow.header.frame_id = "panda_hand";
  arrow.header.stamp = rclcpp::Clock().now();
  arrow.ns = "marker_publisher";
  arrow.action = visualization_msgs::msg::Marker::ADD;
  arrow.id = 1;
  arrow.type = visualization_msgs::msg::Marker::ARROW;

  // Arrow position
  arrow.pose.position.x = 0.0;
  arrow.pose.position.y = 0.0;
  arrow.pose.position.z = 0.0;

  // Arrow orientation
  arrow.pose.orientation.x = 0.0;
  arrow.pose.orientation.y = 0.0;
  arrow.pose.orientation.z = 0.0;
  arrow.pose.orientation.w = 1.0;

  // Arrow dimensions
  arrow.scale.x = 0.04; // Shaft diameter
  arrow.scale.y = 0.06; // Head diameter
  arrow.scale.z = 0.04; // Head length

  // Color
  arrow.color.r = 0.0;
  arrow.color.g = 0.0;
  arrow.color.b = 1.0;
  arrow.color.a = 1.0;
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
  tcp_marker.scale.x = 0.015;
  tcp_marker.scale.y = 0.015;
  tcp_marker.scale.z = 0.015;

  // sphere is red
  tcp_marker.color.r = 1.0;
  tcp_marker.color.a = 1.0;
  
  // zero offset from the panda tcp link frame
  tcp_marker.pose.position.x = 0.0;
  tcp_marker.pose.position.y = 0.0;
  tcp_marker.pose.position.z = 0.0;
}


/////////////////////////// THE MAIN FUNCTION ///////////////////////////
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}