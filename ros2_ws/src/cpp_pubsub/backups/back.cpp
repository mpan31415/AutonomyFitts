#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <chrono>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


using namespace std::chrono_literals;


//////////////////////////////////////////////////////////////////////////////////////////////
class FramePublisher : public rclcpp::Node
{

public:

  std::string panda_link_name = "panda_link0";
  std::string camera_link_name = "camera_base";
  
  double dx = 0.7;
  double dy = 0.0;
  double dz = 0.0;

  double rx = 0.0;
  double ry = 0.0;
  double rz = 3.14;
  

  FramePublisher()
  : Node("camera_frame_broadcaster")
  {
    generate_transformation();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(100ms, std::bind(&FramePublisher::broadcast, this)); ///////// publishing at 10 Hz /////////
  }


private:

  void broadcast() {
    // get current time-stamp & broadcast the transformation
    trans_.header.stamp = this->get_clock()->now();
    
    std::cout << "Sending transformation now!" << std::endl;
    tf_broadcaster_->sendTransform(trans_);
  }


  void generate_transformation()
  {
    // initialize message
    trans_.header.stamp = this->get_clock()->now();
    trans_.header.frame_id = panda_link_name;
    trans_.child_frame_id = camera_link_name;

    // translations
    trans_.transform.translation.x = dx;
    trans_.transform.translation.y = dy;
    trans_.transform.translation.z = dz;

    // rotations
    tf2::Quaternion q;
    q.setRPY(rx, ry, rz);
    trans_.transform.rotation.x = q.x();
    trans_.transform.rotation.y = q.y();
    trans_.transform.rotation.z = q.z();
    trans_.transform.rotation.w = q.w();
  }

  geometry_msgs::msg::TransformStamped trans_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

};



//////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}