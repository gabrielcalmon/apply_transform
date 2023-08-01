#include "rclcpp/rclcpp.hpp"
#include "apply_transform/srv/get_transform.hpp"

#include <memory>
#include <cmath>

# define M_PI 3.14159265358979323846  // pi

void transform(const std::shared_ptr<apply_transform::srv::GetTransform::Request> request,
          std::shared_ptr<apply_transform::srv::GetTransform::Response>     response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nPoint A = [%f %f %f]"  " Theta: %f",
              request->point_a[0], request->point_a[1], request->point_a[2], request->theta);
  
  double theta_rad = (request->theta)*M_PI/180;

  double px = request->point_a[0];
  double py = request->point_a[1];
  double pz = request->point_a[2];
  
  if(request->theta < 0 || request->theta > 90){
    response->sucess = false; 
    response->message = "The transformation failed due to invalid angle input\nTheta angle must be between 0 and 90 degrees";
    return;
  }
  response->sucess = true;
  response->message = "The transformation was completed successfully";
  response->point_b[0] = px + 1;
  response->point_b[1] = cos(theta_rad)*py - sin(theta_rad)*pz + 2;
  response->point_b[2] = sin(theta_rad)*py + cos(theta_rad)*pz + 3;
   

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response");

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("apply_transform_server");

  rclcpp::Service<apply_transform::srv::GetTransform>::SharedPtr service =
    node->create_service<apply_transform::srv::GetTransform>("frame_a2b_transform", &transform);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to transform.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
