#include "rclcpp/rclcpp.hpp"
#include "apply_transform/srv/get_transform.hpp"

#include <memory>
#include <cmath>

# define M_PI 3.14159265358979323846  // pi

void transform(const std::shared_ptr<apply_transform::srv::GetTransform::Request> request,
          std::shared_ptr<apply_transform::srv::GetTransform::Response>     response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nz1: %f" " y: %f" " z2: %f",
              request->point_a[0], request->point_a[1], request->point_a[2]);
  
  // double z1 = (request->z1)*M_PI/180;	// recebe o valor em graus e converte em radianos
  // double y = (request->y)*M_PI/180;
  // double z2 = (request->z2)*M_PI/180;
  
  // sequencia de preenchimento: da primeira linha, para a segunda e, por fim, para a terceira
  response->point_b[0] = 1;
  response->point_b[1] = 0;
  response->point_b[2] = 0;
  response->sucess = true;  

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
