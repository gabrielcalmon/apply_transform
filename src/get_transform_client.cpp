#include "rclcpp/rclcpp.hpp"
#include "apply_transform/srv/get_transform.hpp"                               

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 5) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: get transform Px Py Pz theta");      
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("apply_transform_client");  
  rclcpp::Client<apply_transform::srv::GetTransform>::SharedPtr client =                
    node->create_client<apply_transform::srv::GetTransform>("frame_a2b_transform");          
  auto request = std::make_shared<apply_transform::srv::GetTransform::Request>();       
  request->point_a[0] = atoll(argv[1]);
  request->point_a[1] = atoll(argv[2]);
  request->point_a[2] = atoll(argv[3]);
  request->theta = atoll(argv[4]);                                                             

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result_future = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {  
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_trasform");   
    return -1;
  }

  auto result = result_future.get();

  // referencia: https://github.com/ros2/examples/blob/rolling/rclcpp/services/minimal_client/main.cpp
  // RCLCPP_INFO(
  //   node->get_logger(), "\nR=" "\n\t%.3f\t"  "%.3f\t" "%.3f" "\n\t%.3f\t"  "%.3f\t" "%.3f" "\n\t%.3f\t"  "%.3f\t" "%.3f\n",
  //   result->r[0], result->r[1], result->r[2], result->r[3], result->r[4], result->r[5], result->r[6], result->r[7], result->r[8]);

  if(result->sucess){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result\nPoint B:[ %f %f %f]",
              result->point_b[0], result->point_b[1], result->point_b[2]);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "The transformation failed");
  }
  

  rclcpp::shutdown();
  return 0;
}
