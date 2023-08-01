#include "rclcpp/rclcpp.hpp"
#include "apply_transform/srv/get_transform.hpp"                               

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

// transform the point PA = [aPx aPy aPz] into PB with a translation of [ 1 2 3 ] and a rotation around x axis of theta degrees\n
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc < 5) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "insufficient parameters\nusage: aPx aPy aPz theta");      
    return 1;
  } else if (argc > 5){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "too many parameters were passed\nusage: aPx aPy aPz theta");      
    return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("apply_transform_client");  
  rclcpp::Client<apply_transform::srv::GetTransform>::SharedPtr client =                
    node->create_client<apply_transform::srv::GetTransform>("frame_a2b_transform");          
  auto request = std::make_shared<apply_transform::srv::GetTransform::Request>();

  // receive the values as string and try to convert to a double    
  try {
    request->point_a[0] = std::stod(argv[1]);
    request->point_a[1] = std::stod(argv[2]);
    request->point_a[2] = std::stod(argv[3]);
    request->theta = std::stod(argv[4]);
  } catch (const std::invalid_argument& e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: Invalid argument. The string does not represent a valid number");
  }                                                          

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
 
  if(result->sucess){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s\nResult:\nPoint B = [ %f %f %f]",
              result->message.c_str(),result->point_b[0], result->point_b[1], result->point_b[2]);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "The transformation failed\n%s", result->message.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
