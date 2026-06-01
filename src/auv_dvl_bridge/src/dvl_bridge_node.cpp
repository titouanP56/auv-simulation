#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/dvl_velocity_tracking.pb.h>

/**
 * @class DvlBridgeNode
 * @brief ROS 2 Node bridging Gazebo DVL messages to ROS 2 Twist messages.
 * 
 * Subscribes to Gazebo transport topic `/dvl/velocity` (protobuf) and converts it 
 * to a ROS 2 `geometry_msgs/msg/TwistWithCovarianceStamped` on `/dvl/velocity_ros`.
 * This allows the ROS 2 EKF to consume simulated DVL velocity data.
 */
class DvlBridgeNode : public rclcpp::Node
{
public:
  DvlBridgeNode() : Node("dvl_bridge_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/dvl/velocity_ros", 10);

    // Subscribe to Gazebo topic
    if (!gz_node_.Subscribe("/dvl/velocity", &DvlBridgeNode::OnGzDvlMessage, this))
    {
      RCLCPP_ERROR(this->get_logger(), "Error subscribing to Gazebo topic /dvl/velocity");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Successfully subscribed to Gazebo topic /dvl/velocity");
    }
  }

private:
  void OnGzDvlMessage(const gz::msgs::DVLVelocityTracking &msg)
  {
    auto ros_msg = geometry_msgs::msg::TwistWithCovarianceStamped();
    
    // Check if there's velocity data
    if (msg.has_velocity() && msg.velocity().has_mean())
    {
      if (msg.has_header() && msg.header().has_stamp()) {
          ros_msg.header.stamp.sec = msg.header().stamp().sec();
          ros_msg.header.stamp.nanosec = msg.header().stamp().nsec();
      } else {
          ros_msg.header.stamp = this->now();
      }
      
      // Gazebo often sends fully qualified names like 'BlueROV2::base_link::dvl_sensor' 
      // or doesn't match our ROS TF tree. We force 'base_link' here to guarantee 
      // the EKF accepts the measurement.
      ros_msg.header.frame_id = "base_link";
      
      ros_msg.twist.twist.linear.x = msg.velocity().mean().x();
      // Negate Y: Gazebo DVL outputs Y=right (starboard), ROS base_link uses Y=left (port)
      ros_msg.twist.twist.linear.y = msg.velocity().mean().y();
      ros_msg.twist.twist.linear.z = msg.velocity().mean().z();

      // Ensure the covariance array is correctly sized (9 elements for a 3x3 matrix)
      if (msg.velocity().covariance_size() == 9)
      {
         // Twist covariance is 36 elements (6x6). We only set the linear 3x3 parts.
         ros_msg.twist.covariance[0] = msg.velocity().covariance(0);
         ros_msg.twist.covariance[1] = msg.velocity().covariance(1);
         ros_msg.twist.covariance[2] = msg.velocity().covariance(2);
         ros_msg.twist.covariance[6] = msg.velocity().covariance(3);
         ros_msg.twist.covariance[7] = msg.velocity().covariance(4);
         ros_msg.twist.covariance[8] = msg.velocity().covariance(5);
         ros_msg.twist.covariance[12] = msg.velocity().covariance(6);
         ros_msg.twist.covariance[13] = msg.velocity().covariance(7);
         ros_msg.twist.covariance[14] = msg.velocity().covariance(8);
      }
      else
      {
         // Provide a fallback covariance if Gazebo doesn't provide one
         ros_msg.twist.covariance[0] = 0.01;
         ros_msg.twist.covariance[7] = 0.01;
         ros_msg.twist.covariance[14] = 0.01;
      }
      publisher_->publish(ros_msg);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_;
  gz::transport::Node gz_node_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DvlBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
