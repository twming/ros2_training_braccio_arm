#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"

#define PI 3.1416

using namespace std::chrono_literals;
using std::placeholders::_1;

uint _DataArray[8]={90,90,90,90,90,90};

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode()
  : Node("pub_node")
  {
    publisher_ = create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", 10);
    timer_ = create_wall_timer(
      2000ms, std::bind(&PublisherNode::timer_callback, this));
  }

  void timer_callback()
  {
    message_.data.clear();
    for (int i = 0; i < 6; i++)
    {
        // TODO 1 : Put the DataArray element into the message. 
        //message_.data.push_back( ***your code here*** );

    }
    
    publisher_->publish(message_);
    RCLCPP_INFO(get_logger(),"Arm control data is published");
  }

private:
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::UInt8MultiArray message_;
};

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode()
  : Node("sub_node")
  {
    subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&SubscriberNode::callback, this, _1));
  }

  void callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    int i=0;
    for(i=0; i<6; i++)
    {
      // TODO 2 : Transform the angle from radian into degree 
      // _DataArray[i]= uint( ***your code here*** ); 
      
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
};

int main(int argc, char * argv[]) {
    
    rclcpp::init(argc,argv);

    auto node_pub = std::make_shared<PublisherNode>();
    auto node_sub = std::make_shared<SubscriberNode>();

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node_pub);
    executor.add_node(node_sub);

    // TODO 3 : Spin the executor
    // *** your code here ***
    

    rclcpp::shutdown();
    
    return 0;
}


