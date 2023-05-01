#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

#include <geometry_msgs/msg/twist.hpp>

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : low_udp(LOWLEVEL),
          high_udp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }

    void lowUdpRecv()
    {
      low_udp.Recv();
      low_udp.GetRecv(low_state);
    }
    void highUdpRecv()
    {
      high_udp.Recv();
      high_udp.GetRecv(high_state);
    }
    void lowUdpSend()
    {
      low_udp.SetSend(low_cmd);
      low_udp.Send();
    }
    void highUdpSend()
    {
      high_udp.SetSend(high_cmd);
      high_udp.Send();
    }
};

class HighLevelPublisher : public rclcpp::Node
{
public:
  HighLevelPublisher()
    : Node("high_level_publisher"), 
      high_count(0),
      cmd_vel_count(0),
      count(0)
  {
      timer = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&HighLevelPublisher::timerCallback, this));

      pub_high = this->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("high_state", 1);
      sub_high = this->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1, std::bind(&HighLevelPublisher::highCmdCallback, this, std::placeholders::_1));

      sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1), std::bind(&HighLevelPublisher::cmdVelCallback, this, std::placeholders::_1));
  }
private:
  void timerCallback()
  {
    printf("timerCallback is running !\t%ld\n", count);
    ros2_unitree_legged_msgs::msg::HighState high_state_ros;

    custom.highUdpRecv();

    high_state_ros = state2rosMsg(custom.high_state);
    pub_high->publish(high_state_ros);
    count++;
  }
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    printf("cmdVelCallback is running !\t%ld\n", cmd_vel_count);

    custom.high_cmd.head[0] = 0xFE;
    custom.high_cmd.head[1] = 0xEF;
    custom.high_cmd.levelFlag = 0;
    custom.high_cmd.mode = 0;
    custom.high_cmd.gaitType = 0;
    custom.high_cmd.speedLevel = 0;
    custom.high_cmd.footRaiseHeight = 0;
    custom.high_cmd.bodyHeight = 0;
    custom.high_cmd.euler[0] = 0;
    custom.high_cmd.euler[1] = 0;
    custom.high_cmd.euler[2] = 0;
    custom.high_cmd.velocity[0] = 0.0f;
    custom.high_cmd.velocity[1] = 0.0f;
    custom.high_cmd.yawSpeed = 0.0f;
    custom.high_cmd.reserve = 0;

    custom.high_cmd.velocity[0] = msg->linear.x;
    custom.high_cmd.velocity[1] = msg->linear.y;
    custom.high_cmd.yawSpeed = msg->angular.z;

    custom.high_cmd.mode = 2;
    custom.high_cmd.gaitType = 1;
    
    custom.highUdpSend();

    //ros2_unitree_legged_msgs::msg::HighState high_state_ros;

    //custom.highUdpRecv();

    //high_state_ros = state2rosMsg(custom.high_state);

    //pub_high->publish(high_state_ros);

    printf("cmdVelCallback ending !\t%ld\n\n", cmd_vel_count++);
  }

  void highCmdCallback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
  {
    printf("highCmdCallback is running !\t%ld\n", high_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    custom.high_udp.SetSend(custom.high_cmd);
    custom.high_udp.Send();

    //ros2_unitree_legged_msgs::msg::HighState high_state_ros;

    //custom.high_udp.Recv();
    //custom.high_udp.GetRecv(custom.high_state);

    //high_state_ros = state2rosMsg(custom.high_state);

    //pub_high->publish(high_state_ros);

    printf("highCmdCallback ending !\t%ld\n\n", high_count++);
  }

  Custom custom;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr pub_high;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr sub_high;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
  long high_count;
  long cmd_vel_count;
  long count;
};


/*
rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr sub_low;
rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr pub_low;

long low_count = 0;


void lowCmdCallback(const ros2_unitree_legged_msgs::msg::LowCmd::SharedPtr msg)
{

    printf("lowCmdCallback is running !\t%ld\n", low_count);

    custom.low_cmd = rosMsg2Cmd(msg);

    custom.low_udp.SetSend(custom.low_cmd);
    custom.low_udp.Send();

    ros2_unitree_legged_msgs::msg::LowState low_state_ros;

    custom.low_udp.Recv();
    custom.low_udp.GetRecv(custom.low_state);

    low_state_ros = state2rosMsg(custom.low_state);

    pub_low->publish(low_state_ros);

    printf("lowCmdCallback ending!\t%ld\n\n", ::low_count++);
}
*/

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (strcasecmp(argv[1], "LOWLEVEL") == 0)
    {
        auto node = rclcpp::Node::make_shared("node_ros2_udp");
        printf("low level runing!\n");

        //pub_low = node->create_publisher<ros2_unitree_legged_msgs::msg::LowState>("low_state", 1);
        //sub_low = node->create_subscription<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1, lowCmdCallback);

        rclcpp::spin(node);
    }
    else if (strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        printf("high level runing!\n");
        auto node = std::make_shared<HighLevelPublisher>();
        rclcpp::spin(node);
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel(not case sensitive)" << std::endl;
        exit(-1);
    }

    rclcpp::shutdown();

    return 0;
}
