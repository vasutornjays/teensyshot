#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "teensyshot_interfaces/msg/kiss_telemetry.hpp"
#include "teensyshot_interfaces/msg/dshot_command.hpp"


extern "C" {
  // Get declaration
#include "teensyshot/teensyshot_host_lib.h"
}

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class TeensyshotNode : public rclcpp::Node
{
public:
  TeensyshotNode()
      : Node("teensyshot_node")
  {

    for (int k = 0; k < NB_MAX_ESC; k++)
    {
      dshot[k] = 0;
    }

    // Initialize serial port
    if (Host_init_port(HOST_DEV_SERIALNB))
    {
      fprintf(stderr, "Error initializing serial port.\n");
      exit(-1);
    }

    rclcpp::on_shutdown(std::bind(&TeensyshotNode::shutdown, this));

    dshot_subscription_ = this->create_subscription<teensyshot_interfaces::msg::DshotCommand>("/teensyshot/dshot_cmd", 10, std::bind(&TeensyshotNode::dshot_callback, this, std::placeholders::_1));
    telemetry_publisher_ = this->create_publisher<teensyshot_interfaces::msg::KissTelemetry>("/teensyshot/telemetry", 10);
    std::chrono::milliseconds timer_period(10U);
    timer_ = this->create_wall_timer(timer_period, std::bind(&TeensyshotNode::timer_callback, this));
  }

private:
  void shutdown() {
    RCLCPP_ERROR(this->get_logger(), "Shutdown Node");
    Host_release_port( HOST_DEV_SERIALNB );
  }

  void dshot_callback(const teensyshot_interfaces::msg::DshotCommand::SharedPtr msg)
  {
    for(int i = 0 ; i < 8; i++)
    {
      this->dshot[i] = msg->dshot[i];
      RCLCPP_INFO(this->get_logger(), "Dshot ch: '%d => '%d'", i, msg->dshot[i]);
    }
  }

  void timer_callback()
  {
    int ret;

    // Serial exchange with teensy
    if ( ( ret = Host_comm_update(  HOST_DEV_SERIALNB, dshot, &comm ) ) )  {
      fprintf( stderr, "Error %d in Host_comm_update.\n", ret );
    }

    auto msg = teensyshot_interfaces::msg::KissTelemetry();
    msg.header.stamp.sec = this->get_clock()->now().seconds();
    msg.header.stamp.nanosec = this->get_clock()->now().nanoseconds();

    // Display telemetry
    for (int k = 0; k < NB_ESC; k++ ){
      RCLCPP_INFO(this->get_logger(),
                "#:%d\terr:%d\tdeg:%d\tcmd:%d\tmV:%d\tmA:%d\trpm_r:%d\trpm:%d",
                k,
                comm->err[k],
                comm->deg[k],
                comm->cmd[k],
                comm->volt[k] * 10,
                comm->amp[k] * 10,
                dshot[k],
                comm->rpm[k] * 10 );
      
      msg.error[k] = comm->err[k];
      msg.temperature[k] = comm->deg[k];
      msg.dshot_command[k] = comm->cmd[k];
      msg.volt[k] = comm->volt[k];
      msg.current[k] = comm->amp[k];
      msg.rpm[k] = comm->rpm[k];
    }
    
    telemetry_publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<teensyshot_interfaces::msg::DshotCommand>::SharedPtr dshot_subscription_;
  rclcpp::Publisher<teensyshot_interfaces::msg::KissTelemetry>::SharedPtr telemetry_publisher_;
  int16_t dshot[8];
  ESCPIDcomm_struct_t *comm;
  int dshot_state = 1;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyshotNode>());
  rclcpp::shutdown();
  return 0;
}
