#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


extern "C" {
  // Get declaration for f(int i, char c, float x)
#include "teensyshot/teensyshot_host_lib.h"
}

using namespace std::chrono_literals;

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

    timer_ = this->create_wall_timer(
        10ms, std::bind(&TeensyshotNode::timer_callback, this));
  }

private:

  void shutdown() {
    RCLCPP_ERROR(this->get_logger(), "Shutdown Node");
    Host_release_port( HOST_DEV_SERIALNB );
  }


  void timer_callback()
  {

    int ret;

    // Serial exchange with teensy
    if ( ( ret = Host_comm_update(  HOST_DEV_SERIALNB,
                                    dshot,
                                    &comm ) ) )  {
      fprintf( stderr, "Error %d in Host_comm_update.\n", ret );
      // break;
    }
    
    // Display telemetry
    for (int k = 0; k < NB_ESC; k++ )
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

    
    
    // Update reference
      for (int k = 0; k < NB_MAX_ESC; k++ ){
        if (this->dshot[k] < 5 && this->dshot_state == 1){
            this->dshot[k]++;
        } else if (this->dshot[k] == 5 && this->dshot_state == 1) {
            this->dshot_state = 0;
        } else if (this->dshot[k] > -0 && this->dshot_state == 0) {
            this->dshot[k]--;
        } else if (this->dshot[k] == -0 && this->dshot_state == 0) {
            this->dshot_state = 1;
        } else {
            this->dshot[k] = 0;
        }
      }
  }
  rclcpp::TimerBase::SharedPtr timer_;

  int16_t dshot[NB_MAX_ESC];
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
