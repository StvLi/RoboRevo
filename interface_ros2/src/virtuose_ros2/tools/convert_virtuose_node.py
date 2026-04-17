#!/usr/bin/env python3
"""Convert ROS1 virtuose_node.cpp (stdin) to ROS2-ish output (stdout). Post-edit required for main()."""
import re
import sys

HEADER = r'''#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>

#include "VirtuoseAPI.h"

#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "virtuose_ros2/msg/in_virtuose_force.hpp"
#include "virtuose_ros2/msg/in_virtuose_pose.hpp"
#include "virtuose_ros2/msg/in_virtuose_speed.hpp"
#include "virtuose_ros2/msg/out_virtuose_force.hpp"
#include "virtuose_ros2/msg/out_virtuose_physical_pose.hpp"
#include "virtuose_ros2/msg/out_virtuose_pose.hpp"
#include "virtuose_ros2/msg/out_virtuose_speed.hpp"
#include "virtuose_ros2/msg/out_virtuose_status.hpp"
#include "virtuose_ros2/srv/virtuose_admittance.hpp"
#include "virtuose_ros2/srv/virtuose_calibrate.hpp"
#include "virtuose_ros2/srv/virtuose_impedance.hpp"
#include "virtuose_ros2/srv/virtuose_reset.hpp"

'''

MAIN_REPLACEMENT = r'''// Main function
int main(int argc, char **argv)
{
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("Starting virtuose node\n");
  fflush(stdout);
  rclcpp::init(argc, argv);

  _node = std::make_shared<rclcpp::Node>("virtuose");

  printf("Creating topics\n");
  _out_virtuose_status = _node->create_publisher<virtuose_ros2::msg::OutVirtuoseStatus>("out_virtuose_status", 1);
  _out_virtuose_pose = _node->create_publisher<virtuose_ros2::msg::OutVirtuosePose>("out_virtuose_pose", 1);
  _out_virtuose_physical_pose =
    _node->create_publisher<virtuose_ros2::msg::OutVirtuosePhysicalPose>("out_virtuose_physical_pose", 1);
  _out_virtuose_speed = _node->create_publisher<virtuose_ros2::msg::OutVirtuoseSpeed>("out_virtuose_speed", 1);
  _out_virtuose_force = _node->create_publisher<virtuose_ros2::msg::OutVirtuoseForce>("out_virtuose_force", 1);

  _node->create_subscription<virtuose_ros2::msg::InVirtuosePose>(
    "in_virtuose_pose", 1, in_virtuose_poseCB);
  _node->create_subscription<virtuose_ros2::msg::InVirtuoseSpeed>(
    "in_virtuose_speed", 1, in_virtuose_speedCB);
  _node->create_subscription<virtuose_ros2::msg::InVirtuoseForce>(
    "in_virtuose_force", 1, in_virtuose_forceCB);

  printf("Creating services\n");
  _node->create_service<virtuose_ros2::srv::VirtuoseCalibrate>("virtuose_calibrate", calibrateCB);
  _node->create_service<virtuose_ros2::srv::VirtuoseAdmittance>("virtuose_admittance", admittanceCB);
  _node->create_service<virtuose_ros2::srv::VirtuoseImpedance>("virtuose_impedance", impedanceCB);
  _node->create_service<virtuose_ros2::srv::VirtuoseReset>("virtuose_reset", resetCB);

  printf("Publishing initial status\n");
  virtuose_ros2::msg::OutVirtuoseStatus status;
  status.header.stamp = _node->get_clock()->now();
  status.header.frame_id = "/Ready";
  status.state = _state = STATE_READY;
  _out_virtuose_status->publish(status);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(_node);
  std::thread spin_thread([&]() { executor.spin(); });

  rclcpp::WallRate r(100);
  while (rclcpp::ok()) {
    if (_state == STATE_ERROR) {
      if (_VC != NULL) {
        virtStopLoop(_VC);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        virtClose(_VC);
        _VC = NULL;
        status.header.stamp = _node->get_clock()->now();
        status.header.frame_id = "/Error";
        status.state = _state = STATE_ERROR;
        _out_virtuose_status->publish(status);
        printf("Entered state ERROR\n");
      }
    }
    r.sleep();
  }

  executor.cancel();
  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  return 0;
}
'''


def main():
    text = sys.stdin.read()
    text = re.sub(r"[\s\S]*?#include \"VirtuoseAPI\.h\"\n", HEADER, text, count=1)
    text = text.replace("ros::Publisher*", "rclcpp::Publisher<virtuose_ros2::msg::OutVirtuoseStatus>::SharedPtr")
    # Fix the above over-replace: only first occurrence was wrong - we need distinct types per publisher
    # So do NOT use that - instead replace block manually

if __name__ == "__main__":
    main()
