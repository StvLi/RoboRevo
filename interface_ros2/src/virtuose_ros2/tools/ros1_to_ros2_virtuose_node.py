#!/usr/bin/env python3
"""One-off helper: expand ROS1 virtuose_node.cpp skeleton into ROS2 (best-effort)."""
import re
import sys

def main():
    p = sys.argv[1]
    text = open(p, encoding="utf-8").read()

    # Headers
    text = re.sub(
        r"#include \"ros/ros\.h\"[\s\S]*?#include \"VirtuoseAPI\.h\"",
        '''#include <chrono>
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
#include "virtuose_ros2/srv/virtuose_reset.hpp"''',
        text,
        count=1,
    )

    text = text.replace("virtuose::", "virtuose_ros2::")
    text = text.replace("ros::Publisher*", "rclcpp::Publisher<virtuose_ros2::msg::OutVirtuoseStatus>::SharedPtr /*unused*/\n/*")
    # The above is wrong - do not use this script blindly; kept for reference only.
    print("ERROR: use hand-maintained virtuose_node.cpp", file=sys.stderr)
    sys.exit(1)


if __name__ == "__main__":
    main()
