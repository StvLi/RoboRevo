#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "virtuose_ros2/msg/out_virtuose_status.hpp"
#include "virtuose_ros2/srv/virtuose_calibrate.hpp"
#include "virtuose_ros2/srv/virtuose_reset.hpp"

static uint64_t status_date = 0;
static int status_state = 0;
static void out_virtuose_statusCB(const virtuose_ros2::msg::OutVirtuoseStatus::SharedPtr msg)
{
  status_date = rclcpp::Time(msg->header.stamp).nanoseconds();
  status_state = static_cast<int>(msg->state);
  if (status_state == 3) {
    printf("Please push the force-feedback button!\n");
  }
}

static bool wait_for_service_spinning(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::ClientBase::SharedPtr & client,
  std::chrono::seconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() <= deadline) {
    rclcpp::spin_some(node);
    if (client->service_is_ready()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return client->service_is_ready();
}

int main(int argc, char **argv)
{
  printf("Starting test_calibration node\n");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_calibration");

  printf("Connecting to virtuose node\n");
  auto sub_status = node->create_subscription<virtuose_ros2::msg::OutVirtuoseStatus>(
    "out_virtuose_status", 10, out_virtuose_statusCB);
  auto cli_cal = node->create_client<virtuose_ros2::srv::VirtuoseCalibrate>("virtuose_calibrate");
  auto cli_reset = node->create_client<virtuose_ros2::srv::VirtuoseReset>("virtuose_reset");
  (void)sub_status;

  if (!wait_for_service_spinning(node, cli_reset, std::chrono::seconds(30))) {
    printf("virtuose_reset service not available\n");
    rclcpp::shutdown();
    return 1;
  }
  {
    auto req = std::make_shared<virtuose_ros2::srv::VirtuoseReset::Request>();
    auto fut = cli_reset->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, fut) != rclcpp::FutureReturnCode::SUCCESS) {
      printf("virtuose_reset call failed\n");
      rclcpp::shutdown();
      return 1;
    }
  }

  const uint64_t start = node->now().nanoseconds();

  if (!wait_for_service_spinning(node, cli_cal, std::chrono::seconds(30))) {
    printf("virtuose_calibrate service not available\n");
    rclcpp::shutdown();
    return 1;
  }

  printf("Sending calibration request\n");
  auto cal = std::make_shared<virtuose_ros2::srv::VirtuoseCalibrate::Request>();
  cal->ip_address = "127.0.0.1#5001";
  cal->automatic = true;
  auto cal_fut = cli_cal->async_send_request(cal);
  if (rclcpp::spin_until_future_complete(node, cal_fut, std::chrono::seconds(120)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    printf("calibration async_send_request failed\n");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::Rate r(10);
  int ctr = 0;
  while (rclcpp::ok() && status_state != 1) {
    ctr++;
    if (ctr % 10 == 0) {
      printf("Status: %ld %d\n", static_cast<long>(status_date - start), status_state);
    }
    r.sleep();
    rclcpp::spin_some(node);
  }

  if (wait_for_service_spinning(node, cli_reset, std::chrono::seconds(5))) {
    auto req = std::make_shared<virtuose_ros2::srv::VirtuoseReset::Request>();
    auto fut = cli_reset->async_send_request(req);
    rclcpp::spin_until_future_complete(node, fut);
  }
  printf("Calibration done\n");

  rclcpp::shutdown();
  return 0;
}
