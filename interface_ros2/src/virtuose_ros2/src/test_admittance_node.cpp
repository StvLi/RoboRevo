#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "virtuose_ros2/msg/in_virtuose_pose.hpp"
#include "virtuose_ros2/msg/out_virtuose_pose.hpp"
#include "virtuose_ros2/msg/out_virtuose_status.hpp"
#include "virtuose_ros2/srv/virtuose_admittance.hpp"
#include "virtuose_ros2/srv/virtuose_reset.hpp"

static float cur_pose[7];
static uint64_t pose_date = 0;
static uint64_t status_date = 0;
static int status_state = 0;
static int status_button = 0;

static void out_virtuose_poseCB(const virtuose_ros2::msg::OutVirtuosePose::SharedPtr msg)
{
  pose_date = rclcpp::Time(msg->header.stamp).nanoseconds();
  cur_pose[0] = msg->virtuose_pose.translation.x;
  cur_pose[1] = msg->virtuose_pose.translation.y;
  cur_pose[2] = msg->virtuose_pose.translation.z;
  cur_pose[3] = msg->virtuose_pose.rotation.x;
  cur_pose[4] = msg->virtuose_pose.rotation.y;
  cur_pose[5] = msg->virtuose_pose.rotation.z;
  cur_pose[6] = msg->virtuose_pose.rotation.w;
}

static void out_virtuose_statusCB(const virtuose_ros2::msg::OutVirtuoseStatus::SharedPtr msg)
{
  status_date = rclcpp::Time(msg->header.stamp).nanoseconds();
  status_state = static_cast<int>(msg->state);
  status_button = static_cast<int>(msg->buttons);
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
  printf("Starting test_admittance node\n");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_admittance");

  printf("Connecting to virtuose node\n");
  auto sub_status = node->create_subscription<virtuose_ros2::msg::OutVirtuoseStatus>(
    "out_virtuose_status", 10, out_virtuose_statusCB);
  auto sub_pose = node->create_subscription<virtuose_ros2::msg::OutVirtuosePose>(
    "out_virtuose_pose", 10, out_virtuose_poseCB);
  auto pub_pose = node->create_publisher<virtuose_ros2::msg::InVirtuosePose>("in_virtuose_pose", 10);
  auto cli_adm = node->create_client<virtuose_ros2::srv::VirtuoseAdmittance>("virtuose_admittance");
  auto cli_reset = node->create_client<virtuose_ros2::srv::VirtuoseReset>("virtuose_reset");
  (void)sub_status;
  (void)sub_pose;

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

  if (!wait_for_service_spinning(node, cli_adm, std::chrono::seconds(30))) {
    printf("virtuose_admittance service not available\n");
    rclcpp::shutdown();
    return 1;
  }

  printf("Sending admittance request\n");
  auto adm = std::make_shared<virtuose_ros2::srv::VirtuoseAdmittance::Request>();
  adm->ip_address = "127.0.0.1#5001";
  adm->indexing_mode = 0;
  adm->speed_factor = 1.0f;
  adm->force_factor = 1.0f;
  adm->power_enable = true;
  adm->max_force = 10.0f;
  adm->max_torque = 1.0f;
  adm->ktrans = 2000.0f;
  adm->btrans = 20.0f;
  adm->krot = 5.0f;
  adm->brot = 0.25f;
  adm->base_frame.translation.x = 0.0;
  adm->base_frame.translation.y = 0.0;
  adm->base_frame.translation.z = 0.0;
  adm->base_frame.rotation.x = 0.0;
  adm->base_frame.rotation.y = 0.0;
  adm->base_frame.rotation.z = 0.0;
  adm->base_frame.rotation.w = 1.0;

  auto adm_fut = cli_adm->async_send_request(adm);
  if (rclcpp::spin_until_future_complete(node, adm_fut) != rclcpp::FutureReturnCode::SUCCESS) {
    printf("admittance async_send_request failed\n");
    rclcpp::shutdown();
    return 1;
  }
  auto adm_resp = adm_fut.get();
  if (!adm_resp->success) {
    printf(
      "admittance rejected (success=false), error=%d\n"
      "请查看运行 virtuose_node 的终端：stderr 会打印 [virtuose] 具体失败步骤与 Virtuose 错误码。\n",
      static_cast<int>(adm_resp->error));
    rclcpp::shutdown();
    return 1;
  }
  const uint32_t client_id = adm_resp->client_id;
  printf("Our client ID is %u\n", client_id);

  rclcpp::Rate rate(500);
  int ctr = 0;
  while (rclcpp::ok()) {
    virtuose_ros2::msg::InVirtuosePose pose;
    pose.header.stamp = node->now();
    pose.client_id = client_id;
    pose.virtuose_pose.translation.x = cur_pose[0];
    pose.virtuose_pose.translation.y = cur_pose[1];
    pose.virtuose_pose.translation.z = cur_pose[2];
    pose.virtuose_pose.rotation.x = cur_pose[3];
    pose.virtuose_pose.rotation.y = cur_pose[4];
    pose.virtuose_pose.rotation.z = cur_pose[5];
    pose.virtuose_pose.rotation.w = cur_pose[6];
    if (pose.virtuose_pose.translation.z < -0.1f) {
      pose.virtuose_pose.translation.z = -0.1f;
    }
    pub_pose->publish(pose);
    ctr++;
    if (ctr % 500 == 0) {
      printf(
        "Status: %ld %d %d %f %f %f\n",
        static_cast<long>(status_date - start), status_state, status_button,
        cur_pose[0], cur_pose[1], cur_pose[2]);
    }
    rate.sleep();
    rclcpp::spin_some(node);
  }

  if (wait_for_service_spinning(node, cli_reset, std::chrono::seconds(5))) {
    auto req = std::make_shared<virtuose_ros2::srv::VirtuoseReset::Request>();
    auto fut = cli_reset->async_send_request(req);
    rclcpp::spin_until_future_complete(node, fut);
  }

  rclcpp::shutdown();
  return 0;
}
