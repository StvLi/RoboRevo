#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>

#include "VirtuoseAPI.h"

#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

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
#include "virtuose_ros2/handle_mapping.hpp"

/* Current state of the virtuose node */
#define STATE_UNKNOWN 0
#define STATE_READY 1
#define STATE_ERROR 2
#define STATE_CALIB 3
#define STATE_CART_IMP 4
#define STATE_CART_ADM 5
uint8_t _state = STATE_UNKNOWN;

/* VirtContext */
VirtContext _VC = NULL;

/* Publishers */
rclcpp::Publisher<virtuose_ros2::msg::OutVirtuoseStatus>::SharedPtr _out_virtuose_status;
rclcpp::Publisher<virtuose_ros2::msg::OutVirtuosePose>::SharedPtr _out_virtuose_pose;
rclcpp::Publisher<virtuose_ros2::msg::OutVirtuosePhysicalPose>::SharedPtr _out_virtuose_physical_pose;
rclcpp::Publisher<virtuose_ros2::msg::OutVirtuoseSpeed>::SharedPtr _out_virtuose_speed;
rclcpp::Publisher<virtuose_ros2::msg::OutVirtuoseForce>::SharedPtr _out_virtuose_force;

// 新增：模拟量和布尔量发布者
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _out_handle_analog;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _out_handle_bool_0;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _out_handle_bool_1;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _out_handle_bool_2;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _out_handle_bool_3;


std::shared_ptr<rclcpp::Node> _node;

/* virtSetPeriodicFunction 可能长期保存周期指针，使用静态存储避免悬空 */
static float g_admittance_period_s = 0.001f;
static float g_impedance_period_s = 0.001f;

static VirtIndexingType clamp_indexing_mode(uint8_t mode)
{
  /* VirtuoseAPI.h 中枚举值不含 5，非法值回退为 INDEXING_ALL */
  static const uint8_t k_valid[] = {0, 1, 2, 3, 4, 6, 7};
  for (uint8_t v : k_valid) {
    if (mode == v) {
      return static_cast<VirtIndexingType>(mode);
    }
  }
  return INDEXING_ALL;
}

static void log_virtuose_failure(const char * stage, VirtContext vc)
{
  int code = 0;
  if (vc != NULL) {
    code = virtGetErrorCode(vc);
  }
  char * msg = virtGetErrorMessage(code);
  fprintf(
    stderr, "[virtuose] %s 失败: virt error code=%d (%s)\n", stage, code,
    msg ? msg : "?");
  fflush(stderr);
  if (_node) {
    RCLCPP_ERROR(
      _node->get_logger(), "%s failed: code=%d (%s)", stage, code,
      msg ? msg : "?");
  }
}

/* Setpoints */
float set_pose[7] = { 0, 0, 0, 0, 0, 0, 1 };
uint64_t set_pose_date = 0;
float set_speed[6];
uint64_t set_speed_date = 0;
float set_force[6];
uint64_t set_force_date = 0;

/* Dates */
uint64_t start_loop_date = 0;

/* Storage for current client ID */
uint32_t _client_id = 0;

uint32_t generate_client_id()
{
  srand(static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count()));
  _client_id = static_cast<uint32_t>(rand());
  return _client_id;
}

// Callback for the topic in_virtuose_pose
void in_virtuose_poseCB(const virtuose_ros2::msg::InVirtuosePose::SharedPtr msg)
{
  // Check client id
  if (msg->client_id != _client_id)
    return;
  // Store last set_pose date
  set_pose_date = rclcpp::Time(msg->header.stamp, _node->get_clock()->get_clock_type()).nanoseconds();
  // Store set_pose
  set_pose[0] = msg->virtuose_pose.translation.x;
  set_pose[1] = msg->virtuose_pose.translation.y;
  set_pose[2] = msg->virtuose_pose.translation.z;
  set_pose[3] = msg->virtuose_pose.rotation.x;
  set_pose[4] = msg->virtuose_pose.rotation.y;
  set_pose[5] = msg->virtuose_pose.rotation.z;
  set_pose[6] = msg->virtuose_pose.rotation.w;
  // Normalize quaternion
  float norm_quat = sqrt(set_pose[3]*set_pose[3]+set_pose[4]*set_pose[4]+set_pose[5]*set_pose[5]+set_pose[6]*set_pose[6]);
  if (norm_quat == 0.0)
  {
    set_pose[6] = 1.0;
  }
  else
  {
    set_pose[3] /= norm_quat;
    set_pose[4] /= norm_quat;
    set_pose[5] /= norm_quat;
    set_pose[6] /= norm_quat;
  }
}

// Callback for topic in_virtuose_speed
void in_virtuose_speedCB(const virtuose_ros2::msg::InVirtuoseSpeed::SharedPtr msg)
{
  // Check client id
  if (msg->client_id != _client_id)
    return;
  // Store last set_speed date
  set_speed_date = rclcpp::Time(msg->header.stamp, _node->get_clock()->get_clock_type()).nanoseconds();
  set_speed[0] = msg->virtuose_speed.linear.x;
  set_speed[1] = msg->virtuose_speed.linear.y;
  set_speed[2] = msg->virtuose_speed.linear.z;
  set_speed[3] = msg->virtuose_speed.angular.x;
  set_speed[4] = msg->virtuose_speed.angular.y;
  set_speed[5] = msg->virtuose_speed.angular.z;
}

// Callback for topic in_virtuose_force
void in_virtuose_forceCB(const virtuose_ros2::msg::InVirtuoseForce::SharedPtr msg)
{
  // Check client id
  if (msg->client_id != _client_id)
    return;
  // Store last set_force date
  set_force_date = rclcpp::Time(msg->header.stamp, _node->get_clock()->get_clock_type()).nanoseconds();
  set_force[0] = msg->virtuose_force.force.x;
  set_force[1] = msg->virtuose_force.force.y;
  set_force[2] = msg->virtuose_force.force.z;
  set_force[3] = msg->virtuose_force.torque.x;
  set_force[4] = msg->virtuose_force.torque.y;
  set_force[5] = msg->virtuose_force.torque.z;
}

// Callback for service virtuose_calibrate
void calibrateCB(
  const std::shared_ptr<virtuose_ros2::srv::VirtuoseCalibrate::Request> request,
  std::shared_ptr<virtuose_ros2::srv::VirtuoseCalibrate::Response> response)
{
  printf("Performing calibration on ip address %s\n", request->ip_address.c_str());
  std::this_thread::sleep_for(std::chrono::seconds(1));
  // Change state to CALIB
  virtuose_ros2::msg::OutVirtuoseStatus status;
  status.header.stamp = _node->get_clock()->now();
  status.header.frame_id = "/Calibration";
  status.state = _state = STATE_CALIB;
  _out_virtuose_status->publish(status);
  // Simulate calibration
  std::this_thread::sleep_for(std::chrono::seconds(10));
  // Change state to READY
  status.header.stamp = _node->get_clock()->now();
  status.header.frame_id = "/Calibration";
  status.state = _state = STATE_READY;
  _out_virtuose_status->publish(status);
  // Send response
  response->success = true;
  response->already_calibrated = false;
  response->hardware_down = false;
  response->timeout = false;
  printf("Calibration done\n");
}

// Callback for VirtuoseAPI, in admittance mode
void virtAdmittanceCB(VirtContext VC, void *ignored)
{
  // Current date
  uint64_t now = _node->get_clock()->now().nanoseconds();
  // Physical position
  float disp[7];
  if (!virtGetPhysicalPosition(VC, disp))
  {
    virtuose_ros2::msg::OutVirtuosePhysicalPose ppose;
    ppose.header.stamp = _node->get_clock()->now();
    ppose.virtuose_physical_pose.translation.x = disp[0];
    ppose.virtuose_physical_pose.translation.y = disp[1];
    ppose.virtuose_physical_pose.translation.z = disp[2];
    ppose.virtuose_physical_pose.rotation.x = disp[3];
    ppose.virtuose_physical_pose.rotation.y = disp[4];
    ppose.virtuose_physical_pose.rotation.z = disp[5];
    ppose.virtuose_physical_pose.rotation.w = disp[6];
    _out_virtuose_physical_pose->publish(ppose);
  }
  // Indexed position
  if (!virtGetPosition(VC, disp))
  {
    virtuose_ros2::msg::OutVirtuosePose pose;
    pose.header.stamp = _node->get_clock()->now();
    pose.virtuose_pose.translation.x = disp[0];
    pose.virtuose_pose.translation.y = disp[1];
    pose.virtuose_pose.translation.z = disp[2];
    pose.virtuose_pose.rotation.x = disp[3];
    pose.virtuose_pose.rotation.y = disp[4];
    pose.virtuose_pose.rotation.z = disp[5];
    pose.virtuose_pose.rotation.w = disp[6];
    _out_virtuose_pose->publish(pose);
    printf("Pose: %f %f %f %f %f %f %f\n", disp[0], disp[1], disp[2], disp[3], disp[4], disp[5], disp[6]);
  }
  // Speed
  float speed[6];
  if(!virtGetSpeed(VC, speed))
  {
    virtuose_ros2::msg::OutVirtuoseSpeed vspeed;
    vspeed.header.stamp = _node->get_clock()->now();
    vspeed.virtuose_speed.linear.x = speed[0];
    vspeed.virtuose_speed.linear.y = speed[1];
    vspeed.virtuose_speed.linear.z = speed[2];
    vspeed.virtuose_speed.angular.x = speed[3];
    vspeed.virtuose_speed.angular.y = speed[4];
    vspeed.virtuose_speed.angular.z = speed[5];
    _out_virtuose_speed->publish(vspeed);
  }
  // Force
  float force[6];
  if (virtGetForce(VC, force))
  {
    virtuose_ros2::msg::OutVirtuoseForce vforce;
    vforce.header.stamp = _node->get_clock()->now();
    vforce.virtuose_force.force.x = force[0];
    vforce.virtuose_force.force.y = force[1];
    vforce.virtuose_force.force.z = force[2];
    vforce.virtuose_force.torque.x = force[3];
    vforce.virtuose_force.torque.y = force[4];
    vforce.virtuose_force.torque.z = force[5];
    _out_virtuose_force->publish(vforce);
  }
  // Status
  virtuose_ros2::msg::OutVirtuoseStatus status;
  status.header.stamp = _node->get_clock()->now();
  status.header.frame_id = "/Admittance";
  status.state = _state = STATE_CART_ADM;
  int button[32] = {0};

  // 手柄按钮
  unsigned int tor_inputs = 0;
  int tor_result = virtGetTorInputs(VC, &tor_inputs);
  if (tor_result == VIRT_E_NO_ERROR) {
      // 打印 tor_inputs 的二进制表示，按下不同按钮观察哪一位发生变化
      // printf("Tor Inputs: %u (Hex: 0x%X)\n", tor_inputs, tor_inputs);
      
      // --- 新增：发布布尔量 ---
      std_msgs::msg::Bool bool_0_msg, bool_1_msg, bool_2_msg, bool_3_msg;
      
      bool_0_msg.data = ((tor_inputs & virtuose_ros2::handle_mapping::kButtonAMask) != 0);  // 按钮A
      bool_1_msg.data = ((tor_inputs & virtuose_ros2::handle_mapping::kButtonBMask) != 0);  // 按钮B
      bool_2_msg.data = ((tor_inputs & virtuose_ros2::handle_mapping::kTriggerAMask) != 0);  // 拨杆A
      bool_3_msg.data = ((tor_inputs & virtuose_ros2::handle_mapping::kTriggerBMask) != 0);  // 拨杆B
      
      _out_handle_bool_0->publish(bool_0_msg);
      _out_handle_bool_1->publish(bool_1_msg);
      _out_handle_bool_2->publish(bool_2_msg);
      _out_handle_bool_3->publish(bool_3_msg);
      // ----------------------
  }
  
  // 机械臂按钮
  status.buttons = 0;
  for (int i = 0; i < 32; i++) {
    virtGetButton(VC, i, &button[i]);

    status.buttons += (button[i] == 1) ? (1 << i) : 0;
    // if (button[i] == 1)
    //   printf("Button %d: %d\t", i, button[i]);
  }
  // printf("\n");

  // 模拟量获取
  float analogValues_0[8]; 
  int result = virtGetAnalogicInputs(VC, analogValues_0);
  if (result == VIRT_E_NO_ERROR) {
      // 创建 Float32 消息
      std_msgs::msg::Float32 analog_msg;
      
      // 只赋值第 0 位
      analog_msg.data = analogValues_0[0];
      
      // 发布
      _out_handle_analog->publish(analog_msg);
      
      // 如果需要调试打印
      // printf("Published Analog Channel 0: %f\n", analog_msg.data);
  }

  int emergency_stop;
  if (virtGetEmergencyStop(VC, &emergency_stop))
  {
    printf("Error in virtGetEmergencyStop with code %d, go to ERROR\n",
        virtGetErrorCode(VC));
    virtSetPowerOn(VC, 0);
    virtStopLoop(VC);
    _state = STATE_ERROR;
    return;
  }
  status.emergency_stop = (emergency_stop == 1);
  _out_virtuose_status->publish(status);
  // Watchdog on position
  if (((now - start_loop_date) > 30000000) && (now > set_pose_date) && ((now - set_pose_date) > 30000000))
  {
    printf("Timeout on set pose, going to state ERROR\n");
    virtSetPowerOn(VC, 0);
    virtStopLoop(VC);
    _state = STATE_ERROR;
    return;
  }
  // Set position
  if (virtSetPosition(VC, set_pose))
  {
    printf("Error in virtSetPosition with code %d, go to ERROR\n",
        virtGetErrorCode(VC));
    printf("set_pose was: %f %f %f %f %f %f %f\n",
        set_pose[0], set_pose[1], set_pose[2], 
        set_pose[3], set_pose[4], set_pose[5], set_pose[6]);
    virtSetPowerOn(VC, 0);
    virtStopLoop(VC);
    _state = STATE_ERROR;
    return;
  }
  // Set speed
  if (virtSetSpeed(VC, set_speed))
  {
    printf("Error in virtSetSpeed with code %d, go to ERROR\n",
       virtGetErrorCode(VC));
    virtSetPowerOn(VC, 0);
    virtStopLoop(VC);
    _state = STATE_ERROR;
    return;
  }
}

// Callback for service virtuose_admittance
void admittanceCB(
  const std::shared_ptr<virtuose_ros2::srv::VirtuoseAdmittance::Request> request,
  std::shared_ptr<virtuose_ros2::srv::VirtuoseAdmittance::Response> response)
{
  printf("Received admittance request\n");
  // Check state
  if (_state != STATE_READY)
  {
    printf("Not in state READY, go to ERROR\n");
    _state = STATE_ERROR;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    return;
  }
  // Open connection
  printf("Connecting to IP Address:%s\n", request->ip_address.c_str());

  _VC = virtOpen(request->ip_address.c_str());
  if (_VC == NULL)
  {
    fprintf(
      stderr,
      "[virtuose] virtOpen(\"%s\") 返回 NULL，请确认 Virtuose 设备/仿真服务已启动且地址格式为 IP#<port>\n",
      request->ip_address.c_str());
    fflush(stderr);
    if (_node) {
      RCLCPP_ERROR(
        _node->get_logger(), "virtOpen failed for [%s]", request->ip_address.c_str());
    }
    _state = STATE_ERROR;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    return;
  }
  printf("Virtuose connection OK\n");
  const int pe = request->power_enable ? 1 : 0;
  const VirtIndexingType idx = clamp_indexing_mode(request->indexing_mode);
  if (virtSetIndexingMode(_VC, idx) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetIndexingMode", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtSetForceFactor(_VC, request->force_factor) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetForceFactor", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtSetSpeedFactor(_VC, request->speed_factor) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetSpeedFactor", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtEnableForceFeedback(_VC, pe) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtEnableForceFeedback", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtSetPowerOn(_VC, pe) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetPowerOn", _VC);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtSetCommandType(_VC, COMMAND_TYPE_VIRTMECH) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetCommandType", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtSaturateTorque(_VC, request->max_force, request->max_torque) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSaturateTorque", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  float base[7];
  base[0] = request->base_frame.translation.x;
  base[1] = request->base_frame.translation.y;
  base[2] = request->base_frame.translation.z;
  base[3] = request->base_frame.rotation.x;
  base[4] = request->base_frame.rotation.y;
  base[5] = request->base_frame.rotation.z;
  base[6] = request->base_frame.rotation.w;
  if (virtSetBaseFrame(_VC, base) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetBaseFrame", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtSetTimeStep(_VC, 0.001f) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetTimeStep", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtSetPeriodicFunction(_VC, virtAdmittanceCB, &g_admittance_period_s, NULL) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetPeriodicFunction", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  start_loop_date = _node->get_clock()->now().nanoseconds();
  if (virtStartLoop(_VC) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtStartLoop", _VC);
    virtSetPowerOn(_VC, 0);
    virtStopLoop(_VC);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }

  float K[6], B[6];
  K[0] = K[1] = K[2] = request->krot;
  K[3] = K[4] = K[5] = request->ktrans;
  B[0] = B[1] = B[2] = request->brot;
  B[3] = B[4] = B[5] = request->btrans;
  if (virtAttachQSVO(_VC, K, B) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtAttachQSVO", _VC);
    virtSetPowerOn(_VC, 0);
    virtStopLoop(_VC);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }

  _state = STATE_CART_ADM;
  response->success = true;
  response->error = false;
  response->client_id = generate_client_id();
  printf("Entering state CART_ADM\n");
}

// Callback for VirtuoseAPI, in impedance mode
void virtImpedanceCB(VirtContext VC, void *ignored)
{
  // Current date
  uint64_t now = _node->get_clock()->now().nanoseconds();
  // Physical position
  float disp[7];
  if (!virtGetPhysicalPosition(VC, disp))
  {
    virtuose_ros2::msg::OutVirtuosePhysicalPose ppose;
    ppose.virtuose_physical_pose.translation.x = disp[0];
    ppose.virtuose_physical_pose.translation.y = disp[1];
    ppose.virtuose_physical_pose.translation.z = disp[2];
    ppose.virtuose_physical_pose.rotation.x = disp[3];
    ppose.virtuose_physical_pose.rotation.y = disp[4];
    ppose.virtuose_physical_pose.rotation.z = disp[5];
    ppose.virtuose_physical_pose.rotation.w = disp[6];
    _out_virtuose_physical_pose->publish(ppose);
  }
  // Indexed position
  if (!virtGetPosition(VC, disp))
  {
    virtuose_ros2::msg::OutVirtuosePose pose;
    pose.virtuose_pose.translation.x = disp[0];
    pose.virtuose_pose.translation.y = disp[1];
    pose.virtuose_pose.translation.z = disp[2];
    pose.virtuose_pose.rotation.x = disp[3];
    pose.virtuose_pose.rotation.y = disp[4];
    pose.virtuose_pose.rotation.z = disp[5];
    pose.virtuose_pose.rotation.w = disp[6];
    _out_virtuose_pose->publish(pose);
    printf("Pose: %f %f %f %f %f %f %f\n", disp[0], disp[1], disp[2], disp[3], disp[4], disp[5], disp[6]);
  }
  // Speed
  float speed[6];
  if (!virtGetSpeed(VC, speed))
  {
    virtuose_ros2::msg::OutVirtuoseSpeed vspeed;
    vspeed.virtuose_speed.linear.x = speed[0];
    vspeed.virtuose_speed.linear.y = speed[1];
    vspeed.virtuose_speed.linear.z = speed[2];
    vspeed.virtuose_speed.angular.x = speed[3];
    vspeed.virtuose_speed.angular.y = speed[4];
    vspeed.virtuose_speed.angular.z = speed[5];
    _out_virtuose_speed->publish(vspeed);
  }
  // Status
  virtuose_ros2::msg::OutVirtuoseStatus status;
  status.header.stamp = _node->get_clock()->now();
  status.header.frame_id = "/Admittance";
  status.state = _state = STATE_CART_ADM;
  int button[32] = {0};

  // 手柄按钮
  unsigned int tor_inputs = 0;
  int tor_result = virtGetTorInputs(VC, &tor_inputs);
  if (tor_result == VIRT_E_NO_ERROR) {
      // 打印 tor_inputs 的二进制表示，按下不同按钮观察哪一位发生变化
      // printf("Tor Inputs: %u (Hex: 0x%X)\n", tor_inputs, tor_inputs);
      
      // --- 新增：发布布尔量 ---
      std_msgs::msg::Bool bool_0_msg, bool_1_msg, bool_2_msg, bool_3_msg;
      
      bool_0_msg.data = ((tor_inputs & virtuose_ros2::handle_mapping::kButtonAMask) != 0);  // 按钮A
      bool_1_msg.data = ((tor_inputs & virtuose_ros2::handle_mapping::kButtonBMask) != 0);  // 按钮B
      bool_2_msg.data = ((tor_inputs & virtuose_ros2::handle_mapping::kTriggerAMask) != 0);  // 拨杆A
      bool_3_msg.data = ((tor_inputs & virtuose_ros2::handle_mapping::kTriggerBMask) != 0);  // 拨杆B
      
      _out_handle_bool_0->publish(bool_0_msg);
      _out_handle_bool_1->publish(bool_1_msg);
      _out_handle_bool_2->publish(bool_2_msg);
      _out_handle_bool_3->publish(bool_3_msg);
      // ----------------------
  }
  
  // 机械臂按钮
  status.buttons = 0;
  for (int i = 0; i < 32; i++) {
    virtGetButton(VC, i, &button[i]);

    status.buttons += (button[i] == 1) ? (1 << i) : 0;
    // if (button[i] == 1)
    //   printf("Button %d: %d\t", i, button[i]);
  }
  // printf("\n");

  // 模拟量获取
  float analogValues_0[8]; 
  int result = virtGetAnalogicInputs(VC, analogValues_0);
  if (result == VIRT_E_NO_ERROR) {
      // 创建 Float32 消息
      std_msgs::msg::Float32 analog_msg;
      
      // 只赋值第 0 位
      analog_msg.data = analogValues_0[0];
      
      // 发布
      _out_handle_analog->publish(analog_msg);
      
      // 如果需要调试打印
      // printf("Published Analog Channel 0: %f\n", analog_msg.data);
  }
  int emergency_stop;
  if (virtGetEmergencyStop(VC, &emergency_stop))
  {
    printf("Error in virtGetEmergencyStop with code %d, go to ERROR\n",
        virtGetErrorCode(VC));
    virtSetPowerOn(VC, 0);
    virtStopLoop(VC);
    _state = STATE_ERROR;
    return;
  }
  status.emergency_stop = (emergency_stop == 1);
  _out_virtuose_status->publish(status);
  // Watchdog on force
  if (((now - start_loop_date) > 30000000) && (now > set_force_date) && ((now - set_force_date) > 30000000))
  {
    printf("Timeout on force, going to state ERROR\n");
    virtSetPowerOn(VC, 0);
    virtStopLoop(VC);
    _state = STATE_ERROR;
    return;
  }
  // Set force
  if (virtSetForce(VC, set_force))
  {
    printf("Error in virtSetForce with code %d, go to ERROR\n",
        virtGetErrorCode(VC));
    virtSetPowerOn(VC, 0);
    virtStopLoop(VC);
    _state = STATE_ERROR;
    return;
  }
}

// Callback for service virtuose_impedance
void impedanceCB(
  const std::shared_ptr<virtuose_ros2::srv::VirtuoseImpedance::Request> request,
  std::shared_ptr<virtuose_ros2::srv::VirtuoseImpedance::Response> response)
{
  printf("Received impedance request\n");
  // Check state
  if (_state != STATE_READY)
  {
    printf("Not in state READY, go to ERROR\n");
    _state = STATE_ERROR;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    return;
  }
  // Open connection
  _VC = virtOpen(request->ip_address.c_str());
  if (_VC == NULL)
  {
    fprintf(
      stderr,
      "[virtuose] virtOpen(\"%s\") 返回 NULL（阻抗模式）\n",
      request->ip_address.c_str());
    fflush(stderr);
    if (_node) {
      RCLCPP_ERROR(
        _node->get_logger(), "virtOpen failed (impedance) for [%s]", request->ip_address.c_str());
    }
    _state = STATE_ERROR;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    return;
  }
  const int pe_imp = request->power_enable ? 1 : 0;
  const VirtIndexingType idx_imp = clamp_indexing_mode(request->indexing_mode);
  if (virtSetIndexingMode(_VC, idx_imp) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetIndexingMode", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtSetForceFactor(_VC, request->force_factor) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetForceFactor", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtSetSpeedFactor(_VC, request->speed_factor) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetSpeedFactor", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtEnableForceFeedback(_VC, pe_imp) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtEnableForceFeedback", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtSetPowerOn(_VC, pe_imp) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetPowerOn", _VC);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtSetCommandType(_VC, COMMAND_TYPE_IMPEDANCE) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetCommandType", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtSaturateTorque(_VC, request->max_force, request->max_torque) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSaturateTorque", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  float base[7];
  base[0] = request->base_frame.translation.x;
  base[1] = request->base_frame.translation.y;
  base[2] = request->base_frame.translation.z;
  base[3] = request->base_frame.rotation.x;
  base[4] = request->base_frame.rotation.y;
  base[5] = request->base_frame.rotation.z;
  base[6] = request->base_frame.rotation.w;
  if (virtSetBaseFrame(_VC, base) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetBaseFrame", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  if (virtSetPeriodicFunction(_VC, virtImpedanceCB, &g_impedance_period_s, NULL) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtSetPeriodicFunction", _VC);
    virtSetPowerOn(_VC, 0);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }
  start_loop_date = _node->get_clock()->now().nanoseconds();
  if (virtStartLoop(_VC) != VIRT_E_NO_ERROR) {
    log_virtuose_failure("virtStartLoop", _VC);
    virtSetPowerOn(_VC, 0);
    virtStopLoop(_VC);
    virtClose(_VC);
    _VC = NULL;
    response->success = false;
    response->error = true;
    response->client_id = 0;
    _state = STATE_ERROR;
    return;
  }

  _state = STATE_CART_IMP;
  response->success = true;
  response->error = false;
  response->client_id = generate_client_id();
  printf("Switching to state CART_IMP\n");
}

// Call back for service virtuose_reset
void resetCB(
  const std::shared_ptr<virtuose_ros2::srv::VirtuoseReset::Request> request,
  std::shared_ptr<virtuose_ros2::srv::VirtuoseReset::Response> response)
{
  (void)request;
  (void)response;
  printf("Received Reset request\n");
  if (_VC != NULL)
  {
    virtSetPowerOn(_VC, 0);
    virtStopLoop(_VC);
    virtClose(_VC);
    set_pose_date = 0;
    set_speed_date = 0;
    set_force_date = 0;
    _VC = NULL;
  }
  _state = STATE_READY;
  _client_id = 0;
  printf("Switching to state READY\n");
}

// Main function
int main(int argc, char **argv)
{
  printf("Starting virtuose node\n");
  rclcpp::init(argc, argv);
  _node = std::make_shared<rclcpp::Node>("virtuose");

  printf("Creating topics\n");
  _out_virtuose_status =
    _node->create_publisher<virtuose_ros2::msg::OutVirtuoseStatus>("out_virtuose_status", 10);
  _out_virtuose_pose =
    _node->create_publisher<virtuose_ros2::msg::OutVirtuosePose>("out_virtuose_pose", 10);
  _out_virtuose_physical_pose =
    _node->create_publisher<virtuose_ros2::msg::OutVirtuosePhysicalPose>("out_virtuose_physical_pose", 10);
  _out_virtuose_speed =
    _node->create_publisher<virtuose_ros2::msg::OutVirtuoseSpeed>("out_virtuose_speed", 10);
  _out_virtuose_force =
    _node->create_publisher<virtuose_ros2::msg::OutVirtuoseForce>("out_virtuose_force", 10);
  // 新增：初始化模拟量和布尔量发布者
  _out_handle_analog =
    _node->create_publisher<std_msgs::msg::Float32>(virtuose_ros2::handle_mapping::kHandleAnalogTopic, 10);
  _out_handle_bool_0 =
    _node->create_publisher<std_msgs::msg::Bool>(virtuose_ros2::handle_mapping::kButtonATopic, 10);
  _out_handle_bool_1 =
    _node->create_publisher<std_msgs::msg::Bool>(virtuose_ros2::handle_mapping::kButtonBTopic, 10);
  _out_handle_bool_2 =
    _node->create_publisher<std_msgs::msg::Bool>(virtuose_ros2::handle_mapping::kTriggerATopic, 10);
  _out_handle_bool_3 =
    _node->create_publisher<std_msgs::msg::Bool>(virtuose_ros2::handle_mapping::kTriggerBTopic, 10);

  auto sub_pose = _node->create_subscription<virtuose_ros2::msg::InVirtuosePose>(
    "in_virtuose_pose", 10, in_virtuose_poseCB);
  auto sub_speed = _node->create_subscription<virtuose_ros2::msg::InVirtuoseSpeed>(
    "in_virtuose_speed", 10, in_virtuose_speedCB);
  auto sub_force = _node->create_subscription<virtuose_ros2::msg::InVirtuoseForce>(
    "in_virtuose_force", 10, in_virtuose_forceCB);
  (void)sub_pose;
  (void)sub_speed;
  (void)sub_force;

  printf("Creating services\n");
  auto srv_calibrate = _node->create_service<virtuose_ros2::srv::VirtuoseCalibrate>(
    "virtuose_calibrate", calibrateCB);
  auto srv_admittance = _node->create_service<virtuose_ros2::srv::VirtuoseAdmittance>(
    "virtuose_admittance", admittanceCB);
  auto srv_impedance = _node->create_service<virtuose_ros2::srv::VirtuoseImpedance>(
    "virtuose_impedance", impedanceCB);
  auto srv_reset = _node->create_service<virtuose_ros2::srv::VirtuoseReset>(
    "virtuose_reset", resetCB);
  (void)srv_calibrate;
  (void)srv_admittance;
  (void)srv_impedance;
  (void)srv_reset;

  printf("Publishing initial status\n");
  virtuose_ros2::msg::OutVirtuoseStatus status;
  status.header.stamp = _node->get_clock()->now();
  status.header.frame_id = "/Ready";
  status.state = _state = STATE_READY;
  _out_virtuose_status->publish(status);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(_node);
  std::thread spin_thread([&exec]() { exec.spin(); });

  rclcpp::Rate rate(100);
  while (rclcpp::ok())
  {
    if (_state == STATE_ERROR)
    {
      if (_VC != NULL)
      {
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
    rate.sleep();
  }

  exec.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
