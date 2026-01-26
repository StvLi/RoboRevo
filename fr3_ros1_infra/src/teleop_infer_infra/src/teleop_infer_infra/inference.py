"""
Model inference client for robot control.
北京智源人工智能研究院 具身大模型
李佩泽 TEL:138 1085 0696 E-mail: 3190102290@zju.edu.cn

This module provides inference functionality with support for:
- HTTP REST API communication with inference server
- Image acquisition and preprocessing
- Action prediction and processing
- Integration with robot environment via get_action() interface

Key Classes:
- InferenceClient: Manages inference server communication and action generation
  Follows similar structure to Haption6DExpert for compatibility with teleoperation framework

Design Principles:
1. Compatible interface: get_action() method returns (action, buttons) like Haption6DExpert
2. Server communication: HTTP POST requests with Base64 encoded images
3. Action processing: Converts server predictions to robot-executable actions
4. Error handling: Graceful degradation and fallback modes
"""

import rospy
import numpy as np
import requests
import cv2
import base64
import threading
import time
import copy
from typing import Dict, List, Tuple, Optional, Any
from scipy.spatial.transform import Rotation as R
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from utils import transform_utils, config_manager, logging_utils, validation_utils
class InferenceClient:
    """
    Inference client for model-based robot control.
    
    This class follows a similar structure to Haption6DExpert for compatibility
    with the teleoperation framework. It communicates with an inference server
    to get action predictions based on images and robot state.
    
    Key Features:
    - HTTP REST API communication with inference server
    - Image acquisition and Base64 encoding
    - Action prediction processing
    - Mock mode for testing without hardware
    - get_action() interface compatible with teleoperation agents
    """
    
    def __init__(self, debug: bool = True, use_mock: bool = False):
        """
        Initialize InferenceClient.
        
        Args:
            debug: Enable debug logging
            use_mock: Use mock mode (generate random actions) for testing
        """
        self.debug = debug
        self.use_mock = use_mock
        
        # Configuration
        self.config = config_manager
        
        # Load server configuration
        self.server_ip = self.config.get_param("inference/server_ip", "127.0.0.1")
        self.server_port = self.config.get_param("inference/server_port", 5003)
        self.model_url = f"http://{self.server_ip}:{self.server_port}/predict_action"
        
        # Image parameters
        self.img_width = self.config.get_param("inference/img_width", 640)
        self.img_height = self.config.get_param("inference/img_height", 480)
        
        # Action parameters
        self.exec_steps = self.config.get_param("inference/exec_steps", 5)
        self.action_horizon = self.config.get_param("inference/action_horizon", 30)
        
        # State management (similar to Haption6DExpert structure)
        self.fsm = "idle"  # State machine: idle, inference, execution
        self.current_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 6D action [dx, dy, dz, drx, dry, drz]
        self.current_buttons = np.array([False, False, False])  # 3 buttons for compatibility with SimplifiedFrankaEnv
        self.last_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Inference state
        self.action_buffer = []  # Buffer for action sequence from server
        self.buffer_index = 0  # Current index in action buffer
        self.last_inference_time = time.time()  
    
        self.inference_interval = self.config.get_param("inference/interval", 0.2)  # Seconds
        # self.last_inference_time = 0.0
        # self.inference_interval = self.config.get_param("inference/interval", 0.2)  # Seconds
        
        # Async request state
        self.has_pending_request = False  # Whether a request has been sent and waiting for response
        self.has_received_chunk = False  # Whether a chunk has been received from server
        self.request_thread = None  # Thread for async HTTP request
        
        # Data locks for thread safety
        self.data_lock = threading.Lock()
        
        # Statistics
        self.inference_count = 0
        self.action_count = 0
        self.error_count = 0
        
        # Mock state tracking
        self.mock_chunk_index = 0  # Track which chunk we're on for mock mode
        
        # Image sources (to be set by external code)
        self.image_sources = None  # Should be a callable that returns list of images
        
        # Robot state (to be provided via get_action or separate method)
        self.current_robot_state = None  # [x, y, z, qx, qy, qz, qw, gripper_width]
        
        # HTTP session for connection pooling
        self.session = requests.Session()
        self.session.timeout = self.config.get_param("inference/timeout", 30.0)
        
        rospy.loginfo(f"InferenceClient initialized: server={self.model_url}, mock={use_mock}")
        if self.use_mock:
            rospy.loginfo("Running in MOCK mode - generating safe square pattern actions")
    
    def set_image_sources(self, image_source_func):
        """
        Set function for acquiring images.
        
        Args:
            image_source_func: Callable that returns list of numpy images (RGB format)
        """
        self.image_sources = image_source_func
        rospy.logdebug("Image source function set")
    
    def set_robot_state(self, robot_state: np.ndarray):
        """
        Set current robot state.
        
        Args:
            robot_state: np.ndarray shape (8,) [x, y, z, qx, qy, qz, qw, gripper_width]
        """
        with self.data_lock:
            self.current_robot_state = robot_state.copy()
    
    def get_action(self, obs: Optional[np.ndarray] = None) -> Tuple[np.ndarray, List[bool]]:
        """
        Get current action (compatible with Haption6DExpert interface).
        
        Args:
            obs: Observation array (optional, for compatibility)
            
        Returns:
            Tuple of (action_vector, button_state)
            action_vector: 6D action [dx, dy, dz, drx, dry, drz]
            button_state: List with 3 booleans [button0, button1, button2]
            - button1 is the enable flag for SimplifiedFrankaEnv (True during execution)
        """
        # Update state machine based on current conditions
        self.update_fsm()
        if self.debug and self.fsm == "inference":
            rospy.logdebug("FSM 状态瞬时切换: INFERENCE (准备发送请求)")
        # Handle different states
        if self.fsm == "inference":
            # Inference state: send request and return zero action
            # Note: perform_inference will be called, which sends request and updates state
            self.perform_inference()
            # Return zero action during inference
            self.current_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            
        elif self.fsm == "execution":
            # Execution state: output actions from buffer
            self.update_action_from_buffer()
            
        else:  # idle
            # Idle state: return zero action
            self.current_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Update button state (3 buttons for compatibility with SimplifiedFrankaEnv)
        # button1 is the enable flag (True during execution, False otherwise)
        if self.fsm == "execution":
            self.current_buttons = np.array([False, True, False])
        else:
            self.current_buttons = np.array([False, False, False])
        
        # Update statistics
        self.action_count += 1
        if self.debug and self.action_count % 100 == 0:
            rospy.logdebug(f"InferenceClient: Generated {self.action_count} actions")
        
        return self.current_action.copy(), [bool(self.current_buttons[0]), 
                                           bool(self.current_buttons[1]), 
                                           bool(self.current_buttons[2])]
    
    def update_fsm(self):
        """
        Update finite state machine according to the new design:
        
        State transitions:
        1. idle -> inference: When time interval reached and image sources available
        2. inference -> idle: Immediately after sending request (handled in perform_inference)
        3. idle -> execution: When chunk received from server (has_received_chunk = True)
        4. execution -> idle: When all actions in chunk have been executed
        
        Design purpose:
        - inference state only sends request, doesn't execute actions
        - Avoid blocking in clock callback by using async request handling
        - Pipeline execution: prepare next request while executing current chunk
        - Clear separation: idle decides what to do next based on conditions
        """
        current_time = time.time()
        
        if self.fsm == "idle":
            # In idle state, decide what to do next
            
            # clean buffer index
            self.buffer_index = 0

            # First priority: if we have received a chunk, start execution
            if self.has_received_chunk:
                self.fsm = "execution"
                self.has_received_chunk = False  # Reset flag
                if self.debug:
                    rospy.logdebug("FSM: idle -> execution (chunk received)")
            
            # Second priority: if time interval reached and conditions met, send inference request
            # Modified: Allow inference even without image sources for testing
            elif (current_time - self.last_inference_time > self.inference_interval and
                #   self.image_sources is not None and
                  not self.has_pending_request):
                
                self.fsm = "inference"
                if self.debug:
                    rospy.logdebug("FSM: idle -> inference (sending request)")
        
        elif self.fsm == "inference":
            # Inference state: send request and immediately go back to idle
            # This transition happens in perform_inference after sending request
            # We don't do anything here, just wait for perform_inference to be called

            # clean buffer index
            self.buffer_index = 0

        elif self.fsm == "execution":
            # Check if buffer is exhausted
            if self.buffer_index >= len(self.action_buffer):
                # Buffer exhausted, go back to idle
                self.current_buttons[2] = True # 手动重置
                self.fsm = "idle"
                if self.debug:
                    rospy.logdebug("FSM: execution -> idle (buffer exhausted)")
    
    def perform_inference(self):
        """
        Perform inference with server and update action buffer.
        
        This method:
        1. Sets has_pending_request flag
        2. Acquires images from image sources
        3. Encodes images for transmission
        4. Sends request to inference server (synchronous for simplicity)
        5. Parses response and updates action buffer
        6. Sets has_received_chunk flag and transitions back to idle
        
        Note: In inference state, this method sends request and immediately
        transitions back to idle state. The received chunk will trigger
        transition to execution state in the next update_fsm call.
        """
        # Mark that we're sending a request
        self.has_pending_request = True
        
        # Immediately transition back to idle (as per design)
        self.fsm = "idle"
        if self.debug:
            rospy.logdebug("FSM: inference -> idle (request sent)")
        
        if self.use_mock:
            # For mock mode, generate actions immediately
            self._generate_mock_actions()
            self.has_pending_request = False
            self.has_received_chunk = True
            return
        
        try:
            # 1. Acquire images
            start_time = time.time()
            # images_np = self.image_sources()
            # if not images_np or len(images_np) == 0:
            #     rospy.logwarn("No images available for inference")
            #     self._generate_mock_actions()
            #     self.has_pending_request = False
            #     self.has_received_chunk = True
            #     return
            
            # 2. Encode images
            # images_b64 = [self._encode_image_to_base64(img) for img in images_np]
            
            # 3. Prepare request payload
            # Note: state is set to None as in reference implementation
            # Server will fill with zeros if state is None
            payload = {
                "examples": [
                    {
                        # "image": images_b64,
                        "image": None,
                        "lang": "Perform the task",  # Default instruction
                        "state": None  # Server will fill with zeros
                    }
                ]
            }
            
            # 4. Send request to server (synchronous)
            resp = self.session.post(self.model_url, json=payload, timeout=30.0)
            
            # 5. Check response
            if resp.status_code != 200:
                rospy.logwarn(f"Server error {resp.status_code}: {resp.text}")
                self._generate_mock_actions()
                self.has_pending_request = False
                self.has_received_chunk = True
                return
            
            # 6. Parse response
            result = resp.json()
            all_actions = np.array(result["data"]["unnormalized_actions"])
            
            # Extract action sequence (first sample, first exec_steps)
            # action_sequence = all_actions[0, :self.exec_steps, :]
            action_sequence = all_actions[0, :, :]

            # 7. Update action buffer
            with self.data_lock:
                self.action_buffer = action_sequence.tolist()
                self.buffer_index = 0
            
            # Update statistics
            self.inference_count += 1
            self.last_inference_time = time.time()
            inference_time = time.time() - start_time
            
            # Mark that we've received a chunk
            self.has_pending_request = False
            self.has_received_chunk = True
            
            if self.debug:
                rospy.loginfo(f"Inference successful: {len(action_sequence)} actions, "
                            f"time={inference_time*1000:.1f}ms")
                logging_utils.record_timing("inference", inference_time)
            
        except requests.exceptions.Timeout:
            rospy.logwarn("Inference request timeout")
            self.error_count += 1
            self._generate_mock_actions()
            self.has_pending_request = False
            self.has_received_chunk = True
        except requests.exceptions.ConnectionError:
            rospy.logwarn("Cannot connect to inference server")
            self.error_count += 1
            self._generate_mock_actions()
            self.has_pending_request = False
            self.has_received_chunk = True
        except Exception as e:
            rospy.logerr(f"Inference error: {e}")
            self.error_count += 1
            self._generate_mock_actions()
            self.has_pending_request = False
            self.has_received_chunk = True
    
    def update_action_from_buffer(self):
        """
        Update current action from buffer.
        
        Takes the next action from the buffer and converts it from
        server format (7D: dx,dy,dz,rx,ry,rz,gripper) to robot format
        (6D: dx,dy,dz,drx,dry,drz).
        """
        if not self.action_buffer or self.buffer_index >= len(self.action_buffer):
            self.current_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            return
        
        # Get next action from buffer
        with self.data_lock:
            server_action = self.action_buffer[self.buffer_index]
            self.buffer_index += 1
        
        # Convert from server format to robot format
        # Server format: [delta_x_local, delta_y_local, delta_z_local, 
        #                 rot_x, rot_y, rot_z, gripper_prob]
        # Robot format: [dx, dy, dz, drx, dry, drz] (world frame)
        
        # if self.current_robot_state is not None:
        #     # Convert using current robot state
        #     self.current_action = self._convert_server_action_to_robot_action(
        #         server_action, self.current_robot_state
        #     )
        # else:
        #     # No robot state available, use server action directly (without coordinate conversion)
        #     # This is a simplified conversion - assumes world frame
        self.current_action = np.array([
            server_action[0],  # dx
            server_action[1],  # dy
            server_action[2],  # dz
            server_action[3],  # drx
            server_action[4],  # dry
            server_action[5]   # drz
        ])
    
        # Apply safety limits
        self.current_action = self._apply_action_limits(self.current_action)
        
        if self.debug and self.buffer_index % 5 == 0:
            rospy.logdebug(f"Action from buffer: {self.current_action}, "
                          f"index={self.buffer_index-1}/{len(self.action_buffer)}")
    
    def _convert_server_action_to_robot_action(self, server_action: List[float], 
                                              robot_state: np.ndarray) -> np.ndarray:
        # 待检查
        """
        Convert server action format to robot action format.
        
        Server action is in local frame (relative to current end-effector).
        Robot action needs to be in world frame.
        
        Args:
            server_action: 7D action from server
            robot_state: 8D robot state [x,y,z,qx,qy,qz,qw,gripper]
            
        Returns:
            6D action in world frame [dx, dy, dz, drx, dry, drz]
        """
        # Extract position and orientation from robot state
        curr_pos = robot_state[:3]
        curr_quat = robot_state[3:7]
        
        # Extract server action components
        delta_p_local = np.array(server_action[:3])  # Position delta in local frame
        delta_rot_vec = np.array(server_action[3:6])  # Rotation vector (rotation delta)
        
        # Convert position delta from local to world frame
        curr_rot = R.from_quat(curr_quat)
        delta_p_world = curr_rot.apply(delta_p_local)
        
        # Rotation delta is already in appropriate frame (server returns rotation vector)
        # We use it directly as rotation action
        
        # Combine into 6D action
        robot_action = np.concatenate([delta_p_world, delta_rot_vec])
        
        return robot_action
    
    def _apply_action_limits(self, action: np.ndarray) -> np.ndarray:
        """
        Apply safety limits to action.
        
        Args:
            action: 6D action vector
            
        Returns:
            Limited action vector
        """
        # Position limits (max step size in meters)
        max_pos_step = self.config.get_param("safety/max_pos_step", 0.05)
        # Rotation limits (max rotation in radians)
        max_rot_step = self.config.get_param("safety/max_rot_step", 0.1)
        
        limited_action = action.copy()
        
        # Limit position components
        pos_norm = np.linalg.norm(action[:3])
        if pos_norm > max_pos_step:
            scale = max_pos_step / pos_norm
            limited_action[:3] = action[:3] * scale
            if self.debug and pos_norm > max_pos_step * 1.1:
                rospy.logwarn(f"Position step limited: {pos_norm:.3f} > {max_pos_step:.3f}")
        
        # Limit rotation components
        rot_norm = np.linalg.norm(action[3:])
        if rot_norm > max_rot_step:
            scale = max_rot_step / rot_norm
            limited_action[3:] = action[3:] * scale
            if self.debug and rot_norm > max_rot_step * 1.1:
                rospy.logwarn(f"Rotation step limited: {rot_norm:.3f} > {max_rot_step:.3f}")
        
        return limited_action
    
    def _encode_image_to_base64(self, img_rgb: np.ndarray) -> str:
        """
        Encode RGB image to Base64 JPEG string.
        
        Args:
            img_rgb: numpy array, shape (H, W, 3), dtype uint8, RGB format
            
        Returns:
            Base64 encoded JPEG image string
        """
        # Convert RGB to BGR for OpenCV encoding
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        
        # Encode as JPEG with quality 95
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
        _, encimg = cv2.imencode('.jpg', img_bgr, encode_param)
        
        # Convert to Base64
        b64_string = base64.b64encode(encimg).decode('utf-8')
        
        return b64_string
    
    def _generate_mock_actions(self):
        """
        Generate mock actions for testing without server.
        
        Creates a sequence of safe, constrained actions for testing with real robot.
        Each chunk has length 5 with the following pattern:
        1. x +0.005
        2. y +0.005
        3. x -0.005
        4. y -0.005
        5. zero action (all zeros)
        
        This creates a small square pattern in XY plane for safety testing.
        """
        print("Using MOCK Actions")
        # Generate safe action sequence
        num_actions = self.exec_steps  # Should be 5
        mock_sequence = []
        
        # Define the safe action pattern
        safe_actions = [
            [0.000, 0.000, 0.050, 0.0, 0.0, 0.0, 0.0],  # x +0.005
            [0.000, 0.050, 0.000, 0.0, 0.0, 0.0, 0.0],  # y +0.005
            [0.000, 0.000,-0.050, 0.0, 0.0, 0.0, 0.0], # x -0.005
            [0.000, -0.05, 0.000, 0.0, 0.0, 0.0, 0.0], # y -0.005
            [0.000, 0.000, 0.000, 0.0, 0.0, 0.0, 0.0]   # zero action
        ]
        
        # If exec_steps is different from 5, adjust pattern
        for i in range(num_actions):
            if i < len(safe_actions):
                action = safe_actions[i]
            else:
                # Repeat pattern or use zero actions
                action = [0.000, 0.000, 0.000, 0.0, 0.0, 0.0, 0.0]
            mock_sequence.append(action)
        
        with self.data_lock:
            self.action_buffer = mock_sequence
            self.buffer_index = 0
        
        self.inference_count += 1
        self.last_inference_time = time.time()
        
        if self.debug:
            rospy.loginfo(f"Generated SAFE mock actions: {num_actions} steps (square pattern)")
    
    def get_current_action(self) -> np.ndarray:
        """
        Get current action vector.
        
        Returns:
            Current 6D action vector
        """
        return self.current_action.copy()
    
    def get_current_buttons(self) -> np.ndarray:
        """
        Get current button state.
        
        Returns:
            Current button state array
        """
        return self.current_buttons.copy()
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get inference statistics.
        
        Returns:
            Dictionary with statistics
        """
        return {
            "inference_count": self.inference_count,
            "action_count": self.action_count,
            "error_count": self.error_count,
            "buffer_size": len(self.action_buffer),
            "buffer_index": self.buffer_index,
            "fsm_state": self.fsm
        }
    
    def reset(self):
        """
        Reset inference client state.
        
        Clears action buffer and resets state machine.
        """
        with self.data_lock:
            self.action_buffer = []
            self.buffer_index = 0
            self.fsm = "idle"
            self.current_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.current_buttons = np.array([False, False, False])
            self.has_pending_request = False
            self.has_received_chunk = False
        
        rospy.loginfo("InferenceClient reset")
    
    def cleanup(self):
        """
        Clean up resources.
        
        Closes HTTP session and logs final statistics.
        """
        self.session.close()
        
        stats = self.get_statistics()
        rospy.loginfo(f"InferenceClient cleanup - Statistics: {stats}")
        
        if self.error_count > 0:
            rospy.logwarn(f"InferenceClient had {self.error_count} errors")


# Example usage and test
if __name__ == "__main__":
    # Test the InferenceClient
    rospy.init_node('inference_client_test')
    
    # Create inference client in mock mode
    client = InferenceClient(debug=True, use_mock=False)
    
    # Set up mock image source
    def mock_image_source():
        """Mock image source returning three random images."""
        return [
            np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
            np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
            np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        ]
    
    client.set_image_sources(mock_image_source)
    
    # Set mock robot state
    mock_robot_state = np.array([0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.08])
    client.set_robot_state(mock_robot_state)
    
    try:
        rate = rospy.Rate(10)  # 10Hz
        for i in range(50):  # Run for 5 seconds
            # Get action from inference client
            action, buttons = client.get_action()
            
            rospy.loginfo(f"Step {i}: Action={action}, Buttons={buttons}")
            
            # Print statistics every 10 steps
            if i % 10 == 0:
                stats = client.get_statistics()
                rospy.loginfo(f"Statistics: {stats}")
            
            rate.sleep()
            
    except KeyboardInterrupt:
        rospy.loginfo("Test interrupted")
    finally:
        client.cleanup()
        rospy.loginfo("InferenceClient test completed")


