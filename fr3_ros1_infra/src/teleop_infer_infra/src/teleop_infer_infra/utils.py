"""
Utility functions for teleoperation and inference infrastructure.

This module provides common utilities for coordinate transformations,
ROS message handling, configuration management, and logging.

Key Features:
- Coordinate transformations using scipy.spatial.transform.Rotation
- ROS message conversion utilities
- Configuration loading from ROS parameters and YAML files
- Logging utilities with statistics tracking
- Error handling and validation functions
"""

import rospy
import numpy as np
import yaml
import threading
import time
from typing import Dict, List, Tuple, Optional, Any, Union
from scipy.spatial.transform import Rotation as R

# ROS message imports
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from franka_msgs.msg import FrankaState
from std_msgs.msg import Header


class TransformUtils:
    """Utility class for coordinate transformations and pose manipulations."""
    
    @staticmethod
    def pose_to_numpy(pose: Pose) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convert ROS Pose message to numpy arrays.
        
        Args:
            pose: geometry_msgs/Pose message
            
        Returns:
            Tuple of (position_array, quaternion_array)
            position_array: np.ndarray shape (3,) [x, y, z]
            quaternion_array: np.ndarray shape (4,) [x, y, z, w]
        """
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        quaternion = np.array([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        return position, quaternion
    
    @staticmethod
    def numpy_to_pose(position: np.ndarray, quaternion: np.ndarray) -> Pose:
        """
        Convert numpy arrays to ROS Pose message.
        
        Args:
            position: np.ndarray shape (3,) [x, y, z]
            quaternion: np.ndarray shape (4,) [x, y, z, w]
            
        Returns:
            geometry_msgs/Pose message
        """
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose
    
    @staticmethod
    def franka_state_to_pose(state: FrankaState) -> Tuple[np.ndarray, np.ndarray]:
        """
        Extract pose from FrankaState message.
        
        Args:
            state: franka_msgs/FrankaState message
            
        Returns:
            Tuple of (position_array, quaternion_array)
            position_array: np.ndarray shape (3,) [x, y, z]
            quaternion_array: np.ndarray shape (4,) [x, y, z, w]
        """
        # Extract transform matrix O_T_EE (16 elements, row-major)
        transform_matrix = np.reshape(state.O_T_EE, (4, 4))
        
        # Extract position (last column of transform matrix)
        position = transform_matrix[:3, 3]
        
        # Extract rotation matrix and convert to quaternion
        rotation_matrix = transform_matrix[:3, :3]
        quaternion = R.from_matrix(rotation_matrix).as_quat()  # [x, y, z, w]
        
        return position, quaternion
    
    @staticmethod
    def compute_pose_delta(current_pos: np.ndarray, current_quat: np.ndarray,
                          target_pos: np.ndarray, target_quat: np.ndarray) -> np.ndarray:
        """
        Compute 6D delta between current and target poses.
        
        Args:
            current_pos: Current position [x, y, z]
            current_quat: Current quaternion [x, y, z, w]
            target_pos: Target position [x, y, z]
            target_quat: Target quaternion [x, y, z, w]
            
        Returns:
            6D delta vector [dx, dy, dz, drx, dry, drz]
            where rotation is expressed as rotation vector in world frame
        """
        # Position delta
        pos_delta = target_pos - current_pos
        
        # Rotation delta
        current_rot = R.from_quat(current_quat)
        target_rot = R.from_quat(target_quat)
        
        # Relative rotation in body frame
        rel_rot_body = current_rot.inv() * target_rot
        rotvec_body = rel_rot_body.as_rotvec()
        
        # Convert to world frame
        rotvec_world = current_rot.as_matrix() @ rotvec_body
        
        return np.concatenate([pos_delta, rotvec_world])
    
    @staticmethod
    def apply_pose_delta(current_pos: np.ndarray, current_quat: np.ndarray,
                        delta: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Apply 6D delta to current pose to get target pose.
        
        Args:
            current_pos: Current position [x, y, z]
            current_quat: Current quaternion [x, y, z, w]
            delta: 6D delta vector [dx, dy, dz, drx, dry, drz]
                  where rotation is expressed as rotation vector in world frame
                  
        Returns:
            Tuple of (target_position, target_quaternion)
        """
        # Position update
        target_pos = current_pos + delta[:3]
        
        # Rotation update
        current_rot = R.from_quat(current_quat)
        
        # Convert rotation vector from world to body frame
        rotvec_world = delta[3:]
        rotvec_body = current_rot.as_matrix().T @ rotvec_world
        
        # Apply rotation delta
        delta_rot = R.from_rotvec(rotvec_body)
        target_rot = current_rot * delta_rot
        target_quat = target_rot.as_quat()
        
        return target_pos, target_quat
    
    @staticmethod
    def normalize_quaternion(quat: np.ndarray) -> np.ndarray:
        """
        Normalize quaternion to unit length.
        
        Args:
            quat: Quaternion array [x, y, z, w]
            
        Returns:
            Normalized quaternion
        """
        norm = np.linalg.norm(quat)
        if norm > 1e-8:
            return quat / norm
        else:
            return np.array([0.0, 0.0, 0.0, 1.0])  # Default identity quaternion


class ConfigManager:
    """Manager for configuration loading and parameter handling."""
    
    def __init__(self, node_name: str = "teleop_infer_infra"):
        """
        Initialize configuration manager.
        
        Args:
            node_name: ROS node name for parameter namespace
        """
        self.node_name = node_name
        self.config_lock = threading.Lock()
        self.config_cache = {}
        
    def get_param(self, param_name: str, default: Any = None) -> Any:
        """
        Get parameter from ROS parameter server with caching.
        
        Args:
            param_name: Parameter name (can be namespaced)
            default: Default value if parameter doesn't exist
            
        Returns:
            Parameter value
        """
        full_param_name = f"/{self.node_name}/{param_name}" if not param_name.startswith("/") else param_name
        
        with self.config_lock:
            if full_param_name in self.config_cache:
                return self.config_cache[full_param_name]
            
            if rospy.has_param(full_param_name):
                value = rospy.get_param(full_param_name)
                self.config_cache[full_param_name] = value
                return value
            else:
                rospy.logdebug(f"Parameter {full_param_name} not found, using default: {default}")
                return default
    
    def load_yaml_config(self, file_path: str) -> Dict[str, Any]:
        """
        Load configuration from YAML file.
        
        Args:
            file_path: Path to YAML configuration file
            
        Returns:
            Dictionary containing configuration
        """
        try:
            with open(file_path, 'r') as f:
                config = yaml.safe_load(f)
            rospy.loginfo(f"Loaded configuration from {file_path}")
            return config or {}
        except Exception as e:
            rospy.logwarn(f"Failed to load YAML config from {file_path}: {e}")
            return {}
    
    def update_param(self, param_name: str, value: Any) -> bool:
        """
        Update parameter in ROS parameter server and cache.
        
        Args:
            param_name: Parameter name
            value: New parameter value
            
        Returns:
            True if successful, False otherwise
        """
        full_param_name = f"/{self.node_name}/{param_name}" if not param_name.startswith("/") else param_name
        
        try:
            rospy.set_param(full_param_name, value)
            with self.config_lock:
                self.config_cache[full_param_name] = value
            rospy.logdebug(f"Updated parameter {full_param_name} = {value}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to update parameter {full_param_name}: {e}")
            return False
    
    def clear_cache(self):
        """Clear configuration cache."""
        with self.config_lock:
            self.config_cache.clear()
            rospy.logdebug("Configuration cache cleared")


class LoggingUtils:
    """Utilities for logging and statistics tracking."""
    
    def __init__(self, node_name: str = "teleop_infer_infra"):
        """
        Initialize logging utilities.
        
        Args:
            node_name: ROS node name for log messages
        """
        self.node_name = node_name
        self.stats_lock = threading.Lock()
        self.message_counts = {}
        self.timing_stats = {}
        self.start_time = time.time()
        
    def log_statistics(self, interval: float = 10.0):
        """
        Log statistics at regular intervals.
        
        Args:
            interval: Logging interval in seconds
        """
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed >= interval:
            with self.stats_lock:
                stats_msg = f"[{self.node_name}] Statistics (last {interval:.1f}s):\n"
                for msg_type, count in self.message_counts.items():
                    rate = count / elapsed
                    stats_msg += f"  {msg_type}: {count} messages ({rate:.1f} Hz)\n"
                
                for stat_name, values in self.timing_stats.items():
                    if values:
                        avg_time = np.mean(values)
                        stats_msg += f"  {stat_name}: avg {avg_time*1000:.1f} ms\n"
                
                rospy.loginfo(stats_msg)
                
                # Reset for next interval
                self.message_counts.clear()
                self.timing_stats.clear()
                self.start_time = current_time
    
    def count_message(self, msg_type: str):
        """
        Increment message count for statistics.
        
        Args:
            msg_type: Type of message (e.g., 'pose', 'state', 'action')
        """
        with self.stats_lock:
            self.message_counts[msg_type] = self.message_counts.get(msg_type, 0) + 1
    
    def record_timing(self, stat_name: str, duration: float):
        """
        Record timing information for performance monitoring.
        
        Args:
            stat_name: Name of the timing statistic
            duration: Duration in seconds
        """
        with self.stats_lock:
            if stat_name not in self.timing_stats:
                self.timing_stats[stat_name] = []
            self.timing_stats[stat_name].append(duration)
            # Keep only last 100 samples
            if len(self.timing_stats[stat_name]) > 100:
                self.timing_stats[stat_name] = self.timing_stats[stat_name][-100:]
    
    def log_performance(self, operation_name: str, start_time: float):
        """
        Log performance of an operation.
        
        Args:
            operation_name: Name of the operation
            start_time: Start time from time.time()
        """
        duration = time.time() - start_time
        self.record_timing(operation_name, duration)
        
        if duration > 0.1:  # Log warnings for slow operations
            rospy.logwarn(f"Slow operation {operation_name}: {duration*1000:.1f} ms")


class ValidationUtils:
    """Utilities for data validation and safety checks."""
    
    @staticmethod
    def validate_pose(position: np.ndarray, quaternion: np.ndarray,
                     position_limits: Optional[List[Tuple[float, float]]] = None,
                     max_velocity: float = 0.5) -> Tuple[bool, str]:
        """
        Validate pose for safety.
        
        Args:
            position: Position array [x, y, z]
            quaternion: Quaternion array [x, y, z, w]
            position_limits: Optional list of (min, max) for each axis
            max_velocity: Maximum allowed velocity (m/s) for position change
            
        Returns:
            Tuple of (is_valid, error_message)
        """
        # Check position limits
        if position_limits is not None:
            for i, (min_val, max_val) in enumerate(position_limits):
                if not (min_val <= position[i] <= max_val):
                    return False, f"Position axis {i} out of bounds: {position[i]} not in [{min_val}, {max_val}]"
        
        # Check quaternion normalization
        quat_norm = np.linalg.norm(quaternion)
        if abs(quat_norm - 1.0) > 0.01:
            return False, f"Quaternion not normalized: norm = {quat_norm}"
        
        # Check for NaN or Inf values
        if np.any(np.isnan(position)) or np.any(np.isinf(position)):
            return False, "Position contains NaN or Inf values"
        
        if np.any(np.isnan(quaternion)) or np.any(np.isinf(quaternion)):
            return False, "Quaternion contains NaN or Inf values"
        
        return True, "Pose is valid"
    
    @staticmethod
    def clamp_pose(position: np.ndarray, quaternion: np.ndarray,
                  position_limits: List[Tuple[float, float]]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Clamp pose to within position limits.
        
        Args:
            position: Position array [x, y, z]
            quaternion: Quaternion array [x, y, z, w]
            position_limits: List of (min, max) for each axis
            
        Returns:
            Tuple of (clamped_position, normalized_quaternion)
        """
        # Clamp position
        clamped_position = np.copy(position)
        for i, (min_val, max_val) in enumerate(position_limits):
            clamped_position[i] = np.clip(position[i], min_val, max_val)
        
        # Normalize quaternion
        normalized_quat = TransformUtils.normalize_quaternion(quaternion)
        
        return clamped_position, normalized_quat


# Global instances for convenience
transform_utils = TransformUtils()
config_manager = ConfigManager()
logging_utils = LoggingUtils()
validation_utils = ValidationUtils()
