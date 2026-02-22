"""
Type definitions and data structures for drone racing autonomy.
Emphasizes clarity and type safety.
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np


@dataclass
class Vec3:
    """3D vector with common operations."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])
    
    def magnitude(self) -> float:
        return np.sqrt(self.x**2 + self.y**2 + self.z**2)
    
    def normalize(self) -> 'Vec3':
        mag = self.magnitude()
        if mag < 1e-6:
            return Vec3(0, 0, 0)
        return Vec3(self.x / mag, self.y / mag, self.z / mag)
    
    def __add__(self, other: 'Vec3') -> 'Vec3':
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other: 'Vec3') -> 'Vec3':
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __mul__(self, scalar: float) -> 'Vec3':
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)
    
    def __rmul__(self, scalar: float) -> 'Vec3':
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)
    
    def __truediv__(self, scalar: float) -> 'Vec3':
        if scalar == 0:
            return Vec3(0, 0, 0)
        return Vec3(self.x / scalar, self.y / scalar, self.z / scalar)
    
    def dot(self, other: 'Vec3') -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z
    
    def cross(self, other: 'Vec3') -> 'Vec3':
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )


@dataclass
class DroneState:
    """Complete drone state from simulator."""
    timestamp: float  # Sim time in seconds
    
    # Position (WCS - World Coordinate System)
    position: Vec3 = field(default_factory=Vec3)
    
    # Velocity (WCS)
    velocity: Vec3 = field(default_factory=Vec3)
    
    # Orientation (Euler angles: roll, pitch, yaw in radians)
    attitude: Vec3 = field(default_factory=Vec3)  # roll, pitch, yaw
    
    # Angular velocity (body frame)
    angular_velocity: Vec3 = field(default_factory=Vec3)  # p, q, r
    
    # Computed/filtered values
    speed: float = 0.0  # Total velocity magnitude
    acceleration: Vec3 = field(default_factory=Vec3)  # Estimated from derivatives


@dataclass
class Gate:
    """Gate definition in the race track."""
    index: int  # Gate sequence number
    position: Vec3 = field(default_factory=Vec3)  # Gate center position
    orientation: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # Euler angles
    radius: float = 5.0  # Gate opening radius
    
    def contains_point(self, point: Vec3, margin: float = 0.5) -> bool:
        """Check if a point passes through the gate plane."""
        dist = (point - self.position).magnitude()
        return dist <= (self.radius + margin)


@dataclass
class PlanningResult:
    """Output from planning module."""
    next_gate_index: int  # Target gate to navigate toward
    target_position: Vec3  # Desired position for this step
    target_velocity: float  # Desired speed
    lookahead_gates: List[int] = field(default_factory=list)  # Next 2-3 gates for planning
    path_curvature: float = 0.0  # Estimated path curvature at target
    is_committed: bool = False  # True if fully committed to this gate


@dataclass
class ControlOutput:
    """Commands to send to simulator."""
    timestamp: float
    
    # Attitude commands (setpoints)
    roll_setpoint: float = 0.0  # radians
    pitch_setpoint: float = 0.0  # radians
    yaw_rate_setpoint: float = 0.0  # radians/sec
    
    # Thrust command
    thrust: float = 0.5  # [0, 1] normalized or [0, max_thrust]
    
    # Confidence/health
    is_valid: bool = True
    emergency_stop: bool = False


@dataclass
class RacingConfig:
    """All tunable parameters for the autonomy stack."""
    # Drone physical parameters
    max_roll: float = np.deg2rad(45.0)
    max_pitch: float = np.deg2rad(45.0)
    max_yaw_rate: float = np.deg2rad(360.0)
    max_velocity: float = 30.0  # m/s
    max_acceleration: float = 15.0  # m/s^2
    
    # Control gains
    # Position control PID
    pos_kp: float = 2.0
    pos_ki: float = 0.1
    pos_kd: float = 0.5
    
    # Velocity control PID
    vel_kp: float = 0.5
    vel_ki: float = 0.05
    vel_kd: float = 0.1
    
    # Attitude control PID
    att_kp: float = 3.0
    att_ki: float = 0.1
    att_kd: float = 0.3
    
    # Yaw control
    yaw_kp: float = 2.0
    yaw_kd: float = 0.2
    
    # Planning parameters
    planning_horizon: float = 5.0  # Look ahead 5 gates
    gate_commit_threshold: float = 2.0  # Distance to commit to next gate
    speed_margin: float = 0.85  # Apply 85% of max speed on curves
    
    # State filtering
    position_filter_alpha: float = 0.7  # Low-pass filter coefficient
    attitude_filter_alpha: float = 0.6
    
    # Timing
    control_loop_hz: float = 50.0
    planning_loop_hz: float = 20.0
    state_estimation_hz: float = 50.0
