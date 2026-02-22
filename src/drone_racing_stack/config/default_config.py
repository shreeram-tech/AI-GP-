"""
Default racing configuration.
This file can be modified for different tracks and aircraft.
"""

from src.types import RacingConfig
import numpy as np

# Default configuration (tuned for 8-gate circular track, quadrotor)
DEFAULT_CONFIG = RacingConfig(
    # Drone physical limits
    max_roll=np.deg2rad(45.0),
    max_pitch=np.deg2rad(45.0),
    max_yaw_rate=np.deg2rad(360.0),
    max_velocity=30.0,
    max_acceleration=15.0,
    
    # Position control (outer loop)
    # High gains for aggressive following
    pos_kp=2.5,
    pos_ki=0.15,
    pos_kd=0.8,
    
    # Velocity control (middle loop)
    # Medium gains to translate velocity errors to attitude
    vel_kp=0.6,
    vel_ki=0.08,
    vel_kd=0.15,
    
    # Attitude control (inner loop)
    # High gains for crisp response
    att_kp=4.0,
    att_ki=0.15,
    att_kd=0.4,
    
    # Yaw control
    yaw_kp=2.5,
    yaw_kd=0.25,
    
    # Planning
    planning_horizon=6.0,  # Look 6 gates ahead
    gate_commit_threshold=3.0,
    speed_margin=0.9,  # 90% of max speed on curves
    
    # Filtering
    position_filter_alpha=0.75,  # More responsive
    attitude_filter_alpha=0.65,
    
    # Timing
    control_loop_hz=50.0,
    planning_loop_hz=20.0,
    state_estimation_hz=50.0,
)

# Aggressive tuning (for fast aircraft, wide gates)
AGGRESSIVE_CONFIG = RacingConfig(
    max_velocity=40.0,
    pos_kp=3.5,
    pos_ki=0.2,
    pos_kd=1.0,
    vel_kp=0.8,
    vel_ki=0.1,
    vel_kd=0.2,
    att_kp=5.0,
    att_ki=0.2,
    att_kd=0.5,
    gate_commit_threshold=2.0,
    speed_margin=0.95,
    position_filter_alpha=0.8,
)

# Conservative tuning (for tight gates, low speed)
CONSERVATIVE_CONFIG = RacingConfig(
    max_velocity=15.0,
    pos_kp=1.5,
    pos_ki=0.1,
    pos_kd=0.4,
    vel_kp=0.3,
    vel_ki=0.03,
    vel_kd=0.08,
    att_kp=2.5,
    att_ki=0.08,
    att_kd=0.2,
    gate_commit_threshold=5.0,
    speed_margin=0.7,
    position_filter_alpha=0.6,
)

# Export
__all__ = ['DEFAULT_CONFIG', 'AGGRESSIVE_CONFIG', 'CONSERVATIVE_CONFIG']
