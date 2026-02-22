"""
Control Module: Cascaded PID controllers for position, velocity, and attitude.

Architecture:
- Outer loop: Position control -> velocity setpoint
- Middle loop: Velocity control -> attitude setpoint
- Inner loop: Attitude control -> motor commands (thrust, roll, pitch, yaw rate)

Design: Classical cascaded control for quadrotors, proven in racing.
"""

from typing import Optional, Tuple
import numpy as np
from .types import Vec3, DroneState, ControlOutput, PlanningResult, RacingConfig


class PIDController:
    """
    Basic PID controller with anti-windup and output saturation.
    """
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_min: float = -1.0, output_max: float = 1.0,
                 integral_windup_guard: float = 0.5):
        """
        Args:
            kp, ki, kd: Proportional, integral, derivative gains
            output_min, output_max: Output saturation limits
            integral_windup_guard: Anti-windup coefficient
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_windup_guard = integral_windup_guard
        
        self.error_integral = 0.0
        self.error_previous = 0.0
        self.last_update_time: Optional[float] = None
    
    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        """
        Compute PID output.
        
        Args:
            setpoint: Desired value
            measurement: Current value
            dt: Time step
        
        Returns:
            Control command (saturated)
        """
        error = setpoint - measurement
        
        # Proportional
        p_term = self.kp * error
        
        # Integral with anti-windup
        self.error_integral += error * dt
        # Clamp integral to prevent windup
        self.error_integral = np.clip(
            self.error_integral,
            -self.integral_windup_guard / (self.ki + 1e-9),
            self.integral_windup_guard / (self.ki + 1e-9)
        )
        i_term = self.ki * self.error_integral
        
        # Derivative
        d_term = 0.0
        if dt > 1e-6:
            d_term = self.kd * (error - self.error_previous) / dt
        
        self.error_previous = error
        
        # Output
        output = p_term + i_term + d_term
        output = np.clip(output, self.output_min, self.output_max)
        
        return output
    
    def reset(self):
        """Reset controller state."""
        self.error_integral = 0.0
        self.error_previous = 0.0


class PositionController:
    """
    Position control loop: Maps position error to velocity commands.
    
    Acts as outer loop in cascade:
    desired_position -> velocity_command
    """
    
    def __init__(self, config: RacingConfig):
        self.config = config
        # One controller per axis
        self.pid_x = PIDController(config.pos_kp, config.pos_ki, config.pos_kd,
                                  output_min=-config.max_velocity,
                                  output_max=config.max_velocity)
        self.pid_y = PIDController(config.pos_kp, config.pos_ki, config.pos_kd,
                                  output_min=-config.max_velocity,
                                  output_max=config.max_velocity)
        self.pid_z = PIDController(config.pos_kp, config.pos_ki, config.pos_kd,
                                  output_min=-config.max_velocity,
                                  output_max=config.max_velocity)
    
    def update(self, desired_pos: Vec3, current_pos: Vec3, dt: float) -> Vec3:
        """
        Compute desired velocity from position error.
        
        Returns:
            Desired velocity vector
        """
        vel_cmd_x = self.pid_x.update(desired_pos.x, current_pos.x, dt)
        vel_cmd_y = self.pid_y.update(desired_pos.y, current_pos.y, dt)
        vel_cmd_z = self.pid_z.update(desired_pos.z, current_pos.z, dt)
        
        return Vec3(vel_cmd_x, vel_cmd_y, vel_cmd_z)
    
    def reset(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()


class VelocityController:
    """
    Velocity control loop: Maps velocity error to attitude commands.
    
    Middle loop:
    desired_velocity -> attitude_setpoint (roll, pitch)
    
    Simplified model: 
    - roll affects lateral acceleration
    - pitch affects forward acceleration
    - vertical: thrust directly controls vertical acceleration
    """
    
    def __init__(self, config: RacingConfig):
        self.config = config
        self.pid_vx = PIDController(config.vel_kp, config.vel_ki, config.vel_kd,
                                   output_min=-config.max_pitch,
                                   output_max=config.max_pitch)
        self.pid_vy = PIDController(config.vel_kp, config.vel_ki, config.vel_kd,
                                   output_min=-config.max_roll,
                                   output_max=config.max_roll)
        self.pid_vz = PIDController(config.vel_kp, config.vel_ki, config.vel_kd,
                                   output_min=0.0,
                                   output_max=1.0)  # Thrust normalized
    
    def update(self, desired_vel: Vec3, current_vel: Vec3, 
               current_att: Vec3, dt: float) -> Tuple[Vec3, float]:
        """
        Compute attitude and thrust from velocity error.
        
        Returns:
            (attitude_setpoint, thrust_command)
        """
        # Velocity errors
        vel_error_x = desired_vel.x - current_vel.x
        vel_error_y = desired_vel.y - current_vel.y
        vel_error_z = desired_vel.z - current_vel.z
        
        # Attitude commands (simplified: vel_x -> pitch, vel_y -> roll)
        pitch_cmd = self.pid_vx.update(desired_vel.x, current_vel.x, dt)
        roll_cmd = self.pid_vy.update(desired_vel.y, current_vel.y, dt)
        
        # Thrust
        thrust_cmd = self.pid_vz.update(desired_vel.z, current_vel.z, dt)
        thrust_cmd = max(0.3, thrust_cmd)  # Min thrust to avoid dropping
        
        # Current yaw is preserved
        yaw = current_att.z
        
        attitude_setpoint = Vec3(roll_cmd, pitch_cmd, yaw)
        
        return attitude_setpoint, thrust_cmd
    
    def reset(self):
        self.pid_vx.reset()
        self.pid_vy.reset()
        self.pid_vz.reset()


class AttitudeController:
    """
    Attitude control loop: Maps attitude error to angular rate commands.
    
    Inner loop: Most frequency-critical
    attitude_error -> angular_rate_commands (motor outputs)
    
    Simplified: Direct P control on attitude (high frequency loop assumed to track)
    """
    
    def __init__(self, config: RacingConfig):
        self.config = config
        # Attitude errors mapped to angular rates
        self.pid_roll = PIDController(config.att_kp, config.att_ki, config.att_kd,
                                     output_min=-config.max_roll,
                                     output_max=config.max_roll)
        self.pid_pitch = PIDController(config.att_kp, config.att_ki, config.att_kd,
                                      output_min=-config.max_pitch,
                                      output_max=config.max_pitch)
        self.pid_yaw_rate = PIDController(config.yaw_kp, 0.0, config.yaw_kd,
                                         output_min=-config.max_yaw_rate,
                                         output_max=config.max_yaw_rate)
    
    def update(self, desired_att: Vec3, current_att: Vec3, 
               desired_yaw_rate: float, dt: float) -> Vec3:
        """
        Compute body-frame angular rates from attitude errors.
        
        Returns:
            Angular rate commands (p, q, r) - not used directly but could feed to motor controller
        """
        # Roll and pitch: direct error scaling
        roll_cmd = self.pid_roll.update(desired_att.x, current_att.x, dt)
        pitch_cmd = self.pid_pitch.update(desired_att.y, current_att.y, dt)
        yaw_rate_cmd = self.pid_yaw_rate.update(desired_yaw_rate, 0.0, dt)
        
        return Vec3(roll_cmd, pitch_cmd, yaw_rate_cmd)
    
    def reset(self):
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_yaw_rate.reset()


class CascadedController:
    """
    Integrates all three control loops: Position -> Velocity -> Attitude.
    
    Main control interface.
    """
    
    def __init__(self, config: RacingConfig):
        self.config = config
        self.position_controller = PositionController(config)
        self.velocity_controller = VelocityController(config)
        self.attitude_controller = AttitudeController(config)
        
        self.last_timestamp: Optional[float] = None
    
    def compute_control(self, drone_state: DroneState, 
                       planning_result: PlanningResult,
                       desired_yaw: float = 0.0) -> ControlOutput:
        """
        Compute complete control output from state and plan.
        
        Flow:
        1. Position error -> velocity command
        2. Velocity error -> attitude command
        3. Attitude error -> motor commands
        
        Args:
            drone_state: Current drone state (filtered)
            planning_result: Desired trajectory from planner
            desired_yaw: Desired yaw angle
        
        Returns:
            ControlOutput ready to send to simulator
        """
        if self.last_timestamp is None:
            dt = 0.02
        else:
            dt = drone_state.timestamp - self.last_timestamp
            dt = np.clip(dt, 0.01, 0.1)  # Sanity check on dt
        
        self.last_timestamp = drone_state.timestamp
        
        # *** Outer Loop: Position Control ***
        velocity_cmd = self.position_controller.update(
            planning_result.target_position,
            drone_state.position,
            dt
        )
        
        # Scale by desired speed (planning layer provides target speed)
        if velocity_cmd.magnitude() > 1e-3:
            velocity_cmd = velocity_cmd.normalize() * planning_result.target_velocity
        else:
            velocity_cmd = Vec3(0, 0, 0)
        
        # *** Middle Loop: Velocity Control ***
        attitude_cmd, thrust_cmd = self.velocity_controller.update(
            velocity_cmd,
            drone_state.velocity,
            drone_state.attitude,
            dt
        )
        
        # *** Inner Loop: Attitude Control ***
        # Yaw rate: point toward target or maintain
        yaw_rate_cmd = 0.0  # For now, maintain yaw
        
        _ = self.attitude_controller.update(
            attitude_cmd,
            drone_state.attitude,
            yaw_rate_cmd,
            dt
        )
        
        # Create output command
        output = ControlOutput(
            timestamp=drone_state.timestamp,
            roll_setpoint=attitude_cmd.x,
            pitch_setpoint=attitude_cmd.y,
            yaw_rate_setpoint=yaw_rate_cmd,
            thrust=thrust_cmd,
            is_valid=True
        )
        
        return output
    
    def reset(self):
        """Reset all controller states."""
        self.position_controller.reset()
        self.velocity_controller.reset()
        self.attitude_controller.reset()
        self.last_timestamp = None


class TuningAdvisor:
    """
    Helper to tune control gains based on system response.
    """
    
    @staticmethod
    def suggest_gains_from_response(response_data: dict) -> dict:
        """
        Given system response data, suggest improved gains.
        
        Args:
            response_data: {
                'overshoot_percent': float,
                'settling_time': float,
                'steady_state_error': float,
                'current_kp': float, etc.
            }
        
        Returns:
            Suggested gains
        """
        # Simple heuristics
        overshoot = response_data.get('overshoot_percent', 0)
        settling = response_data.get('settling_time', 1.0)
        kp = response_data.get('current_kp', 1.0)
        
        new_gains = {}
        
        # Too much overshoot: reduce kp, increase kd
        if overshoot > 20:
            new_gains['kp'] = kp * 0.8
            new_gains['kd'] = response_data.get('current_kd', 0) * 1.2
        
        # Too slow: increase kp
        if settling > 2.0:
            new_gains['kp'] = kp * 1.1
        
        return new_gains
