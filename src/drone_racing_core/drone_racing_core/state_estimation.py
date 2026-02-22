"""
State Estimation Module: Filtering and feature extraction.

Responsibilities:
- Low-pass filter noisy state measurements
- Compute derivatives (acceleration, turn rate)
- Provide clean state estimates to controller
- Track state history for trajectory analysis
"""

from collections import deque
from typing import Optional, Tuple
import numpy as np
from .types import DroneState, Vec3


class StateEstimator:
    """
    Stateful filter for drone state.
    Uses exponential moving average (EMA) for low-pass filtering.
    
    Design rationale:
    - EMA is simple, O(1) memory, deterministic
    - Avoids Kalman filter complexity (no covariance tuning)
    - Compute acceleration via finite differences
    """
    
    def __init__(self, alpha_pos: float = 0.7, alpha_att: float = 0.6, 
                 history_length: int = 10):
        """
        Args:
            alpha_pos: Filter coefficient for position [0, 1]. Higher = less filtering.
            alpha_att: Filter coefficient for attitude
            history_length: Number of states to retain for derivative computation
        """
        self.alpha_pos = alpha_pos
        self.alpha_att = alpha_att
        self.history_length = history_length
        
        # State history (for derivative estimation)
        self.position_history: deque = deque(maxlen=history_length)
        self.velocity_history: deque = deque(maxlen=history_length)
        self.attitude_history: deque = deque(maxlen=history_length)
        
        # Last filtered values
        self.filtered_position: Optional[Vec3] = None
        self.filtered_velocity: Optional[Vec3] = None
        self.filtered_attitude: Optional[Vec3] = None
        self.filtered_acceleration: Optional[Vec3] = None
        
        self.last_timestamp: float = 0.0
        self.is_initialized = False
    
    def update(self, state: DroneState) -> DroneState:
        """
        Filter drone state and compute derivatives.
        
        Args:
            state: Raw drone state from simulator
        
        Returns:
            Filtered state with acceleration estimates
        """
        dt = state.timestamp - self.last_timestamp if self.is_initialized else 0.02
        self.last_timestamp = state.timestamp
        
        # Initialize on first call
        if not self.is_initialized:
            self.filtered_position = state.position
            self.filtered_velocity = state.velocity
            self.filtered_attitude = state.attitude
            self.filtered_acceleration = Vec3(0, 0, 0)
            self.is_initialized = True
            return state
        
        # Low-pass filter position
        fp = self.filtered_position
        self.filtered_position = Vec3(
            self.alpha_pos * state.position.x + (1 - self.alpha_pos) * fp.x,
            self.alpha_pos * state.position.y + (1 - self.alpha_pos) * fp.y,
            self.alpha_pos * state.position.z + (1 - self.alpha_pos) * fp.z
        )
        
        # Low-pass filter velocity
        fv = self.filtered_velocity
        self.filtered_velocity = Vec3(
            self.alpha_pos * state.velocity.x + (1 - self.alpha_pos) * fv.x,
            self.alpha_pos * state.velocity.y + (1 - self.alpha_pos) * fv.y,
            self.alpha_pos * state.velocity.z + (1 - self.alpha_pos) * fv.z
        )
        
        # Low-pass filter attitude
        fa = self.filtered_attitude
        self.filtered_attitude = Vec3(
            self.alpha_att * state.attitude.x + (1 - self.alpha_att) * fa.x,
            self.alpha_att * state.attitude.y + (1 - self.alpha_att) * fa.y,
            self.alpha_att * state.attitude.z + (1 - self.alpha_att) * fa.z
        )
        
        # Estimate acceleration from velocity derivative
        if dt > 0:
            accel = Vec3(
                (self.filtered_velocity.x - fv.x) / dt,
                (self.filtered_velocity.y - fv.y) / dt,
                (self.filtered_velocity.z - fv.z) / dt
            )
            # Smooth acceleration estimate with light filtering
            if self.filtered_acceleration is not None:
                alpha_accel = 0.5  # Higher: more responsive
                self.filtered_acceleration = Vec3(
                    alpha_accel * accel.x + (1 - alpha_accel) * self.filtered_acceleration.x,
                    alpha_accel * accel.y + (1 - alpha_accel) * self.filtered_acceleration.y,
                    alpha_accel * accel.z + (1 - alpha_accel) * self.filtered_acceleration.z
                )
        
        # Update history
        self.position_history.append(self.filtered_position)
        self.velocity_history.append(self.filtered_velocity)
        self.attitude_history.append(self.filtered_attitude)
        
        # Create output state
        filtered_state = DroneState(
            timestamp=state.timestamp,
            position=self.filtered_position,
            velocity=self.filtered_velocity,
            attitude=self.filtered_attitude,
            angular_velocity=state.angular_velocity,
            speed=self.filtered_velocity.magnitude(),
            acceleration=self.filtered_acceleration or Vec3(0, 0, 0)
        )
        
        return filtered_state
    
    def get_velocity_derivative(self) -> Optional[Vec3]:
        """Get estimated acceleration (velocity derivative)."""
        return self.filtered_acceleration
    
    def get_position_derivative(self) -> Optional[Vec3]:
        """Get estimated velocity from position history."""
        if len(self.position_history) < 2:
            return None
        
        pos_prev = self.position_history[0]
        pos_curr = self.position_history[-1]
        # dt would be (history_length) * dt_control, so we use filtered_velocity instead
        return self.filtered_velocity
    
    def reset(self):
        """Reset filter state."""
        self.filtered_position = None
        self.filtered_velocity = None
        self.filtered_attitude = None
        self.filtered_acceleration = None
        self.position_history.clear()
        self.velocity_history.clear()
        self.attitude_history.clear()
        self.is_initialized = False


class StateValidator:
    """
    Additional validation for state estimates.
    Detects anomalies or physical inconsistencies.
    """
    
    def __init__(self, max_accel: float = 20.0, max_velocity: float = 40.0):
        self.max_accel = max_accel
        self.max_velocity = max_velocity
        self.anomaly_count = 0
    
    def validate_state(self, state: DroneState) -> Tuple[bool, Optional[str]]:
        """
        Validate filtered state for physical plausibility.
        
        Returns:
            (is_valid, error_message)
        """
        # Check velocity
        if state.speed > self.max_velocity:
            self.anomaly_count += 1
            return False, f"Speed {state.speed:.1f} exceeds max {self.max_velocity}"
        
        # Check acceleration
        accel_mag = state.acceleration.magnitude() if state.acceleration else 0.0
        if accel_mag > self.max_accel:
            self.anomaly_count += 1
            return False, f"Accel {accel_mag:.1f} exceeds max {self.max_accel}"
        
        # Check for sudden position jumps (indicates teleport/simulator reset)
        # This would need position history - skipping for now
        
        return True, None
    
    def get_anomaly_count(self) -> int:
        return self.anomaly_count


# Test state estimator
if __name__ == "__main__":
    from .types import DroneState, Vec3
    
    estimator = StateEstimator(alpha_pos=0.7, alpha_att=0.6)
    
    # Simulate noisy state
    true_pos = Vec3(0, 0, 10)
    for t in np.linspace(0, 1.0, 50):
        noise = Vec3(
            np.random.normal(0, 0.1),
            np.random.normal(0, 0.1),
            np.random.normal(0, 0.05)
        )
        noisy_state = DroneState(
            timestamp=t,
            position=true_pos + noise,
            velocity=Vec3(1.0, 0.5, 0.1),
            attitude=Vec3(0, 0, t/10),
            angular_velocity=Vec3(0, 0, 0.1)
        )
        
        filtered = estimator.update(noisy_state)
        print(f"t={t:.2f}: raw_pos=({noisy_state.position.x:.3f}, {noisy_state.position.y:.3f}), "
              f"filt_pos=({filtered.position.x:.3f}, {filtered.position.y:.3f}), "
              f"accel={filtered.acceleration.magnitude():.3f}")
