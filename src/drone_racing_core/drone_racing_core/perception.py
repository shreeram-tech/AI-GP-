"""
Perception Module: Interface to simulator, data validation, and preprocessing.

Responsibilities:
- Receive drone state from simulator
- Validate data integrity
- Convert to internal representations
- Track timestamps for latency awareness
"""

import time
from typing import Dict, Optional, List, Tuple
import numpy as np
from .types import DroneState, Gate, Vec3


class PerceptionModule:
    """
    Wraps simulator interface and provides validated drone state.
    Design: Stateless & repeatable - input data -> output state.
    """
    
    def __init__(self, data_timeout: float = 1.0):
        """
        Args:
            data_timeout: Seconds to consider data stale
        """
        self.data_timeout = data_timeout
        self.last_update_time: float = 0.0
        self._validation_stats = {
            'good_frames': 0,
            'bad_frames': 0,
            'last_error': ''
        }
    
    def validate_simulator_data(self, sim_data: Dict) -> Tuple[bool, str]:
        """
        Validate incoming simulator data.
        
        Args:
            sim_data: Dict from simulator containing:
                - timestamp, position, velocity, attitude, angular_velocity
                - gates (list), current_gate_index
        
        Returns:
            (is_valid, error_message)
        """
        required_fields = [
            'timestamp', 'position', 'velocity', 'attitude', 
            'angular_velocity', 'gates', 'current_gate_index'
        ]
        
        # Check required fields
        for field in required_fields:
            if field not in sim_data:
                return False, f"Missing field: {field}"
        
        # Validate position (WCS bounds - competition space typically 100m x 100m x 50m)
        pos = sim_data['position']
        if not isinstance(pos, (tuple, list)) or len(pos) != 3:
            return False, "Position must be 3D vector"
        if any(np.isnan(x) or np.isinf(x) for x in pos):
            return False, "Position contains NaN/Inf"
        
        # Validate velocity
        vel = sim_data['velocity']
        if any(np.isnan(x) or np.isinf(x) for x in vel):
            return False, "Velocity contains NaN/Inf"
        speed = np.sqrt(sum(v**2 for v in vel))
        if speed > 100.0:  # Sanity check: max 100 m/s
            return False, f"Velocity too high: {speed} m/s"
        
        # Validate attitude (Euler angles in [-pi, pi] or [0, 2*pi])
        att = sim_data['attitude']
        if any(abs(a) > 2*np.pi for a in att):
            return False, "Attitude angles out of range"
        
        # Validate gates
        gates = sim_data['gates']
        if not isinstance(gates, list) or len(gates) < 1:
            return False, "Gates must be non-empty list"
        
        current_gate = sim_data.get('current_gate_index', 0)
        if current_gate >= len(gates) or current_gate < 0:
            return False, f"Invalid gate index: {current_gate}/{len(gates)}"
        
        return True, "OK"
    
    def process_simulator_data(self, sim_data: Dict) -> Optional[Tuple[DroneState, List[Gate]]]:
        """
        Convert raw simulator data to internal representations.
        
        Args:
            sim_data: Raw dictionary from simulator
        
        Returns:
            (drone_state, gates) or None if validation fails
        """
        is_valid, error_msg = self.validate_simulator_data(sim_data)
        if not is_valid:
            self._validation_stats['bad_frames'] += 1
            self._validation_stats['last_error'] = error_msg
            return None
        
        self._validation_stats['good_frames'] += 1
        current_time = time.time()
        self.last_update_time = current_time
        
        # Extract and convert drone state
        drone_state = DroneState(
            timestamp=sim_data['timestamp'],
            position=Vec3(*sim_data['position']),
            velocity=Vec3(*sim_data['velocity']),
            attitude=Vec3(*sim_data['attitude']),  # (roll, pitch, yaw)
            angular_velocity=Vec3(*sim_data['angular_velocity'])
        )
        
        # Compute speed
        drone_state.speed = drone_state.velocity.magnitude()
        
        # Parse gates
        gates = []
        for i, gate_data in enumerate(sim_data['gates']):
            gate = Gate(
                index=i,
                position=Vec3(*gate_data['position']),
                orientation=tuple(gate_data.get('orientation', (0, 0, 0))),
                radius=gate_data.get('radius', 5.0)
            )
            gates.append(gate)
        
        return drone_state, gates
    
    def is_data_fresh(self) -> bool:
        """Check if data is within timeout."""
        return (time.time() - self.last_update_time) < self.data_timeout
    
    def get_stats(self) -> Dict:
        """Return validation statistics."""
        return self._validation_stats.copy()


class MockSimulator:
    """
    Mock simulator for testing without real simulator connection.
    Generates realistic drone trajectories.
    """
    
    def __init__(self, num_gates: int = 8, track_radius: float = 50.0):
        self.num_gates = num_gates
        self.track_radius = track_radius
        self.sim_time = 0.0
        self.current_gate_index = 0
        
        # Generate gate positions in a circle
        self.gates = self._generate_gates()
        
        # Drone state (initialized at start)
        self.position = np.array([0.0, 0.0, 10.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.attitude = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
    
    def _generate_gates(self) -> List[Dict]:
        """Generate circular track gates."""
        gates = []
        for i in range(self.num_gates):
            angle = 2 * np.pi * i / self.num_gates
            x = self.track_radius * np.cos(angle)
            y = self.track_radius * np.sin(angle)
            z = 10.0 + 5.0 * np.sin(2 * angle)  # Slight height variation
            gates.append({
                'position': (x, y, z),
                'orientation': (0, 0, angle),
                'radius': 5.0
            })
        return gates
    
    def step(self, control_output, dt: float = 0.02) -> Dict:
        """
        Simulate one timestep.
        
        Args:
            control_output: ControlOutput from autonomy
            dt: Timestep
        """
        # Simple drone model: attitude -> velocity -> position
        # (Simplified from full 6-DOF quadrotor dynamics)
        
        # Apply attitude commands
        roll_cmd = control_output.roll_setpoint
        pitch_cmd = control_output.pitch_setpoint
        yaw_rate_cmd = control_output.yaw_rate_setpoint
        thrust_cmd = control_output.thrust
        
        # Attitude dynamics (first-order approximation)
        tau_att = 0.1  # attitude response time
        self.attitude[0] += (roll_cmd - self.attitude[0]) * (dt / tau_att)
        self.attitude[1] += (pitch_cmd - self.attitude[1]) * (dt / tau_att)
        self.attitude[2] += yaw_rate_cmd * dt
        
        # Wrap yaw to [-pi, pi]
        self.attitude[2] = np.arctan2(np.sin(self.attitude[2]), np.cos(self.attitude[2]))
        
        # Velocity from attitude (simplified)
        max_accel = 10.0  # m/s^2
        accel_x = max_accel * np.sin(self.attitude[1])
        accel_y = -max_accel * np.sin(self.attitude[0]) * np.cos(self.attitude[1])
        accel_z = thrust_cmd * 20.0 - 9.81  # Thrust - gravity
        
        # Add small damping
        damping = 0.95
        self.velocity[0] = self.velocity[0] * damping + accel_x * dt
        self.velocity[1] = self.velocity[1] * damping + accel_y * dt
        self.velocity[2] = self.velocity[2] * damping + accel_z * dt
        
        # Integrate position
        self.position += self.velocity * dt
        
        # Add small gravity effect (keep drone roughly at z=10m)
        if self.position[2] < 5.0:
            self.velocity[2] += 5.0 * dt
        
        self.sim_time += dt
        
        # Update current gate based on proximity
        closest_gate = 0
        min_dist = float('inf')
        for i, gate in enumerate(self.gates):
            dist = np.linalg.norm(self.position - np.array(gate['position']))
            if dist < min_dist:
                min_dist = dist
                closest_gate = i
        self.current_gate_index = closest_gate
        
        return self.get_state()
    
    def get_state(self) -> Dict:
        """Return current state as simulator format."""
        return {
            'timestamp': self.sim_time,
            'position': tuple(self.position),
            'velocity': tuple(self.velocity),
            'attitude': tuple(self.attitude),
            'angular_velocity': tuple(self.angular_velocity),
            'gates': self.gates,
            'current_gate_index': self.current_gate_index
        }
