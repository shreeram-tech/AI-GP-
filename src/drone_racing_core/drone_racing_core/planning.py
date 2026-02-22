"""
Planning Module: Gate sequencing and trajectory generation.

Responsibilities:
- Determine next gate to navigate
- Generate smooth trajectory (Pure Pursuit / waypoint-based)
- Compute speed profiles based on path curvature
- Lookahead to next 2-3 gates for smooth planning
"""

from typing import List, Tuple, Optional
import numpy as np
from .types import Vec3, Gate, DroneState, PlanningResult, RacingConfig


class GatePlanner:
    """
    Gate sequencing and commitment logic.
    Decides which gate is next target and when to commit.
    """
    
    def __init__(self, config: RacingConfig):
        self.config = config
        self.current_gate_index = 0
        self.is_committed = False
        self.distance_to_next_gate: float = float('inf')
    
    def update(self, drone_pos: Vec3, gates: List[Gate], 
               current_gate_index: int) -> Tuple[int, bool]:
        """
        Determine next gate target and commitment status.
        
        Args:
            drone_pos: Current drone position
            gates: List of all gates
            current_gate_index: Simulator's current gate index (reference)
        
        Returns:
            (next_gate_index, is_committed)
        """
        if len(gates) == 0:
            return 0, False
        
        # Simple strategy: target is current_gate_index + 1
        # More advanced: lookahead based on predicted trajectory
        next_target = (current_gate_index + 1) % len(gates)
        
        target_gate = gates[next_target]
        self.distance_to_next_gate = (drone_pos - target_gate.position).magnitude()
        
        # Commit to next gate when close enough
        self.is_committed = (self.distance_to_next_gate < self.config.gate_commit_threshold)
        
        return next_target, self.is_committed
    
    def get_lookahead_gates(self, current_idx: int, gates: List[Gate], 
                            num_ahead: int = 3) -> List[int]:
        """Get indices of next N gates for planning."""
        lookahead = []
        for i in range(1, num_ahead + 1):
            idx = (current_idx + i) % len(gates)
            lookahead.append(idx)
        return lookahead


class TrajectoryPlanner:
    """
    Generate smooth trajectory through gates using Pure Pursuit-like approach.
    
    Algorithm:
    1. Compute racing line (center or slightly wide of gate)
    2. Generate waypoints along path
    3. Apply Pure Pursuit to compute steering commands
    4. Adapt speed based on path curvature
    """
    
    def __init__(self, config: RacingConfig, lookahead_dist: float = 15.0):
        """
        Args:
            config: Racing configuration
            lookahead_dist: Pure Pursuit lookahead distance (m)
        """
        self.config = config
        self.lookahead_dist = lookahead_dist
        self.current_waypoint_idx = 0
    
    def compute_racing_line(self, gate: Gate, prev_gate: Optional[Gate] = None,
                           next_gate: Optional[Gate] = None) -> Vec3:
        """
        Compute ideal racing line (entry point to gate).
        
        Strategy:
        - Enter gate through center
        - Slightly offset to smooth turns
        - Account for next gate direction
        
        Returns:
            Target position to pass through gate
        """
        target = gate.position.copy() if hasattr(gate.position, 'copy') else gate.position
        
        # If we have context from adjacent gates, smooth the line
        if prev_gate and next_gate:
            # Vector from previous to current
            in_vec = (gate.position - prev_gate.position).normalize()
            # Vector from current to next
            out_vec = (next_gate.position - gate.position).normalize()
            
            # Average direction for smoother path
            avg_vec = (in_vec + out_vec) * 0.5
            avg_vec = avg_vec.normalize()
            
            # Slight offset to wide line (depends on turn angle)
            turn_angle = np.arccos(max(-1, min(1, in_vec.dot(out_vec))))
            # Larger turn = larger racing line offset
            offset_dist = max(0.5, np.sin(turn_angle) * gate.radius * 0.3)
            
            # Perpendicular to average direction
            perp = Vec3(-avg_vec.y, avg_vec.x, 0).normalize()
            target = gate.position + perp * offset_dist
        
        return target
    
    def generate_waypoints(self, current_pos: Vec3, target_gate: Gate,
                          gates: List[Gate], gate_indices: List[int],
                          num_waypoints: int = 5) -> List[Vec3]:
        """
        Generate smooth waypoint sequence to target gate and beyond.
        
        Strategy:
        - Intermediate waypoint toward target
        - Then waypoints toward lookahead gates
        - Smooth interpolation
        """
        waypoints = [current_pos]
        
        if len(gate_indices) == 0:
            return waypoints
        
        # Primary target
        racing_line = self.compute_racing_line(target_gate)
        waypoints.append(racing_line)
        
        # Waypoints toward lookahead gates
        for idx in gate_indices[1:]:
            gate = gates[idx]
            # Compute midpoint to next lookahead gate
            midpoint = (waypoints[-1] + gate.position) * 0.5
            waypoints.append(midpoint)
        
        return waypoints
    
    def compute_pure_pursuit(self, drone_pos: Vec3, drone_vel: Vec3,
                            waypointsequence: List[Vec3]) -> Tuple[Vec3, float]:
        """
        Pure Pursuit steering law: Generate desired velocity command.
        
        Algorithm:
        1. Find lookahead point ahead on waypoint path
        2. Compute cross-track error
        3. Steering proportional to cross-track error
        
        Args:
            drone_pos: Current position
            drone_vel: Current velocity
            waypointsequence: List of waypoints to follow
        
        Returns:
            (desired_velocity_direction, curvature)
        """
        if len(waypointsequence) < 2:
            return drone_vel.normalize() if drone_vel.magnitude() > 0.1 else Vec3(1, 0, 0), 0.0
        
        # Find current segment (closest waypoint pair)
        min_dist = float('inf')
        segment_idx = 0
        for i in range(len(waypointsequence) - 1):
            # Distance from drone to segment
            p1 = waypointsequence[i]
            p2 = waypointsequence[i + 1]
            dist = self._point_to_segment_distance(drone_pos, p1, p2)
            if dist < min_dist:
                min_dist = dist
                segment_idx = i
        
        # Lookahead point on path
        p1 = waypointsequence[segment_idx]
        p2 = waypointsequence[segment_idx + 1]
        lookahead_point = self._get_lookahead_point_on_segment(
            drone_pos, p1, p2, self.lookahead_dist
        )
        
        # Desired velocity toward lookahead
        desired_dir = (lookahead_point - drone_pos).normalize()
        
        # Compute path curvature for speed adaptation
        curvature = self._estimate_path_curvature(waypointsequence, segment_idx)
        
        return desired_dir, curvature
    
    def _point_to_segment_distance(self, point: Vec3, p1: Vec3, p2: Vec3) -> float:
        """Distance from point to line segment p1-p2."""
        v = p2 - p1
        u = point - p1
        t = max(0, min(1, u.dot(v) / (v.dot(v) + 1e-6)))
        closest = p1 + v * t
        return (point - closest).magnitude()
    
    def _get_lookahead_point_on_segment(self, drone_pos: Vec3, p1: Vec3, 
                                        p2: Vec3, lookahead_dist: float) -> Vec3:
        """Get point lookahead_dist ahead on segment from drone."""
        segment_dir = (p2 - p1).normalize()
        # If drone is before segment, start from p1
        u = (drone_pos - p1).dot(segment_dir)
        u = max(0, u)  # Don't go backwards
        along_segment = p1 + segment_dir * u
        lookahead = along_segment + segment_dir * lookahead_dist
        # Clamp to segment end
        total_len = (p2 - p1).magnitude()
        if u + lookahead_dist > total_len:
            lookahead = p2
        return lookahead
    
    def _estimate_path_curvature(self, waypoints: List[Vec3], 
                                segment_idx: int) -> float:
        """Estimate curvature at current position for speed adaptation."""
        if segment_idx < 1 or segment_idx >= len(waypoints) - 1:
            return 0.0
        
        # Use three consecutive waypoints to estimate curvature
        p0 = waypoints[segment_idx - 1]
        p1 = waypoints[segment_idx]
        p2 = waypoints[segment_idx + 1]
        
        # Compute local curvature via cross-track acceleration
        v01 = p1 - p0
        v12 = p2 - p1
        
        if v01.magnitude() < 1e-6 or v12.magnitude() < 1e-6:
            return 0.0
        
        v01_norm = v01.normalize()
        v12_norm = v12.normalize()
        
        # Angle change
        cos_angle = max(-1, min(1, v01_norm.dot(v12_norm)))
        angle_change = np.arccos(cos_angle)
        
        # Segment length (rough)
        segment_length = v01.magnitude()
        
        # Curvature ~ angle_change / segment_length
        curvature = angle_change / (segment_length + 1e-6)
        return curvature
    
    def compute_speed_profile(self, base_speed: float, 
                             curvature: float) -> float:
        """
        Adapt speed based on path curvature.
        
        Higher curvature -> lower speed (avoid overshoot)
        Straight sections -> higher speed
        """
        # Speed reduction factor: e^(-k * curvature)
        # where k is tuning parameter
        k = 2.0 * base_speed
        speed_factor = np.exp(-k * curvature)
        
        # Apply margin and clamp
        adapted_speed = base_speed * speed_factor * self.config.speed_margin
        adapted_speed = max(5.0, min(self.config.max_velocity, adapted_speed))
        
        return adapted_speed


class PlannerIntegration:
    """
    Combines gate planning and trajectory planning.
    """
    
    def __init__(self, config: RacingConfig):
        self.config = config
        self.gate_planner = GatePlanner(config)
        self.trajectory_planner = TrajectoryPlanner(config, lookahead_dist=15.0)
    
    def plan(self, drone_state: DroneState, gates: List[Gate],
            current_gate_index: int, base_speed: float = 20.0) -> PlanningResult:
        """
        Main planning update.
        
        Returns:
            PlanningResult with next target, speed, and path info
        """
        if len(gates) == 0:
            return PlanningResult(
                next_gate_index=0,
                target_position=drone_state.position,
                target_velocity=0.0
            )
        
        # Determine next gate
        next_gate_idx, is_committed = self.gate_planner.update(
            drone_state.position, gates, current_gate_index
        )
        
        # Lookahead gates for trajectory planning
        lookahead_indices = self.gate_planner.get_lookahead_gates(
            next_gate_idx, gates, num_ahead=3
        )
        
        # Generate trajectory
        target_gate = gates[next_gate_idx]
        waypoints = self.trajectory_planner.generate_waypoints(
            drone_state.position, target_gate, gates, lookahead_indices
        )
        
        # Pure Pursuit steering
        desired_dir, curvature = self.trajectory_planner.compute_pure_pursuit(
            drone_state.position, drone_state.velocity, waypoints
        )
        
        # Speed profile
        target_speed = self.trajectory_planner.compute_speed_profile(base_speed, curvature)
        
        # Target position: next waypoint
        target_pos = waypoints[1] if len(waypoints) > 1 else desired_dir * 20.0 + drone_state.position
        
        return PlanningResult(
            next_gate_index=next_gate_idx,
            target_position=target_pos,
            target_velocity=target_speed,
            lookahead_gates=lookahead_indices,
            path_curvature=curvature,
            is_committed=is_committed
        )
