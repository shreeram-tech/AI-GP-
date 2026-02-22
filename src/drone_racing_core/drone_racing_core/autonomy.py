"""
Main Autonomy Loop: Integration layer orchestrating all modules.

Responsibilities:
- Receive simulator data
- Route through perception -> estimation -> planning -> control
- Send commands back to simulator
- Monitor performance and health
"""

import time
from typing import Dict, Optional, List, Tuple
import numpy as np

from .types import (
    DroneState, Gate, ControlOutput, RacingConfig, Vec3, PlanningResult
)
from .perception import PerceptionModule, MockSimulator
from .state_estimation import StateEstimator, StateValidator
from .planning import PlannerIntegration
from .control import CascadedController


class AutonomyStack:
    """
    Main autonomy loop: Coordinates perception, planning, control.
    Designed for ~50Hz operation (20ms cycle time).
    """
    
    def __init__(self, config: Optional[RacingConfig] = None):
        self.config = config or RacingConfig()
        
        # Modules
        self.perception = PerceptionModule(data_timeout=1.0)
        self.state_estimator = StateEstimator(
            alpha_pos=self.config.position_filter_alpha,
            alpha_att=self.config.attitude_filter_alpha
        )
        self.state_validator = StateValidator(
            max_velocity=self.config.max_velocity,
            max_accel=self.config.max_acceleration
        )
        self.planner = PlannerIntegration(self.config)
        self.controller = CascadedController(self.config)
        
        # Performance tracking
        self.loop_count = 0
        self.total_time = 0.0
        self.timing_stats = {
            'perception_ms': [],
            'estimation_ms': [],
            'planning_ms': [],
            'control_ms': [],
            'total_ms': []
        }
        self.max_loop_time_ms = 0.0
        
        # State history for analysis
        self.state_history: List[Dict] = []
        self.max_history_length = 1000
        
        # Last outputs
        self.last_control_output: Optional[ControlOutput] = None
        self.last_planning_result: Optional[PlanningResult] = None
    
    def step(self, sim_data: Dict) -> Optional[ControlOutput]:
        """
        Main autonomy loop iteration.
        
        Args:
            sim_data: Data from simulator (dict format)
        
        Returns:
            ControlOutput or None if error
        """
        loop_start = time.time()
        
        try:
            # *** PERCEPTION ***
            perc_start = time.time()
            perception_result = self.perception.process_simulator_data(sim_data)
            perc_time_ms = (time.time() - perc_start) * 1000
            
            if perception_result is None:
                print(f"[Autonomy] Perception failed: {self.perception.get_stats()['last_error']}")
                return None
            
            drone_state_raw, gates = perception_result
            
            # *** STATE ESTIMATION ***
            est_start = time.time()
            drone_state = self.state_estimator.update(drone_state_raw)
            
            # Validate
            is_valid, error_msg = self.state_validator.validate_state(drone_state)
            if not is_valid:
                print(f"[Autonomy] State validation failed: {error_msg}")
                # Continue anyway (graceful degradation)
            est_time_ms = (time.time() - est_start) * 1000
            
            # *** PLANNING ***
            plan_start = time.time()
            current_gate_idx = sim_data.get('current_gate_index', 0)
            planning_result = self.planner.plan(
                drone_state, gates, current_gate_idx,
                base_speed=20.0  # m/s
            )
            self.last_planning_result = planning_result
            plan_time_ms = (time.time() - plan_start) * 1000
            
            # *** CONTROL ***
            ctrl_start = time.time()
            control_output = self.controller.compute_control(drone_state, planning_result)
            self.last_control_output = control_output
            ctrl_time_ms = (time.time() - ctrl_start) * 1000
            
            # *** LOGGING & STATS ***
            total_loop_ms = (time.time() - loop_start) * 1000
            self._update_timing_stats(perc_time_ms, est_time_ms, plan_time_ms, 
                                     ctrl_time_ms, total_loop_ms)
            
            # Store history
            self._record_state_history(drone_state, planning_result, control_output)
            
            self.loop_count += 1
            
            if self.loop_count % 50 == 0:  # Log every 1 second at 50Hz
                self._print_diagnostics()
            
            return control_output
        
        except Exception as e:
            print(f"[Autonomy] ERROR in main loop: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _update_timing_stats(self, perc_ms: float, est_ms: float, 
                            plan_ms: float, ctrl_ms: float, total_ms: float):
        """Update performance timing statistics."""
        self.timing_stats['perception_ms'].append(perc_ms)
        self.timing_stats['estimation_ms'].append(est_ms)
        self.timing_stats['planning_ms'].append(plan_ms)
        self.timing_stats['control_ms'].append(ctrl_ms)
        self.timing_stats['total_ms'].append(total_ms)
        
        # Keep only recent history
        max_len = 500
        for key in self.timing_stats:
            if len(self.timing_stats[key]) > max_len:
                self.timing_stats[key] = self.timing_stats[key][-max_len:]
        
        self.max_loop_time_ms = max(self.max_loop_time_ms, total_ms)
    
    def _record_state_history(self, drone_state: DroneState, 
                             planning_result: PlanningResult,
                             control_output: ControlOutput):
        """Record state snapshot for later analysis."""
        record = {
            'timestamp': drone_state.timestamp,
            'position': (drone_state.position.x, drone_state.position.y, drone_state.position.z),
            'velocity': (drone_state.velocity.x, drone_state.velocity.y, drone_state.velocity.z),
            'speed': drone_state.speed,
            'attitude': (drone_state.attitude.x, drone_state.attitude.y, drone_state.attitude.z),
            'target_position': (planning_result.target_position.x, 
                               planning_result.target_position.y,
                               planning_result.target_position.z),
            'target_velocity': planning_result.target_velocity,
            'curvature': planning_result.path_curvature,
            'roll_cmd': control_output.roll_setpoint,
            'pitch_cmd': control_output.pitch_setpoint,
            'thrust': control_output.thrust
        }
        self.state_history.append(record)
        
        if len(self.state_history) > self.max_history_length:
            self.state_history = self.state_history[-self.max_history_length:]
    
    def _print_diagnostics(self):
        """Print diagnostic information."""
        avg_total = np.mean(self.timing_stats['total_ms'][-50:])
        max_total = np.max(self.timing_stats['total_ms'][-50:])
        
        print(f"\n[Diagnostics @ {self.loop_count}]")
        print(f"  Loop time: avg={avg_total:.2f}ms, max={max_total:.2f}ms")
        print(f"  Breakdown: perc={np.mean(self.timing_stats['perception_ms'][-50:]):.2f}ms, "
              f"est={np.mean(self.timing_stats['estimation_ms'][-50:]):.2f}ms, "
              f"plan={np.mean(self.timing_stats['planning_ms'][-50:]):.2f}ms, "
              f"ctrl={np.mean(self.timing_stats['control_ms'][-50:]):.2f}ms")
        
        if self.last_control_output:
            print(f"  Control: roll={self.last_control_output.roll_setpoint:.3f}, "
                  f"pitch={self.last_control_output.pitch_setpoint:.3f}, "
                  f"thrust={self.last_control_output.thrust:.3f}")
        
        if self.last_planning_result:
            print(f"  Planning: gate={self.last_planning_result.next_gate_index}, "
                  f"target_vel={self.last_planning_result.target_velocity:.1f}m/s, "
                  f"curvature={self.last_planning_result.path_curvature:.4f}")
        
        val_stats = self.perception.get_stats()
        print(f"  Perception: good={val_stats['good_frames']}, bad={val_stats['bad_frames']}")
    
    def get_performance_summary(self) -> Dict:
        """Get performance metrics."""
        if len(self.timing_stats['total_ms']) == 0:
            return {}
        
        total_times = self.timing_stats['total_ms']
        
        return {
            'loop_count': self.loop_count,
            'avg_loop_time_ms': np.mean(total_times),
            'max_loop_time_ms': np.max(total_times),
            'min_loop_time_ms': np.min(total_times),
            'p95_loop_time_ms': np.percentile(total_times, 95),
            'avg_perception_ms': np.mean(self.timing_stats['perception_ms']),
            'avg_estimation_ms': np.mean(self.timing_stats['estimation_ms']),
            'avg_planning_ms': np.mean(self.timing_stats['planning_ms']),
            'avg_control_ms': np.mean(self.timing_stats['control_ms']),
            'perception_stat': self.perception.get_stats()
        }
    
    def reset(self):
        """Reset autonomy stack for new run."""
        self.state_estimator.reset()
        self.controller.reset()
        self.state_history.clear()
        self.loop_count = 0
        self.total_time = 0.0
        self.timing_stats = {
            'perception_ms': [],
            'estimation_ms': [],
            'planning_ms': [],
            'control_ms': [],
            'total_ms': []
        }

