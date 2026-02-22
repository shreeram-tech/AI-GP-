"""
Testing and stress-testing framework for autonomy stack.

Includes:
- Unit tests for individual modules
- Integration tests
- Stress tests (latency, noise, edge cases)
- Performance benchmarks
"""

import sys
import unittest
import time
from typing import List, Dict
import numpy as np

sys.path.insert(0, '/home/shree/aigp/drone_racing_stack')

from src.types import Vec3, Gate, DroneState, RacingConfig, ControlOutput
from src.perception import PerceptionModule, MockSimulator
from src.state_estimation import StateEstimator, StateValidator
from src.planning import GatePlanner, TrajectoryPlanner, PlannerIntegration
from src.control import PIDController, CascadedController
from src.autonomy import AutonomyStack


class TestTypes(unittest.TestCase):
    """Test data types and operations."""
    
    def test_vec3_magnitude(self):
        v = Vec3(3, 4, 0)
        self.assertAlmostEqual(v.magnitude(), 5.0)
    
    def test_vec3_normalize(self):
        v = Vec3(3, 0, 4).normalize()
        self.assertAlmostEqual(v.magnitude(), 1.0, places=5)
    
    def test_vec3_dot_product(self):
        v1 = Vec3(1, 0, 0)
        v2 = Vec3(0, 1, 0)
        self.assertEqual(v1.dot(v2), 0.0)


class TestPerception(unittest.TestCase):
    """Test perception module."""
    
    def setUp(self):
        self.perception = PerceptionModule()
    
    def test_valid_data(self):
        """Test validation of correct simulator data."""
        valid_data = {
            'timestamp': 1.0,
            'position': (0, 0, 10),
            'velocity': (0, 0, 0),
            'attitude': (0, 0, 0),
            'angular_velocity': (0, 0, 0),
            'gates': [{'position': (10, 0, 10), 'radius': 5.0}],
            'current_gate_index': 0
        }
        is_valid, msg = self.perception.validate_simulator_data(valid_data)
        self.assertTrue(is_valid)
    
    def test_invalid_data_missing_field(self):
        """Test rejection of data missing required fields."""
        invalid_data = {
            'timestamp': 1.0,
            # Missing position
            'velocity': (0, 0, 0)
        }
        is_valid, msg = self.perception.validate_simulator_data(invalid_data)
        self.assertFalse(is_valid)
    
    def test_invalid_data_nan(self):
        """Test rejection of NaN values."""
        invalid_data = {
            'timestamp': 1.0,
            'position': (np.nan, 0, 10),
            'velocity': (0, 0, 0),
            'attitude': (0, 0, 0),
            'angular_velocity': (0, 0, 0),
            'gates': [{'position': (10, 0, 10), 'radius': 5.0}],
            'current_gate_index': 0
        }
        is_valid, msg = self.perception.validate_simulator_data(invalid_data)
        self.assertFalse(is_valid)
    
    def test_process_valid_data(self):
        """Test data processing."""
        valid_data = {
            'timestamp': 1.5,
            'position': (5, 10, 15),
            'velocity': (1, 2, 3),
            'attitude': (0.1, 0.2, 0.3),
            'angular_velocity': (0, 0, 0.1),
            'gates': [
                {'position': (20, 0, 10), 'radius': 5.0},
                {'position': (-20, 0, 10), 'radius': 5.0}
            ],
            'current_gate_index': 0
        }
        result = self.perception.process_simulator_data(valid_data)
        self.assertIsNotNone(result)
        drone_state, gates = result
        self.assertEqual(drone_state.timestamp, 1.5)
        self.assertEqual(len(gates), 2)


class TestStateEstimation(unittest.TestCase):
    """Test state estimation and filtering."""
    
    def setUp(self):
        self.estimator = StateEstimator(alpha_pos=0.7, alpha_att=0.6)
    
    def test_filtering_response(self):
        """Test that filter smooths noisy measurements."""
        states = []
        for t in np.linspace(0, 1, 50):
            noise = Vec3(np.random.normal(0, 0.2), 
                        np.random.normal(0, 0.2), 
                        np.random.normal(0, 0.1))
            noisy_state = DroneState(
                timestamp=t,
                position=Vec3(0, 0, 10) + noise,
                velocity=Vec3(1, 0, 0),
                attitude=Vec3(0, 0, 0)
            )
            filtered = self.estimator.update(noisy_state)
            states.append(filtered)
        
        # Final state should be close to true position
        final_pos = states[-1].position
        self.assertLess(abs(final_pos.x), 1.0)  # Converged to ~0
    
    def test_acceleration_estimation(self):
        """Test that filter estimates acceleration."""
        state_now = DroneState(
            timestamp=0.1,
            position=Vec3(0, 0, 10),
            velocity=Vec3(1, 0, 0)
        )
        state_next = DroneState(
            timestamp=0.12,
            position=Vec3(0.003, 0, 10),
            velocity=Vec3(1.1, 0, 0)
        )
        
        self.estimator.update(state_now)
        filtered = self.estimator.update(state_next)
        
        # Should have non-zero acceleration
        accel_mag = filtered.acceleration.magnitude()
        self.assertGreater(accel_mag, 0.01)


class TestControl(unittest.TestCase):
    """Test control modules."""
    
    def test_pid_controller_unity_feedback(self):
        """Test PID with simple step response."""
        controller = PIDController(kp=1.0, ki=0.1, kd=0.5)
        
        # Step response: setpoint=1, measurement goes from 0->1
        output_0 = controller.update(1.0, 0.0, 0.01)
        self.assertGreater(output_0, 0)  # Should command positive
        
        output_half = controller.update(1.0, 0.5, 0.01)
        self.assertLess(output_half, output_0)  # Error reduced
    
    def test_cascaded_controller_initialization(self):
        """Test cascaded controller can be created."""
        config = RacingConfig()
        controller = CascadedController(config)
        
        # Should not raise
        self.assertIsNotNone(controller)


class TestPlanning(unittest.TestCase):
    """Test planning modules."""
    
    def setUp(self):
        self.config = RacingConfig()
    
    def test_gate_planner_basic(self):
        """Test gate sequencing."""
        planner = GatePlanner(self.config)
        gates = [
            Gate(0, Vec3(0, 0, 10), radius=5.0),
            Gate(1, Vec3(20, 0, 10), radius=5.0),
            Gate(2, Vec3(20, 20, 10), radius=5.0)
        ]
        
        drone_pos = Vec3(0, 0, 10)
        next_gate, committed = planner.update(drone_pos, gates, 0)
        
        # First gate should target next gate
        self.assertEqual(next_gate, 1)
    
    def test_trajectory_planner_pure_pursuit(self):
        """Test Pure Pursuit steering."""
        planner = TrajectoryPlanner(self.config, lookahead_dist=5.0)
        
        waypoints = [
            Vec3(0, 0, 10),
            Vec3(10, 0, 10),
            Vec3(20, 5, 10)
        ]
        
        drone_pos = Vec3(0, 0, 10)
        drone_vel = Vec3(1, 0, 0)
        
        desired_dir, curvature = planner.compute_pure_pursuit(drone_pos, drone_vel, waypoints)
        
        # Should point roughly toward waypoints
        self.assertGreater(desired_dir.magnitude(), 0.1)
    
    def test_planner_integration(self):
        """Test full planning layer."""
        planner_int = PlannerIntegration(self.config)
        
        gates = [
            Gate(0, Vec3(0, 0, 10), radius=5.0),
            Gate(1, Vec3(30, 0, 10), radius=5.0)
        ]
        
        state = DroneState(
            timestamp=0.0,
            position=Vec3(0, 0, 10),
            velocity=Vec3(0, 0, 0)
        )
        
        plan = planner_int.plan(state, gates, 0, base_speed=20.0)
        
        self.assertIsNotNone(plan.target_position)
        self.assertGreater(plan.target_velocity, 0)


class StressTests(unittest.TestCase):
    """Stress tests for robustness."""
    
    def test_autonomy_with_noisy_data(self):
        """Test autonomy with high-noise simulator data."""
        print("\n[Stress] Testing with noisy data...")
        config = RacingConfig()
        autonomy = AutonomyStack(config)
        simulator = MockSimulator(num_gates=4)
        
        noise_std = 0.5  # High noise
        
        for step in range(200):
            sim_state = simulator.get_state()
            
            # Add noise
            pos = sim_state['position']
            noisy_pos = tuple(p + np.random.normal(0, noise_std) for p in pos)
            sim_state['position'] = noisy_pos
            
            control_output = autonomy.step(sim_state)
            self.assertIsNotNone(control_output)
            
            simulator.step(control_output, dt=0.02)
        
        print(f"  Completed {autonomy.loop_count} steps. Max loop time: {autonomy.max_loop_time_ms:.2f}ms")
    
    def test_autonomy_latency_variation(self):
        """Test autonomy with variable loop latency."""
        print("\n[Stress] Testing with latency variation...")
        config = RacingConfig()
        autonomy = AutonomyStack(config)
        simulator = MockSimulator(num_gates=4)
        
        for step in range(200):
            # Simulate variable latency
            if step % 10 == 0:
                time.sleep(0.005)  # Occasional delay
            
            sim_state = simulator.get_state()
            control_output = autonomy.step(sim_state)
            self.assertIsNotNone(control_output)
            simulator.step(control_output, dt=0.02)
        
        metrics = autonomy.get_performance_summary()
        p95_time = metrics['p95_loop_time_ms']
        print(f"  P95 loop time: {p95_time:.2f}ms")
        self.assertLess(p95_time, 30.0)  # Should handle 30ms
    
    def test_autonomy_large_motion(self):
        """Test autonomy with large commanded motions."""
        print("\n[Stress] Testing with large motions...")
        config = RacingConfig()
        autonomy = AutonomyStack(config)
        simulator = MockSimulator(num_gates=8, track_radius=100.0)  # Large track
        
        for step in range(500):
            sim_state = simulator.get_state()
            control_output = autonomy.step(sim_state)
            self.assertIsNotNone(control_output)
            
            # Check sanity
            self.assertTrue(-1 <= control_output.roll_setpoint <= 1)
            self.assertTrue(-1 <= control_output.pitch_setpoint <= 1)
            self.assertTrue(0 <= control_output.thrust <= 1)
            
            simulator.step(control_output, dt=0.02)
        
        print(f"  Completed {autonomy.loop_count} steps on large track")


def run_all_tests():
    """Run all tests."""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test suites
    suite.addTests(loader.loadTestsFromTestCase(TestTypes))
    suite.addTests(loader.loadTestsFromTestCase(TestPerception))
    suite.addTests(loader.loadTestsFromTestCase(TestStateEstimation))
    suite.addTests(loader.loadTestsFromTestCase(TestControl))
    suite.addTests(loader.loadTestsFromTestCase(TestPlanning))
    suite.addTests(loader.loadTestsFromTestCase(StressTests))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
