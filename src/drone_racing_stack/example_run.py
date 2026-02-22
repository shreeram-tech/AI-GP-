"""
Example: Running autonomy stack with simulator.

Demonstrates:
1. Initialization of autonomy stack
2. Main simulation loop
3. Monitoring performance
"""

import sys
import time
from typing import Optional

# Add src to path
sys.path.insert(0, '/home/shree/aigp/drone_racing_stack')

from src.autonomy import AutonomyStack
from src.perception import MockSimulator
from src.types import RacingConfig


def run_mock_simulation(num_steps: int = 1000, render: bool = False) -> dict:
    """
    Run a complete simulation with mock simulator.
    
    Args:
        num_steps: Number of control steps to execute
        render: If True, print trajectory data
    
    Returns:
        Metrics dictionary
    """
    print("=" * 60)
    print("DRONE RACING AUTONOMY STACK - SIMULATION")
    print("=" * 60)
    
    # Create autonomy stack with default config
    config = RacingConfig()
    autonomy = AutonomyStack(config)
    
    # Create mock simulator
    simulator = MockSimulator(num_gates=8, track_radius=50.0)
    
    print(f"\nConfig:")
    print(f"  Max velocity: {config.max_velocity} m/s")
    print(f"  Max acceleration: {config.max_acceleration} m/s^2")
    print(f"  Position control gains: Kp={config.pos_kp}, Ki={config.pos_ki}, Kd={config.pos_kd}")
    print(f"  Velocity control gains: Kp={config.vel_kp}, Ki={config.vel_ki}, Kd={config.vel_kd}")
    print(f"  Attitude control gains: Kp={config.att_kp}, Ki={config.att_ki}, Kd={config.att_kd}")
    
    print(f"\nSimulation:")
    print(f"  Number of gates: {len(simulator.gates)}")
    print(f"  Track radius: 50 m")
    print(f"  Simulation steps: {num_steps}")
    
    print("\nStarting simulation...")
    print("-" * 60)
    
    sim_time = 0.0
    dt = 0.02  # 50 Hz
    crashed = False
    
    try:
        for step in range(num_steps):
            # Get current simulator state
            sim_state = simulator.get_state()
            
            # Run autonomy
            control_output = autonomy.step(sim_state)
            
            if control_output is None:
                print(f"[ERROR] Autonomy failed at step {step}")
                crashed = True
                break
            
            # Simulate one step
            simulator.step(control_output, dt=dt)
            sim_time += dt
            
            # Check for crashes
            if simulator.position[2] < 1.0:
                print(f"[CRASH] Drone hit ground at step {step} (z={simulator.position[2]:.2f}m)")
                crashed = True
                break
            
            # Progress indicator
            if (step + 1) % 500 == 0:
                print(f"  Step {step+1}/{num_steps}, Sim time: {sim_time:.1f}s")
    
    except Exception as e:
        print(f"[ERROR] Simulation crashed: {e}")
        import traceback
        traceback.print_exc()
        crashed = True
    
    print("-" * 60)
    
    # Results
    metrics = autonomy.get_performance_summary()
    
    print("\nRESULTS")
    print("=" * 60)
    print(f"Status: {'CRASHED' if crashed else 'COMPLETED'}")
    print(f"Total simulation time: {sim_time:.1f}s")
    print(f"Total control steps: {autonomy.loop_count}")
    
    print("\nPerformance:")
    print(f"  Average loop time: {metrics.get('avg_loop_time_ms', 0):.2f}ms")
    print(f"  Max loop time: {metrics.get('max_loop_time_ms', 0):.2f}ms")
    print(f"  P95 loop time: {metrics.get('p95_loop_time_ms', 0):.2f}ms")
    print(f"  Target: < 20ms (50Hz)")
    
    print("\nTiming breakdown (avg):")
    print(f"  Perception: {metrics.get('avg_perception_ms', 0):.2f}ms")
    print(f"  Estimation: {metrics.get('avg_estimation_ms', 0):.2f}ms")
    print(f"  Planning: {metrics.get('avg_planning_ms', 0):.2f}ms")
    print(f"  Control: {metrics.get('avg_control_ms', 0):.2f}ms")
    
    print("\nPerception stats:")
    perc_stat = metrics.get('perception_stat', {})
    print(f"  Good frames: {perc_stat.get('good_frames', 0)}")
    print(f"  Bad frames: {perc_stat.get('bad_frames', 0)}")
    
    print("\nFinal drone state:")
    print(f"  Position: ({simulator.position[0]:.2f}, {simulator.position[1]:.2f}, {simulator.position[2]:.2f})")
    print(f"  Velocity: ({simulator.velocity[0]:.2f}, {simulator.velocity[1]:.2f}, {simulator.velocity[2]:.2f})")
    print(f"  Speed: {np.linalg.norm(simulator.velocity):.2f} m/s")
    print(f"  Current gate: {simulator.current_gate_index}")
    
    # Trajectory analysis (if history available)
    if autonomy.state_history:
        history = autonomy.state_history
        max_speed = max(h['speed'] for h in history)
        avg_speed = sum(h['speed'] for h in history) / len(history)
        print(f"\nTrajectory analysis:")
        print(f"  Max speed achieved: {max_speed:.2f} m/s")
        print(f"  Average speed: {avg_speed:.2f} m/s")
    
    print("=" * 60)
    
    return {
        'crashed': crashed,
        'sim_time': sim_time,
        'loop_count': autonomy.loop_count,
        'metrics': metrics,
        'state_history': autonomy.state_history
    }


if __name__ == "__main__":
    import numpy as np
    
    # Run simulation
    result = run_mock_simulation(num_steps=1000)
    
    # Additional analysis
    if result['state_history']:
        print("\n" + "=" * 60)
        print("TRAJECTORY DUMP (sample)")
        print("=" * 60)
        
        # Print first few states
        for i in range(0, min(10, len(result['state_history'])), 2):
            h = result['state_history'][i]
            print(f"[{h['timestamp']:.2f}s] pos=({h['position'][0]:.1f}, {h['position'][1]:.1f}), "
                  f"speed={h['speed']:.2f}m/s, gate={h['target_velocity']:.1f}")
