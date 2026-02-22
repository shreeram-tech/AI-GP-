"""
Data analysis and visualization for drone racing telemetry.

Analyzes:
- Trajectory (position over time)
- Speed profile
- Control inputs
- Timing metrics
- Gate passages
"""

import sys
import json
import numpy as np
from typing import List, Dict, Optional

sys.path.insert(0, '/home/shree/aigp/drone_racing_stack')

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


class TelemetryAnalyzer:
    """Analyze flight data from autonomy stack."""
    
    def __init__(self, state_history: List[Dict], metrics: Dict):
        """
        Args:
            state_history: List of state records from AutonomyStack
            metrics: Performance summary dict
        """
        self.history = state_history
        self.metrics = metrics
    
    def compute_statistics(self) -> Dict:
        """Compute performance statistics from telemetry."""
        if not self.history:
            return {}
        
        positions = np.array([h['position'] for h in self.history])
        velocities = np.array([h['velocity'] for h in self.history])
        speeds = np.array([h['speed'] for h in self.history])
        
        # Trajectory
        total_distance = np.sum([
            np.linalg.norm(positions[i+1] - positions[i]) 
            for i in range(len(positions)-1)
        ])
        
        # Speed statistics
        max_speed = np.max(speeds)
        avg_speed = np.mean(speeds)
        min_speed = np.min(speeds)
        
        # Acceleration
        accelerations = np.array([
            np.linalg.norm(velocities[i+1] - velocities[i])
            for i in range(len(velocities)-1)
        ]) / 0.02  # dt = 0.02
        max_accel = np.max(accelerations)
        avg_accel = np.mean(accelerations)
        
        # Control inputs
        roll_cmds = np.array([h['roll_cmd'] for h in self.history])
        pitch_cmds = np.array([h['pitch_cmd'] for h in self.history])
        thrusts = np.array([h['thrust'] for h in self.history])
        
        return {
            'total_distance_m': total_distance,
            'max_speed_ms': float(max_speed),
            'avg_speed_ms': float(avg_speed),
            'min_speed_ms': float(min_speed),
            'max_accel_mss': float(max_accel),
            'avg_accel_mss': float(avg_accel),
            'max_roll_cmd': float(np.max(np.abs(roll_cmds))),
            'avg_roll_cmd': float(np.mean(np.abs(roll_cmds))),
            'max_pitch_cmd': float(np.max(np.abs(pitch_cmds))),
            'avg_pitch_cmd': float(np.mean(np.abs(pitch_cmds))),
            'avg_thrust': float(np.mean(thrusts)),
        }
    
    def identify_gate_passes(self, gates_data: List[Dict]) -> List[Dict]:
        """
        Identify when drone passed through gates.
        
        Returns:
            List of gate passage events with timestamps
        """
        positions = np.array([h['position'] for h in self.history])
        times = np.array([h['timestamp'] for h in self.history])
        
        gates_passed = []
        
        for i, gate in enumerate(gates_data):
            gate_pos = np.array(gate['position'])
            gate_radius = gate.get('radius', 5.0)
            
            # Find closest approach
            distances = np.linalg.norm(positions - gate_pos, axis=1)
            min_dist_idx = np.argmin(distances)
            min_dist = distances[min_dist_idx]
            
            if min_dist < gate_radius + 1.0:  # 1m margin
                gates_passed.append({
                    'gate_index': i,
                    'time': float(times[min_dist_idx]),
                    'distance': float(min_dist),
                    'position': positions[min_dist_idx].tolist()
                })
        
        return sorted(gates_passed, key=lambda x: x['time'])
    
    def estimate_lap_time(self) -> Optional[float]:
        """Estimate lap completion time (all gates passed in sequence)."""
        if not self.history:
            return None
        
        total_time = self.history[-1]['timestamp'] - self.history[0]['timestamp']
        return total_time
    
    def print_summary(self):
        """Print analysis summary to console."""
        stats = self.compute_statistics()
        
        print("\n" + "=" * 60)
        print("TRAJECTORY ANALYSIS")
        print("=" * 60)
        
        print("\nDistance & Speed:")
        print(f"  Total distance: {stats.get('total_distance_m', 0):.1f} m")
        print(f"  Max speed: {stats.get('max_speed_ms', 0):.2f} m/s")
        print(f"  Avg speed: {stats.get('avg_speed_ms', 0):.2f} m/s")
        print(f"  Min speed: {stats.get('min_speed_ms', 0):.2f} m/s")
        
        print("\nDynamics:")
        print(f"  Max acceleration: {stats.get('max_accel_mss', 0):.2f} m/s²")
        print(f"  Avg acceleration: {stats.get('avg_accel_mss', 0):.2f} m/s²")
        
        print("\nControl Activity:")
        print(f"  Max roll command: {stats.get('max_roll_cmd', 0):.3f} rad")
        print(f"  Max pitch command: {stats.get('max_pitch_cmd', 0):.3f} rad")
        print(f"  Avg thrust: {stats.get('avg_thrust', 0):.2f}")
        
        lap_time = self.estimate_lap_time()
        if lap_time:
            print(f"\nLap time: {lap_time:.1f} s")
    
    def plot_3d_trajectory(self, show_gates: bool = True):
        """Plot 3D drone trajectory."""
        if not HAS_MATPLOTLIB:
            print("[Warning] matplotlib not available, skipping plots")
            return
        
        positions = np.array([h['position'] for h in self.history])
        
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Trajectory
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
               'b-', linewidth=1, label='Trajectory')
        
        # Start and end
        ax.scatter([positions[0, 0]], [positions[0, 1]], [positions[0, 2]], 
                  c='green', s=100, marker='o', label='Start')
        ax.scatter([positions[-1, 0]], [positions[-1, 1]], [positions[-1, 2]], 
                  c='red', s=100, marker='x', label='End')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.legend()
        ax.set_title('Drone Trajectory (3D)')
        plt.tight_layout()
        return fig
    
    def plot_speed_profile(self):
        """Plot speed over time."""
        if not HAS_MATPLOTLIB:
            print("[Warning] matplotlib not available, skipping plots")
            return
        
        times = np.array([h['timestamp'] for h in self.history])
        speeds = np.array([h['speed'] for h in self.history])
        target_speeds = np.array([h['target_velocity'] for h in self.history])
        
        fig, ax = plt.subplots(figsize=(12, 5))
        
        ax.plot(times, speeds, 'b-', label='Actual speed', linewidth=1.5)
        ax.plot(times, target_speeds, 'r--', label='Target speed', linewidth=1.5)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (m/s)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_title('Speed Profile')
        plt.tight_layout()
        return fig
    
    def plot_control_inputs(self):
        """Plot roll, pitch, thrust commands over time."""
        if not HAS_MATPLOTLIB:
            print("[Warning] matplotlib not available, skipping plots")
            return
        
        times = np.array([h['timestamp'] for h in self.history])
        rolls = np.array([h['roll_cmd'] for h in self.history])
        pitches = np.array([h['pitch_cmd'] for h in self.history])
        thrusts = np.array([h['thrust'] for h in self.history])
        
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
        
        ax1.plot(times, rolls, 'r-', linewidth=1)
        ax1.set_ylabel('Roll (rad)')
        ax1.grid(True, alpha=0.3)
        ax1.set_title('Control Inputs')
        
        ax2.plot(times, pitches, 'g-', linewidth=1)
        ax2.set_ylabel('Pitch (rad)')
        ax2.grid(True, alpha=0.3)
        
        ax3.plot(times, thrusts, 'b-', linewidth=1)
        ax3.set_ylabel('Thrust (norm)')
        ax3.set_xlabel('Time (s)')
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        return fig
    
    def plot_xy_overhead(self, gates_data: Optional[List[Dict]] = None):
        """Plot top-down (X-Y) view with gates."""
        if not HAS_MATPLOTLIB:
            print("[Warning] matplotlib not available, skipping plots")
            return
        
        positions = np.array([h['position'] for h in self.history])
        
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Trajectory
        ax.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=1.5, label='Path')
        ax.scatter([positions[0, 0]], [positions[0, 1]], c='green', s=150, marker='o', 
                  label='Start', zorder=5)
        ax.scatter([positions[-1, 0]], [positions[-1, 1]], c='red', s=150, marker='x', 
                  label='End', zorder=5)
        
        # Gates
        if gates_data:
            for i, gate in enumerate(gates_data):
                pos = gate['position']
                radius = gate.get('radius', 5.0)
                circle = patches.Circle((pos[0], pos[1]), radius, 
                                       fill=False, edgecolor='orange', linewidth=2)
                ax.add_patch(circle)
                ax.text(pos[0], pos[1], f'G{i}', ha='center', va='center')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        ax.set_title('Top-Down View (X-Y)')
        plt.tight_layout()
        return fig


def main():
    """Example analysis run."""
    from example_run import run_mock_simulation
    
    print("\nRunning example simulation for analysis...")
    result = run_mock_simulation(num_steps=500)
    
    if result['state_history']:
        analyzer = TelemetryAnalyzer(result['state_history'], result['metrics'])
        
        # Print summary
        analyzer.print_summary()
        
        # Save plots
        if HAS_MATPLOTLIB:
            print("\nGenerating plots...")
            
            # 3D trajectory
            fig = analyzer.plot_3d_trajectory()
            if fig:
                fig.savefig('/home/shree/aigp/drone_racing_stack/logs/trajectory_3d.png', dpi=100)
                print("  ✓ Saved trajectory_3d.png")
            
            # Speed profile
            fig = analyzer.plot_speed_profile()
            if fig:
                fig.savefig('/home/shree/aigp/drone_racing_stack/logs/speed_profile.png', dpi=100)
                print("  ✓ Saved speed_profile.png")
            
            # Control inputs
            fig = analyzer.plot_control_inputs()
            if fig:
                fig.savefig('/home/shree/aigp/drone_racing_stack/logs/control_inputs.png', dpi=100)
                print("  ✓ Saved control_inputs.png")
            
            # Overhead view
            from src.perception import MockSimulator
            sim = MockSimulator(num_gates=8)
            gates_data = [{'position': g['position'], 'radius': g.get('radius', 5.0)} 
                         for g in sim.gates]
            fig = analyzer.plot_xy_overhead(gates_data)
            if fig:
                fig.savefig('/home/shree/aigp/drone_racing_stack/logs/overhead_view.png', dpi=100)
                print("  ✓ Saved overhead_view.png")
            
            print("\n[Plots saved to logs/]")
        else:
            print("\n[matplotlib not available - install to generate plots]")


if __name__ == "__main__":
    main()
