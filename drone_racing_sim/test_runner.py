#!/usr/bin/env python3
"""
Automated Test Runner for Drone Racing Simulator

Functionality:
  - Spawns multiple simulation trials
  - Configures different race conditions
  - Collects metrics from each run
  - Generates performance reports
  - Supports parallel testing
"""

import os
import sys
import subprocess
import time
import json
import csv
import argparse
from pathlib import Path
from datetime import datetime
import numpy as np


class RaceSimulationRunner:
    """Automated test runner for race simulations."""
    
    def __init__(self, num_gates: int = 8, track_radius: float = 50.0,
                 output_dir: str = '~/aigp/drone_racing_sim/test_results'):
        self.num_gates = num_gates
        self.track_radius = track_radius
        self.output_dir = Path(output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.test_results: List[Dict] = []
        self.start_time = datetime.now()
    
    def run_trial(self, trial_id: int, config: Dict = None,
                  duration: float = 60.0) -> Dict:
        """
        Run a single race trial.
        
        Args:
            trial_id: Unique trial identifier
            config: Configuration overrides (autonomy_config, randomize_gates, etc.)
            duration: Simulation duration in seconds
        
        Returns:
            Results dictionary
        """
        config = config or {}
        trial_name = f'trial_{trial_id:03d}'
        trial_dir = self.output_dir / trial_name
        trial_dir.mkdir(exist_ok=True)
        
        print(f"\n{'='*60}")
        print(f"Starting Trial {trial_id}: {trial_name}")
        print(f"{'='*60}")
        print(f"Config: {config}")
        print(f"Duration: {duration}s")
        
        trial_result = {
            'trial_id': trial_id,
            'start_time': datetime.now().isoformat(),
            'config': config,
            'duration': duration,
            'result': 'PENDING'
        }
        
        try:
            # Build ROS2 launch command with overrides
            cmd = [
                'ros2', 'launch', 'drone_racing_core', 'racing_sim.launch.py',
                f'num_gates:={config.get("num_gates", self.num_gates)}',
                f'track_radius:={config.get("track_radius", self.track_radius)}',
                f'autonomy_config:={config.get("autonomy_config", "default")}',
            ]
            
            # Run simulation
            print(f"Launching: {' '.join(cmd)}")
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                                   stderr=subprocess.PIPE)
            
            # Wait for specified duration
            time.sleep(duration)
            
            # Graceful shutdown
            print("Stopping simulation...")
            proc.terminate()
            proc.wait(timeout=10)
            
            # Check for results
            results_file = self._find_latest_run()
            if results_file:
                with open(results_file, 'r') as f:
                    run_data = json.load(f)
                
                trial_result['result'] = run_data['metadata']['status']
                trial_result['lap_time'] = run_data['metadata'].get('lap_time')
                trial_result['gates_passed'] = run_data['metadata'].get('gates_passed')
                trial_result['stats'] = run_data.get('statistics', {})
            else:
                trial_result['result'] = 'NO_DATA'
            
            trial_result['end_time'] = datetime.now().isoformat()
            
        except subprocess.TimeoutExpired:
            print("ERROR: Simulation timeout")
            trial_result['result'] = 'TIMEOUT'
            proc.kill()
        
        except Exception as e:
            print(f"ERROR: {e}")
            trial_result['result'] = 'FAILED'
            trial_result['error'] = str(e)
        
        self.test_results.append(trial_result)
        
        # Print trial summary
        self._print_trial_summary(trial_result)
        
        return trial_result
    
    def run_multi_trial(self, num_trials: int = 5,
                       base_config: Dict = None,
                       randomize_config: bool = False) -> List[Dict]:
        """
        Run multiple trials with optional configuration variation.
        
        Args:
            num_trials: Number of trials to run
            base_config: Base configuration
            randomize_config: If True, randomize some parameters each trial
        
        Returns:
            List of trial results
        """
        base_config = base_config or {}
        results = []
        
        for i in range(num_trials):
            config = base_config.copy()
            
            if randomize_config:
                # Vary some parameters for stress testing
                config['track_radius'] = base_config.get('track_radius', 50.0) * \
                    np.random.uniform(0.8, 1.2)
                config['num_gates'] = int(base_config.get('num_gates', 8) * \
                    np.random.uniform(0.9, 1.1))
            
            result = self.run_trial(i, config)
            results.append(result)
            
            # Wait between trials
            time.sleep(2)
        
        return results
    
    def _find_latest_run(self) -> Path:
        """Find the most recent run directory."""
        run_dir = Path('/home/shree/aigp/drone_racing_sim/test_runs')
        if not run_dir.exists():
            return None
        
        runs = sorted(run_dir.glob('run_*'), key=lambda p: p.stat().st_mtime)
        if runs:
            latest = runs[-1]
            report = latest / 'report.json'
            if report.exists():
                return report
        
        return None
    
    def generate_report(self) -> Dict:
        """Generate comprehensive test report."""
        if not self.test_results:
            print("No test results to report")
            return {}
        
        report = {
            'test_run_info': {
                'total_trials': len(self.test_results),
                'start_time': self.start_time.isoformat(),
                'end_time': datetime.now().isoformat(),
                'duration_seconds': (datetime.now() - self.start_time).total_seconds()
            },
            'results': self.test_results,
            'summary': self._compute_summary()
        }
        
        # Save report
        report_file = self.output_dir / 'test_report.json'
        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2)
        
        print(f"\nReport saved to: {report_file}")
        
        return report
    
    def _compute_summary(self) -> Dict:
        """Compute summary statistics."""
        if not self.test_results:
            return {}
        
        statuses = [r['result'] for r in self.test_results]
        lap_times = [r.get('lap_time') for r in self.test_results if r.get('lap_time')]
        gates_passed = [r.get('gates_passed', 0) for r in self.test_results]
        
        return {
            'success_count': statuses.count('COMPLETE'),
            'failure_count': statuses.count('FAILED'),
            'timeout_count': statuses.count('TIMEOUT'),
            'success_rate_%': (statuses.count('COMPLETE') / len(statuses) * 100) if statuses else 0,
            'avg_lap_time': np.mean(lap_times) if lap_times else None,
            'best_lap_time': np.min(lap_times) if lap_times else None,
            'worst_lap_time': np.max(lap_times) if lap_times else None,
            'avg_gates_passed': np.mean(gates_passed) if gates_passed else 0,
        }
    
    def _print_trial_summary(self, trial_result: Dict):
        """Print trial summary."""
        print(f"\nTrial Result:")
        print(f"  Status: {trial_result['result']}")
        if trial_result.get('lap_time'):
            print(f"  Lap Time: {trial_result['lap_time']:.2f}s")
        if trial_result.get('gates_passed'):
            print(f"  Gates Passed: {trial_result['gates_passed']}")
        if trial_result.get('stats'):
            stats = trial_result['stats']
            print(f"  Max Speed: {stats.get('max_speed', 0):.2f} m/s")
            print(f"  Avg Speed: {stats.get('avg_speed', 0):.2f} m/s")
    
    def print_summary(self):
        """Print overall test summary."""
        summary = self._compute_summary()
        
        print(f"\n{'='*60}")
        print("OVERALL TEST SUMMARY")
        print(f"{'='*60}")
        print(f"Total Trials: {len(self.test_results)}")
        print(f"Success Rate: {summary.get('success_rate_%', 0):.1f}%")
        print(f"Successful: {summary.get('success_count', 0)}")
        print(f"Failed: {summary.get('failure_count', 0)}")
        print(f"Timeouts: {summary.get('timeout_count', 0)}")
        
        if summary.get('avg_lap_time'):
            print(f"\nLap Times:")
            print(f"  Average: {summary['avg_lap_time']:.2f}s")
            print(f"  Best: {summary['best_lap_time']:.2f}s")
            print(f"  Worst: {summary['worst_lap_time']:.2f}s")
        
        print(f"Average Gates Passed: {summary.get('avg_gates_passed', 0):.1f}")
        print(f"{'='*60}\n")


def main():
    parser = argparse.ArgumentParser(
        description='Automated drone racing simulator test runner'
    )
    parser.add_argument('-n', '--num-trials', type=int, default=5,
                       help='Number of trials to run')
    parser.add_argument('-d', '--duration', type=float, default=60.0,
                       help='Duration of each trial (seconds)')
    parser.add_argument('-c', '--config', default='default',
                       help='Autonomy config (default, aggressive, conservative)')
    parser.add_argument('-r', '--randomize', action='store_true',
                       help='Randomize gate layout each trial')
    parser.add_argument('--stress-test', action='store_true',
                       help='Run stress test with varied configurations')
    
    args = parser.parse_args()
    
    print("Drone Racing Simulator - Automated Test Runner")
    print(f"Configuration: {args.config}")
    print(f"Trials: {args.num_trials}, Duration: {args.duration}s each")
    
    runner = RaceSimulationRunner()
    
    if args.stress_test:
        # Stress test with varied configs
        print("\nRunning STRESS TEST with varied configurations...")
        base_configs = [
            {'autonomy_config': 'conservative', 'randomize': False},
            {'autonomy_config': 'default', 'randomize': False},
            {'autonomy_config': 'aggressive', 'randomize': False},
            {'autonomy_config': 'default', 'randomize': True},
        ]
        
        for i, config in enumerate(base_configs):
            print(f"\n[Stress Test Set {i+1}] Config: {config}")
            runner.run_trial(i, config, duration=args.duration)
    
    else:
        # Normal multi-trial run
        config = {'autonomy_config': args.config}
        runner.run_multi_trial(
            num_trials=args.num_trials,
            base_config=config,
            randomize_config=args.randomize
        )
    
    # Generate report
    runner.generate_report()
    runner.print_summary()


if __name__ == '__main__':
    main()
