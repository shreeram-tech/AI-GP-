#!/usr/bin/env python3
"""
AI Grand Prix Autonomous Drone Racing System - Setup Verification and Navigation

This script helps verify your installation and navigate the project structure.
"""

import os
import sys
import subprocess
from pathlib import Path


def check_file(path: str, description: str) -> bool:
    """Check if a file exists."""
    path_obj = Path(path).expanduser()
    exists = path_obj.exists()
    status = "✓" if exists else "✗"
    print(f"  {status} {description}: {path}")
    return exists


def check_command(cmd: str, description: str) -> bool:
    """Check if a command is available."""
    try:
        subprocess.run(['which', cmd], capture_output=True, check=True)
        print(f"  ✓ {description}: {cmd}")
        return True
    except:
        print(f"  ✗ {description}: {cmd} not found")
        return False


def main():
    print("\n" + "="*70)
    print("AI GRAND PRIX AUTONOMOUS DRONE RACING SYSTEM")
    print("Setup Verification and Navigation")
    print("="*70)
    
    # Check Phase 1: Autonomy Stack
    print("\n[Phase 1] Autonomy Stack")
    print("-" * 70)
    autonomy_files = [
        ('~/aigp/drone_racing_stack/src/autonomy.py', 'autonomy.py'),
        ('~/aigp/drone_racing_stack/src/control.py', 'control.py'),
        ('~/aigp/drone_racing_stack/src/planning.py', 'planning.py'),
        ('~/aigp/drone_racing_stack/src/state_estimation.py', 'state_estimation.py'),
        ('~/aigp/drone_racing_stack/src/types.py', 'types.py'),
        ('~/aigp/drone_racing_stack/example_run.py', 'example_run.py'),
        ('~/aigp/drone_racing_stack/README.md', 'README.md'),
    ]
    autonomy_ok = all(check_file(f, d) for f, d in autonomy_files)
    
    # Check Phase 2: Simulation
    print("\n[Phase 2] ROS2/Gazebo Simulation")
    print("-" * 70)
    sim_files = [
        ('~/aigp/drone_racing_sim/src/drone_racing_core/setup.py', 'setup.py'),
        ('~/aigp/drone_racing_sim/src/drone_racing_core/package.xml', 'package.xml'),
        ('~/aigp/drone_racing_sim/src/drone_racing_core/drone_racing_core/gate_spawner.py', 'gate_spawner.py'),
        ('~/aigp/drone_racing_sim/src/drone_racing_core/drone_racing_core/autonomy_bridge.py', 'autonomy_bridge.py'),
        ('~/aigp/drone_racing_sim/src/drone_racing_core/drone_racing_core/gate_validator.py', 'gate_validator.py'),
        ('~/aigp/drone_racing_sim/src/drone_racing_core/drone_racing_core/visualization.py', 'visualization.py'),
        ('~/aigp/drone_racing_sim/src/drone_racing_core/drone_racing_core/metrics_logger.py', 'metrics_logger.py'),
        ('~/aigp/drone_racing_sim/urdf/quadrotor.urdf.xacro', 'quadrotor.urdf.xacro'),
        ('~/aigp/drone_racing_sim/models/gate/model.sdf', 'gate/model.sdf'),
        ('~/aigp/drone_racing_sim/worlds/racing_track.world', 'racing_track.world'),
        ('~/aigp/drone_racing_sim/launches/racing_sim.launch.py', 'racing_sim.launch.py'),
        ('~/aigp/drone_racing_sim/configs/rviz_config.rviz', 'rviz_config.rviz'),
        ('~/aigp/drone_racing_sim/test_runner.py', 'test_runner.py'),
    ]
    sim_ok = all(check_file(f, d) for f, d in sim_files)
    
    # Check Documentation
    print("\n[Documentation]")
    print("-" * 70)
    doc_files = [
        ('~/aigp/drone_racing_sim/BUILD.md', 'BUILD.md'),
        ('~/aigp/drone_racing_sim/QUICKSTART.md', 'QUICKSTART.md'),
        ('~/aigp/drone_racing_sim/README.md', 'README.md'),
        ('~/aigp/COMPLETE_SYSTEM_SUMMARY.md', 'COMPLETE_SYSTEM_SUMMARY.md'),
    ]
    doc_ok = all(check_file(f, d) for f, d in doc_files)
    
    # Check Dependencies
    print("\n[System Dependencies]")
    print("-" * 70)
    deps_ok = all([
        check_command('ros2', 'ROS2 Humble'),
        check_command('gazebo', 'Gazebo'),
        check_command('python3', 'Python 3'),
        check_command('colcon', 'colcon build system'),
    ])
    
    # Summary and Next Steps
    print("\n" + "="*70)
    print("VERIFICATION SUMMARY")
    print("="*70)
    
    print(f"\nPhase 1 (Autonomy Stack):     {'✓ OK' if autonomy_ok else '✗ MISSING FILES'}")
    print(f"Phase 2 (Simulation):         {'✓ OK' if sim_ok else '✗ MISSING FILES'}")
    print(f"Documentation:                {'✓ OK' if doc_ok else '✗ MISSING FILES'}")
    print(f"System Dependencies:          {'✓ OK' if deps_ok else '✗ MISSING DEPENDENCIES'}")
    
    if autonomy_ok and sim_ok and doc_ok and deps_ok:
        print("\n✓ SYSTEM READY - All components verified!")
    else:
        print("\n✗ SYSTEM INCOMPLETE - Please check missing files above")
        if not deps_ok:
            print("  → See: ~/aigp/drone_racing_sim/BUILD.md for installation")
        sys.exit(1)
    
    # Quick Start Guide
    print("\n" + "="*70)
    print("QUICK START (5 Minutes)")
    print("="*70)
    
    print("""
Step 1: Setup Environment
    source /opt/ros/humble/setup.bash
    source ~/aigp/install/setup.bash
    export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:~/aigp/drone_racing_sim/models"

Step 2: Build (if not already done)
    cd ~/aigp
    colcon build --packages-select drone_racing_core

Step 3: Run Simulation
    ros2 launch drone_racing_core racing_sim.launch.py

Step 4: Monitor Progress
    ros2 topic echo /gate_validation/race_status --rate 1

Step 5: Analyze Results
    cat ~/aigp/drone_racing_sim/test_runs/run_*/report.json
""")
    
    # Navigation Guide
    print("="*70)
    print("PROJECT NAVIGATION")
    print("="*70)
    
    navigation = {
        "Getting Started": [
            ("~/aigp/COMPLETE_SYSTEM_SUMMARY.md", "Complete system overview"),
            ("~/aigp/drone_racing_sim/QUICKSTART.md", "5-minute quick start guide"),
            ("~/aigp/drone_racing_sim/BUILD.md", "Installation and build instructions"),
        ],
        "Implementation": [
            ("~/aigp/drone_racing_stack/", "Phase 1: Autonomy stack (classical control)"),
            ("~/aigp/drone_racing_sim/src/", "Phase 2: ROS2 nodes (integration layer)"),
            ("~/aigp/drone_racing_sim/urdf/", "Drone URDF model"),
            ("~/aigp/drone_racing_sim/models/", "Gate SDF model"),
            ("~/aigp/drone_racing_sim/worlds/", "Gazebo world file"),
        ],
        "Testing & Analysis": [
            ("~/aigp/drone_racing_sim/test_runner.py", "Automated test executor"),
            ("~/aigp/drone_racing_sim/test_runs/", "Simulation results (auto-generated)"),
            ("~/aigp/drone_racing_sim/test_results/", "Test reports (auto-generated)"),
        ],
        "Reference": [
            ("~/aigp/drone_racing_sim/README.md", "Complete architecture documentation"),
            ("~/aigp/drone_racing_stack/README.md", "Autonomy stack documentation"),
        ],
    }
    
    for category, items in navigation.items():
        print(f"\n{category}:")
        for path, desc in items:
            print(f"  → {path:<50} {desc}")
    
    # Next Steps
    print("\n" + "="*70)
    print("NEXT STEPS")
    print("="*70)
    print("""
1. Run Single Race
   ros2 launch drone_racing_core racing_sim.launch.py

2. Run Automated Tests
   cd ~/aigp/drone_racing_sim
   python3 test_runner.py -n 5 -d 60

3. Analyze Results
   python3 -c "import json; r=json.load(open('test_results/test_report.json')); print(r['summary'])"

4. Read Full Documentation
   cat ~/aigp/drone_racing_sim/README.md

5. Customize Configuration
   Edit: ~/aigp/drone_racing_stack/src/configs.py
   Test: python3 test_runner.py --stress-test
""")
    
    # Documentation Quick Links
    print("\n" + "="*70)
    print("DOCUMENTATION QUICK LINKS")
    print("="*70)
    
    links = {
        "Installation": "~/aigp/drone_racing_sim/BUILD.md",
        "Quick Start": "~/aigp/drone_racing_sim/QUICKSTART.md",
        "Main README": "~/aigp/drone_racing_sim/README.md",
        "Complete Summary": "~/aigp/COMPLETE_SYSTEM_SUMMARY.md",
        "Autonomy Stack": "~/aigp/drone_racing_stack/README.md",
    }
    
    for title, path in links.items():
        expanded_path = Path(path).expanduser()
        if expanded_path.exists():
            print(f"\n• {title}")
            print(f"  File: {path}")
            print(f"  Open: gnome-open {path}  # or: cat {path}")
    
    print("\n" + "="*70)
    print("System Status: ✓ READY")
    print("="*70 + "\n")


if __name__ == '__main__':
    main()
