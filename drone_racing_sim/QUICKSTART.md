# Quick Start Guide

## 5-Minute Setup

```bash
# 1. Install dependencies (one-time)
source /opt/ros/humble/setup.bash
cd ~/aigp && colcon build --packages-select drone_racing_core
source ~/aigp/install/setup.bash

# 2. Set environment
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:~/aigp/drone_racing_sim/models"

# 3. Run simulation
ros2 launch drone_racing_core racing_sim.launch.py

# 4. In another terminal, monitor
ros2 topic echo /gate_validation/race_status
```

## Running a Single Race

### Terminal 1: Launch Simulation

```bash
source /opt/ros/humble/setup.bash
source ~/aigp/install/setup.bash
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:~/aigp/drone_racing_sim/models"

ros2 launch drone_racing_core racing_sim.launch.py
```

**Expected Output:**

```
[INFO] [launch]: All requested nodes have been started
[INFO] [Gazebo]: Gazebo loaded
[INFO] [gate_spawner]: Spawning 8 gates at circular track
[INFO] [gate_spawner]: Gate 0 spawned at (50.0, 0.0, 10.0)
...
[INFO] [autonomy_bridge]: Starting autonomy loop (50 Hz)
[INFO] [visualization]: Publishing markers to /visualization_marker_array
```

**Gazebo Window:**
- Drone visible at origin (0, 0, 1)
- 8 gates arranged in circle
- Ground plane and sky visible

**RViz Window (auto-launched):**
- Green cylinders = racing gates
- Red cylinder = current target gate
- White dots = drone trajectory path
- Text overlay = lap timer and current gate

### Terminal 2: Monitor Race Progress

```bash
# Watch gate passages
ros2 topic echo /gate_validation/current_gate

# Watch race status (IN_PROGRESS → COMPLETE or FAILED)
ros2 topic echo /gate_validation/race_status --rate 1

# Monitor lap time
ros2 topic echo /gate_validation/lap_time --rate 1

# View drone position
ros2 topic echo /odom
```

### Terminal 3: Virtual Joystick (Optional Emergency Control)

For manual override testing:

```bash
sudo apt install joystick
# Connect USB joystick
jstest /dev/input/js0

# OR use keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Configuration Parameters

### Launch Time Changes

```bash
# Modify number of gates
ros2 launch drone_racing_core racing_sim.launch.py num_gates:=10

# Aggressive autonomy (faster, more aggressive)
ros2 launch drone_racing_core racing_sim.launch.py autonomy_config:=aggressive

# Conservative autonomy (slower, more stable)
ros2 launch drone_racing_core racing_sim.launch.py autonomy_config:=conservative

# Larger track
ros2 launch drone_racing_core racing_sim.launch.py track_radius:=75.0

# Headless mode (no Gazebo GUI, faster)
ros2 launch drone_racing_core racing_sim.launch.py use_gui:=false
```

### Gazebo Physics Tuning

Edit [worlds/racing_track.world](worlds/racing_track.world):

```xml
<physics name="default_physics" type="ode">
    <max_step_size>0.001</max_step_size>  <!-- Decrease for accuracy, increase for speed -->
    <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

## Understanding Results

### During Simulation

**Command Line Output:**

```
[INFO] [autonomy_bridge]: Autonomy step 150 | v: 3.50 m/s | gate: 2
[INFO] [gate_validator]: Gate passage detected: gate_0, lap_time: 5.23s
[INFO] [gate_validator]: Race COMPLETE at 47.85s
```

**RViz Display:**

- **Green → Red Cylinder**: Target gate changes as passes complete
- **Trajectory Path**: White points show drone path (capped at 5000 points)
- **Text Overlay**: Real-time lap timer shows "Lap Time: 47.85s Gate: 0/8 ✓"
- **Gate Status**: Red = next target, Green = passed or not yet reached

### After Simulation

Results saved in `test_runs/run_YYYY-MM-DD_HH-MM-SS/`:

```
├── trajectory.csv          # Raw 50Hz trajectory log
├── report.json            # High-level summary
└── [timestamp_log.txt]    # Console output
```

**trajectory.csv columns:**

```
timestamp,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,speed,roll,pitch,yaw,current_gate
0.00,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1
0.06,0.1,0.0,1.02,2.0,0.1,-0.3,2.0,0.02,-0.5,0.0,0
...
```

**report.json structure:**

```json
{
  "metadata": {
    "start_time": "2024-01-15T10:30:22.123456",
    "end_time": "2024-01-15T10:31:09.654321",
    "status": "COMPLETE",
    "lap_time": 47.53,
    "gates_passed": 8
  },
  "gate_events": [
    {"gate_index": 0, "timestamp": 5.23, "duration_since_start": 5.23}
  ],
  "statistics": {
    "max_speed": 12.5,
    "avg_speed": 8.3,
    "min_speed": 0.1,
    "total_distance": 398.5,
    "num_samples": 4750
  }
}
```

## Analyze Results Programmatically

```python
import json
import pandas as pd

# Load trajectory
df = pd.read_csv('test_runs/run_XXX/trajectory.csv')
print(f"Max speed: {df['speed'].max():.2f} m/s")
print(f"Total distance: {df['position_x'].diff().abs().sum():.1f} m")

# Load report
with open('test_runs/run_XXX/report.json') as f:
    report = json.load(f)
print(f"Lap time: {report['metadata']['lap_time']:.2f}s")
print(f"Gates passed: {report['metadata']['gates_passed']}")
```

## Running Automated Tests

### Single Configuration

```bash
# Run 5 trials with default autonomy config, 60s each
python3 test_runner.py -n 5 -d 60 -c default

# Output: test_results/test_report.json with statistics
```

### Stress Testing

```bash
# Run permutations: 3 autonomy configs × randomized gates
python3 test_runner.py --stress-test -n 3 -d 120

# Will test: conservative, default, aggressive (each with randomized gates)
```

### Analysis

```bash
# Print summary
python3 -c "
import json
with open('test_results/test_report.json') as f:
    report = json.load(f)
summary = report['summary']
print(f\"Success Rate: {summary['success_rate_%']:.1f}%\")
print(f\"Best Lap: {summary['best_lap_time']:.2f}s\")
print(f\"Avg Lap: {summary['avg_lap_time']:.2f}s\")
"
```

## Troubleshooting

### Gazebo Doesn't Start

```bash
# Check model path
echo $GAZEBO_MODEL_PATH | grep drone_racing_sim
# If empty, run:
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:~/aigp/drone_racing_sim/models"

# Check Gazebo version
gazebo --version
# Expected: Gazebo 6.x or later

# Try headless mode (might work if GUI is broken)
export LIBGL_ALWAYS_INDIRECT=1
gazebo --verbose
```

### ROS2 Nodes Fail to Start

```bash
# Check workspace sourced
echo $ROS_PACKAGE_PATH | grep install
# If empty:
source ~/aigp/install/setup.bash

# Check package built
ros2 pkg list | grep drone_racing
# If not found:
cd ~/aigp && colcon build --packages-select drone_racing_core
source ~/aigp/install/setup.bash

# Check node entry points
ros2 run drone_racing_core gate_spawner
# If "not found", rebuild
```

### Drone Doesn't Move

```bash
# Check if autonomy bridge is publishing commands
ros2 topic list | grep cmd_vel
ros2 topic echo /cmd_vel

# If no messages, check bridge logs
ros2 run drone_racing_core autonomy_bridge --ros-args --log-level DEBUG

# Check if Gazebo is receiving commands
# Open Gazebo → World → Models → quadrotor → View → Properties
# Should show changing linear velocity
```

### No Trajectory Path in RViz

```bash
# RViz needs to be initialized on /drone_trajectory topic
# Manually add:
# 1. Click "Add" button in RViz
# 2. Select "Path"
# 3. Set Topic to "/drone_trajectory"
# 4. Change Color to white or light gray

# Or load provided config (if it exists)
# File → Open Config → configs/rviz_config.rviz
```

### High CPU Usage

```bash
# Reduce simulation rate and RViz path buffer
# Edit launches/racing_sim.launch.py:
# - Reduce max_step_size in world file
# - In visualization.py: reduce MAX_PATH_POINTS (currently 5000)

# Run headless (no GUI)
ros2 launch drone_racing_core racing_sim.launch.py use_gui:=false

# Kill unused terminals (multiple RViz instances are heavy)
ps aux | grep -i rviz
kill [PID]
```

### RViz Markers Not Visible

```bash
# Check if visualization node is running
ros2 node list | grep visualization
# If missing, check if autonomy_bridge crashed

# Check marker topic
ros2 topic list | grep marker
ros2 topic echo /visualization_marker_array --rate 1

# If markers exist but not showing in RViz
# 1. Check RViz Fixed Frame = "map" or "odom"
# 2. Add MarkerArray display with topic="/visualization_marker_array"
# 3. Increase marker marker size (Properties panel)
```

## Next Steps

- **Advanced**: Modify [autonomy stack](../drone_racing_stack/README.md) control parameters
- **Analysis**: Run [test_runner.py](test_runner.py) for multi-trial evaluation
- **Integration**: Connect to real drone hardware (requires Gazebo→Flight adapter)
- **Optimization**: Use Gazebo profiler to identify bottlenecks

## Performance Expectations

Typical single-trial performance:

```
Autonomy Config     | Lap Time  | Max Speed | Success Rate
                   |           |           |
conservative       | 90-120s   | 5-8 m/s   | 95%
default            | 50-70s    | 10-15 m/s | 90%
aggressive         | 35-50s    | 15-20 m/s | 75%
```

*Note: Results vary based on track radius, gate size, and hardware.*

## Getting Help

```bash
# Check if all nodes are healthy
ros2 node list --all

# Monitor node activity
ros2 node info /autonomy_bridge

# Check for errors
ros2 launch drone_racing_core racing_sim.launch.py --launch-prefix='xterm -e gdb -ex run --args'
```

See [BUILD.md](BUILD.md) for detailed installation and [README.md](README.md) for architecture details.
