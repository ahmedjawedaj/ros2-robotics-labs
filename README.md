# ROS 2 Robotics Labs

ROS 2 Python packages covering messages, turtlesim motion, TurtleBot3 perception/mapping and camera-guided track following.

Maintained by [Ahmed Jawed](https://github.com/ahmedjawedaj).

## Methods and engineering work

- Custom ComplexNumber message and publisher/subscriber examples.
- Turtlesim straight, square, circle, go-to-goal and chase behaviours.
- Laser-scan processing, transforms, mapping and robot control.
- OpenCV camera-mask track-centre extraction and velocity commands.

## Repository contents

- [`src/`](src/): ROS 2 packages and custom interfaces.
- [`data/README.md`](data/README.md): required external inputs and data handling.
- [`PROVENANCE.md`](PROVENANCE.md): original source-date evidence, attribution and publication changes.
- [`scripts/validate_sources.py`](scripts/validate_sources.py): notebook/Python syntax and exclusion checks.

## Recorded observations

- This repository publishes source packages rather than robot bag files, recordings or workstation build trees.
- ROS runtime/simulator verification requires a compatible ROS 2 workspace and is not claimed from Python syntax validation.

## Setup and use

Source your installed ROS 2 environment first, then build in a disposable workspace:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 run lab1 publisher
ros2 launch lab2 chase_turtle.launch.py
```

Inspect package `setup.py` entry points and launch files for the other nodes. Start the appropriate turtlesim/TurtleBot3 simulator and verify topic names before running motion nodes. ROS 2 dependencies are supplied by your ROS installation, not pip requirements alone.

## Data and weights

Install ROS 2 with turtlesim, TurtleBot3 interfaces, cv_bridge and OpenCV as appropriate. No captured sensor data or environment recordings are included.

## Validation

```bash
python -m pip install -r requirements-validation.txt
python scripts/validate_sources.py
```

The publication checks validate notebook structure and compile source cells/Python modules. They do not reproduce training, GPU timing or ROS hardware behaviour. Cleaned notebooks have no captured outputs or stale execution counters.

## Scope and limitations

- Physical motion requires supervision and tested velocity limits.
- Track following expects /camera/image_raw and publishes /cmd_vel; verify topics and emergency stop before connecting a robot.
- The empty scaffold package is omitted.

## References and attribution

- [ROS 2 documentation](https://docs.ros.org/en/rolling/)

See PROVENANCE.md for the distinction between original work dates and actual publication dates. This repository excludes instructor handouts, raw data, model weights and private records.
