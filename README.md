# Tractor Launch Packages

This meta-package contains the Autoware vehicle configuration, URDFs, and launch files for simulating and operating the **Heli Tow Tractor 12t**. 

## Project Structure

The project is divided into four main packages:

*   **`tractor_description`**: Contains the vehicle model, URDF definitions, TF logic, and vehicle parameters (`vehicle_info.param.yaml`). It also includes the custom `nmb_odometer_node` used for simulating the complex articulated kinematics of the tractor, drawbar, and trailers.
*   **`tractor_launch`**: Contains the top-level Autoware launch files specific to the tractor, such as the `planning_simulator.launch.xml` which integrates the vehicle into the Autoware stack.
*   **`tractor_sensor_kit_description`**: Contains URDF definitions and mounting parameters for all sensors equipped on the vehicle.
*   **`tractor_sensor_kit_launch`**: Contains launch files for initializing sensor drivers and pointcloud/camera pipelines.

## Heli Tow Tractor 12t Information (Demo Vehicle)

The demo vehicle is configured as an **Articulated Tow Tractor** towing a drawbar and a trailer.

**Tractor Specifications:**
*   **Wheelbase**: 1.4 m
*   **Wheel Tread**: 1.0 m
*   **Front Overhang**: 0.85 m
*   **Rear Overhang**: 0.70 m
*   **Vehicle Height**: 2.0 m
*   **Max Steering Angle**: 0.70 rad (~40 degrees)

**Articulation & Towing:**
*   **Hitch Length (Tractor to Drawbar Pivot)**: 0.70 m
*   **Drawbar Joint Length**: 1.53 m
*   **Trailer Joint Length**: 1.98 m
*   **Trailer Overhang (Left/Right)**: 0.75 m

## Usage

To build the launcher packages, use `colcon build` targeting the packages in this directory:

```bash
cd ~/AutoTract/tracter_ws
colcon build --symlink-install --packages-select tractor_description tractor_launch tractor_sensor_kit_description tractor_sensor_kit_launch
source install/setup.bash
```

## Simulating in RVIZ

To launch the vehicle in the Autoware Planning Simulator (RVIZ):

```bash
ros2 launch tractor_launch planning_simulator.launch.xml map_path:=/home/tacv/autoware_map/minebae vehicle_model:=tractor sensor_model:=tractor_sensor_kit
```

### Manual Control (Teleop)
To manually verify vehicle kinematics and trailer behavior in RVIZ, you can use the custom `tracter_control` teleop node:

1. In a new terminal, source the workspace and run:
   ```bash
   ros2 run tracter_control ackermann_teleop_node
   ```
2. Press `Spacebar` to **Enable** the system and automatically **Engage** the vehicle in REMOTE mode.
3. Use `W`, `A`, `S`, `D` to drive and steer the tractor.
4. Press `Spacebar` again to **Disable** teleop and automatically return the vehicle to AUTONOMOUS (Local) mode.
