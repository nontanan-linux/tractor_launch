# tractor_description

`tractor_description` is a ROS 2 package containing the vehicle model, 3D meshes, URDF (Xacro) definitions, and vehicle parameters for the **Heli Tow Tractor 12t** (including a drawbar and a trailer). This package is designed to integrate the articulated vehicle model into the Autoware Universe ecosystem.

In addition to static descriptions, this package includes a custom kinematic simulator node (`nmb_odometer_node.py`) responsible for calculating the complex articulation angles and publishing the correct TF (Transform) tree for the drawbar and trailer during simulation.

## Package Structure

*   **`config/`**: Contains YAML parameter files used by Autoware.
    *   `vehicle_info.param.yaml`: The core vehicle dimensions (wheelbase, overhangs, steering limits, and trailer joint lengths).
    *   `simulator_model.param.yaml`: Simulator-specific parameters.
    *   `mirror.param.yaml`: Configuration for vehicle mirrors.
*   **`mesh/`**: Contains the 3D visual assets (`.obj`, `.mtl`, `.stl`) for the tractor chassis, wheels, drawbar, and trailer.
*   **`scripts/`**: Contains Python nodes for vehicle simulation.
    *   `nmb_odometer_node.py`: A custom node that subscribes to the tractor's odometry/velocity and mathematically simulates the physics and heading of the towed drawbar and trailer. It publishes the dynamic TFs required for visualization in RViz.
    *   `kinematic_model.py`: Core mathematical models for the articulated kinematics.
*   **`urdf/`**: Contains the Xacro macro files that build the vehicle's TF tree.
    *   `vehicle.xacro`: The top-level file combining the tractor and sensors.
    *   `tractor.xacro`: The main towing vehicle URDF.
    *   `drawbar.xacro` & `trailer.xacro`: The URDFs for the towed attachments.
    *   `tractor_trailer.xacro`: The combined assembly macro.

## Key Features

1.  **Articulated Kinematics Simulation**: Standard Autoware simulators typically only support a single rigid body (Ackermann kinematics). The included `nmb_odometer_node.py` extends this by calculating the off-tracking of the drawbar and trailer, allowing seamless simulation of the entire articulated system in RViz.
2.  **Modular URDF**: The vehicle is split into separate Xacro macros (`tractor`, `drawbar`, `trailer`), making it easy to configure the vehicle with or without trailers.
3.  **Autoware Compatibility**: The parameter formats strictly follow the `autoware_vehicle_info_utils` schema to ensure native compatibility with Autoware's planning and control stack.

## Kinematic Model Equations

The `kinematic_model.py` script implements an extended **Kinematic Bicycle Model** to support multi-articulated towing (Tractor + Drawbar + Trailer) using a **Recursive Matrix Formulation**. The model computes the linear velocity ($v$) and yaw rate ($\dot{\theta}$) iteratively from the front to the back of the vehicle string.

### 1. Tractor Kinematics (Bicycle Model)
The towing vehicle uses standard Ackermann kinematics referenced at the center of the rear axle:
*   $\dot{x}_0 = v_0 \cos(\theta_0)$
*   $\dot{y}_0 = v_0 \sin(\theta_0)$
*   $\dot{\theta}_0 = \frac{v_0}{L_0} \tan(\delta)$

*(Where $v_0$ is the tractor velocity, $L_0$ is the wheelbase, $\delta$ is the front wheel steering angle, and $\theta_0$ is the tractor heading).*

### 2. Drawbar Kinematics (Transform A)
The drawbar is connected to a hitch point located at a distance $d_h$ behind the previous unit's axle. Because of this offset, the hitch point's velocity is affected by the yaw rate of the towing unit (off-tracking effect).
The matrix $M_A$ computes the drawbar's velocity ($v_1$) and yaw rate ($\dot{\theta}_1$):

$$
\begin{bmatrix} v_1 \\ \dot{\theta}_1 \end{bmatrix}
=
\begin{bmatrix} 
\cos(\Delta\theta) & d_h \sin(\Delta\theta) \\
\frac{1}{L_{bar}} \sin(\Delta\theta) & -\frac{d_h}{L_{bar}} \cos(\Delta\theta)
\end{bmatrix}
\begin{bmatrix} v_0 \\ \dot{\theta}_0 \end{bmatrix}
$$

*(Where $\Delta\theta = \theta_0 - \theta_1$ is the articulation angle between the tractor and the drawbar, and $L_{bar}$ is the drawbar length).*

### 3. Trailer Body Kinematics (Transform B)
The trailer body is attached directly above the drawbar's dolly axle. Because there is no hitch offset ($d_h = 0$), the transformation matrix $M_B$ is simplified:

$$
\begin{bmatrix} v_2 \\ \dot{\theta}_2 \end{bmatrix}
=
\begin{bmatrix} 
\cos(\Delta\theta) & 0 \\
\frac{1}{L_{trl}} \sin(\Delta\theta) & 0
\end{bmatrix}
\begin{bmatrix} v_1 \\ \dot{\theta}_1 \end{bmatrix}
$$

*(Where $\Delta\theta = \theta_1 - \theta_2$ is the articulation angle between the drawbar and the trailer body, and $L_{trl}$ is the distance from the dolly pivot to the trailer's rear axle).*

By applying **Transform A** and **Transform B** in sequence for every towed unit, the script can accurately simulate the off-tracking of any number of trailers. The rates are then integrated over time (Euler integration) to update the dynamic TF tree.

## Usage

This package is automatically launched by the top-level launch files in the `tractor_launch` package. You usually do not need to run nodes from this package manually.

However, if you want to inspect the URDF or test the kinematic node in isolation, ensure you build and source your workspace first:

```bash
cd ~/AutoTract/tracter_ws
colcon build --symlink-install --packages-select tractor_description
source install/setup.bash
```
