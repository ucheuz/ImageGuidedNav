# Robotic Trajectory Planning and Execution Pipeline

**Project:** Image-guided Navigation for Robotics  
**Report Linkage:** This codebase corresponds to the architecture, coordinate transformations, and validation protocols detailed in the accompanying "Software and Robotic Integration Final Report."

This repository contains all Python scripts, launch files, and configurations required to run the end-to-end neurosurgical trajectory planning and kinematic execution pipeline.

---

## 1. System Requirements & Dependencies

This pipeline assumes the host environment is pre-configured with:

* **3D Slicer** (v5.0+) with the **SlicerOpenIGTLink** extension installed via the Extension Manager.
* **Ubuntu Linux** (22.04 recommended) running **ROS 2** (Humble/Iron) and **MoveIt 2**.

**Additional Required Libraries:**
* `ros-<distro>-ros2-igtl-bridge`: Required for native OpenIGTLink bridging in ROS 2. 
* Python dependencies are natively satisfied by Slicer’s internal Python environment (`numpy`, `vtk`) and standard ROS 2 packages (`tf2_ros`, `geometry_msgs`).

---

## 2. Code Components (System Architecture)

The system is divided into three functional components, matching the methodology outlined in the Final Report.

### Component A: 3D Slicer Path Planning (`PathPlanner.py`)
This script operates as a Slicer Scripted Loadable Module. 
* **Data Loading (Rubric Check):** The UI seamlessly loads the `BrainPlanning` dataset. Using `qMRMLNodeComboBox`, it accurately ingests `vtkMRMLLabelMapVolumeNode` for the critical/target structures and `vtkMRMLMarkupsFiducialNode` for entry/target coordinates.
* **Logic:** Computes the optimal trajectory by applying hard constraints (length thresholds and a 200mm–400mm physical reachability bubble) and uses `vtkOBBTree` and `vtkImplicitPolyDataDistance` to maximize the Euclidean safety margin from critical structures.
* **Report Reference:** Detailed in **Section 3.1** (Path Planning Logic).

### Component B: 3D Slicer to ROS 2 Communication (OpenIGTLink)
Handled within the `broadcast_to_ros` function of `PathPlanner.py`.
* **Transform to IGTLink (Rubric Check):** Before transmission, the trajectory vector is packed into a `vtkMRMLLinearTransformNode`. A strict spatial parity matrix is applied to rotate the medical RAS coordinate frame by 90° clockwise around the Z-axis, perfectly aligning it with the robotic World frame. 
* **Report Reference:** Detailed in **Section 3.2** (Data Transformation).

### Component C: ROS 2 Robot Listener and Movement (`igtl_listener.py` & `system.launch.py`)
The kinematic execution layer within ROS 2.
* **Data Listener (Rubric Check):** A custom ROS 2 node subscribes to the OpenIGTLink bridge. It parses the incoming transform, applies a 1:1000 scaling factor to convert Slicer's millimeter data into ROS 2's meter format, and packages it into a `geometry_msgs/PoseStamped` message.
* **Robot Movement:** The `RobotModel` utilizes MoveIt 2's Inverse Kinematics (IK) solvers to plan a collision-free path to the requested coordinates. If the point is physically unreachable, the listener safely aborts and outputs an error to the terminal.
* **Report Reference:** Detailed in **Section 3.3** (ROS 2 Robot Movement).

---

## 3. End-to-End Pipeline Execution Guide

To ensure error-free execution and avoid initialization race conditions, the ROS 2 environment has been unified under a single master launch file.

### Step 1: Launch the ROS 2 Environment (Server)
Open a terminal in your Ubuntu ROS 2 environment, source your workspace, and execute the master launch file. 

*Note: This launch file includes an 8-second `TimerAction` delay for the listener node to ensure MoveIt 2 and RViz are fully initialized before attempting to process network packets.*

```bash
# Source the ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch the unified stack (MoveIt + IGTLink Bridge on Port 18945 + Listener Node)
ros2 launch robot_control system.launch.py
```

### Step 2: Configure 3D Slicer (Client)
1. Open 3D Slicer.
2. Navigate to **Modules > OpenIGTLinkIF**.
3. Create a new `Connector`.
4. Set the Type to **Client**, Hostname to `localhost` (or the IP of your Ubuntu machine), and **Port to `18945`** (matching the launch file).
5. Click **Active** to establish the handshake.

### Step 3: Load the BrainPlanning Data
1. Navigate to the **Data** module.
2. Load your binary image volumes (e.g., `Critical_Mask.nii.gz` and `Target_Mask.nii.gz`).
3. Use the **Markups** module to place a list of Fiducials for your **Entry Points** (on the cortex) and **Target Points** (within the hippocampus).

### Step 4: Execute Path Planning
1. Navigate to the custom **PathPlanner** module (under *Examples*).
2. Assign the loaded data nodes to their respective fields in the UI.
3. *(Optional)* Adjust the Trajectory Length Range sliders based on your simulated surgical tool.
4. Click **"Compute Optimal Trajectory"**.

### Step 5: Verification (The Results)
* **In Slicer:** A green `vtkMRMLMarkupsLineNode` will render showing the optimal path. The UI will output the mathematically validated Entry and Target coordinates.
* **In ROS 2 (RViz):** The listener node will intercept the data. The robot arm will execute the trajectory, and the end-effector's Z-axis will co-locate perfectly with the target, verifying the end-to-end integration.