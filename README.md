# Robotic Trajectory Planning and Execution Pipeline

**Project:** Image-guided Navigation for Robotics  
**Report Linkage:** This codebase corresponds to the architecture, coordinate transformations, and validation protocols detailed in the accompanying "Software and Robotic Integration Final Report."

This repository contains all Python scripts, launch files, and configurations required to run the end-to-end neurosurgical trajectory planning and kinematic execution pipeline.

---

## 1. System Requirements & Dependencies

This pipeline assumes the host environment is pre-configured with:

* **3D Slicer** (v5.0+) with the **SlicerOpenIGTLink** extension installed via the Extension Manager.
* **Ubuntu Linux** (24.04 recommended) running **ROS 2 (Jazzy Jalisco)** and **MoveIt 2**.

**Additional Required Libraries:**
* `pyigtl`: Required for Python-based OpenIGTLink communication in the ROS 2 listener node.
* Python dependencies are natively satisfied by Slicer’s internal Python environment (`numpy`, `vtk`) and standard ROS 2 packages (`tf2_ros`, `geometry_msgs`).

---

## 2. Code Components (System Architecture)

The system is divided into three functional components, matching the methodology outlined in the Final Report.

### Component A: 3D Slicer Path Planning (`PathPlanner.py`)
This script operates as a Slicer Scripted Loadable Module. 
* **Data Loading (Rubric Check):** The UI seamlessly loads the `BrainPlanning` dataset. Using `qMRMLNodeComboBox`, it accurately ingests `vtkMRMLLabelMapVolumeNode` for the critical/target structures and `vtkMRMLMarkupsFiducialNode` for entry/target coordinates.
* **Logic:** Computes the optimal trajectory by applying hard constraints (length thresholds and a 200mm–400mm physical reachability bubble) and uses `vtkOBBTree` and `vtkImplicitPolyDataDistance` to maximize the Euclidean safety margin from critical structures.
* **Report Reference:** Detailed in **Section 2.1** (3D Slicer Path Planning).

### Component B: 3D Slicer to ROS 2 Communication (OpenIGTLink)
Handled within the `broadcast_to_ros` function of `PathPlanner.py`.
* **Transform to IGTLink (Rubric Check):** Before transmission, the trajectory vector is packed into a `vtkMRMLLinearTransformNode`. A strict spatial parity matrix is applied to rotate the medical RAS coordinate frame by 90° clockwise around the Z-axis, perfectly aligning it with the robotic World frame. 
* **Report Reference:** Detailed in **Section 2.2** (Trajectory Communication and Transformation).

### Component C: ROS 2 Robot Listener and Movement (`igtl_listener.py` & `system.launch.py`)
The kinematic execution layer within ROS 2.
* **Data Listener (Rubric Check):** A custom ROS 2 node utilizes `pyigtl` to subscribe to the Slicer server. It parses the incoming transform, applies a 10^-3 scaling factor to convert Slicer's millimeter data into ROS 2's SI meter format, and packages it into a `geometry_msgs/TransformStamped` message.
* **Robot Movement & Bidirectional Feedback:** The script utilizes MoveIt 2's IK solvers to plan a collision-free path. Crucially, it acts as a closed-loop system; if the point is unreachable or violates state bounds, the listener serializes an error string and transmits it *back* to Slicer to trigger a clinical GUI alert.
* **Report Reference:** Detailed in **Section 2.3** (ROS 2 Robot Movement).

---

## 3. End-to-End Pipeline Execution Guide

To ensure error-free execution and avoid initialization race conditions, follow this specific startup sequence.

### Step 1: Configure 3D Slicer (Server)
1. Open 3D Slicer.
2. Navigate to **Modules > OpenIGTLinkIF**.
3. Create a new `Connector`.
4. Set the Type to **Server**, and the Port to `18945` (or your configured port).
5. Click **Active (ON)** to open the socket so the ROS 2 listener can connect.

### Step 2: Launch the ROS 2 Environment (Client)
Open a terminal in your Ubuntu ROS 2 environment, source your workspace, and execute the master launch file. 

*Note: This launch file includes an 8-second `TimerAction` delay for the listener node to ensure MoveIt 2 and RViz are fully initialized before attempting the OpenIGTLink handshake.*

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_control system.launch.py
```

### Step 3: Load Anatomical Validation Meshes
Because RViz takes time to boot its visualization server, the anatomical meshes are published manually to bypass CPU throttling in virtual machines.

1. Open a **second** Ubuntu terminal and source your workspace.
2. Run the visualization script:
   `python3 /home/ros2box/ros2_ws/show_brain.py`
3. In RViz, click **Add** (bottom left) -> **MarkerArray** -> set the topic to `/visualization_marker_array`. The green anatomical meshes will appear.

### Step 4: Execute Path Planning
1. Return to 3D Slicer and navigate to the custom **PathPlanner** module.
2. Assign the loaded data nodes (e.g., `Ventricles_and_Vessels` and `r_hippoTest`) to their respective fields in the UI.
3. Adjust the Trajectory Length Range and optional Cortex Roll slider.
4. Click **"Compute Optimal Trajectory"**.

### Step 5: Verification (The Results)
* **In Slicer:** A green `vtkMRMLMarkupsLineNode` will render showing the optimal path.
* **In ROS 2 (RViz):** The listener node will intercept the data. The robot arm will execute the trajectory, aligning perfectly with the target. Slicer will dynamically pop up a "Robot has achieved the approach pose" success box.

---

## 4. Running the Automated Tests (Validation)

To verify the mathematical constraints and collision detection logic described in Section 3.1 of the report, the `PathPlanner` module includes a suite of automated unit tests.

1. Open the **PathPlanner** module in 3D Slicer.
2. Expand the **Reload & Test** section at the top of the module panel.
3. Click **Reload and Test**. The test results will output to the Slicer Python console and display via pop-up messages.

**Note regarding Test Data:** The automated test script attempts to load the `r_hippoTest.nii.gz` volume. The file path is currently configured for the default development environment (`/Users/uchemudiuzoka/Desktop/TestSet`). Please update the `path` variable in the `setUp` method of `PathPlanner.py` to match your local directory before running the test to avoid a `FileNotFound` error.

---

## 5. Troubleshooting Virtual Machine Lag

**Simulation Drift (`START_STATE_INVALID`)** If running inside a Virtual Machine, CPU throttling may cause the `ros2_control` loop to overrun, resulting in the robot's simulated momentum drifting slightly past its mathematical joint limits after a fast movement. If Slicer throws a `START_STATE_INVALID` GUI error:

1. In RViz, navigate to the **MotionPlanning** panel on the left.
2. Select the **Planning** tab.
3. Use the **Goal State** dropdown to select `Home` (or a known safe state) and click **Plan and Execute**.
4. This resets the joints into a clean state, allowing you to send the next trajectory from Slicer without restarting the system.