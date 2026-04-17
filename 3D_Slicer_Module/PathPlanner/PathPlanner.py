import logging
import math
import os
from typing import Annotated, Optional

import vtk, qt, ctk
import numpy as np

import slicer
from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from slicer.parameterNodeWrapper import (
    parameterNodeWrapper,
    WithinRange,
)

from slicer import vtkMRMLScalarVolumeNode, vtkMRMLLabelMapVolumeNode, vtkMRMLMarkupsFiducialNode


# ---------------------------------------------------------------------
# PathPlanner
# ---------------------------------------------------------------------

class PathPlanner(ScriptedLoadableModule):
    """
    Surgical Path Planning module for Image-guided Navigation for Robotics.
    This module identifies collision-free trajectories through brain anatomy 
    and transmits 6-DOF poses to ROS 2 via OpenIGTLink.
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = _("PathPlanner")  
        self.parent.categories = [translate("qSlicerAbstractCoreModule", "Examples")]
        self.parent.dependencies = []  
        self.parent.contributors = ["KCL Student"]  
        self.parent.helpText = "Integrates 3D Slicer targeting with ROS 2 Kinematics."
        self.parent.acknowledgementText = "Developed for the Image Guided Navigation for Robotics module at King's College London."


# ---------------------------------------------------------------------
# PathPlannerParameterNode
# ---------------------------------------------------------------------

@parameterNodeWrapper
class PathPlannerParameterNode:
    """Internal storage for module parameters."""
    inputTargetVolume: vtkMRMLLabelMapVolumeNode
    inputCriticalVolume: vtkMRMLLabelMapVolumeNode
    inputEntryFiducials: vtkMRMLMarkupsFiducialNode
    inputTargetFiducials: vtkMRMLMarkupsFiducialNode
    lengthThreshold: Annotated[float, WithinRange(0, 500)] = 150.0


# ---------------------------------------------------------------------
# PathPlannerWidget
# ---------------------------------------------------------------------

class PathPlannerWidget(ScriptedLoadableModuleWidget):
    def __init__(self, parent=None):
        ScriptedLoadableModuleWidget.__init__(self, parent)
        self.logic = None

    def setup(self):
        """Initialises the UI elements and connects signals."""
        ScriptedLoadableModuleWidget.setup(self)
        self.logic = PathPlannerLogic()

        # UI: Input Selectors
        parametersCollapsibleButton = ctk.ctkCollapsibleButton()
        parametersCollapsibleButton.text = "Path Planning Parameters"
        self.layout.addWidget(parametersCollapsibleButton)
        parametersFormLayout = qt.QFormLayout(parametersCollapsibleButton)

        self.entrySelector = slicer.qMRMLNodeComboBox()
        self.entrySelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.entrySelector.selectNodeUponCreation = True
        self.entrySelector.addEnabled = False
        self.entrySelector.removeEnabled = False
        self.entrySelector.noneEnabled = False
        self.entrySelector.showHidden = False
        self.entrySelector.setMRMLScene(slicer.mrmlScene)
        parametersFormLayout.addRow("Entry Points: ", self.entrySelector)

        self.targetSelector = slicer.qMRMLNodeComboBox()
        self.targetSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.targetSelector.selectNodeUponCreation = True
        self.targetSelector.addEnabled = False
        self.targetSelector.removeEnabled = False
        self.targetSelector.noneEnabled = False
        self.targetSelector.showHidden = False
        self.targetSelector.setMRMLScene(slicer.mrmlScene)
        parametersFormLayout.addRow("Target Points: ", self.targetSelector)

        self.criticalMaskSelector = slicer.qMRMLNodeComboBox()
        self.criticalMaskSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.criticalMaskSelector.selectNodeUponCreation = True
        self.criticalMaskSelector.addEnabled = False
        self.criticalMaskSelector.removeEnabled = False
        self.criticalMaskSelector.noneEnabled = False
        self.criticalMaskSelector.showHidden = False
        self.criticalMaskSelector.setMRMLScene(slicer.mrmlScene)
        parametersFormLayout.addRow("Critical Mask (Vessels/Ventricles): ", self.criticalMaskSelector)

        self.targetMaskSelector = slicer.qMRMLNodeComboBox()
        self.targetMaskSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.targetMaskSelector.selectNodeUponCreation = True
        self.targetMaskSelector.addEnabled = False
        self.targetMaskSelector.removeEnabled = False
        self.targetMaskSelector.noneEnabled = False
        self.targetMaskSelector.showHidden = False
        self.targetMaskSelector.setMRMLScene(slicer.mrmlScene)
        parametersFormLayout.addRow("Target Mask (Hippocampus): ", self.targetMaskSelector)

        # UI: Trajectory Range Sliders
        trajRangeBox = qt.QWidget()
        trajRangeLayout = qt.QVBoxLayout(trajRangeBox)
        self.minLengthSlider = qt.QSlider(qt.Qt.Horizontal)
        self.minLengthSlider.setMinimum(10)
        self.minLengthSlider.setMaximum(490)
        self.minLengthSlider.setValue(40)
        self.minLengthSlider.setToolTip("Reject trajectories shorter than this (mm).")
        self.maxLengthSlider = qt.QSlider(qt.Qt.Horizontal)
        self.maxLengthSlider.setMinimum(20)
        self.maxLengthSlider.setMaximum(500)
        self.maxLengthSlider.setValue(150)
        self.maxLengthSlider.setToolTip("Reject trajectories longer than this (mm).")
        self.minLengthLabel = qt.QLabel()
        self.maxLengthLabel = qt.QLabel()
        for s in (self.minLengthSlider, self.maxLengthSlider):
            s.setTickPosition(qt.QSlider.TicksBelow)
            s.setTickInterval(50)
        self._update_length_labels()
        self.minLengthSlider.connect("valueChanged(int)", self._on_min_length_changed)
        self.maxLengthSlider.connect("valueChanged(int)", self._on_max_length_changed)
        trajRangeLayout.addWidget(qt.QLabel("Trajectory length range (mm) — min"))
        trajRangeLayout.addWidget(self.minLengthSlider)
        trajRangeLayout.addWidget(self.minLengthLabel)
        trajRangeLayout.addWidget(qt.QLabel("Trajectory length range (mm) — max"))
        trajRangeLayout.addWidget(self.maxLengthSlider)
        trajRangeLayout.addWidget(self.maxLengthLabel)
        parametersFormLayout.addRow("Trajectory length: ", trajRangeBox)

        # UI: Orientation Controls
        self.cortexSelector = slicer.qMRMLNodeComboBox()
        self.cortexSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.cortexSelector.selectNodeUponCreation = False
        self.cortexSelector.noneEnabled = True
        self.cortexSelector.addEnabled = False
        self.cortexSelector.removeEnabled = False
        self.cortexSelector.setMRMLScene(slicer.mrmlScene)
        self.cortexSelector.setToolTip(
            "Optional: first fiducial = point on cortex (or near entry) to set approach direction in the plane "
            "perpendicular to the trajectory (hippocampus entry side)."
        )
        parametersFormLayout.addRow("Cortex Reference (optional): ", self.cortexSelector)

        self.cortexRollSlider = qt.QSlider(qt.Qt.Horizontal)
        self.cortexRollSlider.setMinimum(-180)
        self.cortexRollSlider.setMaximum(180)
        self.cortexRollSlider.setValue(0)
        self.cortexRollSlider.setToolTip(
            "Extra roll (degrees) about the trajectory axis after using the cortex reference — "
            "fine-tunes tool orientation toward hippocampus."
        )
        self.cortexRollLabel = qt.QLabel("0°")
        self.cortexRollSlider.connect("valueChanged(int)", self.onRollSliderChanged)
        rollRow = qt.QHBoxLayout()
        rollRow.addWidget(self.cortexRollSlider)
        rollRow.addWidget(self.cortexRollLabel)
        rollWrap = qt.QWidget()
        rollWrap.setLayout(rollRow)
        parametersFormLayout.addRow("Cortex / approach roll: ", rollWrap)

        # Execution Button
        self.applyButton = qt.QPushButton("Compute Optimal Trajectory")
        self.applyButton.enabled = True
        parametersFormLayout.addRow(self.applyButton)
        
        self.resultLabel = qt.QLabel("Status: Waiting for input...")
        self.resultLabel.setTextInteractionFlags(qt.Qt.TextSelectableByMouse)
        parametersFormLayout.addRow("Result: ", self.resultLabel)

        self.applyButton.connect('clicked(bool)', self.onApplyButton)
        self.layout.addStretch(1)

    def cleanup(self):
        """Called when the application closes or the module is reloaded."""
        if self.logic:
            self.logic.cleanup_observer()

    def _update_length_labels(self):
        self.minLengthLabel.setText(f"Min = {self.minLengthSlider.value} mm")
        self.maxLengthLabel.setText(f"Max = {self.maxLengthSlider.value} mm")

    def _on_min_length_changed(self, _value):
        """Triggered only when the minimum slider is moved."""
        if self.minLengthSlider.value >= self.maxLengthSlider.value:
            self.maxLengthSlider.setValue(self.minLengthSlider.value + 5)
        self._update_length_labels()

    def _on_max_length_changed(self, _value):
        """Triggered only when the maximum slider is moved."""
        if self.minLengthSlider.value >= self.maxLengthSlider.value:
            self.minLengthSlider.setValue(max(10, self.maxLengthSlider.value - 5))
        self._update_length_labels()

    def onRollSliderChanged(self, value):
        """Updates the label and re-broadcasts the transform in real-time."""
        self.cortexRollLabel.setText(f"{value}°")
        

    def onApplyButton(self):
        """Main execution loop for finding the best surgical path."""
        entry_node = self.entrySelector.currentNode()
        target_node = self.targetSelector.currentNode()
        critical_node = self.criticalMaskSelector.currentNode()
        target_mask_node = self.targetMaskSelector.currentNode()
        min_length = float(self.minLengthSlider.value)
        max_length = float(self.maxLengthSlider.value)

        if not self.logic.isValidInputOutputData(target_mask_node, critical_node, entry_node, target_node):
            self.resultLabel.setText("Error: Invalid inputs. Check nodes.")
            return

        if not self.logic.hasImageData(target_mask_node) or not self.logic.hasImageData(critical_node):
            self.resultLabel.setText("Error: Volumes are missing image data.")
            return

        self.resultLabel.setText("Status: Computing...")
        slicer.app.processEvents() 
        try:
            # 1. Trajectory Calculation
            best_trajectory = self.logic.compute_optimal_trajectory(
                entry_node, target_node, critical_node, target_mask_node, max_length, min_length
            )
            

            # 2. UI Visualisation (Delete old lines, draw the new optimal path)
            existing_lines = slicer.mrmlScene.GetNodesByClass("vtkMRMLMarkupsLineNode")
            for i in range(existing_lines.GetNumberOfItems()):
                slicer.mrmlScene.RemoveNode(existing_lines.GetItemAsObject(i))

            dist = np.linalg.norm(np.array(best_trajectory[0]) - np.array(best_trajectory[1]))
            line_node_name = f"Active_Trajectory_{dist:.2f}mm"
            line_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsLineNode", line_node_name)
            
            line_node.RemoveAllControlPoints()
            line_node.AddControlPoint(vtk.vtkVector3d(best_trajectory[0][0], best_trajectory[0][1], best_trajectory[0][2]))
            line_node.AddControlPoint(vtk.vtkVector3d(best_trajectory[1][0], best_trajectory[1][1], best_trajectory[1][2]))
            
            display_node = line_node.GetDisplayNode()
            display_node.SetSelectedColor(0, 1, 0) 
            display_node.SetPropertiesLabelVisibility(True) 


            # 3. Orientation and ROS Transmission
            cortex_node = self.cortexSelector.currentNode()
            cortex_pt = None
            if cortex_node and cortex_node.GetNumberOfControlPoints() > 0:
                p = [0.0, 0.0, 0.0]
                cortex_node.GetNthControlPointPosition(0, p)
                cortex_pt = p
            else:
                cortex_pt = [0.0, 0.0, 150.0] # Default 'UP' vector relative to brain origin
                logging.info("No cortex point selected. Using safe default [0, 0, 150].")
            
            roll_deg = float(self.cortexRollSlider.value)
            self.logic.broadcast_to_ros(
                best_trajectory[0], best_trajectory[1], cortex_point_ras=cortex_pt, cortex_roll_deg=roll_deg
            )

            entry_str = f"({best_trajectory[0][0]:.2f}, {best_trajectory[0][1]:.2f}, {best_trajectory[0][2]:.2f})"
            target_str = f"({best_trajectory[1][0]:.2f}, {best_trajectory[1][1]:.2f}, {best_trajectory[1][2]:.2f})"
            self.resultLabel.setText(f"Success! Updated Trajectory_ROS_Transform.\nEntry: {entry_str}\nTarget: {target_str}")
            
        except ValueError as e:
            self.resultLabel.setText(f"Failed Constraint:\n{str(e)}")
            qt.QMessageBox.critical(slicer.util.mainWindow(), "Path Planning Failure", str(e))
        except Exception as e:
            self.resultLabel.setText(f"Unexpected Error:\n{str(e)}")
            logging.error(f"Path planning error: {e}")

# ---------------------------------------------------------------------
# PathPlannerLogic
# ---------------------------------------------------------------------

class PathPlannerLogic(ScriptedLoadableModuleLogic):
    def __init__(self):
        ScriptedLoadableModuleLogic.__init__(self)
        self.observerTag = None
        self.start_listening()

    def start_listening(self):
        """Monitors the scene for feedback strings from ROS 2."""
        if self.observerTag is None:
            self.observerTag = slicer.mrmlScene.AddObserver(slicer.vtkMRMLScene.NodeAddedEvent, self.on_node_added)
            import logging
            logging.info("Reverse-Communication Observer Active. Listening for ROS 2 feedback...")

    def on_node_added(self, caller, event):
        """Triggers pop-ups when ROS 2 sends Error/Success text nodes."""
        import qt
        import slicer
        def process_incoming_text():
            text_nodes = slicer.mrmlScene.GetNodesByClass("vtkMRMLTextNode")
            if not text_nodes:
                return   
            for i in range(text_nodes.GetNumberOfItems()):
                node = text_nodes.GetItemAsObject(i)
                if node:
                    incoming_message = node.GetText()
                    if incoming_message:
                        if incoming_message.startswith("ERROR:"):
                            clean_msg = incoming_message.replace("ERROR:", "").strip()
                            qt.QMessageBox.critical(slicer.util.mainWindow(), "Robotic Execution Failure", clean_msg)
                            slicer.mrmlScene.RemoveNode(node) 
                            
                        elif incoming_message.startswith("SUCCESS:"):
                            clean_msg = incoming_message.replace("SUCCESS:", "").strip()
                            qt.QMessageBox.information(slicer.util.mainWindow(), "Robotic Execution Complete", clean_msg)
                            slicer.mrmlScene.RemoveNode(node) 

        qt.QTimer.singleShot(200, process_incoming_text)

    def cleanup_observer(self):
        if self.observerTag is not None:
            slicer.mrmlScene.RemoveObserver(self.observerTag)
            self.observerTag = None

    def hasImageData(self, volumeNode):
        if not volumeNode:
            return False
        if volumeNode.GetImageData() is None:
            return False
        return True

    def isValidInputOutputData(self, inputTargetVolumeNode, inputCriticalVolumeNode, inputEntryFiducialsNodes, inputTargetFiducialsNode):
        if not inputTargetVolumeNode or not inputCriticalVolumeNode or not inputTargetFiducialsNode or not inputEntryFiducialsNodes:
            return False
        if inputTargetFiducialsNode.GetID() == inputEntryFiducialsNodes.GetID():
            return False
        return True

    def compute_optimal_trajectory(
        self, entry_points_node, target_points_node, critical_mask_node, target_mask_node, max_length, min_length=0.0
    ):
        """
        Iterates through candidate points to find the path that is:
        1. Within tool depth limits.
        2. Within the robot's 200-400mm reachability bubble.
        3. Free of critical structure collisions.
        4. Maximized for safety margin (Euclidean distance to closest vessel).
        """
        # Convert anatomy to meshes for collision detection
        critical_polydata = self._convert_labelmap_to_mesh(critical_mask_node)
        target_polydata = self._convert_labelmap_to_mesh(target_mask_node)

        critical_bvh = vtk.vtkOBBTree()
        critical_bvh.SetDataSet(critical_polydata)
        critical_bvh.BuildLocator()

        target_bvh = vtk.vtkOBBTree()
        target_bvh.SetDataSet(target_polydata)
        target_bvh.BuildLocator()

        valid_trajectories = []
        num_entries = entry_points_node.GetNumberOfControlPoints()
        num_targets = target_points_node.GetNumberOfControlPoints()

        for i in range(num_entries):
            entry_pos = [0.0, 0.0, 0.0]
            entry_points_node.GetNthControlPointPosition(i, entry_pos)
            
            for j in range(num_targets):
                target_pos = [0.0, 0.0, 0.0]
                target_points_node.GetNthControlPointPosition(j, target_pos)

                # Hard Constraint: Tool Depth
                distance = np.linalg.norm(np.array(entry_pos) - np.array(target_pos))
                if distance < min_length or distance > max_length:
                    continue

                # Hard Constraint: Robot Reachability (200mm-400mm from robot base at 0,0,0)
                dist_from_robot_base = np.linalg.norm(np.array(entry_pos))
                if dist_from_robot_base < 200.0 or dist_from_robot_base > 400.0:
                    # Skip points that are physically impossible for this arm model to reach
                    continue
                
                # Hard Constraint: Anatomical Collision
                critical_intersections = vtk.vtkPoints()
                critical_bvh.IntersectWithLine(entry_pos, target_pos, critical_intersections, None)
                if critical_intersections.GetNumberOfPoints() > 0:
                    continue 

                # Target Validation (Ensure the path actually touches the target structure)
                target_intersections = vtk.vtkPoints()
                target_bvh.IntersectWithLine(entry_pos, target_pos, target_intersections, None)
                if target_intersections.GetNumberOfPoints() == 0:
                    continue 

                valid_trajectories.append((entry_pos, target_pos))

        if not valid_trajectories:
            raise ValueError("No trajectories passed the hard constraints.")
        
        # Optimisation: Choose path with the widest 'Safety Corridor'
        distance_calculator = vtk.vtkImplicitPolyDataDistance()
        distance_calculator.SetInput(critical_polydata)

        best_trajectory = None
        max_safety_margin = -float('inf')

        for entry_pos, target_pos in valid_trajectories:
            midpoint = (np.array(entry_pos) + np.array(target_pos)) / 2.0
            dist_to_critical = distance_calculator.EvaluateFunction(midpoint)

            if dist_to_critical > max_safety_margin:
                max_safety_margin = dist_to_critical
                best_trajectory = (entry_pos, target_pos)

        logging.info(f"Optimal trajectory found with safety margin: {max_safety_margin}mm")
        return best_trajectory
    

    def broadcast_to_ros(self, entry_ras, target_ras, cortex_point_ras=None, cortex_roll_deg=0.0):
        """
        Calculates a 6-DOF orthonormal basis and handles the RAS->ROS coordinate swap.
        Applies a 110mm approach standoff to keep the tool away from the mesh.
        """
        import vtk
        import numpy as np
        import logging
        import math

        entry = np.array(entry_ras, dtype=float)
        target = np.array(target_ras, dtype=float)
        

        z_vec = target - entry
        
        distance = np.linalg.norm(z_vec)
        if distance == 0:
            raise ValueError("Entry and Target points are identical.")
        z_vec = z_vec / distance

        if cortex_point_ras is not None:
            cref = np.array(cortex_point_ras, dtype=float) - entry
            cref_perp = cref - float(np.dot(cref, z_vec)) * z_vec
            ncp = np.linalg.norm(cref_perp)
            if ncp > 1e-6:
                x_vec = cref_perp / ncp
                y_vec = np.cross(z_vec, x_vec)
                y_vec = y_vec / (np.linalg.norm(y_vec) + 1e-12)
                x_vec = np.cross(y_vec, z_vec)
                x_vec = x_vec / (np.linalg.norm(x_vec) + 1e-12)
            else:
                arbitrary_vec = np.array([1.0, 0.0, 0.0]) if abs(z_vec[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
                y_vec = np.cross(z_vec, arbitrary_vec)
                y_vec = y_vec / (np.linalg.norm(y_vec) + 1e-12)
                x_vec = np.cross(y_vec, z_vec)
        else:
            arbitrary_vec = np.array([1.0, 0.0, 0.0]) if abs(z_vec[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            y_vec = np.cross(z_vec, arbitrary_vec)
            y_vec = y_vec / (np.linalg.norm(y_vec) + 1e-12)
            x_vec = np.cross(y_vec, z_vec)

        # Apply User Roll
        th = math.radians(float(cortex_roll_deg))
        c, s = math.cos(th), math.sin(th)
        x_rot = c * x_vec - s * y_vec
        y_rot = s * x_vec + c * y_vec
        x_vec, y_vec = x_rot, y_rot

        # Approach Standoff (110mm to keep tool away from the mesh)
        approach_distance = 110.0 
        approach_point = entry - (z_vec * approach_distance)

        ras_matrix = vtk.vtkMatrix4x4()
        for i in range(3):
            ras_matrix.SetElement(i, 0, x_vec[i])
            ras_matrix.SetElement(i, 1, y_vec[i])
            ras_matrix.SetElement(i, 2, z_vec[i])
            # Set the translation column to the safe approach point
            ras_matrix.SetElement(i, 3, approach_point[i])
        # -----------------------------------------------

        # Construct RAS Matrix
        ras_to_ros = vtk.vtkMatrix4x4()
        ras_to_ros.Identity()
        ras_to_ros.SetElement(0, 0, 0.0)
        ras_to_ros.SetElement(0, 1, 1.0)
        ras_to_ros.SetElement(1, 0, -1.0)
        ras_to_ros.SetElement(1, 1, 0.0)

        # Coordinate Frame Swap: Slicer RAS to ROS World
        ros_matrix = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Multiply4x4(ras_to_ros, ras_matrix, ros_matrix)

        transform_node_name = "Trajectory_ROS_Transform"
        ros_transform_node = slicer.mrmlScene.GetFirstNodeByName(transform_node_name)
        if not ros_transform_node:
            ros_transform_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", transform_node_name)
        
        ros_transform_node.SetMatrixTransformToParent(ros_matrix)

        # Publish via OpenIGTLink
        cNodes = slicer.mrmlScene.GetNodesByClass("vtkMRMLIGTLConnectorNode")
        active_connector = None
        for i in range(cNodes.GetNumberOfItems()):
            node = cNodes.GetItemAsObject(i)
            if node.GetState() == 2: # Connected
                active_connector = node
                break

        if active_connector:
            active_connector.RegisterOutgoingMRMLNode(ros_transform_node)
            active_connector.PushNode(ros_transform_node)
            logging.info(f"Pushed {transform_node_name} to {active_connector.GetName()}")
        else:
            logging.error("No active OpenIGTLink connector found. Please turn one ON in Slicer.")

        return ros_transform_node

    def _convert_labelmap_to_mesh(self, labelmap_node):
        image_data = labelmap_node.GetImageData()
        marching_cubes = vtk.vtkDiscreteMarchingCubes()
        marching_cubes.SetInputData(image_data)
        marching_cubes.GenerateValues(1, 1, 1) 
        marching_cubes.Update()

        polydata = marching_cubes.GetOutput()
        ijk_to_ras_matrix = vtk.vtkMatrix4x4()
        labelmap_node.GetIJKToRASMatrix(ijk_to_ras_matrix)

        transform = vtk.vtkTransform()
        transform.SetMatrix(ijk_to_ras_matrix)

        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetInputData(polydata)
        transform_filter.SetTransform(transform)
        transform_filter.Update()

        return transform_filter.GetOutput()

# ---------------------------------------------------------------------
# PathPlannerTest
# ---------------------------------------------------------------------

class PathPlannerTest(ScriptedLoadableModuleTest):
    """
    Automated tests for validating surgical planning logic.
    These tests verify the constraints detailed in Section 3.1 of the Final Report.
    """
    def setUp(self):
        """Clears the scene and loads the brain dataset for testing."""
        slicer.mrmlScene.Clear()
        
        # Update path to your BrainPlanning Dataset
        path = '/Users/uchemudiuzoka/Desktop/TestSet'
        volumePath = os.path.join(path, 'r_hippoTest.nii.gz')
        
        if os.path.exists(volumePath):
            slicer.util.loadVolume(volumePath)
        else:
            self.delayDisplay(f"CRITICAL ERROR: Test data not found at {volumePath}")

    def runTest(self):
        """Orchestrates the execution of all validation test cases."""
        self.setUp()
        self.test_OutofBoundsConstraint()
        self.test_LengthConstraint()
        self.test_CollisionConstraint()

    def test_OutofBoundsConstraint(self):
        """Validates 'Edge Case Testing (Out-of-Bounds)' from report section 3.1."""
        self.delayDisplay("Testing: Out-of-Bounds Point Rejection")
        mask = slicer.util.getNode('r_hippoTest')
        if not mask: return

        # Point at (-1, -1, -1) violates the 200mm-400mm reachability bubble
        eNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        eNode.AddControlPoint(-1, -1, -1) 
        
        tNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        tNode.AddControlPoint(*mask.GetImageData().GetOrigin()) 
    
        logic = PathPlannerLogic()
        with self.assertRaises(ValueError):
            # This should raise ValueError because the point is too close to the robot base
            logic.compute_optimal_trajectory(eNode, tNode, mask, mask, 150.0, 10.0)
        self.delayDisplay("Test Passed: Reachability violation correctly caught.")

    def test_LengthConstraint(self):
        """Validates 'Case B (Length Filter)' from report section 3.1."""
        self.delayDisplay("Testing: Tool Length Thresholds")
        mask = slicer.util.getNode('r_hippoTest')
        if not mask: return

        # Create points that are exactly 20mm apart but within the reachability zone
        eNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        eNode.AddControlPoint(250, 0, 0) # 250mm from base (Valid Reachability)
        tNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        tNode.AddControlPoint(270, 0, 0) # 20mm total length
    
        logic = PathPlannerLogic()
        # Set min_length to 40mm. 20mm should fail the constraint.
        with self.assertRaises(ValueError):
            logic.compute_optimal_trajectory(eNode, tNode, mask, mask, 150.0, 40.0)
        self.delayDisplay("Test Passed: Short trajectory (20mm < 40mm) correctly rejected.")

    def test_CollisionConstraint(self):
        """Validates 'Case A (Collision)' from report section 3.1."""
        self.delayDisplay("Testing: Critical Structure Collision Detection")
        mask = slicer.util.getNode('r_hippoTest')
        if not mask: return

        # Place target INSIDE the hippo and entry OUTSIDE so the path crosses through it
        origin = np.array(mask.GetImageData().GetOrigin())
        eNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        eNode.AddControlPoint(*(origin + [0, 0, 100])) 
        
        tNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        tNode.AddControlPoint(*(origin - [0, 0, 100])) 
    
        logic = PathPlannerLogic()
        # Using the mask as both 'Target' and 'Critical'. Path through it should be discarded.
        with self.assertRaises(ValueError):
            logic.compute_optimal_trajectory(eNode, tNode, mask, mask, 500.0, 0.0)
        self.delayDisplay("Test Passed: Path through critical structure correctly rejected.")