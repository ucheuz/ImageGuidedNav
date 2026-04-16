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


#
# PathPlanner
#

class PathPlanner(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class."""

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = _("PathPlanner")  
        self.parent.categories = [translate("qSlicerAbstractCoreModule", "Examples")]
        self.parent.dependencies = []  
        self.parent.contributors = ["KCL Student"]  
        self.parent.helpText = _("""
Image-guided Navigation for Robotics path planning script.
Integrates Slicer targeting with ROS 2 Inverse Kinematics via OpenIGTLink.
""")
        self.parent.acknowledgementText = _("""
Modified for Image-guided Navigation for Robotics taught through King's College London.
Original framework by Jean-Christophe Fillion-Robin, Kitware Inc., Andras Lasso, PerkLab.
""")


#
# PathPlannerParameterNode
#

@parameterNodeWrapper
class PathPlannerParameterNode:
    inputTargetVolume: vtkMRMLLabelMapVolumeNode
    inputCriticalVolume: vtkMRMLLabelMapVolumeNode
    inputEntryFiducials: vtkMRMLMarkupsFiducialNode
    inputTargetFiducials: vtkMRMLMarkupsFiducialNode
    lengthThreshold: Annotated[float, WithinRange(0, 500)] = 150.0


#
# PathPlannerWidget
#

class PathPlannerWidget(ScriptedLoadableModuleWidget):
    """Uses ScriptedLoadableModuleWidget base class."""

    def __init__(self, parent=None):
        ScriptedLoadableModuleWidget.__init__(self, parent)
        self.logic = None

    def setup(self):
        ScriptedLoadableModuleWidget.setup(self)
        self.logic = PathPlannerLogic()

        # ---------------------------------------------------------------------
        # UI Setup: Parameters Area
        # ---------------------------------------------------------------------
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
        parametersFormLayout.addRow("Critical Mask: ", self.criticalMaskSelector)

        self.targetMaskSelector = slicer.qMRMLNodeComboBox()
        self.targetMaskSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.targetMaskSelector.selectNodeUponCreation = True
        self.targetMaskSelector.addEnabled = False
        self.targetMaskSelector.removeEnabled = False
        self.targetMaskSelector.noneEnabled = False
        self.targetMaskSelector.showHidden = False
        self.targetMaskSelector.setMRMLScene(slicer.mrmlScene)
        parametersFormLayout.addRow("Target Mask: ", self.targetMaskSelector)

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
        parametersFormLayout.addRow("Cortex reference (optional): ", self.cortexSelector)

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

        self.applyButton = qt.QPushButton("Compute Optimal Trajectory")
        self.applyButton.enabled = True
        parametersFormLayout.addRow(self.applyButton)
        
        self.resultLabel = qt.QLabel("Status: Waiting for input...")
        self.resultLabel.setTextInteractionFlags(qt.Qt.TextSelectableByMouse)
        parametersFormLayout.addRow("Result: ", self.resultLabel)

        self.applyButton.connect('clicked(bool)', self.onApplyButton)
        self.layout.addStretch(1)

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
        
        # Only update if we already have a successful trajectory
        entry_node = self.entrySelector.currentNode()
        target_node = self.targetSelector.currentNode()
        if entry_node and target_node:
            # Re-running the UI logic to update the ROS transform
            self.onApplyButton()

    def onApplyButton(self):
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
            best_trajectory = self.logic.compute_optimal_trajectory(
                entry_node, target_node, critical_node, target_mask_node, max_length, min_length
            )
            
            # --- NEW: Cleanup logic to prevent "ghost" lines ---
            # This deletes any old trajectory lines before drawing the new one
            existing_lines = slicer.mrmlScene.GetNodesByClass("vtkMRMLMarkupsLineNode")
            for i in range(existing_lines.GetNumberOfItems()):
                slicer.mrmlScene.RemoveNode(existing_lines.GetItemAsObject(i))

            # --- NEW: Create a fresh line and calculate length ---
            dist = np.linalg.norm(np.array(best_trajectory[0]) - np.array(best_trajectory[1]))
            
            # Name the node with the actual distance so it's clear in the Subject Hierarchy
            line_node_name = f"Active_Trajectory_{dist:.2f}mm"
            line_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsLineNode", line_node_name)
            
            line_node.RemoveAllControlPoints()
            line_node.AddControlPoint(vtk.vtkVector3d(best_trajectory[0][0], best_trajectory[0][1], best_trajectory[0][2]))
            line_node.AddControlPoint(vtk.vtkVector3d(best_trajectory[1][0], best_trajectory[1][1], best_trajectory[1][2]))
            
            # Make it look professional
            display_node = line_node.GetDisplayNode()
            display_node.SetSelectedColor(0, 1, 0) # Clinical Green
            display_node.SetPropertiesLabelVisibility(True) # Ensure the length is visible in 3D

        # try:
        #     best_trajectory = self.logic.compute_optimal_trajectory(
        #         entry_node, target_node, critical_node, target_mask_node, max_length, min_length
        #     )
            
        #     line_node_name = "Calculated_Trajectory"
        #     line_node = slicer.mrmlScene.GetFirstNodeByName(line_node_name)
        #     if not line_node:
        #         line_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsLineNode", line_node_name)
        #     line_node.RemoveAllControlPoints()
        #     line_node.AddControlPoint(vtk.vtkVector3d(best_trajectory[0][0], best_trajectory[0][1], best_trajectory[0][2]))
        #     line_node.AddControlPoint(vtk.vtkVector3d(best_trajectory[1][0], best_trajectory[1][1], best_trajectory[1][2]))
        #     line_node.GetDisplayNode().SetSelectedColor(0, 1, 0)

            # Automated Cortex Selection 
            cortex_node = self.cortexSelector.currentNode()
            cortex_pt = None
            if cortex_node and cortex_node.GetNumberOfControlPoints() > 0:
                # Always grab the first point even if a list is used
                p = [0.0, 0.0, 0.0]
                cortex_node.GetNthControlPointPosition(0, p)
                cortex_pt = p
            else:
                # If no cortex point is provided, we assume an 'Up' orientation 
                # relative to the entry (150mm above the brain origin).
                cortex_pt = [0.0, 0.0, 150.0] 
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
        except Exception as e:
            self.resultLabel.setText(f"Unexpected Error:\n{str(e)}")
            logging.error(f"Path planning error: {e}")

#
# PathPlannerLogic
#

class PathPlannerLogic(ScriptedLoadableModuleLogic):
    def __init__(self):
        ScriptedLoadableModuleLogic.__init__(self)

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

                # 1. Coordinate Length Check (Tool Limit)
                distance = np.linalg.norm(np.array(entry_pos) - np.array(target_pos))
                if distance < min_length or distance > max_length:
                    continue

                # 2. ROBOT REACHABILITY CHECK (The 200mm-400mm Bubble)
                # Since the robot base is at (0,0,0) in RViz, we check the Entry Point distance.
                dist_from_robot_base = np.linalg.norm(np.array(entry_pos))
                if dist_from_robot_base < 200.0 or dist_from_robot_base > 400.0:
                    # Skip points that are physically impossible for this arm model to reach
                    continue
                
                # 3. Critical Structure Collision
                critical_intersections = vtk.vtkPoints()
                critical_bvh.IntersectWithLine(entry_pos, target_pos, critical_intersections, None)
                if critical_intersections.GetNumberOfPoints() > 0:
                    continue 

                target_intersections = vtk.vtkPoints()
                target_bvh.IntersectWithLine(entry_pos, target_pos, target_intersections, None)
                if target_intersections.GetNumberOfPoints() == 0:
                    continue 

                valid_trajectories.append((entry_pos, target_pos))

        if not valid_trajectories:
            raise ValueError("No trajectories passed the hard constraints.")
        
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
    
    # def broadcast_to_ros(self, entry_ras, target_ras, cortex_point_ras=None, cortex_roll_deg=0.0):
    #     import vtk
    #     import numpy as np

    #     entry = np.array(entry_ras, dtype=float)
    #     target = np.array(target_ras, dtype=float)
    #     z_vec = target - entry
    #     distance = np.linalg.norm(z_vec)
    #     if distance == 0:
    #         raise ValueError("Entry and Target points are identical.")
    #     z_vec = z_vec / distance

    #     if cortex_point_ras is not None:
    #         cref = np.array(cortex_point_ras, dtype=float) - entry
    #         cref_perp = cref - float(np.dot(cref, z_vec)) * z_vec
    #         ncp = np.linalg.norm(cref_perp)
    #         if ncp > 1e-6:
    #             x_vec = cref_perp / ncp
    #             y_vec = np.cross(z_vec, x_vec)
    #             y_vec = y_vec / (np.linalg.norm(y_vec) + 1e-12)
    #             x_vec = np.cross(y_vec, z_vec)
    #             x_vec = x_vec / (np.linalg.norm(x_vec) + 1e-12)
    #         else:
    #             arbitrary_vec = np.array([1.0, 0.0, 0.0]) if abs(z_vec[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    #             y_vec = np.cross(z_vec, arbitrary_vec)
    #             y_vec = y_vec / (np.linalg.norm(y_vec) + 1e-12)
    #             x_vec = np.cross(y_vec, z_vec)
    #     else:
    #         arbitrary_vec = np.array([1.0, 0.0, 0.0]) if abs(z_vec[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    #         y_vec = np.cross(z_vec, arbitrary_vec)
    #         y_vec = y_vec / (np.linalg.norm(y_vec) + 1e-12)
    #         x_vec = np.cross(y_vec, z_vec)

    #     th = math.radians(float(cortex_roll_deg))
    #     c, s = math.cos(th), math.sin(th)
    #     x_rot = c * x_vec - s * y_vec
    #     y_rot = s * x_vec + c * y_vec
    #     x_vec, y_vec = x_rot, y_rot

    #     # ras_matrix = vtk.vtkMatrix4x4()
    #     # for i in range(3):
    #     #     ras_matrix.SetElement(i, 0, x_vec[i])
    #     #     ras_matrix.SetElement(i, 1, y_vec[i])
    #     #     ras_matrix.SetElement(i, 2, z_vec[i])
    #     #     ras_matrix.SetElement(i, 3, entry[i])

    #     # --- NEW: 150mm Pre-Surgical Approach Offset ---
    #     # z_vec points FROM entry TO target, so subtracting it pulls the robot BACK along the trajectory
    #     approach_distance = 150.0 
    #     approach_point = entry - (z_vec * approach_distance)

    #     ras_matrix = vtk.vtkMatrix4x4()
    #     for i in range(3):
    #         ras_matrix.SetElement(i, 0, x_vec[i])
    #         ras_matrix.SetElement(i, 1, y_vec[i])
    #         ras_matrix.SetElement(i, 2, z_vec[i])
    #         # Set the translation column to the safe approach point instead of the entry point
    #         ras_matrix.SetElement(i, 3, approach_point[i])
    #     # -----------------------------------------------

    #     ras_to_ros = vtk.vtkMatrix4x4()
    #     ras_to_ros.Identity()
    #     ras_to_ros.SetElement(0, 0, 0.0)
    #     ras_to_ros.SetElement(0, 1, 1.0)
    #     ras_to_ros.SetElement(1, 0, -1.0)
    #     ras_to_ros.SetElement(1, 1, 0.0)

    #     ros_matrix = vtk.vtkMatrix4x4()
    #     vtk.vtkMatrix4x4.Multiply4x4(ras_to_ros, ras_matrix, ros_matrix)

    #     transform_node_name = "Trajectory_ROS_Transform"
    #     ros_transform_node = slicer.mrmlScene.GetFirstNodeByName(transform_node_name)
    #     if not ros_transform_node:
    #         ros_transform_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", transform_node_name)
        
    #     ros_transform_node.SetMatrixTransformToParent(ros_matrix)

    #     # Find the connector you manually turned ON
    #     cNodes = slicer.mrmlScene.GetNodesByClass("vtkMRMLIGTLConnectorNode")
    #     active_connector = None
    #     for i in range(cNodes.GetNumberOfItems()):
    #         node = cNodes.GetItemAsObject(i)
    #         if node.GetState() == 2: # State 2 = Connected/ON
    #             active_connector = node
    #             break

    #     if active_connector:
    #         # This ensures the node is registered for broadcast
    #         active_connector.RegisterOutgoingMRMLNode(ros_transform_node)
    #         # This forces the data out immediately
    #         active_connector.PushNode(ros_transform_node)
    #         logging.info(f"Pushed {transform_node_name} to {active_connector.GetName()}")
    #     else:
    #         logging.error("No active OpenIGTLink connector found. Please turn one ON in Slicer.")

    #     return ros_transform_node

    # def broadcast_to_ros(self, entry_ras, target_ras, cortex_point_ras=None, cortex_roll_deg=0.0):
    #     import vtk
    #     import numpy as np

    #     entry = np.array(entry_ras, dtype=float)
    #     target = np.array(target_ras, dtype=float)
        
    #     # --- FIX 1: FLIP THE Z-VECTOR ---
    #     # Your robot's tool Z-axis is inverted in the URDF. 
    #     # Changing this to 'entry - target' flips the orientation 180 degrees so the grippers point DOWN.
    #     z_vec = entry - target
        
    #     distance = np.linalg.norm(z_vec)
    #     if distance == 0:
    #         raise ValueError("Entry and Target points are identical.")
    #     z_vec = z_vec / distance

    #     if cortex_point_ras is not None:
    #         cref = np.array(cortex_point_ras, dtype=float) - entry
    #         cref_perp = cref - float(np.dot(cref, z_vec)) * z_vec
    #         ncp = np.linalg.norm(cref_perp)
    #         if ncp > 1e-6:
    #             x_vec = cref_perp / ncp
    #             y_vec = np.cross(z_vec, x_vec)
    #             y_vec = y_vec / (np.linalg.norm(y_vec) + 1e-12)
    #             x_vec = np.cross(y_vec, z_vec)
    #             x_vec = x_vec / (np.linalg.norm(x_vec) + 1e-12)
    #         else:
    #             arbitrary_vec = np.array([1.0, 0.0, 0.0]) if abs(z_vec[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    #             y_vec = np.cross(z_vec, arbitrary_vec)
    #             y_vec = y_vec / (np.linalg.norm(y_vec) + 1e-12)
    #             x_vec = np.cross(y_vec, z_vec)
    #     else:
    #         arbitrary_vec = np.array([1.0, 0.0, 0.0]) if abs(z_vec[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    #         y_vec = np.cross(z_vec, arbitrary_vec)
    #         y_vec = y_vec / (np.linalg.norm(y_vec) + 1e-12)
    #         x_vec = np.cross(y_vec, z_vec)

    #     th = math.radians(float(cortex_roll_deg))
    #     c, s = math.cos(th), math.sin(th)
    #     x_rot = c * x_vec - s * y_vec
    #     y_rot = s * x_vec + c * y_vec
    #     x_vec, y_vec = x_rot, y_rot

    #     # --- FIX 2: REVERSE THE OFFSET DIRECTION ---
    #     # Because z_vec now points UP, we ADD it to the entry point to move 150mm UP into the air.
    #     approach_distance = 150.0 
    #     approach_point = entry + (z_vec * approach_distance)

    #     ras_matrix = vtk.vtkMatrix4x4()
    #     for i in range(3):
    #         ras_matrix.SetElement(i, 0, x_vec[i])
    #         ras_matrix.SetElement(i, 1, y_vec[i])
    #         ras_matrix.SetElement(i, 2, z_vec[i])
    #         # Set the translation column to the safe approach point
    #         ras_matrix.SetElement(i, 3, approach_point[i])
    #     # -----------------------------------------------

    #     ras_to_ros = vtk.vtkMatrix4x4()
    #     ras_to_ros.Identity()
    #     ras_to_ros.SetElement(0, 0, 0.0)
    #     ras_to_ros.SetElement(0, 1, 1.0)
    #     ras_to_ros.SetElement(1, 0, -1.0)
    #     ras_to_ros.SetElement(1, 1, 0.0)

    #     ros_matrix = vtk.vtkMatrix4x4()
    #     vtk.vtkMatrix4x4.Multiply4x4(ras_to_ros, ras_matrix, ros_matrix)

    #     transform_node_name = "Trajectory_ROS_Transform"
    #     ros_transform_node = slicer.mrmlScene.GetFirstNodeByName(transform_node_name)
    #     if not ros_transform_node:
    #         ros_transform_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", transform_node_name)
        
    #     ros_transform_node.SetMatrixTransformToParent(ros_matrix)

    #     # Find the connector you manually turned ON
    #     cNodes = slicer.mrmlScene.GetNodesByClass("vtkMRMLIGTLConnectorNode")
    #     active_connector = None
    #     for i in range(cNodes.GetNumberOfItems()):
    #         node = cNodes.GetItemAsObject(i)
    #         if node.GetState() == 2: # State 2 = Connected/ON
    #             active_connector = node
    #             break

    #     if active_connector:
    #         active_connector.RegisterOutgoingMRMLNode(ros_transform_node)
    #         active_connector.PushNode(ros_transform_node)
    #         logging.info(f"Pushed {transform_node_name} to {active_connector.GetName()}")
    #     else:
    #         logging.error("No active OpenIGTLink connector found. Please turn one ON in Slicer.")

    #     return ros_transform_node

    def broadcast_to_ros(self, entry_ras, target_ras, cortex_point_ras=None, cortex_roll_deg=0.0):
        import vtk
        import numpy as np
        import logging
        import math

        entry = np.array(entry_ras, dtype=float)
        target = np.array(target_ras, dtype=float)
        
        # --- FIX 1: REVERT TO ORIGINAL CORRECT ORIENTATION ---
        # This points perfectly from outside the skull into the hippocampus
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

        th = math.radians(float(cortex_roll_deg))
        c, s = math.cos(th), math.sin(th)
        x_rot = c * x_vec - s * y_vec
        y_rot = s * x_vec + c * y_vec
        x_vec, y_vec = x_rot, y_rot

        # --- FIX 2: THE REACHABLE OFFSET ---
        # 80mm pulls it out of the brain, but keeps it close enough so the joints don't fail (no singularity)
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

        ras_to_ros = vtk.vtkMatrix4x4()
        ras_to_ros.Identity()
        ras_to_ros.SetElement(0, 0, 0.0)
        ras_to_ros.SetElement(0, 1, 1.0)
        ras_to_ros.SetElement(1, 0, -1.0)
        ras_to_ros.SetElement(1, 1, 0.0)

        ros_matrix = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Multiply4x4(ras_to_ros, ras_matrix, ros_matrix)

        transform_node_name = "Trajectory_ROS_Transform"
        ros_transform_node = slicer.mrmlScene.GetFirstNodeByName(transform_node_name)
        if not ros_transform_node:
            ros_transform_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", transform_node_name)
        
        ros_transform_node.SetMatrixTransformToParent(ros_matrix)

        # Find the connector you manually turned ON
        cNodes = slicer.mrmlScene.GetNodesByClass("vtkMRMLIGTLConnectorNode")
        active_connector = None
        for i in range(cNodes.GetNumberOfItems()):
            node = cNodes.GetItemAsObject(i)
            if node.GetState() == 2: # State 2 = Connected/ON
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

#
# PathPlannerTest (Teacher's Official Test Set)
#

class PathPlannerTest(ScriptedLoadableModuleTest):
    def setUp(self):
        slicer.mrmlScene.Clear()
        
        path = '/Users/uchemudiuzoka/Desktop/TestSet'
        isLoaded = slicer.util.loadVolume(path + '/r_hippoTest.nii.gz')

    def runTest(self):
        self.setUp()
        self.test_PathPlanningTestOutsidePoint()

    def test_PathPlanningTestOutsidePoint(self):
        mask = slicer.util.getNode('r_hippoTest')

        outsidePointsEntry = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        outsidePointsEntry.AddControlPoint(-1, -1, -1) 
        
        cornerPoint = mask.GetImageData().GetOrigin()
        outsidePointsTarget = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode")
        outsidePointsTarget.AddControlPoint(cornerPoint[0], cornerPoint[1], cornerPoint[2]) 
    
        logic = PathPlannerLogic()
        passed = False
        
        try:
            logic.compute_optimal_trajectory(outsidePointsEntry, outsidePointsTarget, mask, mask, 150.0, 0.0)
        except ValueError:
            passed = True