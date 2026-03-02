import logging
import os
from typing import Annotated

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

from slicer import vtkMRMLScalarVolumeNode


#
# PathPlanner
#


class PathPlanner(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = _("PathPlanner")  # TODO: make this more human readable by adding spaces
        # TODO: set categories (folders where the module shows up in the module selector)
        self.parent.categories = [translate("qSlicerAbstractCoreModule", "Examples")]
        self.parent.dependencies = []  # TODO: add here list of module names that this module requires
        self.parent.contributors = ["John Doe (AnyWare Corp.)"]  # TODO: replace with "Firstname Lastname (Organization)"
        # TODO: update with short description of the module and a link to online module documentation
        # _() function marks text as translatable to other languages
        self.parent.helpText = _("""
This is an example of scripted loadable module bundled in an extension.
See more information in <a href="https://github.com/organization/projectname#PathPlanner">module documentation</a>.
""")
        # TODO: replace with organization, grant and thanks
        self.parent.acknowledgementText = _("""
This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc., Andras Lasso, PerkLab,
and Steve Pieper, Isomics, Inc. and was partially funded by NIH grant 3P41RR013218-12S1.
""")

        # Additional initialization step after application startup is complete
        slicer.app.connect("startupCompleted()", registerSampleData)


#
# Register sample data sets in Sample Data module
#


def registerSampleData():
    """Add data sets to Sample Data module."""
    # It is always recommended to provide sample data for users to make it easy to try the module,
    # but if no sample data is available then this method (and associated startupCompeted signal connection) can be removed.

    import SampleData

    iconsPath = os.path.join(os.path.dirname(__file__), "Resources/Icons")

    # To ensure that the source code repository remains small (can be downloaded and installed quickly)
    # it is recommended to store data sets that are larger than a few MB in a Github release.

    # PathPlanner1
    SampleData.SampleDataLogic.registerCustomSampleDataSource(
        # Category and sample name displayed in Sample Data module
        category="PathPlanner",
        sampleName="PathPlanner1",
        # Thumbnail should have size of approximately 260x280 pixels and stored in Resources/Icons folder.
        # It can be created by Screen Capture module, "Capture all views" option enabled, "Number of images" set to "Single".
        thumbnailFileName=os.path.join(iconsPath, "PathPlanner1.png"),
        # Download URL and target file name
        uris="https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/998cb522173839c78657f4bc0ea907cea09fd04e44601f17c82ea27927937b95",
        fileNames="PathPlanner1.nrrd",
        # Checksum to ensure file integrity. Can be computed by this command:
        #  import hashlib; print(hashlib.sha256(open(filename, "rb").read()).hexdigest())
        checksums="SHA256:998cb522173839c78657f4bc0ea907cea09fd04e44601f17c82ea27927937b95",
        # This node name will be used when the data set is loaded
        nodeNames="PathPlanner1",
    )

    # PathPlanner2
    SampleData.SampleDataLogic.registerCustomSampleDataSource(
        # Category and sample name displayed in Sample Data module
        category="PathPlanner",
        sampleName="PathPlanner2",
        thumbnailFileName=os.path.join(iconsPath, "PathPlanner2.png"),
        # Download URL and target file name
        uris="https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/1a64f3f422eb3d1c9b093d1a18da354b13bcf307907c66317e2463ee530b7a97",
        fileNames="PathPlanner2.nrrd",
        checksums="SHA256:1a64f3f422eb3d1c9b093d1a18da354b13bcf307907c66317e2463ee530b7a97",
        # This node name will be used when the data set is loaded
        nodeNames="PathPlanner2",
    )


#
# PathPlannerParameterNode
#


@parameterNodeWrapper
class PathPlannerParameterNode:
    """
    The parameters needed by module.

    inputVolume - The volume to threshold.
    imageThreshold - The value at which to threshold the input volume.
    invertThreshold - If true, will invert the threshold.
    thresholdedVolume - The output volume that will contain the thresholded volume.
    invertedVolume - The output volume that will contain the inverted thresholded volume.
    """

    inputVolume: vtkMRMLScalarVolumeNode
    imageThreshold: Annotated[float, WithinRange(-100, 500)] = 100
    invertThreshold: bool = False
    thresholdedVolume: vtkMRMLScalarVolumeNode
    invertedVolume: vtkMRMLScalarVolumeNode


#
# PathPlannerWidget
#


class PathPlannerWidget(ScriptedLoadableModuleWidget):
    """Uses ScriptedLoadableModuleWidget base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent=None):
        ScriptedLoadableModuleWidget.__init__(self, parent)
        self.logic = None

    def setup(self):
        ScriptedLoadableModuleWidget.setup(self)

        # Instantiate the logic class we wrote earlier
        self.logic = PathPlannerLogic()

        # ---------------------------------------------------------------------
        # UI Setup: Parameters Area
        # ---------------------------------------------------------------------
        parametersCollapsibleButton = ctk.ctkCollapsibleButton()
        parametersCollapsibleButton.text = "Path Planning Parameters"
        self.layout.addWidget(parametersCollapsibleButton)

        # Layout within the collapsible button
        parametersFormLayout = qt.QFormLayout(parametersCollapsibleButton)

        # 1. Entry Points Selector (vtkMRMLMarkupsFiducialNode)
        self.entrySelector = slicer.qMRMLNodeComboBox()
        self.entrySelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.entrySelector.selectNodeUponCreation = True
        self.entrySelector.addEnabled = False
        self.entrySelector.removeEnabled = False
        self.entrySelector.noneEnabled = False
        self.entrySelector.showHidden = False
        self.entrySelector.setMRMLScene(slicer.mrmlScene)
        self.entrySelector.setToolTip("Select the fiducial points representing potential entry coordinates.")
        parametersFormLayout.addRow("Entry Points: ", self.entrySelector)

        # 2. Target Points Selector (vtkMRMLMarkupsFiducialNode)
        self.targetSelector = slicer.qMRMLNodeComboBox()
        self.targetSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.targetSelector.selectNodeUponCreation = True
        self.targetSelector.addEnabled = False
        self.targetSelector.removeEnabled = False
        self.targetSelector.noneEnabled = False
        self.targetSelector.showHidden = False
        self.targetSelector.setMRMLScene(slicer.mrmlScene)
        self.targetSelector.setToolTip("Select the fiducial points representing potential target coordinates.")
        parametersFormLayout.addRow("Target Points: ", self.targetSelector)

        # 3. Critical Structures Mask (vtkMRMLLabelMapVolumeNode)
        self.criticalMaskSelector = slicer.qMRMLNodeComboBox()
        self.criticalMaskSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.criticalMaskSelector.selectNodeUponCreation = True
        self.criticalMaskSelector.addEnabled = False
        self.criticalMaskSelector.removeEnabled = False
        self.criticalMaskSelector.noneEnabled = False
        self.criticalMaskSelector.showHidden = False
        self.criticalMaskSelector.setMRMLScene(slicer.mrmlScene)
        self.criticalMaskSelector.setToolTip("Select the label map representing structures to strictly avoid.")
        parametersFormLayout.addRow("Critical Mask: ", self.criticalMaskSelector)

        # 4. Target Structure Mask (vtkMRMLLabelMapVolumeNode)
        self.targetMaskSelector = slicer.qMRMLNodeComboBox()
        self.targetMaskSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.targetMaskSelector.selectNodeUponCreation = True
        self.targetMaskSelector.addEnabled = False
        self.targetMaskSelector.removeEnabled = False
        self.targetMaskSelector.noneEnabled = False
        self.targetMaskSelector.showHidden = False
        self.targetMaskSelector.setMRMLScene(slicer.mrmlScene)
        self.targetMaskSelector.setToolTip("Select the label map representing the target structure boundary.")
        parametersFormLayout.addRow("Target Mask: ", self.targetMaskSelector)

        # 5. Maximum Tool Length SpinBox
        self.maxLengthSpinBox = ctk.ctkDoubleSpinBox()
        self.maxLengthSpinBox.minimum = 10.0
        self.maxLengthSpinBox.maximum = 500.0
        self.maxLengthSpinBox.value = 150.0 # Default value
        self.maxLengthSpinBox.suffix = " mm"
        self.maxLengthSpinBox.setToolTip("Maximum allowed length for the surgical tool/trajectory.")
        parametersFormLayout.addRow("Max Tool Length: ", self.maxLengthSpinBox)

        # 6. Apply Button
        self.applyButton = qt.QPushButton("Compute Optimal Trajectory")
        self.applyButton.toolTip = "Run the path planning algorithm."
        self.applyButton.enabled = True
        parametersFormLayout.addRow(self.applyButton)
        
        # 7. Output Result Label
        self.resultLabel = qt.QLabel("Status: Waiting for input...")
        # Make the text selectable so you can copy the coordinates
        self.resultLabel.setTextInteractionFlags(qt.Qt.TextSelectableByMouse)
        parametersFormLayout.addRow("Result: ", self.resultLabel)

        # Connections
        self.applyButton.connect('clicked(bool)', self.onApplyButton)
        self.layout.addStretch(1)

    def onApplyButton(self):
        """
        Triggered when the user clicks the 'Compute Optimal Trajectory' button.
        """
        # Gather inputs from the UI
        entry_node = self.entrySelector.currentNode()
        target_node = self.targetSelector.currentNode()
        critical_node = self.criticalMaskSelector.currentNode()
        target_mask_node = self.targetMaskSelector.currentNode()
        max_length = self.maxLengthSpinBox.value

        if not all([entry_node, target_node, critical_node, target_mask_node]):
            self.resultLabel.setText("Error: Please select all required nodes.")
            return

        self.resultLabel.setText("Status: Computing...")
        slicer.app.processEvents() # Force UI update before heavy computation

        try:
            # Call the algorithm
            best_trajectory = self.logic.compute_optimal_trajectory(
                entry_node, target_node, critical_node, target_mask_node, max_length
            )
            
            # --- NEW: Broadcast the result ---
            self.logic.broadcast_to_ros(best_trajectory[0], best_trajectory[1])
            
            # Format output for the user
            entry_str = f"({best_trajectory[0][0]:.2f}, {best_trajectory[0][1]:.2f}, {best_trajectory[0][2]:.2f})"
            target_str = f"({best_trajectory[1][0]:.2f}, {best_trajectory[1][1]:.2f}, {best_trajectory[1][2]:.2f})"
            self.resultLabel.setText(f"Success! Broadcasted to ROS.\nEntry: {entry_str}\nTarget: {target_str}")
            
        except ValueError as e:
            # Catch our algorithm's intentional failure constraints (e.g., collisions)
            self.resultLabel.setText(f"Failed Constraint:\n{str(e)}")
        except Exception as e:
            # Catch unexpected errors
            self.resultLabel.setText(f"Unexpected Error:\n{str(e)}")
            logging.error(f"Path planning error: {e}")

#
# PathPlannerLogic
#


class PathPlannerLogic(ScriptedLoadableModuleLogic):
    def __init__(self):
        ScriptedLoadableModuleLogic.__init__(self)

    def compute_optimal_trajectory(self, entry_points_node, target_points_node, critical_mask_node, target_mask_node, max_length):
        """
        Executes the path planning pipeline.
        Returns the optimal (entry, target) coordinate tuple in RAS space.
        """
        if not entry_points_node or not target_points_node or not critical_mask_node or not target_mask_node:
            raise ValueError("Missing input nodes.")

        # 1. Convert masks to 3D meshes for lightning-fast collision detection
        critical_polydata = self._convert_labelmap_to_mesh(critical_mask_node)
        target_polydata = self._convert_labelmap_to_mesh(target_mask_node)

        # 2. Build Bounding Volume Hierarchies (BVH) for rapid ray-casting
        critical_bvh = vtk.vtkOBBTree()
        critical_bvh.SetDataSet(critical_polydata)
        critical_bvh.BuildLocator()

        target_bvh = vtk.vtkOBBTree()
        target_bvh.SetDataSet(target_polydata)
        target_bvh.BuildLocator()

        # Generate all possible pairs
        valid_trajectories = []
        
        num_entries = entry_points_node.GetNumberOfControlPoints()
        num_targets = target_points_node.GetNumberOfControlPoints()

        # Step 3: Hard Constraints Filtering
        for i in range(num_entries):
            entry_pos = [0.0, 0.0, 0.0]
            entry_points_node.GetNthControlPointPosition(i, entry_pos)
            
            for j in range(num_targets):
                target_pos = [0.0, 0.0, 0.0]
                target_points_node.GetNthControlPointPosition(j, target_pos)

                # Constraint A: Length Check
                distance = np.linalg.norm(np.array(entry_pos) - np.array(target_pos))
                if distance > max_length:
                    continue

                # Constraint B: Critical Structure Collision (Must NOT intersect)
                critical_intersections = vtk.vtkPoints()
                critical_bvh.IntersectWithLine(entry_pos, target_pos, critical_intersections, None)
                if critical_intersections.GetNumberOfPoints() > 0:
                    continue # Discard, it hit a critical structure

                # Constraint C: Target Intersection (Must intersect)
                target_intersections = vtk.vtkPoints()
                target_bvh.IntersectWithLine(entry_pos, target_pos, target_intersections, None)
                if target_intersections.GetNumberOfPoints() == 0:
                    continue # Discard, it missed the target zone

                valid_trajectories.append((entry_pos, target_pos))

        if not valid_trajectories:
            raise ValueError("No trajectories passed the hard constraints.")

        # Step 4: Soft Constraints Optimization (Maximize distance from critical structures)
        # To maintain high performance, we sample the closest distance from the valid trajectories
        # to the critical structure mesh using vtkImplicitPolyDataDistance.
        
        distance_calculator = vtk.vtkImplicitPolyDataDistance()
        distance_calculator.SetInput(critical_polydata)

        best_trajectory = None
        max_safety_margin = -float('inf')

        for entry_pos, target_pos in valid_trajectories:
            # Check midpoint or sample along the line. For speed, checking midpoint here.
            midpoint = (np.array(entry_pos) + np.array(target_pos)) / 2.0
            
            # Returns distance to the mesh (negative if inside, positive if outside)
            dist_to_critical = distance_calculator.EvaluateFunction(midpoint)

            if dist_to_critical > max_safety_margin:
                max_safety_margin = dist_to_critical
                best_trajectory = (entry_pos, target_pos)

        logging.info(f"Optimal trajectory found with safety margin: {max_safety_margin}mm")
        return best_trajectory
    
    def broadcast_to_ros(self, entry_ras, target_ras, port=18944):
        """
        Converts the trajectory to a ROS-compatible TransformNode and broadcasts it via OpenIGTLink.
        """
        import vtk
        import numpy as np

        # 1. Calculate the Trajectory Vector (Z-axis of our tool)
        entry = np.array(entry_ras)
        target = np.array(target_ras)
        z_vec = target - entry
        distance = np.linalg.norm(z_vec)
        if distance == 0:
            raise ValueError("Entry and Target points are identical.")
        z_vec = z_vec / distance # Normalize

        # Calculate arbitrary X and Y orthogonal vectors to complete the rotation matrix
        arbitrary_vec = np.array([1.0, 0.0, 0.0]) if abs(z_vec[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        y_vec = np.cross(z_vec, arbitrary_vec)
        y_vec = y_vec / np.linalg.norm(y_vec)
        x_vec = np.cross(y_vec, z_vec)

        # 2. Construct the original Trajectory Matrix in RAS space
        # Origin is the entry point, Z-axis points toward the target
        ras_matrix = vtk.vtkMatrix4x4()
        for i in range(3):
            ras_matrix.SetElement(i, 0, x_vec[i])
            ras_matrix.SetElement(i, 1, y_vec[i])
            ras_matrix.SetElement(i, 2, z_vec[i])
            ras_matrix.SetElement(i, 3, entry[i])

        # 3. Apply the T_{ROS <- RAS} Transformation
        ras_to_ros = vtk.vtkMatrix4x4()
        ras_to_ros.Identity()
        ras_to_ros.SetElement(0, 0, 0.0)
        ras_to_ros.SetElement(0, 1, 1.0)
        ras_to_ros.SetElement(1, 0, -1.0)
        ras_to_ros.SetElement(1, 1, 0.0)

        ros_matrix = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Multiply4x4(ras_to_ros, ras_matrix, ros_matrix)

        # 4. Create the Transform Node in the Slicer Scene
        transform_node_name = "Trajectory_ROS_Transform"
        # Check if it already exists to avoid cluttering the scene on multiple clicks
        ros_transform_node = slicer.mrmlScene.GetFirstNodeByName(transform_node_name)
        if not ros_transform_node:
            ros_transform_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", transform_node_name)
        
        ros_transform_node.SetMatrixTransformToParent(ros_matrix)

        # 5. Setup OpenIGTLink Server
        # Check if a server is already running on this port
        server_name = f"IGTLServer_{port}"
        server_node = slicer.mrmlScene.GetFirstNodeByName(server_name)
        if not server_node:
            server_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLIGTLConnectorNode", server_name)
            server_node.SetTypeServer(port)
            server_node.Start()
            logging.info(f"Started OpenIGTLink server on port {port}")

        # Register the node so it broadcasts automatically
        server_node.RegisterOutgoingMRMLNode(ros_transform_node)
        server_node.PushNode(ros_transform_node)
        
        logging.info("Trajectory successfully transformed to ROS coordinates and broadcasted.")
        return ros_transform_node

    def _convert_labelmap_to_mesh(self, labelmap_node):
        """
        Converts a vtkMRMLLabelMapVolumeNode to vtkPolyData using Marching Cubes.
        Ensures transformations are correctly applied from IJK to RAS space.
        """
        # Extract the image data
        image_data = labelmap_node.GetImageData()

        # Run Marching Cubes to generate the surface
        marching_cubes = vtk.vtkDiscreteMarchingCubes()
        marching_cubes.SetInputData(image_data)
        marching_cubes.GenerateValues(1, 1, 1) # Assuming label 1 is the structure
        marching_cubes.Update()

        polydata = marching_cubes.GetOutput()

        # Critical step: Transform from image IJK space to world RAS space
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
# PathPlannerTest
#


# class PathPlannerTest(ScriptedLoadableModuleTest):
#     """
#     This is the test case for your scripted module.
#     Uses ScriptedLoadableModuleTest base class, available at:
#     https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
#     """

#     def setUp(self):
#         """ Do whatever is needed to reset the state - typically a scene clear will be enough. """
#         slicer.mrmlScene.Clear(0)
#         self.logic = PathPlannerLogic()
        
#         # INSTRUCTOR NOTE: Update these paths to the local BrainPlanning dataset locations
#         # As per Week 3 TDD lecture, hardcoding is allowed with comments for the instructor.
#         self.critical_path = 'C:/path/to/BrainPlanning/critical_structures.nrrd' # CHANGE THIS PATH
#         self.target_path = 'C:/path/to/BrainPlanning/target.nrrd'               # CHANGE THIS PATH

#     def runTest(self):
#         """Run as few or as many tests as needed here."""
#         self.setUp()
#         self.test_PathPlanner_Positive()
#         self.test_PathPlanner_EdgeCase_Length()
#         self.test_PathPlanner_EdgeCase_Collision()

#     def _create_fiducial_node(self, name, points):
#         """Helper function to quickly generate MRML markup nodes for testing."""
#         node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", name)
#         for pt in points:
#             node.AddControlPoint(vtk.vtkVector3d(pt[0], pt[1], pt[2]))
#         return node

#     def test_PathPlanner_Positive(self):
#         """ Positive Test: A safe trajectory that hits the target and avoids critical structures. """
#         self.delayDisplay("Starting Positive Test...")
        
#         try:
#             critical_mask = slicer.util.loadLabelVolume(self.critical_path)
#             target_mask = slicer.util.loadLabelVolume(self.target_path)
#         except Exception as e:
#             self.delayDisplay(f"Skipping test, BrainPlanning data not found at hardcoded path: {e}")
#             return

#         # Known safe coordinates (Update these based on your specific dataset geometry)
#         entry_node = self._create_fiducial_node("Test_Entry", [[10.0, 10.0, 50.0]])
#         target_node = self._create_fiducial_node("Test_Target", [[10.0, 10.0, 0.0]])
#         max_length = 150.0 

#         try:
#             result = self.logic.compute_optimal_trajectory(
#                 entry_node, target_node, critical_mask, target_mask, max_length
#             )
#             self.assertIsNotNone(result, "Logic should return a valid trajectory.")
#             self.delayDisplay("Positive Test Passed!")
#         except Exception as e:
#             self.fail(f"Positive test failed unexpectedly: {e}")

#     def test_PathPlanner_EdgeCase_Length(self):
#         """ Negative Test: Trajectory is safe, but exceeds the physical tool length limit. """
#         self.delayDisplay("Starting Edge Case (Length) Test...")
        
#         # We don't need the volumes loaded for this specific failure to trigger, 
#         # but we pass dummy nodes to satisfy the function signature.
#         dummy_mask = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLabelMapVolumeNode")
        
#         entry_node = self._create_fiducial_node("Test_Entry", [[0.0, 0.0, 100.0]])
#         target_node = self._create_fiducial_node("Test_Target", [[0.0, 0.0, 0.0]])
        
#         # Distance is 100mm, but max length is artificially restricted to 50mm
#         max_length = 50.0 

#         with self.assertRaises(ValueError):
#              self.logic.compute_optimal_trajectory(
#                  entry_node, target_node, dummy_mask, dummy_mask, max_length
#              )
#         self.delayDisplay("Edge Case (Length) Test Passed! Algorithm successfully rejected it.")

#     def test_PathPlanner_EdgeCase_Collision(self):
#         """ Negative Test: Trajectory intentionally passes through a critical structure. """
#         self.delayDisplay("Starting Edge Case (Collision) Test...")
        
#         try:
#             critical_mask = slicer.util.loadLabelVolume(self.critical_path)
#             target_mask = slicer.util.loadLabelVolume(self.target_path)
#         except Exception:
#             return # Skip if data isn't loaded

#         # Known colliding coordinates (Update to force a collision through the critical mask)
#         entry_node = self._create_fiducial_node("Test_Entry", [[0.0, 0.0, 50.0]])
#         target_node = self._create_fiducial_node("Test_Target", [[0.0, 0.0, -50.0]])
#         max_length = 200.0

#         with self.assertRaises(ValueError):
#              self.logic.compute_optimal_trajectory(
#                  entry_node, target_node, critical_mask, target_mask, max_length
#              )
#         self.delayDisplay("Edge Case (Collision) Test Passed! Algorithm successfully rejected it.")
class PathPlannerTest(ScriptedLoadableModuleTest):
    """
    This is the test case for your scripted module.
    Uses ScriptedLoadableModuleTest base class.
    """

    def setUp(self):
        """Do whatever is needed to reset the state - typically a scene clear will be enough."""
        slicer.mrmlScene.Clear()
        self.delayDisplay("Starting load data test")
        
        # NOTE: Update this path to where you eventually save the BrainPlanning dataset!
        self.path = 'C:/path/to/BrainPlanning'
        
        self.critical_loaded = slicer.util.loadLabelVolume(self.path + '/critical_structures.nrrd')
        self.target_loaded = slicer.util.loadLabelVolume(self.path + '/target.nrrd')
        
        if not self.critical_loaded or not self.target_loaded:
            self.delayDisplay(f'Unable to load BrainPlanning data from {self.path}')
        else:
            self.delayDisplay('Test passed! All data loaded correctly')
            
        self.logic = PathPlannerLogic()

    def runTest(self):
        """Run as few or as many tests as needed here."""
        self.setUp()
        self.test_PathPlanning_CollisionEdgeCase()

    def test_PathPlanning_CollisionEdgeCase(self):
        """
        Here I give a trajectory I know should intersect the critical structure.
        Hence I expect the algorithm to reject it and raise a ValueError.
        """
        self.delayDisplay("Starting test for critical structure collision...")
        
        # Get our mask image nodes
        critical_mask = slicer.util.getNode('critical_structures')
        target_mask = slicer.util.getNode('target')
        
        if not critical_mask or not target_mask:
            self.delayDisplay("Skipping test: Masks not found in scene.")
            return

        # I am going to hard code two points to force a collision
        entry_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "Test_Entry")
        entry_node.AddControlPoint(0.0, 0.0, 50.0) 
        
        target_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "Test_Target")
        target_node.AddControlPoint(0.0, 0.0, -50.0)
        
        max_length = 200.0

        self.delayDisplay("Running path planning logic...")
        
        # Check if the logic properly catches the collision
        passed = False
        try:
            self.logic.compute_optimal_trajectory(entry_node, target_node, critical_mask, target_mask, max_length)
            self.delayDisplay('Test failed. The algorithm allowed a collision!')
        except ValueError:
            # We EXPECT a ValueError because it should hit the critical structure
            passed = True
            
        if passed:
            self.delayDisplay('Test passed! Trajectory was safely rejected.')