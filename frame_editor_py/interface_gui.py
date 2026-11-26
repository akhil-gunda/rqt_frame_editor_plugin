#!/usr/bin/env python3
from python_qt_binding import QtWidgets, QtCore
from python_qt_binding.QtWidgets import QWidget, QPushButton, QColorDialog
from python_qt_binding.QtCore import Slot
from python_qt_binding.QtGui import QColor

from frame_editor_py.commands import *
from frame_editor_py import urdf_model

from frame_editor_py.interface import Interface
from ament_index_python import get_package_share_directory
import math
import os



def get_package_name_from_path(file_path):
    # Walk upwards from the file path to find the package
    current_dir = os.path.abspath(file_path)
    
    while current_dir != os.path.dirname(current_dir):  # Until reaching the root
        package_xml = os.path.join(current_dir, 'package.xml')
        if os.path.exists(package_xml):
            # If package.xml exists, return the package directory
            # The parent directory of package.xml is the package directory
            return os.path.basename(current_dir)
        current_dir = os.path.dirname(current_dir)
    
    return None  # No package found

def get_rel_path_in_ros2(file_path, package_name):
    # Get the package share directory path
    rospackage_path = get_package_share_directory(package_name)
    
    # Calculate the relative path
    rel_path = os.path.relpath(file_path, rospackage_path)
    return rel_path

class _ShortDoubleSpinBox(QtWidgets.QDoubleSpinBox):
    """Shows 1.57 instead of 1.570000, so three boxes fit in one row."""

    def textFromValue(self, value):
        text = super(_ShortDoubleSpinBox, self).textFromValue(value)
        point = self.locale().decimalPoint()
        return text.rstrip("0").rstrip(point) if point in text else text


class FrameEditor_JointWidget(Interface):
    """How the selected frame is attached to its parent: fixed, or a revolute joint (hinge)."""

    # (label, [(attribute, minimum, maximum, step), ...]): one panel row each
    ROWS = [("Axis x y z:", [("axis_x", -1000.0, 1000.0, 1.0),
                             ("axis_y", -1000.0, 1000.0, 1.0),
                             ("axis_z", -1000.0, 1000.0, 1.0)]),
            ("Limits (rad):", [("lower", -1000.0, 1000.0, 0.1),
                               ("upper", -1000.0, 1000.0, 0.1)]),
            ("Effort, speed:", [("effort", 0.0, 1000000.0, 1.0),
                                ("velocity", 0.0, 1000000.0, 0.1)])]
    DECIMALS = 6
    TIPS = {"axis_x": "Axis X: the joint turns around this axis, given in this frame's coordinates",
            "axis_y": "Axis Y: the joint turns around this axis, given in this frame's coordinates",
            "axis_z": "Axis Z: the joint turns around this axis, given in this frame's coordinates",
            "lower": "Smallest angle in radians. 0 is the pose entered for this frame",
            "upper": "Largest angle in radians (1.570796 = 90 degrees)",
            "effort": "Effort: maximum torque in Nm",
            "velocity": "Velocity: maximum speed in rad/s"}

    def __init__(self, frame_editor):
        self.editor = frame_editor
        self.editor.observers.append(self)

        self.widget = QWidget()
        layout = QtWidgets.QGridLayout()
        layout.setContentsMargins(4, 4, 4, 4)
        self.widget.setLayout(layout)

        self.type_combo = QtWidgets.QComboBox()
        self.type_combo.addItems(urdf_model.JOINT_TYPES)
        self.type_combo.setToolTip("Joint between the parent frame and this frame (URDF only: "
                                   "the editor always shows the frame at angle 0)")
        self.type_combo.activated.connect(lambda _: self.type_changed())
        layout.addWidget(QtWidgets.QLabel("Joint:"), 0, 0)
        layout.addWidget(self.type_combo, 0, 1)

        self.labels, self.spinboxes = [], {}
        for row, (text, fields) in enumerate(self.ROWS, 1):
            label = QtWidgets.QLabel(text)
            boxes = QtWidgets.QHBoxLayout()
            for key, low, high, step in fields:
                spinbox = _ShortDoubleSpinBox()
                spinbox.setDecimals(self.DECIMALS)
                spinbox.setRange(low, high)
                spinbox.setSingleStep(step)
                spinbox.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
                spinbox.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
                spinbox.setMinimumWidth(48)
                spinbox.setToolTip(self.TIPS[key])
                spinbox.editingFinished.connect(lambda key=key: self.value_changed(key))
                boxes.addWidget(spinbox)
                self.spinboxes[key] = spinbox
            layout.addWidget(label, row, 0)
            layout.addLayout(boxes, row, 1)
            self.labels.append(label)

        self.info = QtWidgets.QLabel()
        self.info.setWordWrap(True)
        layout.addWidget(self.info, len(self.ROWS) + 1, 0, 1, 2)

        self.update_values(None)

    def get_widget(self):
        return self.widget

    def update(self, editor, level, elements):
        if level & (2 + 4):
            self.update_values(self.editor.active_frame)

    @staticmethod
    def _values(joint):
        return {"axis_x": joint.axis[0], "axis_y": joint.axis[1], "axis_z": joint.axis[2],
                "lower": joint.lower, "upper": joint.upper,
                "effort": joint.effort, "velocity": joint.velocity}

    def update_values(self, frame):
        self.widget.setEnabled(frame is not None)
        joint = frame.joint if frame is not None else None

        self.type_combo.setCurrentText(joint.type if joint else "fixed")
        for label in self.labels:
            label.setVisible(joint is not None)
        for key, spinbox in self.spinboxes.items():
            spinbox.setVisible(joint is not None)
            if joint is not None:
                spinbox.blockSignals(True)
                spinbox.setValue(self._values(joint)[key])
                spinbox.blockSignals(False)

        self.info.setStyleSheet("")
        self.info.setVisible(joint is not None)
        if joint is not None:
            self.info.setText("Range: {:.1f}\u00b0 to {:.1f}\u00b0".format(
                math.degrees(joint.lower), math.degrees(joint.upper)))

    def type_changed(self):
        frame = self.editor.active_frame
        if frame is None:
            return
        kind = self.type_combo.currentText()
        if kind == (frame.joint.type if frame.joint else "fixed"):
            return
        joint = urdf_model.DEFAULT_REVOLUTE if kind == "revolute" else None
        self.editor.command(Command_SetJoint(self.editor, frame, joint))

    def value_changed(self, key):
        frame = self.editor.active_frame
        if frame is None or frame.joint is None:
            return
        values = self._values(frame.joint)
        new = self.spinboxes[key].value()
        # The box shows rounded numbers: only a real edit may change the stored value
        if abs(values[key] - new) <= 0.51 * 10 ** -self.DECIMALS:
            return
        values[key] = new
        joint = frame.joint._replace(
            axis=(values["axis_x"], values["axis_y"], values["axis_z"]),
            lower=values["lower"], upper=values["upper"],
            effort=values["effort"], velocity=values["velocity"])
        error = urdf_model.joint_error(joint)
        if error:
            self.update_values(frame)  # back to the stored values
            self.info.setStyleSheet("QLabel { color: red; }")
            self.info.setText("Not changed: {}.".format(error))
            return
        self.editor.command(Command_SetJoint(self.editor, frame, joint))


class FrameEditor_StyleWidget(Interface):

    def __init__(self, frame_editor):
        self.editor = frame_editor
        self.editor.observers.append(self)

        self.old_frame = None

        self.layout = QtWidgets.QGridLayout()
        self.widget = QWidget()
        self.widget.setLayout(self.layout)

        self.mesh_label = QtWidgets.QLineEdit("File:")
        self.mesh_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        self.mesh_button = QtWidgets.QPushButton("Open")
        self.mesh_button.clicked.connect(lambda: self.btn_open_mesh_clicked())

        self.size_label = QtWidgets.QLabel("Size:")
        self.size_spinbox = QtWidgets.QDoubleSpinBox()
        self.size_spinbox.editingFinished.connect(lambda: self.size_changed())
        self.size_spinbox.setDecimals(4) 
        self.size_spinbox.setMinimum(0.0)

        self.diameter_label = QtWidgets.QLabel("Diameter:")
        self.diameter_spinbox = QtWidgets.QDoubleSpinBox()
        self.diameter_spinbox.editingFinished.connect(lambda: self.diameter_changed())

        self.length_label = QtWidgets.QLabel("Length:")
        self.length_spinbox = QtWidgets.QDoubleSpinBox()
        self.length_spinbox.editingFinished.connect(lambda: self.length_changed())

        self.width_label = QtWidgets.QLabel("Width:")
        self.width_spinbox = QtWidgets.QDoubleSpinBox()
        self.width_spinbox.editingFinished.connect(lambda: self.width_changed())

        self.height_label = QtWidgets.QLabel("Height:")
        self.height_spinbox = QtWidgets.QDoubleSpinBox()
        self.height_spinbox.editingFinished.connect(lambda: self.height_changed())

        self.color_label = QtWidgets.QLabel()
        self.color_label.setAutoFillBackground(True)
        self.update_color_label(None)
        self.color_button = QtWidgets.QPushButton("Set Color")
        self.color_button.clicked.connect(lambda: self.btn_color_clicked())

        self.alpha_label = QtWidgets.QLabel("Alpha:")
        self.alpha_label.setToolTip("Transparency: 0 = invisible, 1 = solid (URDF rgba, 4th value)")
        self.alpha_spinbox = QtWidgets.QDoubleSpinBox()
        self.alpha_spinbox.setRange(0.0, 1.0)
        self.alpha_spinbox.setSingleStep(0.05)
        self.alpha_spinbox.setDecimals(2)
        self.alpha_spinbox.setToolTip(self.alpha_label.toolTip())
        self.alpha_spinbox.editingFinished.connect(lambda: self.alpha_changed())

        self.layout.addWidget(self.mesh_label, 0, 0)
        self.layout.addWidget(self.mesh_button, 0, 1)
        self.layout.addWidget(self.diameter_label, 1, 0)
        self.layout.addWidget(self.diameter_spinbox, 1, 1)
        self.layout.addWidget(self.length_label, 2, 0)
        self.layout.addWidget(self.length_spinbox, 2, 1)
        self.layout.addWidget(self.width_label, 3, 0)
        self.layout.addWidget(self.width_spinbox, 3, 1)
        self.layout.addWidget(self.height_label, 4, 0)
        self.layout.addWidget(self.height_spinbox, 4, 1)
        self.layout.addWidget(self.color_label, 5, 0)
        self.layout.addWidget(self.color_button, 5, 1)
        self.layout.addWidget(self.size_label, 6, 0)
        self.layout.addWidget(self.size_spinbox, 6, 1)
        self.layout.addWidget(self.alpha_label, 7, 0)
        self.layout.addWidget(self.alpha_spinbox, 7, 1)


        self.update_widget(None)

    def get_widget(self):
        return self.widget

    def update(self, editor, level, elements):
        
        if level & 2:
            ## Check for change
            if self.editor.active_frame is not self.old_frame:
                self.update_widget(self.editor.active_frame)
                self.update_values(self.editor.active_frame)
            self.update_color_label(self.editor.active_frame)

        elif level & 4:
            if self.editor.active_frame is not None:
                self.update_values(self.editor.active_frame)
            self.update_color_label(self.editor.active_frame)
    
    def update_widget(self, frame):
        ## Clear layout
        #while self.layout.count():
        #    child = self.layout.takeAt(0)
        #    child.widget().deleteLater()

        self.mesh_label.hide()
        self.mesh_button.hide()
        self.diameter_label.hide()
        self.diameter_spinbox.hide()
        self.length_label.hide()
        self.length_spinbox.hide()
        self.width_label.hide()
        self.width_spinbox.hide()
        self.height_label.hide()
        self.height_spinbox.hide()
        self.size_label.hide()
        self.size_spinbox.hide()

        if frame is None or frame.style == "none":
            self.widget.setEnabled(False)
            return

        if frame.style == "mesh":
            self.mesh_label.show()
            self.mesh_button.show()
            self.size_label.show()
            self.size_spinbox.show()

        elif frame.style == "sphere":
            self.diameter_label.show()
            self.diameter_spinbox.show()
        elif frame.style == "cylinder":
            self.diameter_label.show()
            self.diameter_spinbox.show()
            self.height_label.show()
            self.height_spinbox.show()
        else:
            self.length_label.show()
            self.length_spinbox.show()
            self.width_label.show()
            self.width_spinbox.show()
            if frame.style == "cube":
                self.height_label.show()
                self.height_spinbox.show()

        self.widget.setEnabled(True)

    def update_values(self, frame):
        if frame is None or frame.style == "none":
            return

        if frame.style == "mesh":
            self.mesh_label.setText(frame.path)
            self.size_spinbox.setValue(frame.scale)
        elif frame.style == "sphere":
            self.diameter_spinbox.setValue(frame.diameter)
        elif frame.style == "cylinder":
            self.diameter_spinbox.setValue(frame.diameter)
            self.height_spinbox.setValue(frame.height)
        else:
            self.length_spinbox.setValue(frame.length)
            self.width_spinbox.setValue(frame.width)
            if frame.style == "cube":
                self.height_spinbox.setValue(frame.height)

    def update_color_label(self, frame):
        if frame is None:
            values = "{}, {}, {}, {}".format(200, 200, 200, 255)
        else:
            values = "{}, {}, {}, {}".format(frame.color[0]*255, frame.color[1]*255, frame.color[2]*255, frame.color[3]*255)
        self.color_label.setStyleSheet("QLabel { background-color: rgba("+values+"); }")
        if frame is not None:
            self.alpha_spinbox.blockSignals(True)
            self.alpha_spinbox.setValue(frame.color[3])
            self.alpha_spinbox.blockSignals(False)

    @Slot()
    def alpha_changed(self):
        frame = self.editor.active_frame
        if frame is None or frame.marker is None:
            return
        alpha = self.alpha_spinbox.value()
        if abs(frame.color[3] - alpha) < 1e-6:
            return
        r, g, b = frame.color[0], frame.color[1], frame.color[2]
        self.editor.command(Command_SetStyleColor(self.editor, frame, (r, g, b, alpha)))

    @Slot(float)
    def diameter_changed(self):
        if self.editor.active_frame.diameter != self.diameter_spinbox.value():
            self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "diameter", self.diameter_spinbox.value()))

    @Slot(float)
    def size_changed(self):
        if self.editor.active_frame.scale != self.diameter_spinbox.value():
            self.editor.command(Command_SetSize(self.editor, self.editor.active_frame, self.size_spinbox.value()))


    @Slot(float)
    def length_changed(self):
        if self.editor.active_frame.length != self.length_spinbox.value():
            self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "length", self.length_spinbox.value()))

    @Slot(float)
    def width_changed(self):
        if self.editor.active_frame.width != self.width_spinbox.value():
            self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "width", self.width_spinbox.value()))

    @Slot(float)
    def height_changed(self):
        if self.editor.active_frame.height != self.height_spinbox.value():
            self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "height", self.height_spinbox.value()))

    @Slot(bool)
    def btn_open_mesh_clicked(self):
        path = QtWidgets.QFileDialog.getOpenFileName(None, 'Open Mesh', '/home', 'Mesh Files (*.stl)')[0]
        try:
            rospackage = get_package_name_from_path(path)
            if rospackage is None:
                QtWidgets.QMessageBox.warning(self.widget, "Saving absolute path to mesh",
                "Cannot find rospackage with selected mesh in it!\nSaving absolute path to mesh instead!")
                self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "package", ""))
                self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "path", path))
            else:
                rel_path = get_rel_path_in_ros2(path, rospackage)
                print("Saving: package: {} + relative path: {}".format(rospackage, rel_path))
                self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "package", rospackage))
                self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "path", rel_path))
        except:
            QtWidgets.QMessageBox.warning(self.widget, "Saving absolute path to mesh",
            "The found rospackage with selected mesh in it is not sourced in your ROS workspace!\n"+
            "Cannot resolve the packagepath\nSaving absolute path to mesh instead!")
            self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "package", ""))
            self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "path", path))

    @Slot(bool)
    def btn_color_clicked(self):
        frame = self.editor.active_frame
        color = QtWidgets.QColorDialog.getColor(
            QColor(*[int(round(c*255)) for c in frame.color]),  # Qt needs ints
            None,
            "Select Color",
            options=QtWidgets.QColorDialog.ShowAlphaChannel)
        if not color.isValid():
            return  # dialog cancelled
        self.editor.command(Command_SetStyleColor(self.editor, frame, color.getRgbF()))

# eof
