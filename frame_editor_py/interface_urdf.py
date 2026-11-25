#!/usr/bin/env python3
"""URDF view of the frame editor: a live preview panel and a save_urdf service.

Both are additional observers / services. They only read the editor model,
except for "Apply code to frames", which goes through the normal undoable commands.
"""
import difflib
import os
import re

from python_qt_binding import QtCore, QtGui, QtWidgets
from python_qt_binding.QtCore import QObject, Slot

from frame_editor_py.interface import Interface
from frame_editor_py import urdf_model as um
from frame_editor_py import urdf_apply
from frame_editor_py.urdf_model import URDFError

from frame_editor.srv import SaveYaml  # same request/response shape: filename -> success, message


def robot_name_for(editor):
    base = os.path.splitext(editor.get_file_name())[0]
    base = re.sub(r"[^A-Za-z0-9_]", "_", base)
    return base or um.DEFAULT_ROBOT_NAME


def generate_urdf(editor):
    # urdf_collision is set by the panel's checkbox; the service uses the same setting.
    return um.frames_to_urdf(list(editor.frames.values()), robot_name_for(editor),
                             collision=getattr(editor, "urdf_collision", False))


def changed_lines(old_text, new_text):
    """Line numbers (0-based) in new_text that are new or changed, and how many old lines were removed."""
    old_lines, new_lines = old_text.splitlines(), new_text.splitlines()
    changed, removed = [], 0
    for tag, i1, i2, j1, j2 in difflib.SequenceMatcher(None, old_lines, new_lines, autojunk=False).get_opcodes():
        if tag in ("replace", "insert"):
            changed.extend(range(j1, j2))
        if tag in ("replace", "delete"):
            removed += max(0, (i2 - i1) - (j2 - j1))
    return changed, removed


def default_urdf_path(editor):
    yaml_path = editor.get_full_file_path()
    if not yaml_path:
        return ""
    return os.path.splitext(yaml_path)[0] + ".urdf"


def write_urdf(editor, filename):
    text, warnings = generate_urdf(editor)
    with open(filename, "w") as f:
        f.write(text)
    return warnings


def xacro_paths(filename):
    """'cell.urdf.xacro' (or 'cell.xacro', 'cell') -> ('cell.urdf.xacro', 'cell_macro.xacro')."""
    for ending in (".urdf.xacro", ".xacro"):
        if filename.endswith(ending):
            filename = filename[:-len(ending)]
            break
    return filename + ".urdf.xacro", filename + "_macro.xacro"


def default_xacro_path(editor):
    yaml_path = editor.get_full_file_path()
    return xacro_paths(os.path.splitext(yaml_path)[0])[0] if yaml_path else ""


def write_xacro(editor, filename):
    """Writes the top file and the macro file next to each other. Returns (top, macro, warnings)."""
    top_path, macro_path = xacro_paths(filename)
    macro, top, warnings = um.frames_to_xacro(
        list(editor.frames.values()), robot_name_for(editor), os.path.basename(macro_path),
        collision=getattr(editor, "urdf_collision", False))
    with open(macro_path, "w") as f:
        f.write(macro)
    with open(top_path, "w") as f:
        f.write(top)
    return top_path, macro_path, warnings


class FrameEditor_URDFService(Interface):
    """Service <node>/save_urdf (frame_editor/srv/SaveYaml).

    Empty filename: write next to the loaded YAML, with the .urdf extension.
    """

    def __init__(self, frame_editor):
        self.editor = frame_editor
        self.editor.node.create_service(
            SaveYaml, "{}/save_urdf".format(self.editor.node.get_name()), self.callback_save_urdf)
        self.editor.node.create_service(
            SaveYaml, "{}/save_xacro".format(self.editor.node.get_name()), self.callback_save_xacro)

    def callback_save_xacro(self, request, response):
        filename = os.path.expanduser(request.filename) or default_xacro_path(self.editor)
        self.editor.node.get_logger().info("> Request to save xacro file to:'{}'".format(filename))
        if not filename:
            response.success = False
            response.message = "No filename given and no YAML file loaded"
            return response
        try:
            top, macro, warnings = write_xacro(self.editor, filename)
            response.success = True
            response.message = "saved {} and {}".format(top, macro) + "".join("\nwarning: " + w for w in warnings)
        except Exception as e:
            response.success = False
            response.message = "Exception: {}".format(e)
        return response

    def callback_save_urdf(self, request, response):
        filename = os.path.expanduser(request.filename) or default_urdf_path(self.editor)
        self.editor.node.get_logger().info("> Request to save urdf file to:'{}'".format(filename))
        if not filename:
            response.success = False
            response.message = "No filename given and no YAML file loaded"
            return response
        try:
            warnings = write_urdf(self.editor, filename)
            response.success = True
            response.message = "file saved" + "".join("\nwarning: " + w for w in warnings)
        except Exception as e:
            response.success = False
            response.message = "Exception: {}".format(e)
        return response


class FrameEditor_URDFPanel(Interface):
    """Live URDF text of all editor frames. After editing the code: "Apply code to frames" or
    "Rewrite code from frames"."""

    signal_changed = QtCore.Signal()

    REFRESH_DELAY_MS = 200  # debounce: dragging in RViz sends many updates

    def __init__(self, frame_editor):
        QObject.__init__(self)  # Interface.__init__ skips QObject; signals need it
        self.editor = frame_editor
        self.editor.observers.append(self)

        self.dirty = False          # text edited by the user, not applied yet
        self.stale = True           # editor changed while the panel was hidden or dirty
        self._setting_text = False

        ## Widgets
        self.widget = QtWidgets.QGroupBox("URDF")
        layout = QtWidgets.QVBoxLayout(self.widget)

        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        self.status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.status)

        self.text = QtWidgets.QPlainTextEdit()
        self.text.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.text.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        self.text.textChanged.connect(self.text_changed)
        layout.addWidget(self.text)

        code_buttons = QtWidgets.QHBoxLayout()
        self.btn_apply = QtWidgets.QPushButton("Apply code to frames")
        self.btn_apply.setToolTip("Reads the URDF code above and changes the frames in the editor to match it "
                                  "(one undo step). Code the editor cannot represent is refused.")
        self.btn_apply.clicked.connect(self.apply_clicked)
        self.btn_discard = QtWidgets.QPushButton("Rewrite code from frames")
        self.btn_discard.setToolTip("Throws away your edits to the code and writes it again from the "
                                    "frames currently in the editor")
        self.btn_discard.clicked.connect(self.discard_clicked)
        code_buttons.addWidget(self.btn_apply)
        code_buttons.addWidget(self.btn_discard)
        layout.addLayout(code_buttons)

        export_buttons = QtWidgets.QHBoxLayout()
        self.btn_export = QtWidgets.QPushButton("Export URDF...")
        self.btn_export.clicked.connect(self.export_clicked)
        self.btn_export_xacro = QtWidgets.QPushButton("Export xacro...")
        self.btn_export_xacro.setToolTip("Writes <name>.urdf.xacro and a reusable macro <name>_macro.xacro "
                                         "(with prefix, parent and origin parameters)")
        self.btn_export_xacro.clicked.connect(self.export_xacro_clicked)
        export_buttons.addWidget(self.btn_export)
        export_buttons.addWidget(self.btn_export_xacro)
        layout.addLayout(export_buttons)

        self.chk_save_with_yaml = QtWidgets.QCheckBox("Also write .urdf next to the YAML on Save")
        self.chk_save_with_yaml.setChecked(True)
        layout.addWidget(self.chk_save_with_yaml)

        self.chk_collision = QtWidgets.QCheckBox("Add collision shapes (same as the visual shape)")
        self.chk_collision.setToolTip("Writes each shape a second time as <collision>, which planners "
                                      "such as MoveIt use for collision checking")
        self.chk_collision.toggled.connect(self.collision_toggled)
        layout.addWidget(self.chk_collision)

        self.chk_highlight = QtWidgets.QCheckBox("Highlight lines changed by the last edit")
        self.chk_highlight.setChecked(True)
        self.chk_highlight.toggled.connect(lambda on: on or self.text.setExtraSelections([]))
        layout.addWidget(self.chk_highlight)

        self.timer = QtCore.QTimer()
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.refresh)

        self.signal_changed.connect(self.editor_changed)
        self.set_dirty(False)
        self.widget.hide()

    def get_widget(self):
        return self.widget

    ## Observer ##
    ##
    def update(self, editor, level, elements):
        # May be called from a ROS thread (services, interactive marker): hand over to Qt.
        self.signal_changed.emit()

    @Slot()
    def editor_changed(self):
        self.stale = True
        if not self.widget.isVisible():
            return
        if self.dirty:
            self.show_status("The frames changed while you were editing the code. 'Apply code to frames' "
                             "overwrites them with your code; 'Rewrite code from frames' replaces your "
                             "code with the current frames.", warn=True)
            return
        self.timer.start(self.REFRESH_DELAY_MS)

    ## Show / hide ##
    ##
    def set_visible(self, visible):
        self.widget.setVisible(visible)
        if visible and self.stale and not self.dirty:
            self.refresh(highlight=False)

    @Slot(bool)
    def collision_toggled(self, on):
        self.editor.urdf_collision = on
        if self.dirty:
            self.show_status("The collision setting takes effect after 'Apply code to frames' or 'Rewrite code from frames'.", warn=True)
        else:
            self.refresh()

    ## Text ##
    ##
    def refresh(self, highlight=True):
        if self.dirty:
            return
        old_text = self.text.toPlainText()
        text, warnings = generate_urdf(self.editor)
        scroll = self.text.verticalScrollBar().value()
        self._setting_text = True
        self.text.setPlainText(text)
        self._setting_text = False
        self.text.verticalScrollBar().setValue(scroll)
        self.stale = False

        changed, removed = [], 0
        if highlight and old_text and self.chk_highlight.isChecked():
            changed, removed = changed_lines(old_text, text)
        self.highlight_lines(changed)

        status = "Live: {} frames".format(len(self.editor.frames))
        if changed or removed:
            status += " | last edit: {} line(s) changed or added, {} removed".format(len(changed), removed)
        if warnings:
            self.show_status(status + "\n" + "\n".join(warnings), warn=True)
        else:
            self.show_status(status)

    def highlight_lines(self, line_numbers):
        """Yellow background on the given lines; scrolls the first one into view."""
        selections = []
        fmt = QtGui.QTextCharFormat()
        fmt.setBackground(QtGui.QColor(255, 236, 150))
        fmt.setForeground(QtGui.QColor(0, 0, 0))
        fmt.setProperty(QtGui.QTextFormat.FullWidthSelection, True)
        document = self.text.document()
        for line in line_numbers:
            selection = QtWidgets.QTextEdit.ExtraSelection()
            selection.format = fmt
            selection.cursor = QtGui.QTextCursor(document.findBlockByLineNumber(line))
            selections.append(selection)
        self.text.setExtraSelections(selections)
        if line_numbers:
            self.text.setTextCursor(QtGui.QTextCursor(document.findBlockByLineNumber(line_numbers[0])))
            self.text.ensureCursorVisible()

    @Slot()
    def text_changed(self):
        if not self._setting_text and not self.dirty:
            self.set_dirty(True)
            self.show_status("Code edited: press 'Apply code to frames' to update the frames, or 'Rewrite code from frames' to drop your edits.")

    def set_dirty(self, dirty):
        self.dirty = dirty
        self.btn_apply.setEnabled(dirty)
        self.btn_discard.setEnabled(dirty)

    def show_status(self, message, warn=False, error=False):
        color = "#b00020" if error else ("#9a6700" if warn else "")
        self.status.setStyleSheet("QLabel {{ color: {}; }}".format(color) if color else "")
        self.status.setText(message)

    ## Buttons ##
    ##
    @Slot()
    def apply_clicked(self):
        try:
            changes = urdf_apply.apply(self.editor, self.text.toPlainText())
        except URDFError as e:
            self.show_status("Not applied: {}".format(e), error=True)
            return
        self.set_dirty(False)
        self.refresh(highlight=False)
        if changes:
            self.show_status("Applied {} change(s) to the frames. Undo reverts them in one step.".format(len(changes)))
        else:
            self.show_status("Nothing to apply: the code already matches the frames.")

    @Slot()
    def discard_clicked(self):
        self.set_dirty(False)
        self.refresh(highlight=False)

    @Slot()
    def export_clicked(self):
        if self.dirty:
            QtWidgets.QMessageBox.information(
                self.widget, "Export URDF",
                "Your text edits are not applied yet. The export uses the current frames.")
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self.widget, "Export URDF", default_urdf_path(self.editor), "URDF files (*.urdf)")
        if not filename:
            return
        if not filename.endswith(".urdf"):
            filename += ".urdf"
        self.export_to(filename)

    def export_to(self, filename):
        try:
            warnings = write_urdf(self.editor, filename)
        except Exception as e:
            self.show_status("Export failed: {}".format(e), error=True)
            return False
        message = "Exported {}".format(filename)
        if warnings:
            message += "\n" + "\n".join(warnings)
        self.show_status(message, warn=bool(warnings))
        return True

    @Slot()
    def export_xacro_clicked(self):
        if self.dirty:
            QtWidgets.QMessageBox.information(
                self.widget, "Export xacro",
                "Your text edits are not applied yet. The export uses the current frames.")
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self.widget, "Export xacro", default_xacro_path(self.editor), "xacro files (*.xacro)")
        if filename:
            self.export_xacro_to(filename)

    def export_xacro_to(self, filename):
        try:
            top, macro, warnings = write_xacro(self.editor, filename)
        except Exception as e:
            self.show_status("xacro export failed: {}".format(e), error=True)
            return False
        message = "Exported {}\nand macro {}".format(top, macro)
        if warnings:
            message += "\n" + "\n".join(warnings)
        self.show_status(message, warn=bool(warnings))
        return True

    def yaml_saved(self, yaml_path):
        """Called by the GUI after a successful YAML save."""
        if self.widget.isVisible() and self.chk_save_with_yaml.isChecked() and yaml_path:
            self.export_to(os.path.splitext(yaml_path)[0] + ".urdf")

# eof
