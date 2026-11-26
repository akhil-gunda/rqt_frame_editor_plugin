#!/usr/bin/env python3
"""Apply edited URDF text to the frame editor, using its normal undoable commands.

Everything is checked first; then all changes are pushed as one undo macro
("Apply URDF"), so a single Undo reverts the whole edit.
"""

from frame_editor_py.commands import (Command_AddElement, Command_RemoveElement,
                                      Command_SetParent, Command_SetPose,
                                      Command_SetStyle, Command_SetGeometry,
                                      Command_SetSize, Command_SetStyleColor)
from frame_editor_py.objects import (Frame, Object_Cube, Object_Plane,
                                     Object_Sphere, Object_Cylinder, Object_Mesh)
from frame_editor_py import urdf_model as um
from frame_editor_py.urdf_model import URDFError


def current_specs(editor):
    return um.specs_from_frames(list(editor.frames.values()))


def style_of(geometry):
    """URDF geometry -> (editor style, attributes to set)."""
    if geometry is None:
        return "none", {}
    kind = geometry[0]
    if kind == "box":
        l, w, h = geometry[1]
        if abs(h - um.PLANE_THICKNESS) < um.SIZE_TOL:
            return "plane", {"length": l, "width": w}
        return "cube", {"length": l, "width": w, "height": h}
    if kind == "sphere":
        return "sphere", {"diameter": 2.0 * geometry[1]}
    if kind == "cylinder":
        return "cylinder", {"diameter": 2.0 * geometry[1], "height": geometry[2]}
    if kind == "mesh":
        package, path = um.split_mesh_uri(geometry[1])
        return "mesh", {"package": package, "path": path, "scale": geometry[2]}
    raise URDFError("unknown geometry {}".format(kind))


def new_element(spec):
    style, a = style_of(spec.geometry)
    args = (spec.name, spec.position, spec.orientation, spec.parent)
    if style == "cube":
        element = Object_Cube(*args, length=a["length"], width=a["width"], height=a["height"])
    elif style == "plane":
        element = Object_Plane(*args, length=a["length"], width=a["width"])
    elif style == "sphere":
        element = Object_Sphere(*args, diameter=a["diameter"])
    elif style == "cylinder":
        element = Object_Cylinder(*args, diameter=a["diameter"], height=a["height"])
    elif style == "mesh":
        element = Object_Mesh(*args, package=a["package"], mesh_path=a["path"], scale=a["scale"])
    else:
        element = Frame(*args)
    if spec.geometry is not None:
        element.set_color(spec.color)
    return element


def _check_mesh_package(geometry, name):
    if geometry is None or geometry[0] != "mesh":
        return
    package, _ = um.split_mesh_uri(geometry[1])
    if not package:
        return
    import rospkg  # same lookup the editor uses to show meshes
    try:
        rospkg.RosPack().get_path(package)
    except Exception:
        raise URDFError("link '{}': package '{}' not found (is it built and sourced?)".format(name, package))


def plan(editor, text):
    """Parse and check `text`. Returns the list of changes; raises URDFError."""
    wanted, roots = um.urdf_to_specs(text)
    current = current_specs(editor)

    for root in roots:
        if root in current:
            raise URDFError("link '{}' is an editor frame and needs a parent joint "
                            "(only links not owned by the editor may be roots)".format(root))

    changes = um.diff_specs(current, wanted)

    tf_names = set(editor.tf_dict().keys())
    for change in changes:
        if change.kind == "add" and change.name in tf_names \
                and not Frame.was_published_by_frameeditor(change.name):
            raise URDFError("link '{}': a frame with this name is already published by another "
                            "node".format(change.name))
        if change.kind in ("add", "geometry"):
            _check_mesh_package(change.spec.geometry, change.name)
    return changes


def apply(editor, text):
    """Apply `text` to the editor as one undo step. Returns the applied changes."""
    changes = plan(editor, text)
    if not changes:
        return changes

    stack = editor.undo_stack
    stack.beginMacro("Apply URDF")
    try:
        for change in changes:
            _apply_one(editor, change)
    except Exception as e:
        stack.endMacro()
        stack.undo()  # roll back the partial macro
        raise URDFError("applying failed, nothing changed: {}".format(e))
    stack.endMacro()
    return changes


def _apply_one(editor, change):
    name, spec = change.name, change.spec

    if change.kind == "remove":
        editor.command(Command_RemoveElement(editor, editor.frames[name]))

    elif change.kind == "add":
        editor.command(Command_AddElement(editor, new_element(spec)))

    elif change.kind == "parent":
        editor.command(Command_SetParent(editor, editor.frames[name], spec.parent, False))

    elif change.kind == "pose":
        editor.command(Command_SetPose(editor, editor.frames[name], spec.position, spec.orientation))

    elif change.kind == "geometry":
        style, attrs = style_of(spec.geometry)
        element = editor.frames[name]
        if element.style != style:
            command = Command_SetStyle(editor, element, style)
            editor.command(command)
            element = command.new_element
        for attr in ("length", "width", "height", "diameter", "package", "path"):
            if attr in attrs and getattr(element, attr, None) != attrs[attr]:
                editor.command(Command_SetGeometry(editor, element, attr, attrs[attr]))
        if "scale" in attrs and element.scale != attrs["scale"]:
            editor.command(Command_SetSize(editor, element, attrs["scale"]))
        if spec.geometry is not None and tuple(element.color) != tuple(spec.color):
            editor.command(Command_SetStyleColor(editor, element, spec.color))

    elif change.kind == "color":
        editor.command(Command_SetStyleColor(editor, editor.frames[name], spec.color))

# eof
