#!/usr/bin/env python3
"""Convert frame editor frames to URDF text and back.

Pure Python: no ROS or Qt imports, so it can be unit tested on its own.

Mapping (frame editor -> URDF):
    frame name                     -> <link name>
    parent + position + orientation -> fixed <joint> with <origin xyz rpy>
    style cube   (length/width/height) -> <box size="l w h"/>
    style plane  (length/width)        -> thin <box>, see PLANE_THICKNESS
    style sphere (diameter)            -> <sphere radius="d/2"/>
    style mesh   (package/path/scale)  -> <mesh filename="package://..." scale/>
    style axis / none                  -> link without <visual>
    color                              -> <material><color rgba/></material>
Parents that are not editor frames (e.g. "world") become empty root links.
"""

import math
import xml.etree.ElementTree as ET
from xml.sax.saxutils import quoteattr

DEFAULT_ROBOT_NAME = "frame_editor_cell"
PLANE_THICKNESS = 0.001
DECIMALS = 6
POS_TOL = 1e-5
ANG_TOL = 1e-5
SIZE_TOL = 1e-6
COLOR_TOL = 1e-3


class FrameSpec(object):
    """What URDF can say about one editor frame."""

    def __init__(self, name, parent, position=(0.0, 0.0, 0.0),
                 orientation=(0.0, 0.0, 0.0, 1.0), geometry=None, color=None):
        self.name = name
        self.parent = parent
        self.position = tuple(float(v) for v in position)
        self.orientation = tuple(float(v) for v in orientation)
        # None | ("box", (l, w, h)) | ("sphere", radius) | ("mesh", filename, scale)
        self.geometry = geometry
        self.color = tuple(float(c) for c in color) if color is not None else None

    def __repr__(self):
        return "FrameSpec({!r}, parent={!r}, pos={}, quat={}, geom={}, color={})".format(
            self.name, self.parent, self.position, self.orientation, self.geometry, self.color)


## Math ##
##
def quaternion_to_rpy(q):
    """(x, y, z, w) -> (roll, pitch, yaw), same convention as URDF and tf ('sxyz')."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


def rpy_to_quaternion(rpy):
    """(roll, pitch, yaw) -> (x, y, z, w)."""
    r, p, y = rpy
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quaternions_equal(a, b, tol=ANG_TOL):
    """True if both quaternions describe the same rotation (q and -q are equal)."""
    na = math.sqrt(sum(v * v for v in a)) or 1.0
    nb = math.sqrt(sum(v * v for v in b)) or 1.0
    dot = abs(sum(x * y for x, y in zip(a, b))) / (na * nb)
    angle = 2.0 * math.acos(min(1.0, dot))
    return angle < tol


def fmt(value):
    """Short, stable number text: rounded, no trailing zeros, no '-0'."""
    s = "{:.{}f}".format(round(float(value), DECIMALS), DECIMALS).rstrip("0").rstrip(".")
    if s in ("-0", ""):
        s = "0"
    return s


def fmt_vec(values):
    return " ".join(fmt(v) for v in values)


## Mesh paths ##
##
def mesh_uri(package, path):
    if package:
        return "package://{}/{}".format(package, path.lstrip("/"))
    if path.startswith("/"):
        return "file://" + path
    return path


def split_mesh_uri(uri):
    """URDF mesh filename -> (package, path) as the editor stores them."""
    if uri.startswith("package://"):
        rest = uri[len("package://"):]
        package, _, path = rest.partition("/")
        return package, path
    if uri.startswith("file://"):
        return "", uri[len("file://"):]
    return "", uri


## Editor frames -> specs ##
##
def is_temporary(name):
    return name.startswith("_")


def frame_to_spec(frame):
    """Duck-typed: works with frame_editor_py.objects.Frame and subclasses."""
    style = getattr(frame, "style", "none")
    geometry = None
    if style == "cube":
        geometry = ("box", (frame.length, frame.width, frame.height))
    elif style == "plane":
        geometry = ("box", (frame.length, frame.width, PLANE_THICKNESS))
    elif style == "sphere":
        geometry = ("sphere", frame.diameter / 2.0)
    elif style == "mesh" and getattr(frame, "path", ""):
        geometry = ("mesh", mesh_uri(frame.package or "", frame.path), frame.scale)
    color = frame.color if geometry is not None else None
    return FrameSpec(frame.name, frame.parent, frame.position, frame.orientation, geometry, color)


def specs_from_frames(frames):
    specs = {}
    for frame in frames:
        if is_temporary(frame.name):
            continue
        specs[frame.name] = frame_to_spec(frame)
    return specs


## Tree helpers ##
##
def tree_order(specs):
    """Returns (external_roots, ordered_names, cyclic_names).

    ordered_names lists every spec reachable from a root, parent before child,
    siblings alphabetically. cyclic_names are specs in a parent loop.
    """
    children = {}
    for spec in specs.values():
        children.setdefault(spec.parent, []).append(spec.name)
    roots = sorted({s.parent for s in specs.values() if s.parent not in specs})

    ordered = []
    seen = set()
    # Depth first, alphabetical among siblings
    def visit(name):
        for child in sorted(children.get(name, [])):
            if child in seen:
                continue
            seen.add(child)
            ordered.append(child)
            visit(child)
    for root in roots:
        visit(root)
    cyclic = sorted(set(specs) - seen)
    return roots, ordered, cyclic


## Specs -> URDF text ##
##
def _geometry_lines(geometry, indent):
    kind = geometry[0]
    if kind == "box":
        shape = '<box size="{}"/>'.format(fmt_vec(geometry[1]))
    elif kind == "sphere":
        shape = '<sphere radius="{}"/>'.format(fmt(geometry[1]))
    else:
        shape = '<mesh filename={} scale="{}"/>'.format(quoteattr(geometry[1]), fmt_vec((geometry[2],) * 3))
    return [indent + "<geometry>", indent + "  " + shape, indent + "</geometry>"]


def frames_to_urdf(frames, robot_name=DEFAULT_ROBOT_NAME, collision=False):
    """Editor frames -> (urdf_text, warnings).

    collision: also write each shape as a <collision> element (same geometry).
    """
    frames = [f for f in frames if not is_temporary(f.name)]
    specs = specs_from_frames(frames)
    styles = {f.name: getattr(f, "style", "none") for f in frames}
    warnings = []

    roots, ordered, cyclic = tree_order(specs)
    if len(roots) > 1:
        warnings.append("More than one root link ({}). A URDF must have exactly one root: "
                        "give these a common parent.".format(", ".join(roots)))
    if cyclic:
        warnings.append("Parent loop, not exported: {}".format(", ".join(cyclic)))

    joint_names = set()
    out = []
    out.append('<?xml version="1.0"?>')
    out.append("<!-- Generated by frame_editor. Edit here and press Apply, or use the editor. -->")
    out.append("<robot name={}>".format(quoteattr(robot_name)))

    for root in roots:
        out.append("")
        out.append("  <!-- {} is not an editor frame: exported as an empty root link -->".format(root))
        out.append("  <link name={}/>".format(quoteattr(root)))

    for name in ordered:
        spec = specs[name]
        style = styles.get(name, "none")
        out.append("")
        if style == "plane":
            out.append("  <!-- frame_editor style: plane (exported as a thin box) -->")
        elif style == "axis":
            out.append("  <!-- frame_editor style: axis (no URDF equivalent, not exported) -->")
            warnings.append("'{}': style axis has no URDF equivalent, exported without geometry.".format(name))
        elif style == "mesh" and spec.geometry is None:
            warnings.append("'{}': mesh without a file, exported without geometry.".format(name))

        if spec.geometry is None:
            out.append("  <link name={}/>".format(quoteattr(name)))
        else:
            out.append("  <link name={}>".format(quoteattr(name)))
            if spec.geometry[0] == "mesh" and not spec.geometry[1].startswith("package://"):
                warnings.append("'{}': mesh uses an absolute path; package:// paths are more portable.".format(name))
            out.append("    <visual>")
            out.extend(_geometry_lines(spec.geometry, "      "))
            out.append("      <material name={}>".format(quoteattr(name + "_material")))
            out.append('        <color rgba="{}"/>'.format(fmt_vec(spec.color)))
            out.append("      </material>")
            out.append("    </visual>")
            if collision:
                out.append("    <collision>")
                out.extend(_geometry_lines(spec.geometry, "      "))
                out.append("    </collision>")
            out.append("  </link>")

        joint_name = "{}_to_{}".format(spec.parent, name)
        base, i = joint_name, 2
        while joint_name in joint_names:
            joint_name = "{}_{}".format(base, i)
            i += 1
        joint_names.add(joint_name)
        out.append("  <joint name={} type=\"fixed\">".format(quoteattr(joint_name)))
        out.append("    <parent link={}/>".format(quoteattr(spec.parent)))
        out.append("    <child link={}/>".format(quoteattr(name)))
        out.append('    <origin xyz="{}" rpy="{}"/>'.format(
            fmt_vec(spec.position), fmt_vec(quaternion_to_rpy(spec.orientation))))
        out.append("  </joint>")

    out.append("")
    out.append("</robot>")
    return "\n".join(out) + "\n", warnings


## URDF text -> specs ##
##
class URDFError(Exception):
    pass


def _floats(text, count, what):
    try:
        values = [float(v) for v in text.split()]
    except ValueError:
        raise URDFError("{}: '{}' is not a list of numbers".format(what, text))
    if len(values) != count:
        raise URDFError("{}: expected {} numbers, got '{}'".format(what, count, text))
    return values


def _origin(element, what):
    origin = element.find("origin")
    if origin is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = _floats(origin.get("xyz", "0 0 0"), 3, what + " origin xyz")
    rpy = _floats(origin.get("rpy", "0 0 0"), 3, what + " origin rpy")
    return tuple(xyz), tuple(rpy)


def _parse_visual(link, name):
    visuals = link.findall("visual")
    collisions = link.findall("collision")
    if not visuals:
        if collisions:
            raise URDFError("link '{}': a <collision> needs a matching <visual>".format(name))
        return None, None
    if len(visuals) > 1:
        raise URDFError("link '{}': only one <visual> per link is supported".format(name))
    visual = visuals[0]
    geometry = _parse_shape(visual, name, "visual")

    # <collision> is accepted only as a copy of the visual shape: the editor keeps one shape.
    if len(collisions) > 1:
        raise URDFError("link '{}': only one <collision> per link is supported".format(name))
    if collisions and not _geometry_equal(_parse_shape(collisions[0], name, "collision"), geometry):
        raise URDFError("link '{}': the <collision> shape must be the same as the <visual> shape "
                        "(the frame editor stores one shape per frame)".format(name))

    color = None
    material = visual.find("material")
    if material is not None:
        color_el = material.find("color")
        if color_el is not None:
            color = tuple(_floats(color_el.get("rgba", ""), 4, "link '{}' color rgba".format(name)))
    return geometry, color


def _parse_shape(element, name, what):
    """<visual> or <collision> -> geometry tuple."""
    xyz, rpy = _origin(element, "link '{}' {}".format(name, what))
    if any(abs(v) > POS_TOL for v in xyz + rpy):
        raise URDFError("link '{}': a <{}> <origin> is not supported, the shape sits on the "
                        "link origin. Move the joint instead, or add a child link.".format(name, what))
    geometry_el = element.find("geometry")
    if geometry_el is None or len(list(geometry_el)) != 1:
        raise URDFError("link '{}': <geometry> needs exactly one shape".format(name))
    shape = list(geometry_el)[0]
    if shape.tag == "box":
        geometry = ("box", tuple(_floats(shape.get("size", ""), 3, "link '{}' box size".format(name))))
    elif shape.tag == "sphere":
        geometry = ("sphere", _floats(shape.get("radius", ""), 1, "link '{}' sphere radius".format(name))[0])
    elif shape.tag == "mesh":
        filename = shape.get("filename", "")
        if not filename:
            raise URDFError("link '{}': <mesh> needs a filename".format(name))
        scale = _floats(shape.get("scale", "1 1 1"), 3, "link '{}' mesh scale".format(name))
        if max(scale) - min(scale) > SIZE_TOL:
            raise URDFError("link '{}': the editor only supports a uniform mesh scale".format(name))
        geometry = ("mesh", filename, scale[0])
    else:
        raise URDFError("link '{}': <{}> is not supported (use box, sphere or mesh)".format(name, shape.tag))
    return geometry


SUPPORTED_LINK_TAGS = {"visual", "collision"}
SUPPORTED_JOINT_TAGS = {"parent", "child", "origin"}


def urdf_to_specs(text):
    """URDF text -> (specs, roots). specs only holds links that have a parent joint.

    Raises URDFError for anything the editor cannot represent, so nothing is
    dropped silently.
    """
    try:
        root = ET.fromstring(text)
    except ET.ParseError as e:
        raise URDFError("XML error: {}".format(e))
    if root.tag != "robot":
        raise URDFError("the top element must be <robot>, not <{}>".format(root.tag))

    links = {}
    joints = []
    for element in root:
        if element.tag == "link":
            name = element.get("name", "")
            if not name:
                raise URDFError("a <link> has no name")
            if name in links:
                raise URDFError("link '{}' is defined twice".format(name))
            for child in element:
                if child.tag not in SUPPORTED_LINK_TAGS:
                    raise URDFError("link '{}': <{}> is not supported by the frame editor".format(name, child.tag))
            links[name] = element
        elif element.tag == "joint":
            joints.append(element)
        else:
            raise URDFError("<{}> is not supported by the frame editor (only <link> and <joint>)".format(element.tag))

    parent_of = {}
    origin_of = {}
    for joint in joints:
        jname = joint.get("name", "")
        what = "joint '{}'".format(jname)
        if joint.get("type") != "fixed":
            raise URDFError("{}: only type=\"fixed\" is supported, got '{}'".format(what, joint.get("type")))
        for child in joint:
            if child.tag not in SUPPORTED_JOINT_TAGS:
                raise URDFError("{}: <{}> is not supported".format(what, child.tag))
        parent_el, child_el = joint.find("parent"), joint.find("child")
        if parent_el is None or child_el is None:
            raise URDFError("{}: needs <parent link> and <child link>".format(what))
        parent, child = parent_el.get("link", ""), child_el.get("link", "")
        for link_name in (parent, child):
            if link_name not in links:
                raise URDFError("{}: link '{}' is not defined".format(what, link_name))
        if child in parent_of:
            raise URDFError("link '{}' has more than one parent joint".format(child))
        if is_temporary(child):
            raise URDFError("link '{}': names starting with '_' are reserved for temporary frames".format(child))
        parent_of[child] = parent
        origin_of[child] = _origin(joint, what)

    specs = {}
    roots = []
    for name, link in links.items():
        geometry, color = _parse_visual(link, name)
        if name not in parent_of:
            if geometry is not None:
                raise URDFError("root link '{}' cannot have a <visual>: the editor stores a shape only "
                                "on frames that have a parent joint".format(name))
            roots.append(name)
            continue
        xyz, rpy = origin_of[name]
        if geometry is not None and color is None:
            color = (0.0, 0.5, 0.5, 0.75)  # editor default
        specs[name] = FrameSpec(name, parent_of[name], xyz, rpy_to_quaternion(rpy), geometry, color)

    _, _, cyclic = tree_order(specs)
    if cyclic:
        raise URDFError("parent loop between: {}".format(", ".join(cyclic)))
    return specs, sorted(roots)


## Diff ##
##
def _geometry_equal(a, b):
    if a is None or b is None:
        return a is None and b is None
    if a[0] != b[0]:
        return False
    if a[0] == "box":
        return all(abs(x - y) < SIZE_TOL for x, y in zip(a[1], b[1]))
    if a[0] == "sphere":
        return abs(a[1] - b[1]) < SIZE_TOL
    if a[0] == "mesh":
        return a[1] == b[1] and abs(a[2] - b[2]) < SIZE_TOL
    return False


def _color_equal(a, b):
    if a is None or b is None:
        return True  # no geometry on one side: colour is irrelevant
    return all(abs(x - y) < COLOR_TOL for x, y in zip(a, b))


class Change(object):
    def __init__(self, kind, name, spec=None):
        self.kind = kind    # "remove" | "add" | "parent" | "pose" | "geometry" | "color"
        self.name = name
        self.spec = spec

    def __repr__(self):
        return "Change({}, {})".format(self.kind, self.name)


def diff_specs(current, wanted):
    """Changes that turn `current` specs into `wanted` specs.

    Order: removes, then adds (parents first), then edits of kept frames.
    """
    changes = []
    for name in sorted(set(current) - set(wanted)):
        changes.append(Change("remove", name))

    _, ordered, _ = tree_order(wanted)
    for name in ordered:
        if name not in current:
            changes.append(Change("add", name, wanted[name]))

    for name in ordered:
        if name not in current:
            continue
        old, new = current[name], wanted[name]
        if old.parent != new.parent:
            changes.append(Change("parent", name, new))
        if (any(abs(a - b) > POS_TOL for a, b in zip(old.position, new.position))
                or not quaternions_equal(old.orientation, new.orientation)):
            changes.append(Change("pose", name, new))
        if not _geometry_equal(old.geometry, new.geometry):
            changes.append(Change("geometry", name, new))
        elif not _color_equal(old.color, new.color):
            changes.append(Change("color", name, new))
    return changes

# eof
