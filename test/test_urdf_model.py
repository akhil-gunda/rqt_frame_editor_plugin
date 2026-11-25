"""Unit tests for frame_editor_py.urdf_model (pure Python, no ROS needed).

Run: python3 -m pytest test/ -q
"""
import math
import os
import sys
from types import SimpleNamespace

import pytest

from frame_editor_py import urdf_model as um  # noqa: E402
from frame_editor_py.urdf_model import URDFError  # noqa: E402

COLOR = (0.0, 0.5, 0.5, 0.75)
Q_ID = (0.0, 0.0, 0.0, 1.0)


def frame(name, parent="world", position=(0, 0, 0), orientation=Q_ID, style="none", **data):
    data.setdefault("color", COLOR)
    return SimpleNamespace(name=name, parent=parent, position=position,
                           orientation=orientation, style=style, **data)


def cell():
    yaw90 = um.rpy_to_quaternion((0, 0, math.pi / 2))
    return [
        frame("table", "world", (0, 0, 0.4), style="cube", length=1.2, width=0.8, height=0.8),
        frame("robot_base", "table", (0.3, 0, 0.4), yaw90),
        frame("camera", "robot_base", (0, 0, 1.0), style="sphere", diameter=0.1, color=(1, 0, 0, 1)),
        frame("floor", "world", style="plane", length=3, width=3),
        frame("marker_axis", "table", style="axis", length=0.2, width=0.01),
        frame("gripper", "robot_base", style="mesh", package="my_pkg", path="meshes/g.stl", scale=0.001),
        frame("_temp", "world"),
    ]


def test_rpy_quaternion_roundtrip():
    for rpy in [(0, 0, 0), (0.1, -0.2, 0.3), (math.pi, 0, 0), (0, 0, -math.pi / 2), (1.0, 1.2, -2.0)]:
        q = um.rpy_to_quaternion(rpy)
        assert um.quaternions_equal(q, um.rpy_to_quaternion(um.quaternion_to_rpy(q)))


def test_rpy_matches_tf_transformations():
    tft = pytest.importorskip("tf_transformations")
    for rpy in [(0.1, -0.2, 0.3), (1.0, 1.2, -2.0), (math.pi, 0.0, 0.5)]:
        assert um.quaternions_equal(um.rpy_to_quaternion(rpy), tuple(tft.quaternion_from_euler(*rpy)))
        q = tft.quaternion_from_euler(*rpy)
        assert all(abs(a - b) < 1e-9 for a, b in zip(um.quaternion_to_rpy(q), tft.euler_from_quaternion(q)))


def test_fmt():
    assert um.fmt(-0.0000001) == "0"
    assert um.fmt(0.47346997261047363) == "0.47347"
    assert um.fmt(2.0) == "2"


def test_export_contents():
    text, warnings = um.frames_to_urdf(cell(), "my_cell")
    assert '<robot name="my_cell">' in text
    assert '<link name="world"/>' in text
    assert '<box size="1.2 0.8 0.8"/>' in text
    assert '<sphere radius="0.05"/>' in text
    assert '<box size="3 3 0.001"/>' in text
    assert 'filename="package://my_pkg/meshes/g.stl" scale="0.001 0.001 0.001"' in text
    assert '<origin xyz="0.3 0 0.4" rpy="0 0 1.570796"/>' in text
    assert "_temp" not in text
    assert any("axis" in w for w in warnings)
    # parent before child
    assert text.index('<link name="table">') < text.index('<link name="robot_base"/>')


def test_export_is_deterministic():
    frames = cell()
    assert um.frames_to_urdf(frames)[0] == um.frames_to_urdf(list(reversed(frames)))[0]


def test_roundtrip_has_no_changes():
    frames = cell()
    text, _ = um.frames_to_urdf(frames)
    wanted, roots = um.urdf_to_specs(text)
    assert roots == ["world"]
    assert um.diff_specs(um.specs_from_frames(frames), wanted) == []


def test_multiple_roots_warning():
    _, warnings = um.frames_to_urdf([frame("a", "world"), frame("b", "tool0")])
    assert any("root" in w for w in warnings)


def edited(old, new):
    text, _ = um.frames_to_urdf(cell())
    assert old in text, old
    return text.replace(old, new)


def kinds(text):
    wanted, _ = um.urdf_to_specs(text)
    return [(c.kind, c.name) for c in um.diff_specs(um.specs_from_frames(cell()), wanted)]


def test_diff_pose():
    assert kinds(edited('xyz="0.3 0 0.4"', 'xyz="0.5 0 0.4"')) == [("pose", "robot_base")]


def test_diff_parent():
    text = edited('<parent link="robot_base"/>\n    <child link="camera"/>',
                  '<parent link="table"/>\n    <child link="camera"/>')
    assert kinds(text) == [("parent", "camera")]


def test_diff_geometry_and_color():
    assert kinds(edited('<sphere radius="0.05"/>', '<sphere radius="0.2"/>')) == [("geometry", "camera")]
    assert kinds(edited('rgba="1 0 0 1"', 'rgba="0 1 0 1"')) == [("color", "camera")]


def test_diff_add_and_remove():
    text, _ = um.frames_to_urdf(cell())
    text = text.replace("</robot>",
                        '<link name="shelf"><visual><geometry><box size="1 1 1"/></geometry></visual></link>\n'
                        '<joint name="j" type="fixed"><parent link="table"/><child link="shelf"/></joint>\n'
                        "</robot>")
    assert kinds(text) == [("add", "shelf")]


@pytest.mark.parametrize("old,new,message", [
    ('type="fixed"', 'type="revolute"', "only type"),
    ("<visual>", "<inertial/><visual>", "inertial"),
    ('<sphere radius="0.05"/>', '<cylinder radius="0.05" length="1"/>', "cylinder"),
    ('<parent link="table"/>', '<parent link="nope"/>', "not defined"),
    ('<link name="world"/>', '<link name="world"><visual><geometry><box size="1 1 1"/></geometry></visual></link>', "root link"),
    ('<sphere radius="0.05"/>', '<sphere radius="0.05"/><box size="1 1 1"/>', "exactly one shape"),
    ("</robot>", "<gazebo/></robot>", "gazebo"),
    ("</robot>", "<link name=\"table\"/></robot>", "twice"),
    ("</robot", "</rob", "XML error"),
])
def test_rejects_what_the_editor_cannot_hold(old, new, message):
    with pytest.raises(URDFError, match=message):
        um.urdf_to_specs(edited(old, new))


def test_visual_origin_rejected():
    text = edited('<sphere radius="0.05"/>\n      </geometry>',
                  '<sphere radius="0.05"/>\n      </geometry>\n      <origin xyz="0 0 1"/>')
    with pytest.raises(URDFError, match="origin"):
        um.urdf_to_specs(text)


def test_parent_loop_rejected():
    text = ('<robot name="r"><link name="a"/><link name="b"/>'
            '<joint name="1" type="fixed"><parent link="a"/><child link="b"/></joint>'
            '<joint name="2" type="fixed"><parent link="b"/><child link="a"/></joint></robot>')
    with pytest.raises(URDFError, match="loop"):
        um.urdf_to_specs(text)


def test_collision_export_and_roundtrip():
    frames = cell()
    text, _ = um.frames_to_urdf(frames, collision=True)
    assert text.count("<collision>") == 4  # table, camera, floor, gripper have shapes
    assert "<collision>\n      <geometry>\n        <box size=\"1.2 0.8 0.8\"/>" in text
    wanted, _ = um.urdf_to_specs(text)
    assert um.diff_specs(um.specs_from_frames(frames), wanted) == []
    assert "<collision>" not in um.frames_to_urdf(frames)[0]  # off by default


def test_collision_must_match_visual():
    text, _ = um.frames_to_urdf(cell(), collision=True)
    bad = text.replace('<sphere radius="0.05"/>\n      </geometry>\n    </collision>',
                       '<sphere radius="0.5"/>\n      </geometry>\n    </collision>')
    assert bad != text
    with pytest.raises(URDFError, match="same as the <visual>"):
        um.urdf_to_specs(bad)


def test_collision_without_visual_rejected():
    text = edited('<link name="robot_base"/>',
                  '<link name="robot_base"><collision><geometry><box size="1 1 1"/></geometry></collision></link>')
    with pytest.raises(URDFError, match="needs a matching <visual>"):
        um.urdf_to_specs(text)
