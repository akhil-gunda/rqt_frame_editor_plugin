"""Revolute joints: URDF text, the editor commands, the joint panel, YAML and xacro."""
import pytest
import yaml

from frame_editor_py import urdf_model as um
from frame_editor_py.urdf_model import URDFError
from test_urdf_model import frame
from test_urdf_integration import Ctx, app, editor, urdf  # noqa: F401 (fixtures)

HINGE = um.Joint("revolute", (0.0, 0.0, -1.0), -0.5, 1.2, 25.0, 0.5)


def door():
    hinge = frame("hinge", "post", (0, -0.045, 0))
    hinge.joint = HINGE
    return [frame("post", "world"), hinge, frame("panel", "hinge", (0, -0.35, 0))]


def test_export_and_roundtrip():
    text, _ = um.frames_to_urdf(door())
    assert '<joint name="post_to_hinge" type="revolute">' in text
    assert '<axis xyz="0 0 -1"/>' in text
    assert '<limit lower="-0.5" upper="1.2" effort="25" velocity="0.5"/>' in text
    assert text.count('type="fixed"') == 2
    specs, _ = um.urdf_to_specs(text)
    assert specs["hinge"].joint == HINGE and specs["panel"].joint is None
    assert um.diff_specs(um.specs_from_frames(door()), specs) == []


@pytest.mark.parametrize("old,new", [
    ('upper="1.2"', 'upper="1.5"'),
    ('<axis xyz="0 0 -1"/>', '<axis xyz="0 1 0"/>'),
    ('velocity="0.5"', 'velocity="2"'),
])
def test_joint_edits_are_detected(old, new):
    text = um.frames_to_urdf(door())[0]
    wanted, _ = um.urdf_to_specs(text.replace(old, new))
    changes = um.diff_specs(um.specs_from_frames(door()), wanted)
    assert [(c.kind, c.name) for c in changes] == [("joint", "hinge")]


def test_urdf_default_axis_is_x():
    text = um.frames_to_urdf(door())[0].replace('<axis xyz="0 0 -1"/>', "")
    assert um.urdf_to_specs(text)[0]["hinge"].joint.axis == (1.0, 0.0, 0.0)


@pytest.mark.parametrize("old,new,message", [
    ('<limit lower="-0.5" upper="1.2" effort="25" velocity="0.5"/>', "", "needs <limit"),
    (' effort="25"', "", "needs <limit"),
    ('lower="-0.5"', 'lower="2"', "lower limit"),
    ('<axis xyz="0 0 -1"/>', '<axis xyz="0 0 0"/>', "0 0 0"),
    ('<axis xyz="0 0 -1"/>', '<axis xyz="0 0 -1"/><dynamics damping="1"/>', "dynamics"),
    ('type="revolute"', 'type="fixed"', "not supported on a fixed joint"),
    ('type="revolute"', 'type="prismatic"', "only type"),
])
def test_rejects(old, new, message):
    text = um.frames_to_urdf(door())[0]
    assert old in text
    with pytest.raises(URDFError, match=message):
        um.urdf_to_specs(text.replace(old, new))


def test_yaml_dict_roundtrip_and_validation():
    assert um.joint_from_dict(um.joint_to_dict(HINGE)) == HINGE
    assert um.joint_from_dict(None) is None and um.joint_from_dict({"type": "fixed"}) is None
    assert um.joint_from_dict({"type": "revolute"}) == um.DEFAULT_REVOLUTE
    with pytest.raises(ValueError):
        um.joint_from_dict({"type": "revolute", "lower": 3.0})


def test_xacro_keeps_revolute_joints(tmp_path):
    xacro = pytest.importorskip("xacro")
    macro, top, _ = um.frames_to_xacro(door(), "d", "d_macro.xacro")
    (tmp_path / "d_macro.xacro").write_text(macro)
    (tmp_path / "d.urdf.xacro").write_text(top)
    specs, _ = um.urdf_to_specs(xacro.process_file(str(tmp_path / "d.urdf.xacro")).toxml())
    assert specs["hinge"].joint == HINGE


def test_apply_set_joint_and_undo(editor):  # noqa: F811
    before = urdf(editor)
    revolute = before.replace(
        '<joint name="world_to_frame1" type="fixed">', '<joint name="world_to_frame1" type="revolute">'
    ).replace('<child link="frame1"/>', '<child link="frame1"/>\n    <axis xyz="0 0 1"/>\n'
              '    <limit lower="0" upper="1" effort="10" velocity="1"/>')
    changes = urdf_apply_(editor, revolute)
    assert [(c.kind, c.name) for c in changes] == [("joint", "frame1")]
    assert editor.frames["frame1"].joint.upper == 1.0
    assert urdf_apply_(editor, urdf(editor)) == []
    editor.undo_stack.undo()
    assert editor.frames["frame1"].joint is None and urdf(editor) == before
    editor.undo_stack.redo()
    assert urdf_apply_(editor, before)[0].kind == "joint"  # back to fixed from the text
    assert editor.frames["frame1"].joint is None


def urdf_apply_(ed, text):
    from frame_editor_py import urdf_apply
    return urdf_apply.apply(ed, text)


def test_joint_survives_style_change_duplicate_and_set_frame(editor, monkeypatch):  # noqa: F811
    from frame_editor.srv import SetFrame
    from frame_editor_py import commands
    from frame_editor_py.interface_services import FrameEditor_Services

    editor.command(commands.Command_SetJoint(editor, editor.frames["frame1"], HINGE))
    editor.command(commands.Command_SetStyle(editor, editor.frames["frame1"], "cube"))
    assert editor.frames["frame1"].joint == HINGE
    editor.undo_stack.undo()
    assert editor.frames["frame1"].joint == HINGE

    # duplicate: the TF lookup for the pose is replaced, only the copy itself is tested here
    monkeypatch.setattr(commands, "FromTransformStamped", lambda t: ((0, 0, 0), (0, 0, 0, 1)))
    monkeypatch.setattr(type(editor.frames["frame1"].tf_buffer), "lookup_transform", lambda *a: None)
    editor.command(commands.Command_CopyElement(editor, "copy", "frame1", "world"))
    assert editor.frames["copy"].joint == HINGE

    request = SetFrame.Request()
    request.name = "frame1"
    request.pose.orientation.w = 1.0
    service = FrameEditor_Services.__new__(FrameEditor_Services)
    service.editor = editor
    service.callback_set_frame(request, SetFrame.Response())
    assert editor.frames["frame1"].joint == HINGE


def test_joint_panel(editor):  # noqa: F811
    from frame_editor_py.interface_gui import FrameEditor_JointWidget
    from frame_editor_py.commands import Command_SelectElement, Command_SetJoint

    panel = FrameEditor_JointWidget(editor)
    assert not panel.get_widget().isEnabled()  # nothing selected
    editor.command(Command_SelectElement(editor, editor.frames["frame1"]))
    assert panel.type_combo.currentText() == "fixed"
    assert all(s.isHidden() for s in panel.spinboxes.values())

    panel.type_combo.setCurrentText("revolute")
    panel.type_changed()
    assert editor.frames["frame1"].joint == um.DEFAULT_REVOLUTE
    assert not any(s.isHidden() for s in panel.spinboxes.values())
    assert "90.0" in panel.info.text()

    # leaving a box without editing it must not round the stored value or add an undo step
    precise = um.DEFAULT_REVOLUTE._replace(upper=1.5707963267948966)
    editor.command(Command_SetJoint(editor, editor.frames["frame1"], precise))
    index = editor.undo_stack.index()
    for key in panel.spinboxes:
        panel.value_changed(key)
    assert editor.undo_stack.index() == index
    assert editor.frames["frame1"].joint.upper == precise.upper

    panel.spinboxes["lower"].setValue(-1.0)
    panel.value_changed("lower")
    assert editor.frames["frame1"].joint == precise._replace(lower=-1.0)

    panel.spinboxes["lower"].setValue(3.0)  # above the upper limit: refused
    panel.value_changed("lower")
    assert editor.frames["frame1"].joint.lower == -1.0
    assert panel.spinboxes["lower"].value() == -1.0 and "Not changed" in panel.info.text()

    editor.undo_stack.undo()
    assert editor.frames["frame1"].joint == precise
    assert panel.spinboxes["lower"].value() == 0.0


def test_yaml_save_and_load(editor, tmp_path):  # noqa: F811
    from frame_editor_py.commands import Command_SetJoint
    from frame_editor_py.editor import FrameEditor

    plain = str(tmp_path / "plain.yaml")
    editor.save_file(plain)
    assert "joint" not in open(plain).read()  # fixed projects are saved as before

    editor.command(Command_SetJoint(editor, editor.frames["frame1"], HINGE))
    path = str(tmp_path / "door.yaml")
    editor.save_file(path)
    saved = yaml.safe_load(open(path))["frames"]  # plain YAML, no python tags
    assert saved["frame1"]["joint"]["axis"] == [0.0, 0.0, -1.0]
    other = FrameEditor(Ctx())
    try:
        other.load_file(path)
        assert other.frames["frame1"].joint == HINGE
        assert other.frames["frame_cube"].joint is None
    finally:
        other.node.destroy_node()
