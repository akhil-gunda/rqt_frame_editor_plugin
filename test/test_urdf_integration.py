"""Integration tests: URDF panel and Apply against the real FrameEditor.

Needs a sourced ROS 2 workspace with frame_editor built. Uses Qt offscreen.
Run: QT_QPA_PLATFORM=offscreen python3 -m pytest test/ -q
"""
import os

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("frame_editor.srv")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding.QtWidgets import QApplication  # noqa: E402

from frame_editor_py import urdf_apply, urdf_model as um  # noqa: E402
from frame_editor_py.urdf_model import URDFError  # noqa: E402

FRAMES_YAML = os.path.join(os.path.dirname(__file__), "..", "etc", "frames.yaml")


class Ctx(object):
    def argv(self):
        return []


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance() or QApplication([])
    rclpy.init()
    yield application
    rclpy.try_shutdown()


@pytest.fixture
def editor(app):
    from frame_editor_py.editor import FrameEditor
    ed = FrameEditor(Ctx())
    ed.load_file(FRAMES_YAML)
    yield ed
    ed.node.destroy_node()


def urdf(ed):
    from frame_editor_py.interface_urdf import generate_urdf
    return generate_urdf(ed)[0]


def test_unchanged_text_applies_nothing(editor):
    assert urdf_apply.apply(editor, urdf(editor)) == []


def test_apply_is_one_undo_step(editor):
    before = urdf(editor)
    index = editor.undo_stack.index()
    text = before.replace('<sphere radius="0.5"/>', '<sphere radius="0.25"/>')
    text = text.replace("</robot>",
                        '<link name="shelf"><visual><geometry><box size="1 2 3"/></geometry></visual></link>'
                        '<joint name="j" type="fixed"><parent link="frame1"/><child link="shelf"/>'
                        '<origin xyz="1 2 3" rpy="0 0 1.5"/></joint></robot>')
    changes = urdf_apply.apply(editor, text)
    assert {(c.kind, c.name) for c in changes} == {("geometry", "frame_sphere"), ("add", "shelf")}
    assert editor.frames["frame_sphere"].diameter == pytest.approx(0.5)
    shelf = editor.frames["shelf"]
    assert (shelf.style, shelf.parent, shelf.height) == ("cube", "frame1", 3)
    assert um.quaternion_to_rpy(shelf.orientation)[2] == pytest.approx(1.5)
    assert editor.undo_stack.index() == index + 1

    editor.undo_stack.undo()
    assert "shelf" not in editor.frames
    assert urdf(editor) == before


def test_style_change_and_remove(editor):
    text = urdf(editor).replace(
        '<link name="frame_cube"/>',
        '<link name="frame_cube"><visual><geometry><box size="0.1 0.2 0.001"/></geometry></visual></link>')
    urdf_apply.apply(editor, text)
    assert editor.frames["frame_cube"].style == "plane"
    assert editor.frames["frame_cube"].group == ""  # untouched editor-only data stays

    text = urdf(editor)
    start = text.index('<link name="frame_axis"/>')
    end = text.index("</joint>", start) + len("</joint>")
    urdf_apply.apply(editor, text[:start] + text[end:])
    assert "frame_axis" not in editor.frames


def test_reparent_keeps_numbers(editor):
    text = urdf(editor).replace(
        '<parent link="frame1"/>\n    <child link="frame_plane"/>',
        '<parent link="world"/>\n    <child link="frame_plane"/>')
    old_position = editor.frames["frame_plane"].position
    urdf_apply.apply(editor, text)
    assert editor.frames["frame_plane"].parent == "world"
    assert editor.frames["frame_plane"].position == pytest.approx(old_position, abs=1e-5)


def test_invalid_text_changes_nothing(editor):
    before = urdf(editor)
    index = editor.undo_stack.index()
    with pytest.raises(URDFError):
        urdf_apply.apply(editor, before.replace('type="fixed"', 'type="revolute"', 1))
    # drop frame1's joint: frame1 would become a root, but it is an editor frame
    start = before.index('<joint name="world_to_frame1"')
    end = before.index("</joint>", start) + len("</joint>")
    with pytest.raises(URDFError, match="needs a parent joint"):
        urdf_apply.apply(editor, before[:start] + before[end:])
    assert urdf(editor) == before
    assert editor.undo_stack.index() == index


def test_panel_follows_editor_and_saves_with_yaml(editor, tmp_path):
    from frame_editor_py.commands import Command_SetValue
    from frame_editor_py.interface_urdf import FrameEditor_URDFPanel

    panel = FrameEditor_URDFPanel(editor)
    panel.set_visible(True)
    panel.get_widget().show()
    assert panel.text.toPlainText() == urdf(editor)

    editor.command(Command_SetValue(editor, editor.frames["frame1"], "x", 0.75))
    QApplication.processEvents()
    panel.refresh()  # what the debounce timer does
    assert 'xyz="0.75 0.5 0.5"' in panel.text.toPlainText()

    # user edit -> dirty; editor changes no longer overwrite the text
    panel.text.setPlainText(panel.text.toPlainText().replace('xyz="0.75 0.5 0.5"', 'xyz="0.1 0.5 0.5"'))
    assert panel.dirty
    editor.command(Command_SetValue(editor, editor.frames["frame1"], "y", 0.6))
    QApplication.processEvents()
    assert 'xyz="0.1 0.5 0.5"' in panel.text.toPlainText()
    panel.discard_clicked()
    assert 'xyz="0.75 0.6 0.5"' in panel.text.toPlainText()

    # Save YAML -> .urdf alongside
    yaml_path = str(tmp_path / "cell.yaml")
    editor.save_file(yaml_path)
    panel.yaml_saved(yaml_path)
    assert open(str(tmp_path / "cell.urdf")).read() == urdf(editor)
    assert "Exported" in panel.status.text()


def test_color_edit_in_text_applies(editor):
    text = urdf(editor).replace('rgba="0 0.5 0.5 0.75"', 'rgba="1 0 0 1"')
    changes = urdf_apply.apply(editor, text)
    assert {(c.kind, c.name) for c in changes} == {("color", "frame_sphere")}
    assert tuple(editor.frames["frame_sphere"].color) == (1.0, 0.0, 0.0, 1.0)


def test_changed_lines_are_highlighted(editor):
    from frame_editor_py.commands import Command_SetValue
    from frame_editor_py.interface_urdf import FrameEditor_URDFPanel

    panel = FrameEditor_URDFPanel(editor)
    panel.set_visible(True)
    assert panel.text.extraSelections() == []  # first fill: nothing highlighted

    editor.command(Command_SetValue(editor, editor.frames["frame1"], "z", 0.9))
    QApplication.processEvents()
    panel.refresh()
    highlighted = [s.cursor.block().text().strip() for s in panel.text.extraSelections()]
    assert highlighted == ['<origin xyz="0.5 0.5 0.9" rpy="0 0 0"/>']
    assert "1 line(s) changed" in panel.status.text()

    panel.chk_highlight.setChecked(False)
    assert panel.text.extraSelections() == []


def test_collision_checkbox_feeds_panel_and_service(editor, tmp_path):
    from frame_editor_py.interface_urdf import FrameEditor_URDFPanel, write_urdf

    panel = FrameEditor_URDFPanel(editor)
    panel.set_visible(True)
    assert "<collision>" not in panel.text.toPlainText()
    panel.chk_collision.setChecked(True)
    assert panel.text.toPlainText().count("<collision>") == 1  # only frame_sphere has a shape
    assert len(panel.text.extraSelections()) == 5              # the new <collision> block
    assert urdf_apply.apply(editor, panel.text.toPlainText()) == []  # round trip: nothing to change

    write_urdf(editor, str(tmp_path / "c.urdf"))                 # what save_urdf calls
    assert "<collision>" in open(str(tmp_path / "c.urdf")).read()
