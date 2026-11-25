"""The style panel's colour button: picking a colour changes the frame, cancelling does not."""
import pytest
from python_qt_binding import QtWidgets
from python_qt_binding.QtGui import QColor

from frame_editor_py.interface_gui import FrameEditor_StyleWidget

from test_urdf_integration import app, editor  # noqa: F401  (fixtures)


def press_color_button(editor, monkeypatch, picked):
    monkeypatch.setattr(QtWidgets.QColorDialog, "getColor", staticmethod(lambda *a, **k: picked))
    editor.active_frame = editor.frames["frame_sphere"]
    FrameEditor_StyleWidget(editor).btn_color_clicked()
    return editor.frames["frame_sphere"]


def test_picking_a_colour_changes_frame_and_marker(editor, monkeypatch):
    frame = press_color_button(editor, monkeypatch, QColor(255, 0, 0, 128))
    assert tuple(frame.color) == pytest.approx((1.0, 0.0, 0.0, 128 / 255.0), abs=1e-3)
    assert frame.marker.color.r == pytest.approx(1.0)


def test_cancelling_keeps_the_colour(editor, monkeypatch):
    before = tuple(editor.frames["frame_sphere"].color)
    frame = press_color_button(editor, monkeypatch, QColor())  # invalid colour = cancelled
    assert tuple(frame.color) == before


def test_alpha_field_changes_only_transparency(editor):
    from frame_editor_py.interface_urdf import generate_urdf
    editor.active_frame = editor.frames["frame_sphere"]
    widget = FrameEditor_StyleWidget(editor)
    widget.update_color_label(editor.active_frame)
    assert widget.alpha_spinbox.value() == pytest.approx(0.75)

    widget.alpha_spinbox.setValue(0.3)
    widget.alpha_changed()
    frame = editor.frames["frame_sphere"]
    assert tuple(frame.color) == pytest.approx((0.0, 0.5, 0.5, 0.3))
    assert frame.marker.color.a == pytest.approx(0.3)
    assert 'rgba="0 0.5 0.5 0.3"' in generate_urdf(editor)[0]

    editor.undo_stack.undo()
    assert editor.frames["frame_sphere"].color[3] == pytest.approx(0.75)
