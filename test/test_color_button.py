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
