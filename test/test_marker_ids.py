"""Duplicated frames must get their own RViz marker id (RViz keys markers by ns + id)."""
from types import SimpleNamespace

from frame_editor_py.commands import Command_CopyElement
from frame_editor_py.objects import Frame

from test_urdf_integration import app, editor  # noqa: F401  (fixtures)


def fake_tf_buffer():
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    return SimpleNamespace(lookup_transform=lambda *a, **k: SimpleNamespace(transform=transform))


def test_duplicates_get_unique_marker_ids(editor, monkeypatch):
    monkeypatch.setattr(Frame, "tf_buffer", fake_tf_buffer())
    for i in range(4):
        editor.command(Command_CopyElement(editor, "pillar{}".format(i), "frame_cube", "frame1"))
    editor.command(Command_CopyElement(editor, "ball", "frame_sphere", "frame1"))

    ids = [f.marker.id for f in editor.frames.values() if f.marker is not None]
    assert len(ids) == len(set(ids)), "marker ids collide: {}".format(sorted(ids))
