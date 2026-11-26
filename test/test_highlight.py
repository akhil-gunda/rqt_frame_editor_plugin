"""The yellow lines in the URDF panel must be exactly the lines that changed."""
import pytest

from frame_editor_py import urdf_model as um
from frame_editor_py.interface_urdf import changed_lines
from test_urdf_model import frame

from test_urdf_integration import app, editor, urdf  # noqa: F401  (fixtures)


def block_of(text, name):
    """The non-blank lines of one frame: its <link> up to the end of its <joint>."""
    lines = text.splitlines()
    start = next(i for i, l in enumerate(lines) if l.strip().startswith('<link name="{}"'.format(name)))
    end = next(i for i in range(start, len(lines)) if lines[i].strip() == "</joint>")
    return [i for i in range(start, end + 1) if lines[i].strip()]


@pytest.mark.parametrize("new", ["a_pillar", "b_pillar", "d_pillar"])  # sorts first, middle, last
@pytest.mark.parametrize("style", [{}, {"style": "cube", "length": 1, "width": 1, "height": 1}])
def test_added_frame_highlights_exactly_its_block(new, style):
    shape = {"style": "cube", "length": 1, "width": 1, "height": 1}
    old = [frame("table"), frame("a_post", "table", **shape), frame("c_pillar", "table", **style)]
    before = um.frames_to_urdf(old)[0]
    after = um.frames_to_urdf(old + [frame(new, "table", **style)])[0]
    assert changed_lines(before, after) == (block_of(after, new), 0)


def test_removed_frame_highlights_nothing_and_counts_its_lines():
    old = [frame("table"), frame("a_pillar", "table"), frame("b_pillar", "table")]
    before = um.frames_to_urdf(old)[0]
    after = um.frames_to_urdf([old[0], old[2]])[0]
    assert changed_lines(before, after) == ([], len(block_of(before, "a_pillar")))


def test_rewrite_code_from_frames_highlights_what_it_restores(editor):
    from frame_editor_py.interface_urdf import FrameEditor_URDFPanel
    panel = FrameEditor_URDFPanel(editor)
    panel.set_visible(True)
    original = panel.text.toPlainText()

    # the user changes two values in the code, then asks for it to be rewritten
    edited = original.replace('xyz="0.5 0.5 0.5"', 'xyz="9 9 9"').replace('radius="0.5"', 'radius="7"')
    panel.text.setPlainText(edited)
    panel.discard_clicked()

    assert panel.text.toPlainText() == original == urdf(editor)
    yellow = [s.cursor.block().text().strip() for s in panel.text.extraSelections()]
    assert yellow == ['<origin xyz="0.5 0.5 0.5" rpy="0 0 0"/>', '<sphere radius="0.5"/>']
    assert "2 line(s)" in panel.status.text()


def test_rewrite_without_differences_highlights_nothing(editor):
    from frame_editor_py.interface_urdf import FrameEditor_URDFPanel
    panel = FrameEditor_URDFPanel(editor)
    panel.set_visible(True)
    panel.text.appendPlainText("")
    panel.discard_clicked()
    assert panel.text.extraSelections() == []
