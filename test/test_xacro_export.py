"""xacro export: the macro must expand to the same cell as the direct URDF export."""
import pytest

from frame_editor_py import urdf_model as um
from frame_editor_py.urdf_model import URDFError
from test_urdf_model import cell, frame

xacro = pytest.importorskip("xacro")


def expand(tmp_path, macro_text, top_text, macro_file="my_cell_macro.xacro"):
    (tmp_path / macro_file).write_text(macro_text)
    top = tmp_path / "my_cell.urdf.xacro"
    top.write_text(top_text)
    return xacro.process_file(str(top)).toprettyxml(indent="  ")


def test_expands_to_the_same_cell(tmp_path):
    frames = cell()
    macro, top, _ = um.frames_to_xacro(frames, "my_cell", "my_cell_macro.xacro", collision=True)
    assert "prefix" not in macro + top
    assert '<link name="table">' in macro and '<xacro:insert_block name="origin"/>' in macro

    expanded, roots = um.urdf_to_specs(expand(tmp_path, macro, top))
    assert roots == ["world"]
    base = expanded.pop("my_cell_base")  # the macro's base link stands in for the editor root
    assert base.parent == "world"

    original = um.specs_from_frames(frames)
    assert set(expanded) == set(original)
    for name, spec in original.items():
        if spec.parent == "world":
            spec.parent = "my_cell_base"
    assert um.diff_specs(original, expanded) == []


def test_macro_attaches_to_another_robot(tmp_path):
    macro, _, _ = um.frames_to_xacro(cell(), "my_cell", "my_cell_macro.xacro")
    top = ('<robot name="big" xmlns:xacro="http://www.ros.org/wiki/xacro">'
           '<xacro:include filename="my_cell_macro.xacro"/><link name="hall_floor"/>'
           '<xacro:my_cell parent="hall_floor"><origin xyz="3 0 0" rpy="0 0 0"/></xacro:my_cell>'
           "</robot>")
    specs, roots = um.urdf_to_specs(expand(tmp_path, macro, top))
    assert roots == ["hall_floor"]
    assert specs["my_cell_base"].parent == "hall_floor"
    assert specs["my_cell_base"].position == (3.0, 0.0, 0.0)
    assert specs["robot_base"].parent == "table"


def test_needs_a_single_root():
    with pytest.raises(URDFError, match="exactly one root"):
        um.frames_to_xacro([frame("a", "world"), frame("b", "tool0")])


def test_macro_name_is_a_valid_identifier():
    assert um.macro_name_for("1st cell-v2") == "cell_1st_cell_v2"
