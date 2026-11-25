# Test the source tree, not an installed copy of frame_editor_py.
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
