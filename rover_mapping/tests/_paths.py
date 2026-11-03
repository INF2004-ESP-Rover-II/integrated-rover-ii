"""Make Buddy 1's dashboard modules importable (dfs_explorer, laptop_navigation, vision_feed)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for candidate in (os.path.join(HERE, "..", "..", "dashboard"), os.path.join(HERE, "..", "dashboard")):
    if os.path.isdir(candidate) and candidate not in sys.path:
        sys.path.insert(0, os.path.abspath(candidate))
