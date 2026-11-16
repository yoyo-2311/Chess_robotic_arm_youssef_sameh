"""Make the sibling chess_brain_handover package (and its vendored python-chess) importable for the tests."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRC = os.path.dirname(PKG)
for p in (PKG, os.path.join(SRC, 'chess_brain_handover'), os.path.join(SRC, 'chess_brain_handover', 'chess_brain_handover', '_vendor')):
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)
