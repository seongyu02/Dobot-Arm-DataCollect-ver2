#!/usr/bin/env python3
"""Run merge.py from project root. Use: python run_merge.py (from src or from Dobot_E6_Moveit2)."""
import os
import sys

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
merge_py = os.path.join(root, 'merge.py')
os.chdir(root)
sys.path.insert(0, root)
sys.path.insert(0, os.path.join(root, 'src'))

# Run merge.py with __file__ set so its path logic works
with open(merge_py, encoding='utf-8') as f:
    code = f.read()
globals_ = {'__name__': '__main__', '__file__': merge_py, '__builtins__': __builtins__}
exec(compile(code, merge_py, 'exec'), globals_)
