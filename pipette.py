#!/usr/bin/env python3
"""Thin wrapper: the driver + command line live in ros/harness_pipette/harness_pipette/pipette.py (shared with the node)."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "ros", "harness_pipette"))
from harness_pipette.pipette import main
if __name__ == "__main__":
    main()
